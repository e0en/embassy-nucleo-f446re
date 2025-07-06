use libm::fmodf;

use crate::{
    angle_input::AngleReading,
    lowpass_filter::LowPassFilter,
    pid::{PID, PIDController},
    pwm::FocError,
    pwm_output::DutyCycle3Phase,
    svpwm,
    units::{Radian, RadianPerSecond, Second},
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Clockwise,
    CounterClockWise,
}

#[derive(Clone, Copy)]
pub enum Command {
    Angle(Radian),
    Velocity(RadianPerSecond),
}

#[derive(Clone, Copy)]
pub struct FocState {
    pub filtered_velocity: RadianPerSecond,
    pub velocity_error: RadianPerSecond,
    pub velocity_integral: f32,
    pub angle_error: Option<Radian>,
    pub v_ref: f32,
    pub electrical_angle: Radian,
    pub dt: Second,
}

pub struct FocController {
    pub bias_angle: Radian,
    pub sensor_direction: Direction,
    pub state: Option<FocState>,
    command: Option<Command>,
    psu_voltage: f32,
    angle_pid: PIDController,
    velocity_pid: PIDController,
    _velocity_output_limit: f32, // Volts per second
    velocity_filter: LowPassFilter,
    pole_pair_count: u16,
}

impl FocController {
    pub fn new(
        psu_voltage: f32,
        pole_pair_count: u16,
        angle_pid_gains: PID,
        velocity_pid_gains: PID,
        max_voltage: f32,
        max_voltage_ramp: f32,
    ) -> Self {
        Self {
            bias_angle: Radian(0.0),
            sensor_direction: Direction::Clockwise,
            command: None,
            state: None,
            psu_voltage,
            angle_pid: PIDController::new(angle_pid_gains, max_voltage, max_voltage_ramp),
            velocity_pid: PIDController::new(velocity_pid_gains, max_voltage, max_voltage_ramp),
            velocity_filter: LowPassFilter::new(0.01),
            _velocity_output_limit: 1000.0,
            pole_pair_count,
        }
    }
    pub async fn align_sensor<'a, FSensor, FMotor, FWait, FutUnit>(
        &mut self,
        align_voltage: f32,
        mut read_sensor: FSensor,
        mut set_motor: FMotor,
        wait_function: FWait,
    ) -> Result<(), FocError>
    where
        FSensor: AsyncFnMut() -> AngleReading + Send + 'a,
        FMotor: FnMut(DutyCycle3Phase),
        FWait: Fn(Second) -> FutUnit + 'a,
        FutUnit: Future<Output = ()> + Send + 'a,
    {
        // keep sending zero
        let zero_signal = svpwm(
            align_voltage,
            Radian(3.0 * core::f32::consts::FRAC_PI_2),
            self.psu_voltage,
        )?;
        set_motor(zero_signal);

        // monitor mechanical angle until convergence
        let mut angle = Radian(0.0);
        let mut last_angle = Radian(0.0);
        for _ in 0..100 {
            angle = read_sensor().await.angle;
            wait_function(Second(0.01)).await;
            if angle == last_angle {
                break;
            }
            last_angle = angle;
        }
        self.bias_angle = angle;

        // start rotating slowly and monitor velocity
        let mut target_angle = Radian(0.0);
        let mut velocity_sum = RadianPerSecond(0.0);
        for _ in 0..100 {
            target_angle.0 += 0.01;
            let signal = svpwm(align_voltage, target_angle, self.psu_voltage)?;
            set_motor(signal);
            wait_function(Second(0.01)).await;
            velocity_sum += read_sensor().await.velocity;
        }
        let direction_forward = velocity_sum.0 > 0.0;

        // do the same for reversed direction
        velocity_sum = RadianPerSecond(0.0);
        for _ in 0..100 {
            target_angle.0 -= 0.01;
            let signal = svpwm(align_voltage, target_angle, self.psu_voltage)?;
            set_motor(signal);
            wait_function(Second(0.01)).await;
            velocity_sum += read_sensor().await.velocity;
        }
        let direction_backward = velocity_sum.0 < 0.0;

        match (direction_forward, direction_backward) {
            (true, true) => self.sensor_direction = Direction::Clockwise,
            (false, false) => self.sensor_direction = Direction::CounterClockWise,
            _ => return Err(FocError::AlignError),
        }
        Ok(())
    }

    fn to_electrical_angle(&self, mechanical_angle: Radian) -> Radian {
        let mut full_angle = (mechanical_angle - self.bias_angle) * (self.pole_pair_count as f32);
        if self.sensor_direction == Direction::CounterClockWise {
            full_angle = full_angle * -1.0;
        }
        let angle = fmodf(full_angle.0, 2.0 * core::f32::consts::PI);
        Radian(angle)
    }

    pub fn set_command(&mut self, command: Command) {
        self.command = Some(command);
    }
    pub fn get_duty_cycle(
        &mut self,
        angle_reading: &AngleReading,
    ) -> Result<DutyCycle3Phase, FocError> {
        let mut new_state = FocState {
            angle_error: None,
            filtered_velocity: RadianPerSecond(0.0),
            velocity_error: RadianPerSecond(0.0),
            velocity_integral: 0.0,
            dt: Second(0.0),
            electrical_angle: Radian(0.0),
            v_ref: 0.0,
        };

        let s = angle_reading;
        new_state.dt = s.dt;
        let velocity = RadianPerSecond(self.velocity_filter.apply(s.velocity.0, s.dt));
        new_state.filtered_velocity = velocity;
        let v_ref = match &self.command {
            Some(Command::Angle(target_angle)) => {
                let angle_error = *target_angle - s.angle;
                new_state.angle_error = Some(angle_error);
                let target_velocity = self.angle_pid.update(angle_error.0, s.dt);
                let velocity_error = RadianPerSecond(target_velocity) - velocity;
                new_state.velocity_error = velocity_error;
                new_state.velocity_integral = self.velocity_pid.integral;
                self.velocity_pid.update(velocity_error.0, s.dt)
            }
            Some(Command::Velocity(target_velocity)) => {
                let velocity_error = *target_velocity - velocity;
                new_state.velocity_error = velocity_error;
                new_state.velocity_integral = self.velocity_pid.integral;
                self.velocity_pid.update(velocity_error.0, s.dt)
            }
            None => 0.0,
        };
        new_state.v_ref = v_ref;
        let electrical_angle = self.to_electrical_angle(s.angle);
        new_state.electrical_angle = electrical_angle;
        let duty_cycle = svpwm(v_ref, electrical_angle, self.psu_voltage)?;
        self.state = Some(new_state);
        Ok(duty_cycle)
    }
}
