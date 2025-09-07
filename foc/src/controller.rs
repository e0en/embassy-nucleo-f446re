use libm::fmodf;

const INV_SQRT3: f32 = 0.577_350_26;

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

pub struct Target {
    pub angle: Radian,
    pub velocity: RadianPerSecond,
    pub torque: f32,
    pub spring: f32,
    pub damping: f32,
}

#[derive(Clone, Copy)]
pub enum RunMode {
    Angle,
    Velocity,
    Torque,
    Impedance,
}

#[derive(Clone, Copy)]
pub struct ImpedanceParameter {
    pub angle: Radian,
    pub velocity: RadianPerSecond,
    pub spring: f32,
    pub damping: f32,
    pub torque: f32,
}

#[derive(Clone, Copy)]
pub struct FocState {
    pub is_running: bool,
    pub mode: RunMode,
    pub filtered_velocity: RadianPerSecond,
    pub velocity_error: RadianPerSecond,
    pub velocity_integral: f32,
    pub angle_error: Option<Radian>,
    pub v_ref: f32,
    pub i_q: f32,
    pub i_d: f32,
    pub electrical_angle: Radian,
    pub dt: Second,
}

pub struct FocController {
    pub bias_angle: Radian,
    pub sensor_direction: Direction,
    pub current_mapping: (u8, u8, u8),
    pub state: Option<FocState>,
    psu_voltage: f32,
    torque_limit: Option<f32>,
    current_limit: Option<f32>,
    velocity_limit: Option<RadianPerSecond>,
    angle_pid: PIDController,
    velocity_pid: PIDController,
    _velocity_output_limit: f32, // Volts per second
    velocity_filter: LowPassFilter,
    pole_pair_count: u16,
    mode: RunMode,
    target: Target,
    is_running: bool,
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
            bias_angle: Radian::new(0.0),
            sensor_direction: Direction::Clockwise,
            current_mapping: (0, 1, 2),
            state: None,
            psu_voltage,
            torque_limit: None,
            current_limit: None,
            velocity_limit: None,
            angle_pid: PIDController::new(angle_pid_gains, max_voltage, max_voltage_ramp),
            velocity_pid: PIDController::new(velocity_pid_gains, max_voltage, max_voltage_ramp),
            velocity_filter: LowPassFilter::new(0.01),
            _velocity_output_limit: 1000.0,
            pole_pair_count,
            mode: RunMode::Impedance,
            target: Target {
                angle: Radian::new(0.0),
                velocity: RadianPerSecond(0.0),
                torque: 0.0,
                spring: 0.0,
                damping: 0.0,
            },
            is_running: false,
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
            0.0,
            Radian::new(3.0 * core::f32::consts::FRAC_PI_2),
            self.psu_voltage,
        )?;
        set_motor(zero_signal);

        // monitor mechanical angle until convergence
        let mut angle = Radian::new(0.0);
        let mut last_angle = Radian::new(0.0);
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
        let mut target_angle = Radian::new(0.0);
        let mut velocity_sum = RadianPerSecond(0.0);
        for _ in 0..100 {
            target_angle.angle += 0.01;
            let signal = svpwm(align_voltage, 0.0, target_angle, self.psu_voltage)?;
            set_motor(signal);
            wait_function(Second(0.01)).await;
            velocity_sum += read_sensor().await.velocity;
        }
        let direction_forward = velocity_sum.0 > 0.0;

        // do the same for reversed direction
        velocity_sum = RadianPerSecond(0.0);
        for _ in 0..100 {
            target_angle.angle -= 0.01;
            let signal = svpwm(align_voltage, 0.0, target_angle, self.psu_voltage)?;
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
        let angle = fmodf(full_angle.angle, 2.0 * core::f32::consts::PI);
        Radian::new(angle)
    }

    pub fn stop(&mut self) {
        self.is_running = false;
    }

    pub fn enable(&mut self) {
        self.is_running = true;
    }

    pub fn set_run_mode(&mut self, mode: RunMode) {
        self.mode = mode;
    }

    pub fn set_target_angle(&mut self, angle: Radian) {
        self.target.angle = angle;
    }

    pub fn set_target_velocity(&mut self, velocity: RadianPerSecond) {
        self.target.velocity = velocity;
    }

    pub fn set_target_torque(&mut self, torque: f32) {
        self.target.torque = torque;
    }

    pub fn set_spring(&mut self, spring: f32) {
        self.target.spring = spring;
    }

    pub fn set_damping(&mut self, damping: f32) {
        self.target.damping = damping;
    }

    pub fn set_velocity_limit(&mut self, velocity: RadianPerSecond) {
        self.velocity_limit = Some(velocity);
    }

    pub fn set_torque_limit(&mut self, torque: f32) {
        self.torque_limit = Some(torque);
    }

    pub fn set_current_limit(&mut self, current: f32) {
        self.current_limit = Some(current);
    }

    pub fn set_velocity_kp(&mut self, kp: f32) {
        self.velocity_pid.gains.p = kp;
    }

    pub fn set_velocity_ki(&mut self, ki: f32) {
        self.velocity_pid.gains.i = ki;
    }

    pub fn set_velocity_gain(&mut self, tf: f32) {
        self.velocity_filter = LowPassFilter::new(tf);
    }

    pub fn set_angle_kp(&mut self, kp: f32) {
        self.angle_pid.gains.p = kp;
    }

    fn map_currents(&self, ia: f32, ib: f32, ic: f32) -> (f32, f32, f32) {
        let (ma, mb, mc) = self.current_mapping;
        (
            self.get_current_at(ma, ia, ib, ic),
            self.get_current_at(mb, ia, ib, ic),
            self.get_current_at(mc, ia, ib, ic),
        )
    }
    fn get_current_at(&self, index: u8, ia: f32, ib: f32, ic: f32) -> f32 {
        match index {
            0 => ia,
            1 => ib,
            2 => ic,
            _ => panic!(),
        }
    }

    pub fn get_duty_cycle(
        &mut self,
        angle_reading: &AngleReading,
        ia: f32,
        ib: f32,
        ic: f32,
    ) -> Result<DutyCycle3Phase, FocError> {
        let mut new_state = FocState {
            is_running: self.is_running,
            mode: self.mode,
            angle_error: None,
            filtered_velocity: RadianPerSecond(0.0),
            velocity_error: RadianPerSecond(0.0),
            velocity_integral: 0.0,
            dt: Second(0.0),
            electrical_angle: Radian::new(0.0),
            i_q: 0.0,
            i_d: 0.0,
            v_ref: 0.0,
        };

        let s = angle_reading;
        new_state.dt = s.dt;
        let velocity = RadianPerSecond(self.velocity_filter.apply(s.velocity.0, s.dt));
        new_state.filtered_velocity = velocity;
        let mut v_ref = match &self.mode {
            RunMode::Angle => {
                let angle_error = self.target.angle - s.angle;
                new_state.angle_error = Some(angle_error);
                let mut target_velocity = self.angle_pid.update(angle_error.angle, s.dt);
                if let Some(v) = self.velocity_limit {
                    target_velocity = target_velocity.min(v.0).max(-v.0);
                }
                let velocity_error = RadianPerSecond(target_velocity) - velocity;
                new_state.velocity_error = velocity_error;
                new_state.velocity_integral = self.velocity_pid.integral;
                self.velocity_pid.update(velocity_error.0, s.dt)
            }
            RunMode::Velocity => {
                let velocity_error = self.target.velocity - velocity;
                new_state.velocity_error = velocity_error;
                new_state.velocity_integral = self.velocity_pid.integral;
                self.velocity_pid.update(velocity_error.0, s.dt)
            }
            RunMode::Impedance => {
                let angle_error = self.target.angle - s.angle;
                let velocity_error = self.target.velocity - velocity;
                new_state.angle_error = Some(angle_error);
                new_state.velocity_error = velocity_error;
                self.target.spring * angle_error.angle
                    + self.target.damping * velocity_error.0
                    + self.target.torque
            }
            RunMode::Torque => self.target.torque,
        };

        if self.sensor_direction == Direction::CounterClockWise {
            v_ref *= -1.0;
        }

        if let Some(c) = self.current_limit {
            v_ref = v_ref.min(c).max(-c);
        }

        // TODO: implement actual torque calculation
        if let Some(t) = self.torque_limit {
            v_ref = v_ref.min(t).max(-t);
        }

        if !self.is_running {
            v_ref = 0.0;
        }

        new_state.v_ref = v_ref;
        let mut electrical_angle = self.to_electrical_angle(s.angle);
        new_state.electrical_angle = electrical_angle;

        let (ia, ib, ic) = self.map_currents(ia, ib, ic);
        let mid = (ia + ib + ic) / 3.0;
        let a = ia - mid;
        let b = ib - mid;
        // clarke transform
        let i_alpha = a - mid;
        let i_beta = INV_SQRT3 * a + 2.0 * INV_SQRT3 * b;

        // park transform
        let (sin, cos) = electrical_angle.get_sin_cos();
        new_state.i_q = i_beta * cos - i_alpha * sin;
        new_state.i_d = i_alpha * cos + i_beta * sin;

        let duty_cycle = svpwm(v_ref, 0.0, electrical_angle, self.psu_voltage)?;
        self.state = Some(new_state);
        Ok(duty_cycle)
    }
}
