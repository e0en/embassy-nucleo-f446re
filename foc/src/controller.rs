use crate::{
    pid::PIDController,
    pwm::FocError,
    svpwm,
    units::{Radian, RadianPerSecond},
};

pub enum Direction {
    Clockwise,
    CounterClockWise,
}

#[derive(Clone, Copy)]
pub enum Command {
    Angle(Radian),
    Velocity(RadianPerSecond),
}

pub struct FocController {
    bias_angle: Radian,
    sensor_direction: Direction,
    command: Option<Command>,
    psu_voltage: f32,
    angle_pid: PIDController,
    velocity_pid: PIDController,
}

pub struct DutyCycle3Phase {
    pub t1: f32,
    pub t2: f32,
    pub t3: f32,
}

impl DutyCycle3Phase {
    pub fn new(phases: (f32, f32, f32)) -> Self {
        Self {
            t1: phases.0,
            t2: phases.1,
            t3: phases.2,
        }
    }
}

impl FocController {
    pub fn align_sensor(
        &mut self,
        align_voltage: f32,
        read_sensor: fn() -> (Radian, RadianPerSecond, f32),
        set_motor: fn(DutyCycle3Phase) -> (),
    ) -> Result<(), FocError> {
        // keep sending zero
        let zero_signal = svpwm(align_voltage, Radian(0.0), self.psu_voltage)?;
        set_motor(DutyCycle3Phase::new(zero_signal));

        // monitor mechanical angle until convergence
        let mut angle = Radian(0.0);
        for _ in 0..10 {
            angle = read_sensor().0;
        }
        self.bias_angle = angle;

        // start rotating slowly and monitor velocity
        let mut target_angle = Radian(0.0);
        for _ in 0..10 {
            target_angle.0 += 0.1;
            let signal = svpwm(align_voltage, target_angle, self.psu_voltage)?;
            set_motor(DutyCycle3Phase::new(signal));
            let (_, _measured_velocity, _) = read_sensor();
        }

        // do the same for reversed direction
        for _ in 0..10 {
            target_angle.0 -= 0.1;
            let signal = svpwm(align_voltage, Radian(target_angle.0), self.psu_voltage)?;
            set_motor(DutyCycle3Phase::new(signal));
            let (_, _measured_velocity, _) = read_sensor();
        }
        self.sensor_direction = Direction::Clockwise;

        Ok(())
    }
    pub fn set_command(&mut self, command: Command) {
        self.command = Some(command);
    }
    pub fn get_duty_cycle(
        &mut self,
        read_sensor: fn() -> (Radian, RadianPerSecond, f32),
    ) -> Result<DutyCycle3Phase, FocError> {
        let (mechanical_angle, measured_velocity, dt) = read_sensor();
        let v_ref = match &self.command {
            Some(Command::Angle(target_angle)) => {
                let angle_error = target_angle.0 - mechanical_angle.0;
                let target_velocity = self.angle_pid.update(angle_error, dt);
                let velocity_error = target_velocity - measured_velocity.0;
                self.velocity_pid.update(velocity_error, dt)
            }
            Some(Command::Velocity(target_velocity)) => {
                let error = target_velocity.0 - measured_velocity.0;
                self.velocity_pid.update(error, dt)
            }
            None => 0.0,
        };
        let _electrical_angle = mechanical_angle.0 - self.bias_angle.0;
        let uptimes = svpwm(v_ref, mechanical_angle, self.psu_voltage)?;
        Ok(DutyCycle3Phase {
            t1: uptimes.0,
            t2: uptimes.1,
            t3: uptimes.2,
        })
    }
}
