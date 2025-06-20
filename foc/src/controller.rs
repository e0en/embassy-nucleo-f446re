use libm::fmodf;

use crate::{
    motor::DutyCycle3Phase,
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
    pole_count: u16,
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
        set_motor(zero_signal);

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
            set_motor(signal);
            let (_, _measured_velocity, _) = read_sensor();
        }

        // do the same for reversed direction
        for _ in 0..10 {
            target_angle.0 -= 0.1;
            let signal = svpwm(align_voltage, Radian(target_angle.0), self.psu_voltage)?;
            set_motor(signal);
            let (_, _measured_velocity, _) = read_sensor();
        }
        self.sensor_direction = Direction::Clockwise;

        Ok(())
    }

    fn to_electrical_angle(&self, mechanical_angle: Radian) -> Radian {
        let full_angle = (mechanical_angle.0 - self.bias_angle.0) / (self.pole_count as f32);
        let angle = fmodf(full_angle, 2.0 * core::f32::consts::PI);
        Radian(angle)
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
        let electrical_angle = self.to_electrical_angle(mechanical_angle);
        let duty_cycle = svpwm(v_ref, electrical_angle, self.psu_voltage)?;
        Ok(duty_cycle)
    }
}
