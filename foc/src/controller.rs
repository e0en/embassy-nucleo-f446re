use libm::fmodf;

use crate::{
    motor::DutyCycle3Phase,
    pid::PIDController,
    pwm::FocError,
    sensor::SensorReading,
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
    pub async fn align_sensor<FSensor, FutSensor, FMotor, FutMotor>(
        &mut self,
        align_voltage: f32,
        read_sensor: FSensor,
        set_motor: FMotor,
    ) -> Result<(), FocError>
    where
        FSensor: Fn() -> FutSensor,
        FutSensor: Future<Output = SensorReading> + Send,
        FMotor: Fn(DutyCycle3Phase) -> FutMotor,
        FutMotor: Future<Output = ()> + Send,
    {
        // keep sending zero
        let zero_signal = svpwm(align_voltage, Radian(0.0), self.psu_voltage)?;
        set_motor(zero_signal).await;

        // monitor mechanical angle until convergence
        let mut angle = Radian(0.0);
        for _ in 0..10 {
            angle = read_sensor().await.angle;
        }
        self.bias_angle = angle;

        // start rotating slowly and monitor velocity
        let mut target_angle = Radian(0.0);
        for _ in 0..10 {
            target_angle.0 += 0.1;
            let signal = svpwm(align_voltage, target_angle, self.psu_voltage)?;
            set_motor(signal).await;
            let _velocity = read_sensor().await.velocity;
        }

        // do the same for reversed direction
        for _ in 0..10 {
            target_angle.0 -= 0.1;
            let signal = svpwm(align_voltage, Radian(target_angle.0), self.psu_voltage)?;
            set_motor(signal).await;
            let _measured_velocity = read_sensor().await.velocity;
        }
        self.sensor_direction = Direction::Clockwise;

        Ok(())
    }

    fn to_electrical_angle(&self, mechanical_angle: Radian) -> Radian {
        let full_angle = (mechanical_angle - self.bias_angle) / (self.pole_count as f32);
        let angle = fmodf(full_angle.0, 2.0 * core::f32::consts::PI);
        Radian(angle)
    }

    pub fn set_command(&mut self, command: Command) {
        self.command = Some(command);
    }
    pub fn get_duty_cycle(
        &mut self,
        read_sensor: fn() -> SensorReading,
    ) -> Result<DutyCycle3Phase, FocError> {
        let s = read_sensor();
        let v_ref = match &self.command {
            Some(Command::Angle(target_angle)) => {
                let angle_error = *target_angle - s.angle;
                let target_velocity = self.angle_pid.update(angle_error.0, s.dt);
                let velocity_error = RadianPerSecond(target_velocity) - s.velocity;
                self.velocity_pid.update(velocity_error.0, s.dt)
            }
            Some(Command::Velocity(target_velocity)) => {
                let error = *target_velocity - s.velocity;
                self.velocity_pid.update(error.0, s.dt)
            }
            None => 0.0,
        };
        let electrical_angle = self.to_electrical_angle(s.angle);
        let duty_cycle = svpwm(v_ref, electrical_angle, self.psu_voltage)?;
        Ok(duty_cycle)
    }
}
