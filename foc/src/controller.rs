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
pub struct MotorSetup {
    pub pole_pair_count: u8,
    pub phase_resistance: f32,
}

#[derive(Clone, Copy)]
pub struct FocState {
    pub is_running: bool,
    pub mode: RunMode,
    pub filtered_velocity: RadianPerSecond,
    pub velocity_error: RadianPerSecond,
    pub velocity_integral: f32,
    pub angle_error: Radian,
    pub i_ref: f32,

    pub i_q: f32,
    pub i_d: f32,
    pub v_q: f32,
    pub v_d: f32,

    pub i_q_error: f32,
    pub i_q_integral: f32,
    pub electrical_angle: Radian,
    pub dt: Second,
}

pub struct FocController {
    pub motor: MotorSetup,
    pub bias_angle: Radian,
    pub current_phase_bias: Radian,
    pub sensor_direction: Direction,
    pub current_mapping: (u8, u8, u8),
    pub state: Option<FocState>,
    psu_voltage: f32,
    torque_limit: Option<f32>,
    current_limit: Option<f32>,
    velocity_limit: Option<RadianPerSecond>,
    current_q_pid: PIDController,
    current_d_pid: PIDController,
    angle_pid: PIDController,
    velocity_pid: PIDController,
    _velocity_output_limit: f32, // Volts per second
    velocity_filter: LowPassFilter,
    current_q_filter: LowPassFilter,
    current_d_filter: LowPassFilter,
    mode: RunMode,
    target: Target,
    is_running: bool,
}

impl FocController {
    pub fn new(
        motor: MotorSetup,
        psu_voltage: f32,
        current_pid_gains: PID,
        angle_pid_gains: PID,
        velocity_pid_gains: PID,
        max_voltage: f32,
        max_voltage_ramp: f32,
    ) -> Self {
        Self {
            motor,
            bias_angle: Radian::new(0.0),
            current_phase_bias: Radian::new(0.0),
            sensor_direction: Direction::Clockwise,
            current_mapping: (0, 1, 2),
            state: None,
            psu_voltage,
            torque_limit: None,
            current_limit: None,
            velocity_limit: None,
            current_q_pid: PIDController::new(current_pid_gains, max_voltage, max_voltage_ramp),
            current_d_pid: PIDController::new(current_pid_gains, max_voltage, max_voltage_ramp),
            angle_pid: PIDController::new(angle_pid_gains, max_voltage, max_voltage_ramp),
            velocity_pid: PIDController::new(velocity_pid_gains, max_voltage, max_voltage_ramp),
            velocity_filter: LowPassFilter::new(0.01),
            _velocity_output_limit: 1000.0,
            current_q_filter: LowPassFilter::new(0.003),
            current_d_filter: LowPassFilter::new(0.003),
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

    pub fn to_electrical_angle(&self, mechanical_angle: Radian) -> Radian {
        let mut full_angle =
            (mechanical_angle - self.bias_angle) * (self.motor.pole_pair_count as f32);
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

    pub fn reset(&mut self) {
        self.current_q_pid.reset();
        self.current_d_pid.reset();
        self.angle_pid.reset();
        self.velocity_pid.reset();
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

    pub fn map_currents(&self, ia: f32, ib: f32, ic: f32) -> (f32, f32, f32) {
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
            angle_error: Radian::new(0.0),
            filtered_velocity: RadianPerSecond(0.0),
            velocity_error: RadianPerSecond(0.0),
            velocity_integral: 0.0,
            dt: Second(0.0),
            electrical_angle: Radian::new(0.0),

            i_q: 0.0,
            i_d: 0.0,
            v_q: 0.0,
            v_d: 0.0,

            i_q_error: 0.0,
            i_q_integral: 0.0,
            i_ref: 0.0,
        };

        let s = angle_reading;
        new_state.dt = s.dt;
        let velocity = RadianPerSecond(self.velocity_filter.apply(s.velocity.0, s.dt));
        new_state.filtered_velocity = velocity;
        let mut i_ref = match &self.mode {
            RunMode::Angle => {
                let angle_error = self.target.angle - s.angle;
                new_state.angle_error = angle_error;
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
                new_state.angle_error = angle_error;
                new_state.velocity_error = velocity_error;
                self.target.spring * angle_error.angle
                    + self.target.damping * velocity_error.0
                    + self.target.torque
            }
            RunMode::Torque => self.target.torque,
        };

        if self.sensor_direction == Direction::CounterClockWise {
            i_ref *= -1.0;
        }

        if let Some(c) = self.current_limit {
            i_ref = i_ref.min(c).max(-c);
        }

        // TODO: implement actual torque calculation
        if let Some(t) = self.torque_limit {
            i_ref = i_ref.min(t).max(-t);
        }

        if !self.is_running {
            i_ref = 0.0;
        }

        new_state.i_ref = i_ref;
        let electrical_angle = self.to_electrical_angle(s.angle);
        new_state.electrical_angle = electrical_angle;

        let (ia, ib, ic) = self.map_currents(ia, ib, ic);

        // clarke transform
        let (i_alpha, i_beta) = clarke_transform(ia, ib, ic);

        // park transform
        let current_angle = electrical_angle + self.current_phase_bias;
        let (mut i_q, mut i_d) = park_transform(i_alpha, i_beta, current_angle);

        i_q = self.current_q_filter.apply(i_q, s.dt);
        i_d = self.current_d_filter.apply(i_d, s.dt);

        new_state.i_q = i_q;
        new_state.i_d = i_d;

        let v_q = self.current_q_pid.update(i_ref - i_q, s.dt);
        let v_d = self.current_d_pid.update(-i_d, s.dt);

        new_state.i_q_error = self.current_q_pid.last_error;
        new_state.i_q_integral = self.current_q_pid.integral;

        new_state.v_q = v_q;
        new_state.v_d = v_d;

        let duty_cycle = svpwm(v_q, v_d, electrical_angle, self.psu_voltage)?;
        self.state = Some(new_state);
        Ok(duty_cycle)
    }

    pub fn get_vq_duty_cycle(
        &mut self,
        v_q: f32,
        angle: Radian,
    ) -> Result<DutyCycle3Phase, FocError> {
        let duty_cycle = svpwm(v_q, 0.0, self.to_electrical_angle(angle), self.psu_voltage)?;
        Ok(duty_cycle)
    }

    pub fn set_current_phase_bias(&mut self, angle: f32) {
        self.current_phase_bias = Radian::new(angle);
    }
}

pub fn clarke_transform(ia: f32, ib: f32, ic: f32) -> (f32, f32) {
    let mid = (ia + ib + ic) / 3.0;
    let a = ia - mid;
    let b = ib - mid;

    let i_alpha = a;
    let i_beta = INV_SQRT3 * a + 2.0 * INV_SQRT3 * b;
    (i_alpha, i_beta)
}

pub fn park_transform(alpha: f32, beta: f32, mut angle: Radian) -> (f32, f32) {
    let (sin, cos) = angle.get_sin_cos();
    let i_q = beta * cos - alpha * sin;
    let i_d = alpha * cos + beta * sin;
    (i_q, i_d)
}
