use libm::fmodf;

use crate::current::PhaseCurrent;

const INV_SQRT3: f32 = 0.577_350_26;

use crate::{
    angle_input::AngleReading,
    lowpass_filter::LowPassFilter,
    pid::{PID, PIDController},
    pwm::FocError,
    pwm_output::DutyCycle3Phase,
    svpwm,
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Clockwise,
    CounterClockWise,
}

pub struct Target {
    pub angle: f32,
    pub velocity: f32,
    pub torque: f32,
    pub spring: f32,
    pub damping: f32,
    pub voltage: f32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    Angle,
    Velocity,
    Torque,
    Impedance,
    Voltage,
}

#[derive(Clone, Copy)]
pub struct ImpedanceParameter {
    pub angle: f32,
    pub velocity: f32,
    pub spring: f32,
    pub damping: f32,
    pub torque: f32,
}

#[derive(Clone, Copy)]
pub struct MotorSetup {
    pub pole_pair_count: u8,
    pub phase_resistance: f32,
    pub max_velocity: f32,
}

#[derive(Clone, Copy)]
pub struct FocState {
    pub angle: f32,
    pub angular_change: f32,
    pub velocity: f32,

    pub velocity_error: f32,
    pub velocity_integral: f32,
    pub angle_error: f32,
    pub angle_integral: f32,

    pub i_ref: f32,

    pub i_q: f32,
    pub i_d: f32,
    pub v_q: f32,
    pub v_d: f32,

    pub i_q_error: f32,
    pub i_q_integral: f32,
    pub electrical_angle: f32,
    pub dt: f32,
}

pub struct FocController<Fsincos: Fn(f32) -> (f32, f32)> {
    pub motor: MotorSetup,
    pub bias_angle: f32,
    pub current_phase_bias: f32,
    pub sensor_direction: Direction,
    pub current_mapping: (u8, u8, u8),
    pub state: FocState,

    pub use_current_sensing: bool,
    current_offset: PhaseCurrent,

    psu_voltage: f32,
    pub torque_limit: Option<f32>,
    pub current_limit: Option<f32>,
    pub velocity_limit: Option<f32>,
    pub current_q_pid: PIDController,
    pub current_d_pid: PIDController,
    pub angle_pid: PIDController,
    pub velocity_pid: PIDController,
    _velocity_output_limit: f32, // Volts per second
    pub current_q_filter: LowPassFilter,
    pub current_d_filter: LowPassFilter,
    pub mode: RunMode,
    pub target: Target,
    is_running: bool,

    f_sincos: Fsincos,
}

pub struct OutputLimit {
    pub max_value: f32,
    pub ramp: f32,
}

impl<Fsincos> FocController<Fsincos>
where
    Fsincos: Fn(f32) -> (f32, f32),
{
    pub fn new(
        motor: MotorSetup,
        psu_voltage: f32,
        current_pid_gains: PID,
        angle_pid_gains: PID,
        velocity_pid_gains: PID,
        output_limit: OutputLimit,
        f_sincos: Fsincos,
    ) -> Self {
        let max_velocity = motor.max_velocity;
        let max_velocity_ramp = 1000.0;
        let max_current = output_limit.max_value; // / motor.phase_resistance;
        let max_current_ramp = output_limit.ramp / motor.phase_resistance;
        let state = FocState {
            angle_error: 0.0,
            angle_integral: 0.0,

            angle: 0.0,
            angular_change: 0.0,
            velocity: 0.0,
            velocity_error: 0.0,
            velocity_integral: 0.0,
            dt: 0.0,
            electrical_angle: 0.0,

            i_q: 0.0,
            i_d: 0.0,
            v_q: 0.0,
            v_d: 0.0,

            i_q_error: 0.0,
            i_q_integral: 0.0,
            i_ref: 0.0,
        };
        Self {
            use_current_sensing: false,
            motor,
            bias_angle: 0.0,
            current_phase_bias: 0.0,
            sensor_direction: Direction::Clockwise,
            current_mapping: (0, 1, 2),
            state,
            psu_voltage,
            torque_limit: None,
            current_limit: None,
            velocity_limit: None,
            current_q_pid: PIDController::new(
                current_pid_gains,
                output_limit.max_value,
                output_limit.ramp,
            ),
            current_d_pid: PIDController::new(
                current_pid_gains,
                output_limit.max_value,
                output_limit.ramp,
            ),
            angle_pid: PIDController::new(angle_pid_gains, max_velocity, max_velocity_ramp),
            velocity_pid: PIDController::new(velocity_pid_gains, max_current, max_current_ramp),
            _velocity_output_limit: 1000.0,
            current_q_filter: LowPassFilter::new(0.001),
            current_d_filter: LowPassFilter::new(0.001),

            current_offset: PhaseCurrent::new(0.0, 0.0, 0.0),

            mode: RunMode::Impedance,
            target: Target {
                angle: 0.0,
                velocity: 0.0,
                torque: 0.0,
                spring: 0.0,
                damping: 0.0,
                voltage: 0.0,
            },
            is_running: false,
            f_sincos,
        }
    }

    pub fn enable_current_sensing(&mut self) {
        self.use_current_sensing = true;
    }

    pub fn disable_current_sensing(&mut self) {
        self.use_current_sensing = false;
    }

    pub fn set_current_offset(&mut self, offset: PhaseCurrent) {
        self.current_offset = offset;
    }

    pub async fn align_sensor<'a, FSensor, FMotor, FWait, FutUnit>(
        &mut self,
        align_voltage: f32,
        read_sensor: FSensor,
        mut set_motor: FMotor,
        wait_function: FWait,
    ) -> Result<(), FocError>
    where
        FSensor: Fn() -> AngleReading,
        FMotor: FnMut(DutyCycle3Phase),
        FWait: Fn(f32) -> FutUnit + 'a,
        FutUnit: Future<Output = ()> + Send + 'a,
    {
        // keep sending zero
        let zero_signal = svpwm(
            align_voltage,
            0.0,
            3.0 * core::f32::consts::FRAC_PI_2,
            self.psu_voltage,
            &self.f_sincos,
        )?;
        set_motor(zero_signal);

        // monitor mechanical angle until convergence
        let mut angle = 0.0;
        let mut last_angle = 0.0;
        for _ in 0..100 {
            angle = read_sensor().angle;
            wait_function(0.01).await;
            if angle == last_angle {
                break;
            }
            last_angle = angle;
        }
        self.bias_angle = angle;

        // start rotating slowly and monitor velocity
        let mut target_angle = 0.0;
        let mut velocity_sum = 0.0;
        for _ in 0..100 {
            target_angle += 0.01;
            let signal = svpwm(
                align_voltage,
                0.0,
                target_angle,
                self.psu_voltage,
                &self.f_sincos,
            )?;
            set_motor(signal);
            wait_function(0.01).await;
            velocity_sum += read_sensor().velocity;
        }
        let direction_forward = velocity_sum > 0.0;

        // do the same for reversed direction
        velocity_sum = 0.0;
        for _ in 0..100 {
            target_angle -= 0.01;
            let signal = svpwm(
                align_voltage,
                0.0,
                target_angle,
                self.psu_voltage,
                &self.f_sincos,
            )?;
            set_motor(signal);
            wait_function(0.01).await;
            velocity_sum += read_sensor().velocity;
        }
        let direction_backward = velocity_sum < 0.0;

        match (direction_forward, direction_backward) {
            (true, true) => self.sensor_direction = Direction::Clockwise,
            (false, false) => self.sensor_direction = Direction::CounterClockWise,
            _ => return Err(FocError::AlignError),
        }
        Ok(())
    }

    pub fn to_electrical_angle(&self, mechanical_angle: f32) -> f32 {
        let mut full_angle =
            (mechanical_angle - self.bias_angle) * (self.motor.pole_pair_count as f32);
        if self.sensor_direction == Direction::CounterClockWise {
            full_angle *= -1.0;
        }
        fmodf(full_angle, 2.0 * core::f32::consts::PI)
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

    pub fn set_target_angle(&mut self, angle: f32) {
        self.target.angle = angle;
    }

    pub fn set_target_velocity(&mut self, velocity: f32) {
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

    pub fn set_target_voltage(&mut self, v_q: f32) {
        self.target.voltage = v_q;
    }

    pub fn set_velocity_limit(&mut self, velocity: f32) {
        self.velocity_limit = Some(velocity);
    }

    pub fn set_torque_limit(&mut self, torque: f32) {
        self.torque_limit = Some(torque);
    }

    pub fn set_current_limit(&mut self, current: f32) {
        self.current_limit = Some(current);
    }

    pub fn set_current_kp(&mut self, kp: f32) {
        self.current_q_pid.gains.p = kp;
        self.current_d_pid.gains.p = kp;
        self.current_q_pid.reset();
        self.current_d_pid.reset();
    }

    pub fn set_current_ki(&mut self, ki: f32) {
        self.current_q_pid.gains.i = ki;
        self.current_d_pid.gains.i = ki;
        self.current_q_pid.reset();
        self.current_d_pid.reset();
    }

    pub fn set_velocity_kp(&mut self, kp: f32) {
        self.velocity_pid.gains.p = kp;
    }

    pub fn set_velocity_ki(&mut self, ki: f32) {
        self.velocity_pid.gains.i = ki;
    }

    pub fn set_current_filter(&mut self, f: f32) {
        self.current_q_filter = LowPassFilter::new(f);
        self.current_d_filter = LowPassFilter::new(f);
    }

    pub fn set_angle_kp(&mut self, kp: f32) {
        self.angle_pid.gains.p = kp;
    }

    pub fn normalize_current(&self, c: PhaseCurrent) -> PhaseCurrent {
        self.map_currents(c - self.current_offset)
    }

    pub fn map_currents(&self, c: PhaseCurrent) -> PhaseCurrent {
        let (ma, mb, mc) = self.current_mapping;
        PhaseCurrent::new(
            self.get_current_at(ma, c),
            self.get_current_at(mb, c),
            self.get_current_at(mc, c),
        )
    }
    fn get_current_at(&self, index: u8, c: PhaseCurrent) -> f32 {
        match index {
            0 => c.a,
            1 => c.b,
            2 => c.c,
            _ => panic!(),
        }
    }

    pub fn get_duty_cycle(
        &mut self,
        angle_reading: &AngleReading,
        measured_current: PhaseCurrent,
    ) -> Result<DutyCycle3Phase, FocError> {
        let s = angle_reading;
        self.state.dt = s.dt;

        self.state.angle = s.angle;
        self.state.velocity = s.velocity;
        let mut i_ref = match &self.mode {
            RunMode::Angle => {
                let angle_error = self.target.angle - s.angle;
                self.state.angle_error = angle_error;
                let mut target_velocity = self.angle_pid.update(angle_error, s.dt);
                self.state.angle_integral = self.angle_pid.integral;
                if let Some(v) = self.velocity_limit {
                    target_velocity = target_velocity.min(v).max(-v);
                }
                let velocity_error = target_velocity - s.velocity;
                self.state.velocity_error = velocity_error;
                self.state.velocity_integral = self.velocity_pid.integral;
                self.velocity_pid.update(velocity_error, s.dt)
            }
            RunMode::Velocity => {
                let velocity_error = self.target.velocity - s.velocity;
                self.state.velocity_error = velocity_error;
                self.state.velocity_integral = self.velocity_pid.integral;
                self.velocity_pid.update(velocity_error, s.dt)
            }
            RunMode::Impedance => {
                let angle_error = self.target.angle - s.angle;
                let velocity_error = self.target.velocity - s.velocity;
                self.state.angle_error = angle_error;
                self.state.velocity_error = velocity_error;
                self.target.spring * angle_error
                    + self.target.damping * velocity_error
                    + self.target.torque
            }
            RunMode::Torque => self.target.torque,
            RunMode::Voltage => self.target.voltage,
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

        self.state.i_ref = i_ref;
        let electrical_angle = self.to_electrical_angle(s.angle);

        let (v_q, v_d) = {
            if self.mode == RunMode::Voltage {
                (self.target.voltage, 0.0)
            } else if self.use_current_sensing {
                self.state.electrical_angle = electrical_angle;

                let (mut i_q, mut i_d) =
                    self.calculate_pq_currents(measured_current, electrical_angle);

                i_q = self.current_q_filter.apply(i_q, s.dt);
                i_d = self.current_d_filter.apply(i_d, s.dt);

                self.state.i_q = i_q;
                self.state.i_d = i_d;

                let v_q = self.current_q_pid.update(i_ref - i_q, s.dt);
                let v_d = self.current_d_pid.update(-i_d, s.dt);

                self.state.i_q_error = self.current_q_pid.last_error;
                self.state.i_q_integral = self.current_q_pid.integral;
                (v_q, v_d)
            } else {
                (i_ref, 0.0)
            }
        };
        self.state.v_q = v_q;
        self.state.v_d = v_d;

        let duty_cycle = svpwm(v_q, v_d, electrical_angle, self.psu_voltage, &self.f_sincos)?;
        Ok(duty_cycle)
    }

    pub fn calculate_pq_currents(
        &mut self,
        measured_current: PhaseCurrent,
        electrical_angle: f32,
    ) -> (f32, f32) {
        let c = self.normalize_current(measured_current);

        // clarke transform
        let (i_alpha, i_beta) = clarke_transform(c.a, c.b, c.c);

        // park transform
        let current_angle = electrical_angle + self.current_phase_bias;
        park_transform(i_alpha, i_beta, current_angle, &self.f_sincos)
    }

    pub fn get_vq_duty_cycle(&mut self, v_q: f32, angle: f32) -> Result<DutyCycle3Phase, FocError> {
        let duty_cycle = svpwm(
            v_q,
            0.0,
            self.to_electrical_angle(angle),
            self.psu_voltage,
            &self.f_sincos,
        )?;
        Ok(duty_cycle)
    }

    pub fn get_vd_duty_cycle(&mut self, v_d: f32, angle: f32) -> Result<DutyCycle3Phase, FocError> {
        let duty_cycle = svpwm(
            0.0,
            v_d,
            self.to_electrical_angle(angle),
            self.psu_voltage,
            &self.f_sincos,
        )?;
        Ok(duty_cycle)
    }

    pub fn set_current_phase_bias(&mut self, angle: f32) {
        self.current_phase_bias = angle;
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

pub fn park_transform<Fsincos>(alpha: f32, beta: f32, angle: f32, sincos: Fsincos) -> (f32, f32)
where
    Fsincos: Fn(f32) -> (f32, f32),
{
    let (sin, cos) = sincos(angle);
    let i_q = beta * cos - alpha * sin;
    let i_d = alpha * cos + beta * sin;
    (i_q, i_d)
}
