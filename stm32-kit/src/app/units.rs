use super::{ACTUATOR_IQ_TO_TORQUE_NM, ACTUATOR_REDUCTION_RATIO};

#[inline]
pub(crate) fn input_to_output_shaft(value: f32) -> f32 {
    value / ACTUATOR_REDUCTION_RATIO
}

#[inline]
pub(crate) fn output_to_input_shaft(value: f32) -> f32 {
    value * ACTUATOR_REDUCTION_RATIO
}

#[inline]
pub(crate) fn torque_nm_to_iq(torque_nm: f32) -> f32 {
    torque_nm / ACTUATOR_IQ_TO_TORQUE_NM
}

#[inline]
pub(crate) fn output_torque_nm_to_iq(torque_nm: f32) -> f32 {
    torque_nm_to_iq(torque_nm) / ACTUATOR_REDUCTION_RATIO
}

#[inline]
pub(crate) fn iq_to_torque_nm(iq: f32) -> f32 {
    iq * ACTUATOR_IQ_TO_TORQUE_NM
}

#[inline]
pub(crate) fn motion_gain_to_internal_iq_gain(gain: f32) -> f32 {
    gain / (ACTUATOR_REDUCTION_RATIO * ACTUATOR_REDUCTION_RATIO * ACTUATOR_IQ_TO_TORQUE_NM)
}

#[inline]
pub(crate) fn internal_iq_gain_to_motion_gain(gain: f32) -> f32 {
    gain * ACTUATOR_REDUCTION_RATIO * ACTUATOR_REDUCTION_RATIO * ACTUATOR_IQ_TO_TORQUE_NM
}
