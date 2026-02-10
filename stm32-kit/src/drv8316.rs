use foc::current::PhaseCurrent;

// ADC and voltage reference constants
const ADC_REFERENCE_VOLTAGE: f32 = 3.3;
const ADC_RESOLUTION: f32 = 4096.0;
const ADC_MIDPOINT_VOLTAGE: f32 = ADC_REFERENCE_VOLTAGE / 2.0;

// CSA gain values in V/A (from DRV8316 datasheet)
const CSA_GAIN_0_15: f32 = 0.15;
const CSA_GAIN_0_3: f32 = 0.3;
const CSA_GAIN_0_6: f32 = 0.6;
const CSA_GAIN_1_2: f32 = 1.2;

// Current sensing cross-coupling compensation matrix (calibrated)
const CROSSTALK_AA: f32 = 0.995832;
const CROSSTALK_AB: f32 = -0.028119;
const CROSSTALK_AC: f32 = -0.014988;
const CROSSTALK_BA: f32 = 0.037737;
const CROSSTALK_BB: f32 = 1.007723;
const CROSSTALK_BC: f32 = -0.033757;
const CROSSTALK_CA: f32 = 0.009226;
const CROSSTALK_CB: f32 = 0.029805;
const CROSSTALK_CC: f32 = 1.003268;

pub const MAX_CURRENT: f32 = 8.0;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum CsaGain {
    Gain0_15V = 0b00, // 0.15 V/A
    Gain0_3V = 0b01,  // 0.3 V/A
    Gain0_6V = 0b10,  // 0.6 V/A
    Gain1_2V = 0b11,  // 1.2 V/A
}

pub fn convert_csa_readings(ia_raw: u16, ib_raw: u16, ic_raw: u16, gain: CsaGain) -> PhaseCurrent {
    let mut ia = (ia_raw as f32) * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION - ADC_MIDPOINT_VOLTAGE;
    let mut ib = (ib_raw as f32) * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION - ADC_MIDPOINT_VOLTAGE;
    let mut ic = (ic_raw as f32) * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION - ADC_MIDPOINT_VOLTAGE;

    ia = CROSSTALK_AA * ia + CROSSTALK_AB * ib + CROSSTALK_AC * ic;
    ib = CROSSTALK_BA * ia + CROSSTALK_BB * ib + CROSSTALK_BC * ic;
    ic = CROSSTALK_CA * ia + CROSSTALK_CB * ib + CROSSTALK_CC * ic;
    let gain_value = match gain {
        CsaGain::Gain0_15V => CSA_GAIN_0_15,
        CsaGain::Gain0_3V => CSA_GAIN_0_3,
        CsaGain::Gain0_6V => CSA_GAIN_0_6,
        CsaGain::Gain1_2V => CSA_GAIN_1_2,
    };

    ia /= gain_value;
    ib /= gain_value;
    ic /= gain_value;
    PhaseCurrent::new(ia, ib, ic)
}
