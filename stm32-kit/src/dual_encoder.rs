#![allow(dead_code)]

use foc::{
    encoder::{AngleInput, EncoderReading},
    output_angle::estimate_output_rev_count,
};

pub enum DualEncoderReadError<PE, SE> {
    Primary(PE),
    Secondary(SE),
}

pub struct DualEncoderReading {
    pub primary: EncoderReading,
    pub secondary_phase: f32,
}

pub struct DualEncoder<P, S> {
    primary: P,
    secondary: S,
    reduction_ratio: i32,
    last_rev_count_estimate: Option<i32>,
}

impl<P, S> DualEncoder<P, S> {
    pub fn new(primary: P, secondary: S, reduction_ratio: i32) -> Self {
        Self {
            primary,
            secondary,
            reduction_ratio,
            last_rev_count_estimate: None,
        }
    }

    pub fn primary_mut(&mut self) -> &mut P {
        &mut self.primary
    }

    pub fn secondary_mut(&mut self) -> &mut S {
        &mut self.secondary
    }

    pub fn last_rev_count_estimate(&self) -> Option<i32> {
        self.last_rev_count_estimate
    }

    pub fn observe(&mut self, primary_reading: &EncoderReading, secondary_phase: f32) {
        let rev_count =
            estimate_output_rev_count(primary_reading.phase, secondary_phase, self.reduction_ratio);
        self.last_rev_count_estimate = Some(rev_count);
    }
}

impl<P, S> DualEncoder<P, S>
where
    P: AngleInput,
    S: AngleInput,
{
    pub async fn read_pair_async(
        &mut self,
    ) -> Result<DualEncoderReading, DualEncoderReadError<P::ReadError, S::ReadError>> {
        let primary = self
            .primary
            .read_async()
            .await
            .map_err(DualEncoderReadError::Primary)?;
        let secondary = self
            .secondary
            .read_async()
            .await
            .map_err(DualEncoderReadError::Secondary)?;
        self.observe(&primary, secondary.phase);

        Ok(DualEncoderReading {
            primary,
            secondary_phase: secondary.phase,
        })
    }

    pub async fn read_async(
        &mut self,
    ) -> Result<EncoderReading, DualEncoderReadError<P::ReadError, S::ReadError>> {
        self.read_pair_async().await.map(|reading| reading.primary)
    }
}
