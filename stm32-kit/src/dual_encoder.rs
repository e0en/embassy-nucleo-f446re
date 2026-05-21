#![allow(dead_code)]

use foc::{
    encoder::{AngleInput, EncoderReading},
    output_angle::estimate_output_rev_count,
};

const MISMATCH_FAULT_STREAK: u8 = 10;

pub enum DualEncoderReadError<PE, SE> {
    Primary(PE),
    Secondary(SE),
}

pub struct DualEncoderReading {
    pub primary: EncoderReading,
    pub secondary_phase: f32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DualEncoderStatus {
    Ok,
    Mismatch(u8),
    Fault,
}

impl DualEncoderStatus {
    pub fn reset(self) -> Self {
        Self::Ok
    }

    pub fn extend_streak(self) -> Self {
        match self {
            Self::Ok => Self::Mismatch(1),
            Self::Mismatch(streak) => {
                let next_streak = streak.saturating_add(1);
                if next_streak >= MISMATCH_FAULT_STREAK {
                    Self::Fault
                } else {
                    Self::Mismatch(next_streak)
                }
            }
            Self::Fault => Self::Fault,
        }
    }
}

pub struct DualEncoder<P, S> {
    primary: P,
    secondary: S,
    reduction_ratio: i32,
    last_rev_count_estimate: Option<i32>,
    last_mismatch_rev_count: Option<i32>,
    status: DualEncoderStatus,
}

impl<P, S> DualEncoder<P, S> {
    pub fn new(primary: P, secondary: S, reduction_ratio: i32) -> Self {
        Self {
            primary,
            secondary,
            reduction_ratio,
            last_rev_count_estimate: None,
            last_mismatch_rev_count: None,
            status: DualEncoderStatus::Ok,
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

    pub fn status(&self) -> DualEncoderStatus {
        self.status
    }

    pub fn observe(&mut self, primary_reading: &EncoderReading, secondary_phase: f32) {
        let rev_count =
            estimate_output_rev_count(primary_reading.phase, secondary_phase, self.reduction_ratio);
        self.last_rev_count_estimate = Some(rev_count);

        let primary_residue = primary_reading
            .full_rotations
            .rem_euclid(self.reduction_ratio);
        if primary_residue == rev_count {
            self.last_mismatch_rev_count = None;
            self.status = self.status.reset();
        } else if self.last_mismatch_rev_count == Some(rev_count) {
            self.status = self.status.extend_streak();
        } else {
            self.last_mismatch_rev_count = Some(rev_count);
            self.status = DualEncoderStatus::Ok.extend_streak();
        }
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
