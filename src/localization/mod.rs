use crate::robot::control::ControlData;
use crate::robot::sensors::SensorMeasurement;
use crate::robot::state::RobotState;

pub mod extended_kalman_filter;
pub mod kalman_filter;
pub mod dead_reckoning;

pub trait LocalizationAlgorithm {
    fn localize(
        &mut self,
        state: &RobotState,
        measurements: &[SensorMeasurement],
        u: &ControlData,
        dt: f32,
    ) -> RobotState;
}
