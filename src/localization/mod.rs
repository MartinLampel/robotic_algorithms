
use crate::robot::state::RobotState;
use crate::robot::sensors::SensorMeasurement;


pub trait LocalizationAlgorithm {
    fn update(
        &mut self,
        state: &RobotState,
        measurements: &Vec<SensorMeasurement>,
        dt: f32,
    ) -> RobotState;
}