use nalgebra::DVector;

use crate::robot::state::RobotState;

#[derive(Clone, Debug)]
pub struct ControlData {
    pub velocity: f32,
    pub angular_velocity: f32,
}

impl ControlData {
    pub fn new(velocity: f32, angular_velocity: f32) -> Self {
        Self {
            velocity,
            angular_velocity,
        }
    }
}

impl From<&ControlData> for DVector<f64> {
    fn from(data: &ControlData) -> Self {
        DVector::from_row_slice(&[data.velocity as f64, data.angular_velocity as f64])
    }
}

impl From<&DVector<f64>> for ControlData {
    fn from(vec: &DVector<f64>) -> Self {
        ControlData {
            velocity: vec[0] as f32,
            angular_velocity: vec[1] as f32,
        }
    }
}

pub trait ControlAlgorithm {
    fn calculate_input(
        &self,
        state: &RobotState,
        target_velocity: &ControlData,
        dt: f32,
    ) -> ControlData;
}
