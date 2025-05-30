
use crate::robot::state::RobotState;

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

pub trait ControlAlgorithm {
    fn calculate_input(&self, state: &RobotState, target_velocity: &ControlData, dt: f32) -> ControlData;
}