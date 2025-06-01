use crate::robot::control::ControlData;
use crate::robot::state::RobotState;

use super::state::Position;

pub trait KinematicsModel {
    fn predict_state(
        &self,
        current_state: &RobotState,
        control_input: &ControlData,
        dt: f32,
    ) -> RobotState;
}

pub struct UnicyleModel;

impl KinematicsModel for UnicycleModel {
    fn predict_state(
        &self,
        current_state: &RobotState,
        control_input: &ControlData,
        dt: f32,
    ) -> RobotState {
        let v = control_input.velocity;
        let omega = control_input.angular_velocity;

        let delta_x = v * dt * current_state.orientation.cos();
        let delta_y = v * dt * current_state.orientation.sin();
        let delta_theta = omega * dt;

        let cur = Position {
            x: current_state.position.x + delta_x,
            y: current_state.position.y + delta_y,
        };

        RobotState {
            position: cur,
            orientation: current_state.orientation + delta_theta,
            ..*current_state
        }
    }
}
