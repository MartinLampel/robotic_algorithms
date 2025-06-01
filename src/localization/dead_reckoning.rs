use crate::localization::LocalizationAlgorithm;
use crate::robot::control::ControlData;
use crate::robot::sensors::SensorMeasurement;
use crate::robot::state::RobotState;

pub struct DeadReckoning<K: crate::robot::kinematics::KinematicsModel> {
    pub state: RobotState,
    pub kinematics_model: K,
}

impl<K: crate::robot::kinematics::KinematicsModel> DeadReckoning<K> {
    pub fn new(initial_state: RobotState, kinematics_model: K) -> Self {
        Self {
            state: initial_state,
            kinematics_model,
        }
    }
}

impl<K: crate::robot::kinematics::KinematicsModel> LocalizationAlgorithm for DeadReckoning<K> {
    fn localize(
        &mut self,
        _state: &RobotState,
        _measurements: &[SensorMeasurement],
        u: &ControlData,
        dt: f32,
    ) -> RobotState {
        // Use the kinematics model to propagate the state
        self.state = self.kinematics_model.predict_state(&self.state, u, dt);
        self.state.clone()
    }
}
