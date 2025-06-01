use crate::environment::Environment;
use crate::planning::{GlobalPathPlanner, LocalPathPlanner};
use state::RobotState;

use crate::localization::LocalizationAlgorithm;

pub mod control;
pub mod kinematics;
pub mod sensors;
pub mod state;

use control::ControlData;
use kinematics::KinematicsModel;
use sensors::Sensor;
use state::Position;

pub struct Robot<S, L, G, LP, K> {
    pub sensors: Vec<S>,
    pub localization: L,
    pub global_planner: G,
    pub local_planner: LP,
    pub kinematics: K,
    pub dt: f32,
}

impl<S, L, G, LP, K> Robot<S, L, G, LP, K>
where
    S: Sensor,
    L: LocalizationAlgorithm,
    G: GlobalPathPlanner,
    LP: LocalPathPlanner,
    K: KinematicsModel,
{
    pub fn with_sensor(mut self, sensor: S) -> Self {
        self.sensors.push(sensor);
        self
    }

    // pub fn localize(
    //     &mut self,
    //     state: &RobotState,
    //     environment: &Environment,
    //     dt: f32,
    // ) -> RobotState {
    //     let mut measurements = Vec::new();
    //     for sensor in &self.sensors {
    //         measurements.push(sensor.measure(state, environment));
    //     }

    //     self.localization.localize(state, &measurements, dt)
    // }

    pub fn run_motion_control(
        &mut self,
        dt: f32,
        state: &RobotState,
        u: &ControlData,
    ) -> RobotState {
        self.kinematics.predict_state(state, u, dt)
    }

    pub fn plan_path_and_execute(
        &mut self,
        goal: Position,
        state: &mut RobotState,
        environment: &Environment,
    ) -> Vec<RobotState> {
        let path = self
            .global_planner
            .plan_path(&state.position, &goal, environment);
        let mut robot_states: Vec<RobotState> = Vec::new();

        for waypoint in &path {
            while !self.reached_waypoint(&state.position, &waypoint, None) {
                let target_velocity =
                    self.local_planner
                        .plan_path(&state.position, waypoint, environment);
                self.run_motion_control(self.dt, state, &target_velocity);
            //    self.localize(environment, self.dt);
                robot_states.push(state.clone());
            }
        }

        println!("Reached the goal!");

        robot_states
    }

    fn reached_waypoint(
        &self,
        position: &Position,
        waypoint: &Position,
        tolerance: Option<f32>,
    ) -> bool {
        let distance = position.norm(waypoint);
        distance < tolerance.unwrap_or(0.1)
    }
}
