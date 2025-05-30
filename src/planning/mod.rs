

use crate::environment::Environment;
use crate::robot::state::Position;
use crate::robot::control::ControlData;

pub trait GlobalPathPlanner {
    fn plan_path(&self, start: &Position, goal: &Position, environment: &Environment) -> Vec<Position>;
}


pub trait LocalPathPlanner {
    fn plan_path(&self, start: &Position, goal: &Position, environment: &Environment) -> ControlData;
}