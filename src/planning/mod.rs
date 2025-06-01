use crate::environment::Environment;
use crate::robot::control::ControlData;
use crate::robot::state::Position;

pub trait GlobalPathPlanner {
    fn plan_path(
        &self,
        start: &Position,
        goal: &Position,
        environment: &Environment,
    ) -> Vec<Position>;
}

pub trait LocalPathPlanner {
    fn plan_path(
        &self,
        start: &Position,
        goal: &Position,
        environment: &Environment,
    ) -> ControlData;
}
