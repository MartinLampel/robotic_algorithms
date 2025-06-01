/// A simple GPS sensor that returns the robot's current position as a measurement.
pub struct GPSSensor;

impl Sensor for GPSSensor {
    fn measure(&self, state: &crate::robot::state::RobotState, _environment: &crate::environment::Environment) -> super::sensors::SensorMeasurement {
        // Output the robot's (x, y) position as a GPS measurement with no accuracy info
        super::sensors::SensorMeasurement::GPS {
            x: state.position.x,
            y: state.position.y,
        }
    }
}
use crate::environment::Environment;
use crate::robot::state::RobotState;
use nalgebra::DVector;

use std::convert::TryFrom;

pub enum SensorMeasurement {
    Range {
        distance: f32, // Distance measured by a range sensor (e.g., LIDAR)
        angle: f32,    // Angle of the measurement relative to the robot
    },
    Data(Vec<SensorMeasurement>),
    GPS {
        x: f32,        
        y: f32,       
    },
}

pub trait Sensor {
    fn measure(&self, state: &RobotState, environment: &Environment) -> SensorMeasurement;
}

impl TryFrom<&SensorMeasurement> for DVector<f64> {
    type Error = &'static str;

    fn try_from(measurement: &SensorMeasurement) -> Result<Self, Self::Error> {
        match measurement {
            SensorMeasurement::Range { distance, angle } => {
                Ok(DVector::from_vec(vec![*distance as f64, *angle as f64]))
            }
            SensorMeasurement::GPS { x, y, .. } => {
                Ok(DVector::from_vec(vec![*x as f64, *y as f64]))
            }
            _ => Err("Unsupported measurement type"),
        }
    }
}
