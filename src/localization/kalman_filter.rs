use crate::robot::control::ControlData;
use crate::robot::sensors::SensorMeasurement;
use nalgebra::DVector;

pub trait KalmanFilter {
    fn predict(&mut self, u: &ControlData, dt: f32);
    fn update(&mut self, measurements: &[SensorMeasurement]) -> DVector<f64>;
}
