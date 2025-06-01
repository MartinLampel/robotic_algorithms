use crate::localization::LocalizationAlgorithm;
use crate::localization::kalman_filter::KalmanFilter;
use crate::robot::control::ControlData;
use crate::robot::kinematics::KinematicsModel;
use crate::robot::sensors::SensorMeasurement;
use crate::robot::state::RobotState;
use nalgebra::{DMatrix, DVector};

pub struct ExtendedKalmanFilter<K: KinematicsModel> {
    state: DVector<f64>,
    pub p: DMatrix<f64>,
    q: DMatrix<f64>,
    r: DMatrix<f64>,
    kinematics_model: K,
    f_jacobi: Box<dyn Fn(&DVector<f64>, &DVector<f64>) -> DMatrix<f64>>,
    h_jacobi: Box<dyn Fn(&DVector<f64>) -> DMatrix<f64>>,
}

impl<K: KinematicsModel> ExtendedKalmanFilter<K> {
    pub fn new(
        initial_state: DVector<f64>,
        p: DMatrix<f64>,
        q: DMatrix<f64>,
        r: DMatrix<f64>,
        kinematics_model: K,
        f_jacobi: Box<dyn Fn(&DVector<f64>, &DVector<f64>) -> DMatrix<f64>>,
        h_jacobi: Box<dyn Fn(&DVector<f64>) -> DMatrix<f64>>,
    ) -> Self {
        ExtendedKalmanFilter {
            state: initial_state,
            p,
            q,
            r,
            kinematics_model,
            f_jacobi,
            h_jacobi,
        }
    }
}

impl<K> KalmanFilter for ExtendedKalmanFilter<K>
where
    K: KinematicsModel,
{
    fn predict(&mut self, u: &ControlData, dt: f32) {
        let rs = RobotState::from(&self.state);
        let u_vec: DVector<f64> = u.into();

        let f_jacobian = (self.f_jacobi)(&self.state, &u_vec);
        self.state = self.kinematics_model.predict_state(&rs, &u, dt).into();

        self.p = &f_jacobian * &self.p * f_jacobian.transpose() + &self.q;
    }

    fn update(&mut self, measurements: &[SensorMeasurement]) -> DVector<f64> {
        let h_jacobian = (self.h_jacobi)(&self.state);
        let identity_matrix = DMatrix::identity(self.state.len(), self.state.len());

        for measurement in measurements {
            let z: DVector<f64> = match measurement.try_into() {
                Ok(v) => v,
                Err(_) => continue,
            };

            let y = z - &h_jacobian * &self.state; // Innovation
            let s = &h_jacobian * &self.p * &h_jacobian.transpose() + &self.r; // Innovation covariance
            let k = &self.p * &h_jacobian.transpose() * s.try_inverse().unwrap(); // Kalman gain

            self.state += &k * y; // Update state
            self.p = (&identity_matrix - &k * &h_jacobian) * &self.p;
        }

        self.state.clone()
    }
}

impl<K> LocalizationAlgorithm for ExtendedKalmanFilter<K>
where
    K: KinematicsModel,
{
    fn localize(
        &mut self,
        state: &RobotState,
        measurements: &[SensorMeasurement],
        u: &ControlData,
        dt: f32,
    ) -> RobotState {
        self.predict(u, dt);
        let updated_state_vec: DVector<f64> = self.update(measurements);
        RobotState::from(updated_state_vec)
    }
}
