use rand_distr::{Normal, Distribution};
use eframe::egui;
use egui_plot::{Plot, PlotPoints, Line};
use robotic_algorithms::robot::state::{RobotState, Position, Velocity};
use robotic_algorithms::robot::control::ControlData;
use robotic_algorithms::robot::kinematics::{KinematicsModel, UnicycleModel};
use robotic_algorithms::localization::dead_reckoning::DeadReckoning;
use robotic_algorithms::localization::extended_kalman_filter::ExtendedKalmanFilter;
use robotic_algorithms::localization::LocalizationAlgorithm;
use robotic_algorithms::robot::sensors::{GPSSensor, Sensor, SensorMeasurement};
use nalgebra::{DMatrix, DVector};

fn f_jacobi(state: &DVector<f64>, u: &DVector<f64>) -> DMatrix<f64> {
    // Unicycle model Jacobian wrt state
    // state = [x, y, theta, v, omega]
    // u = [v, omega]
    let theta = state[2];
    let v = u[0];
    let dt = 0.01; // Will be scaled by dt in EKF predict
    let mut f = DMatrix::identity(5, 5);
    f[(0, 2)] = -v * theta.sin() * dt;
    f[(0, 3)] = dt * theta.cos();
    f[(1, 2)] = v * theta.cos() * dt;
    f[(1, 3)] = dt * theta.sin();
    f[(2, 4)] = dt;
    f
}

fn h_jacobi(_state: &DVector<f64>) -> DMatrix<f64> {
    // GPS measures x, y
    let mut h = DMatrix::zeros(2, 5);
    h[(0, 0)] = 1.0;
    h[(1, 1)] = 1.0;
    h
}

struct SimData {
    gt: Vec<[f64; 2]>,
    dr: Vec<[f64; 2]>,
    ekf: Vec<[f64; 2]>,
    gps: Vec<[f64; 2]>,
    dr_rmse: f32,
    ekf_rmse: f32,
    step: usize,
    steps: usize,
    gt_full: Vec<[f64; 2]>,
    dr_full: Vec<[f64; 2]>,
    ekf_full: Vec<[f64; 2]>,
    gps_full: Vec<[f64; 2]>,
    play_mode: bool,
    last_update: Option<std::time::Instant>,
}

impl SimData {
    fn new() -> Self {
        let dt = 0.1;
        let steps = 500; // Only a partial circle
        let v = 1.0;
        let omega = 0.1;

        let mut gt = RobotState {
            position: Position { x: 0.0, y: 0.0 },
            orientation: 0.0,
            velocity: Velocity { linear: v, angular: omega },
        };
        let mut dr = gt.clone();
        let mut dead_reckoning = DeadReckoning::new(dr.clone(), UnicycleModel);
        let initial_state = DVector::from_vec(vec![0.0, 0.0, 0.0, v as f64, omega as f64]);
        let p = DMatrix::identity(5, 5) * 0.5;
        // Q: process noise covariance
        let mut q = DMatrix::zeros(5, 5);
        q[(0, 0)] = 0.01f64.powi(2); // x
        q[(1, 1)] = 0.01f64.powi(2); // y
        q[(2, 2)] = (0.5 * std::f64::consts::PI / 180.0).powi(2); // heading (0.5 deg)
        q[(3, 3)] = 0.05f64.powi(2); // velocity
        q[(4, 4)] = (1.0 * std::f64::consts::PI / 180.0).powi(2); // angular velocity
        let mut r = DMatrix::zeros(2, 2);
        r[(0, 0)] = 0.1f64.powi(2); // x
        r[(1, 1)] = 0.1f64.powi(2); // y
        let kinematics_model = UnicycleModel;
        let mut ekf = ExtendedKalmanFilter::new(
            initial_state,
            p,
            q,
            r,
            kinematics_model,
            Box::new(f_jacobi),
            Box::new(h_jacobi),
        );
        let gps = GPSSensor;
        let mut rng = rand::thread_rng();
        // GPS noise: stddev 0.5 for both x and y
        let gps_noise = Normal::new(0.0, 0.1).unwrap();
        // Control noise: stddev 1.0 for velocity, 30 deg (in radians) for angular velocity
        let control_noise_v = Normal::new(0.0, 0.05).unwrap();
        let control_noise_w = Normal::new(0.0, 2.0 / 180.0 * std::f64::consts::PI ).unwrap();
        let mut gt_full = Vec::new();
        let mut dr_full = Vec::new();
        let mut ekf_full = Vec::new();
        let mut gps_full = Vec::new();
        let mut gt_traj = Vec::new();
        let mut dr_traj = Vec::new();
        let mut ekf_traj = Vec::new();
        let mut gps_traj = Vec::new();
        for _ in 0..steps {
            let control = ControlData { velocity: v, angular_velocity: omega };
            gt = UnicycleModel.predict_state(&gt, &control, dt);
            gt_full.push([gt.position.x as f64, gt.position.y as f64]);
            let mut noisy_control = control.clone();
            noisy_control.velocity += control_noise_v.sample(&mut rng) as f32;
            noisy_control.angular_velocity += control_noise_w.sample(&mut rng) as f32;
            let dr_est = dead_reckoning.localize(&dr, &[], &noisy_control, dt);
            dr = dr_est.clone();
            dr_full.push([dr.position.x as f64, dr.position.y as f64]);
            let mut gps_meas = match gps.measure(&gt, &robotic_algorithms::environment::Environment) {
                SensorMeasurement::GPS { x, y } => (x, y),
                _ => panic!("Expected GPS measurement"),
            };
            gps_meas.0 += gps_noise.sample(&mut rng) as f32;
            gps_meas.1 += gps_noise.sample(&mut rng) as f32;
            let gps_measurement = SensorMeasurement::GPS { x: gps_meas.0, y: gps_meas.1 };
            gps_full.push([gps_meas.0 as f64, gps_meas.1 as f64]);
            let ekf_state = ekf.localize(&gt, &[gps_measurement], &noisy_control, dt);
            ekf_full.push([ekf_state.position.x as f64, ekf_state.position.y as f64]);
        }
        // Start with only the first point visible
        gt_traj.push(gt_full[0]);
        dr_traj.push(dr_full[0]);
        ekf_traj.push(ekf_full[0]);
        gps_traj.push(gps_full[0]);
        let dr_rmse = 0.0;
        let ekf_rmse = 0.0;
        Self {
            gt: gt_traj,
            dr: dr_traj,
            ekf: ekf_traj,
            gps: gps_traj,
            dr_rmse,
            ekf_rmse,
            step: 1,
            steps,
            gt_full,
            dr_full,
            ekf_full,
            gps_full,
            play_mode: false,
            last_update: None,
        }
    }
}

impl eframe::App for SimData {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("EKF vs Dead Reckoning vs Ground Truth");
            ui.label("Black: Ground Truth, Red: Dead Reckoning, Blue: EKF");
            ui.label("Use the button to step through the simulation.");

            let next_step_clicked = ui.button("Next step").clicked();
            let play_clicked = ui.button(if self.play_mode { "Pause" } else { "Play" })
                .on_hover_text("Animate the simulation").clicked();

            // Handle play/pause toggle
            if play_clicked {
                self.play_mode = !self.play_mode;
                if self.play_mode {
                    self.last_update = Some(std::time::Instant::now());
                }
            }

            // Step logic
            let mut do_step = false;
            let mut steps_per_frame = 1;
            if next_step_clicked {
                do_step = true;
                self.play_mode = false; // Pause if user steps manually
            }
            if self.play_mode {
                // Animate at ~60Hz, but only step if enough time has passed
                let now = std::time::Instant::now();
                let interval = std::time::Duration::from_millis(16); // ~60 FPS
                let enough_time = self.last_update.map_or(true, |last| now.duration_since(last) >= interval);
                if enough_time {
                    do_step = true;
                    steps_per_frame = 1;
                    self.last_update = Some(now);
                }
            }

            if do_step {
                for _ in 0..steps_per_frame {
                    if self.step < self.steps {
                        self.gt.push(self.gt_full[self.step]);
                        self.dr.push(self.dr_full[self.step]);
                        self.ekf.push(self.ekf_full[self.step]);
                        self.gps.push(self.gps_full[self.step]);
                        self.step += 1;
                        // Update RMSE
                        let rmse = |traj: &Vec<[f64; 2]>| {
                            let mut sum = 0.0;
                            for (i, [x, y]) in traj.iter().enumerate() {
                                let [gtx, gty] = self.gt_full[i];
                                sum += (x - gtx).powi(2) + (y - gty).powi(2);
                            }
                            (sum / traj.len() as f64).sqrt() as f32
                        };
                        self.dr_rmse = rmse(&self.dr);
                        self.ekf_rmse = rmse(&self.ekf);
                    } else {
                        self.play_mode = false;
                    }
                }
                ctx.request_repaint();
            }

            ui.label(format!("Step: {}/{}", self.step, self.steps));
            ui.label(format!("Dead Reckoning RMSE: {:.3}", self.dr_rmse));
            ui.label(format!("EKF RMSE: {:.3}", self.ekf_rmse));
            Plot::new("Trajectories").show(ui, |plot_ui| {
                plot_ui.line(Line::new("Ground Truth", PlotPoints::from(self.gt.clone())).color(egui::Color32::DARK_GREEN));
                plot_ui.line(Line::new("Dead Reckoning", PlotPoints::from(self.dr.clone())).color(egui::Color32::RED));
                plot_ui.line(Line::new("EKF", PlotPoints::from(self.ekf.clone())).color(egui::Color32::BLUE));
                // Plot GPS as magenta points
                let gps_points: PlotPoints = self.gps.iter().map(|[x, y]| [*x, *y]).collect();
                plot_ui.points(
                    egui_plot::Points::new("GPS", gps_points)
                        .color(egui::Color32::YELLOW)
                        .radius(2.5)
                );
            });
        });
    }
}

fn main() -> eframe::Result<()> {
    let sim_data = SimData::new();
    let options = eframe::NativeOptions::default();
    eframe::run_native("EKF Demo", options, Box::new(|_cc| Ok(Box::new(sim_data))))
}
