#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release
#![allow(rustdoc::missing_crate_level_docs)] // it's an example

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints, Points};

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([350.0, 200.0]),
        ..Default::default()
    };
    let graph: Vec<[f64; 2]> = vec![[0.0, 1.0], [2.0, 3.0], [3.0, 2.0]];
    let graph2: Vec<[f64; 2]> = vec![[0.0, 2.0], [2.0, 4.0], [3.0, 3.0]];
    let graph3: Vec<[f64; 2]> = vec![[0.0, 3.0], [2.0, 5.0], [3.0, 4.0]];

    eframe::run_native(
        "My egui App with a plot",
        options,
        Box::new(|_cc| {
            Ok(Box::new(MyApp {
                insert_order: false,
                graph,
                graph2,
                graph3,
                new_point_x: 0.0,
                new_point_y: 0.0,
            }))
        }),
    )
}

#[derive(Default)]
struct MyApp {
    insert_order: bool,
    graph: Vec<[f64; 2]>,
    graph2: Vec<[f64; 2]>,
    graph3: Vec<[f64; 2]>,
    // Add fields for new point input
    new_point_x: f64,
    new_point_y: f64,
}

fn covariance_ellipse_points(
    mean: [f64; 2],
    cov: [[f64; 2]; 2],
    n_points: usize,
    scale: f64,
) -> Vec<[f64; 2]> {
    use nalgebra::{Matrix2, Vector2};
    let cov = Matrix2::new(cov[0][0], cov[0][1], cov[1][0], cov[1][1]);
    let eig = cov.symmetric_eigen();
    let angle = eig.eigenvectors[(1, 0)].atan2(eig.eigenvectors[(0, 0)]);
    let (mx, my) = (mean[0], mean[1]);
    let (a, b) = (
        eig.eigenvalues[0].sqrt() * scale,
        eig.eigenvalues[1].sqrt() * scale,
    );

    (0..=n_points)
        .map(|i| {
            let theta = i as f64 * std::f64::consts::TAU / n_points as f64;
            let x = a * theta.cos();
            let y = b * theta.sin();
            let xr = x * angle.cos() - y * angle.sin();
            let yr = x * angle.sin() + y * angle.cos();
            [mx + xr, my + yr]
        })
        .collect()
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.label("If checked the legend will follow the order as the curves are inserted");
            ui.checkbox(&mut self.insert_order, "Insert order");

            ui.horizontal(|ui| {
                ui.label("Add point to 1st Curve:");
                ui.add(egui::DragValue::new(&mut self.new_point_x).prefix("x: "));
                ui.add(egui::DragValue::new(&mut self.new_point_y).prefix("y: "));
                if ui.button("Add Point").clicked() {
                    self.graph.push([self.new_point_x, self.new_point_y]);
                }
            });

            Plot::new("My Plot")
                .legend(Legend::default().follow_insertion_order(self.insert_order))
                .show(ui, |plot_ui| {
                    plot_ui.line(Line::new(
                        "3rd Curve",
                        PlotPoints::from(self.graph3.clone()),
                    ));
                    //  let points = Points::new("test", &self.graph)
                    //      .name("1st Curve")
                    //    .color(egui::Color32::from_rgb(255, 0, 0));
                    // plot_ui.points(points);
                    // plot_ui.line(Line::new(
                    //    "2nd Curve",
                    //    PlotPoints::from(self.graph2.clone()),
                    //));

                    // Example: plot a covariance ellipse
                    let mean = [1.5, 2.5];
                    let cov = [[0.5, 0.2], [0.2, 0.3]];
                    let ellipse = covariance_ellipse_points(mean, cov, 64, 2.0);
                    plot_ui.line(Line::new("Covariance", PlotPoints::from(ellipse)));
                });
        });
    }
}
