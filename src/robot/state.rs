use nalgebra::DVector;

#[derive(Clone, Copy, Debug)]
pub struct Position {
    pub x: f32,
    pub y: f32,
}

impl Position {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn translate(&mut self, dx: f32, dy: f32) {
        self.x += dx;
        self.y += dy;
    }

    pub fn norm(&self, other: &Self) -> f32 {
        ((other.x - self.x).powi(2) + (other.y - self.y).powi(2)).sqrt()
    }
}

#[derive(Clone, Copy)]
pub struct Velocity {
    pub linear: f32,
    pub angular: f32,
}

impl Velocity {
    pub fn new(linear: f32, angular: f32) -> Self {
        Self { linear, angular }
    }
}

#[derive(Clone)]
pub struct RobotState {
    pub position: Position,
    pub orientation: f32,
    pub velocity: Velocity,
}

impl From<RobotState> for DVector<f64> {
    fn from(state: RobotState) -> Self {
        DVector::from_row_slice(&[
            state.position.x as f64,
            state.position.y as f64,
            state.orientation as f64,
            state.velocity.linear as f64,
            state.velocity.angular as f64,
        ])
    }
}

impl From<&RobotState> for DVector<f64> {
    fn from(state: &RobotState) -> Self {
        DVector::from_row_slice(&[
            state.position.x as f64,
            state.position.y as f64,
            state.orientation as f64,
            state.velocity.linear as f64,
            state.velocity.angular as f64,
        ])
    }
}

impl From<&DVector<f64>> for RobotState {
    fn from(vec: &DVector<f64>) -> Self {
        RobotState {
            position: Position::new(vec[0] as f32, vec[1] as f32),
            orientation: vec[2] as f32,
            velocity: Velocity::new(vec[3] as f32, vec[4] as f32),
        }
    }
}

impl From<DVector<f64>> for RobotState {
    fn from(vec: DVector<f64>) -> Self {
        RobotState {
            position: Position::new(vec[0] as f32, vec[1] as f32),
            orientation: vec[2] as f32,
            velocity: Velocity::new(vec[3] as f32, vec[4] as f32),
        }
    }
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            position: Position::new(0.0, 0.0),
            orientation: 0.0,
            velocity: Velocity::new(0.0, 0.0),
        }
    }
}

#[derive(Clone)]
pub struct Odometry {
    pub delta_position: Position,
    pub delta_orientation: f32,
}

impl Odometry {
    pub fn new() -> Self {
        Self {
            delta_position: Position::new(0.0, 0.0),
            delta_orientation: 0.0,
        }
    }

    pub fn update(&mut self, delta_x: f32, delta_y: f32, delta_theta: f32) {
        self.delta_position.x += delta_x;
        self.delta_position.y += delta_y;
        self.delta_orientation += delta_theta;
    }
}
