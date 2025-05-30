


pub struct GridMap {
    pub width: usize,
    pub height: usize,
    pub map: Vec<Vec<u8>>,
}


impl GridMap {
    pub fn new(width: usize, height: usize, map: Vec<Vec<u8>>) -> Self {
        Self { width, height, map }
    }
}

pub struct Environment;