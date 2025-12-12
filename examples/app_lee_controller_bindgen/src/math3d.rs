//! Math types for no_std - f32 version for Crazyflie
//! Adapted from assignment1 quaternion_math.rs

#![allow(dead_code)]

use core::ops::{Add, Sub, Mul, Neg};
use libm::sqrtf;

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }
    
    pub fn scale(&self, s: f32) -> Vec3 {
        Vec3 { x: self.x * s, y: self.y * s, z: self.z * s }
    }
    
    /// Element-wise multiplication (for diagonal matrix * vector)
    pub fn eltmul(&self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }
    
    pub fn dot(&self, other: &Vec3) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    
    pub fn cross(&self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
    
    pub fn magnitude(&self) -> f32 {
        sqrtf(self.x * self.x + self.y * self.y + self.z * self.z)
    }
    
    pub fn normalize(&self) -> Vec3 {
        let mag = self.magnitude();
        if mag > 1e-6 {
            self.scale(1.0 / mag)
        } else {
            *self
        }
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, other: Vec3) -> Vec3 {
        Vec3 { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, other: Vec3) -> Vec3 {
        Vec3 { x: self.x - other.x, y: self.y - other.y, z: self.z - other.z }
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3 { x: -self.x, y: -self.y, z: -self.z }
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, scalar: f32) -> Vec3 {
        self.scale(scalar)
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3 {
    pub const fn identity() -> Self {
        Self { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] }
    }
    
    /// Create rotation matrix from column vectors
    pub fn from_columns(x: &Vec3, y: &Vec3, z: &Vec3) -> Self {
        Self {
            m: [
                [x.x, y.x, z.x],
                [x.y, y.y, z.y],
                [x.z, y.z, z.z],
            ],
        }
    }
    
    /// Get column i of the matrix
    pub fn column(&self, i: usize) -> Vec3 {
        Vec3 { x: self.m[0][i], y: self.m[1][i], z: self.m[2][i] }
    }
    
    /// Transpose
    pub fn transpose(&self) -> Mat3 {
        Mat3 {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2]],
            ],
        }
    }
    
    /// Matrix-vector multiplication
    pub fn mul_vec(&self, v: &Vec3) -> Vec3 {
        Vec3 {
            x: self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            y: self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            z: self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        }
    }
    
    /// Matrix-matrix multiplication
    pub fn mul_mat(&self, other: &Mat3) -> Mat3 {
        let mut result = Mat3 { m: [[0.0; 3]; 3] };
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    result.m[i][j] += self.m[i][k] * other.m[k][j];
                }
            }
        }
        result
    }
}

impl Sub for Mat3 {
    type Output = Mat3;
    fn sub(self, other: Mat3) -> Mat3 {
        let mut result = Mat3 { m: [[0.0; 3]; 3] };
        for i in 0..3 {
            for j in 0..3 {
                result.m[i][j] = self.m[i][j] - other.m[i][j];
            }
        }
        result
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }
}

/// Convert quaternion to rotation matrix
pub fn quat_to_rotmat(q: &Quaternion) -> Mat3 {
    let w = q.w;
    let x = q.x;
    let y = q.y;
    let z = q.z;
    
    // Normalize quaternion
    let norm = sqrtf(w*w + x*x + y*y + z*z);
    let w = w / norm;
    let x = x / norm;
    let y = y / norm;
    let z = z / norm;
    
    Mat3 {
        m: [
            [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z),       2.0*(x*z + w*y)],
            [2.0*(x*y + w*z),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)],
            [2.0*(x*z - w*y),       2.0*(y*z + w*x),       1.0 - 2.0*(x*x + y*y)],
        ],
    }
}

/// Vee-map: extract vector from skew-symmetric matrix
pub fn vee(m: &Mat3) -> Vec3 {
    Vec3 {
        x: m.m[2][1],  // (3,2) element
        y: m.m[0][2],  // (1,3) element  
        z: m.m[1][0],  // (2,1) element
    }
}
