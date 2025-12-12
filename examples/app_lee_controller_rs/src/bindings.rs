//! Minimal C bindings for Crazyflie Lee controller
//! Hand-crafted to match stabilizer_types.h - no fat

// Field names must match C exactly for ABI compatibility
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

/// Attitude in euler angles (degrees)
#[repr(C)]
#[derive(Copy, Clone)]
pub struct attitude_t {
    pub timestamp: u32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// 3D point/vector with timestamp
#[repr(C)]
#[derive(Copy, Clone)]
pub struct vec3_s {
    pub timestamp: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub type point_t = vec3_s;
pub type velocity_t = vec3_s;
pub type acc_t = vec3_s;
pub type jerk_t = vec3_s;

/// Quaternion (x, y, z, w order)
#[repr(C)]
#[derive(Copy, Clone)]
pub struct quaternion_t {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

/// State estimate from Kalman filter
#[repr(C)]
#[derive(Copy, Clone)]
pub struct state_t {
    pub attitude: attitude_t,
    pub attitudeQuaternion: quaternion_t,
    pub position: point_t,
    pub velocity: velocity_t,
    pub acc: acc_t,
}

/// Stabilization mode
#[repr(C)]
#[derive(Copy, Clone, PartialEq)]
pub enum stab_mode_t {
    ModeDisable = 0,
    ModeAbs = 1,
    ModeVelocity = 2,
}

/// Setpoint mode flags
#[repr(C)]
#[derive(Copy, Clone)]
pub struct setpoint_mode_t {
    pub x: stab_mode_t,
    pub y: stab_mode_t,
    pub z: stab_mode_t,
    pub roll: stab_mode_t,
    pub pitch: stab_mode_t,
    pub yaw: stab_mode_t,
    pub quat: stab_mode_t,
}

/// Trajectory setpoint from commander
#[repr(C)]
#[derive(Copy, Clone)]
pub struct setpoint_t {
    pub timestamp: u32,
    pub attitude: attitude_t,
    pub attitudeRate: attitude_t,
    pub attitudeQuaternion: quaternion_t,
    pub thrust: f32,
    pub position: point_t,
    pub velocity: velocity_t,
    pub acceleration: acc_t,
    pub jerk: jerk_t,
    pub velocity_body: bool,
    pub mode: setpoint_mode_t,
}

/// 3-axis sensor reading (union in C, we use the xyz variant)
#[repr(C)]
#[derive(Copy, Clone)]
pub struct Axis3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Barometer data
#[repr(C)]
#[derive(Copy, Clone)]
pub struct baro_t {
    pub pressure: f32,
    pub temperature: f32,
    pub asl: f32,
}

/// Sensor readings (IMU + baro)
#[repr(C)]
#[derive(Copy, Clone)]
pub struct sensorData_t {
    pub acc: Axis3f,
    pub gyro: Axis3f,
    pub mag: Axis3f,
    pub baro: baro_t,
    pub interruptTimestamp: u64,
}

/// Control output mode
#[repr(C)]
#[derive(Copy, Clone, PartialEq)]
pub enum control_mode_t {
    Legacy = 0,
    ForceTorque = 1,
    Force = 2,
}

/// Control output - we use the force/torque variant
#[repr(C)]
#[derive(Copy, Clone)]
pub struct control_t {
    pub thrustSi: f32,
    pub torqueX: f32,
    pub torqueY: f32,
    pub torqueZ: f32,
    _padding: [u8; 4],  // union alignment padding
    pub controlMode: control_mode_t,
}
