//! Rust Lee Controller for Crazyflie
//! Replaces controller_lee.c with a Rust implementation

#![no_std]

mod bindings;
mod math3d;
mod lee_controller;

use panic_halt as _;
use bindings::*;
use math3d::{Vec3, Quaternion};
use lee_controller::LeeController;
use core::cell::UnsafeCell;

/// Global controller (single-threaded embedded, so this is safe)
struct SyncController(UnsafeCell<Option<LeeController>>);
unsafe impl Sync for SyncController {}

static CONTROLLER: SyncController = SyncController(UnsafeCell::new(None));

/// Initialize the Lee controller - called once at firmware startup
#[no_mangle]
pub extern "C" fn controllerLeeFirmwareInit() {
    unsafe {
        *CONTROLLER.0.get() = Some(LeeController::new());
    }
}

/// Test the controller
#[no_mangle]
pub extern "C" fn controllerLeeFirmwareTest() -> bool {
    true
}

/// Main control loop - called at ~500Hz by the stabilizer
#[no_mangle]
pub extern "C" fn controllerLeeFirmware(
    control: *mut control_t,
    setpoint: *const setpoint_t,
    sensors: *const sensorData_t,
    state: *const state_t,
    _tick: u32,
) {
    let controller = unsafe {
        match (*CONTROLLER.0.get()).as_ref() {
            Some(c) => c,
            None => return,
        }
    };
    
    let setpoint = unsafe { &*setpoint };
    let state = unsafe { &*state };
    let sensors = unsafe { &*sensors };
    let control = unsafe { &mut *control };
    
    // Current state
    let pos = Vec3::new(state.position.x, state.position.y, state.position.z);
    let vel = Vec3::new(state.velocity.x, state.velocity.y, state.velocity.z);
    let quat = Quaternion::new(
        state.attitudeQuaternion.w,
        state.attitudeQuaternion.x,
        state.attitudeQuaternion.y,
        state.attitudeQuaternion.z,
    );
    let omega = Vec3::new(sensors.gyro.x, sensors.gyro.y, sensors.gyro.z);
    
    // Setpoint
    let pos_des = Vec3::new(setpoint.position.x, setpoint.position.y, setpoint.position.z);
    let vel_des = Vec3::new(setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z);
    let acc_des = Vec3::new(setpoint.acceleration.x, setpoint.acceleration.y, setpoint.acceleration.z);
    let jerk_des = Vec3::new(setpoint.jerk.x, setpoint.jerk.y, setpoint.jerk.z);
    let yaw_des = setpoint.attitude.yaw * (core::f32::consts::PI / 180.0);
    
    // Run controller
    let output = controller.control(
        pos, vel, quat, omega,
        pos_des, vel_des, acc_des, jerk_des,
        yaw_des,
    );
    
    // Write output
    control.controlMode = control_mode_t::ForceTorque;
    control.thrustSi = output.thrust;
    control.torqueX = output.torque.x;
    control.torqueY = output.torque.y;
    control.torqueZ = output.torque.z;
}

/// Dummy appMain - required by app layer but we're a controller replacement
#[no_mangle]
pub extern "C" fn appMain() {
    // Nothing to do ^^
}
