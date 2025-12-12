//! Rust Lee Controller for Crazyflie (bindgen version)
//! Uses auto-generated bindings from bindgen - uglier but guaranteed ABI-safe(maybe,idk)

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
pub extern "C" fn controllerOutOfTreeInit() {
    unsafe {
        *CONTROLLER.0.get() = Some(LeeController::new());  //the.0.get() is cause Synccontroller unsafe cell a tuple struct
    }
}

/// Test the controller
#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest() -> bool {
    true
}

/// Main control loop - called at ~500Hz by the stabilizer
#[no_mangle]
pub extern "C" fn controllerOutOfTree(
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
    
    // Quaternion from bindgen union (access the x,y,z,w variant)
    let quat = unsafe {
        let q = &state.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_2;
        Quaternion::new(q.w, q.x, q.y, q.z)
    };
    
    // Angular velocity from gyro (bindgen union)convert degrees to radians mult by 0.0174533
    let omega = unsafe {
        let  g = &sensors.gyro.__bindgen_anon_1;
        Vec3::new(g.x * 0.0174533, g.y * 0.0174533, g.z * 0.0174533)
    };
    
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
    
    // Write output using bindgen union access
    control.controlMode = control_mode_e_controlModeForceTorque;
    unsafe {
        control.__bindgen_anon_1.__bindgen_anon_2.thrustSi = output.thrust;
        control.__bindgen_anon_1.__bindgen_anon_2.__bindgen_anon_1.torque[0] = output.torque.x;
        control.__bindgen_anon_1.__bindgen_anon_2.__bindgen_anon_1.torque[1] = output.torque.y;
        control.__bindgen_anon_1.__bindgen_anon_2.__bindgen_anon_1.torque[2] = output.torque.z;
    }
}

/// Dummy appMain - required by app layer 
#[no_mangle]
pub extern "C" fn appMain() {
    // Nothing to do ^
}
