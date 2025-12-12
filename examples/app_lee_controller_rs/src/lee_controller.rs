//! Lee Geometric Controller - no_std version for Crazyflie
//! Based on: T. Lee, M. Leok, N.H. McClamroch - "Geometric Tracking Control
//! of a Quadrotor UAV on SE(3)" CDC 2010

use crate::math3d::{Vec3, Mat3, Quaternion, quat_to_rotmat, vee};
use libm::{cosf, sinf, fabsf};

/// Controller gains
#[derive(Copy, Clone)]
pub struct LeeGains {
    pub kp_pos: Vec3,    // position proportional (Kx in paper)
    pub kd_pos: Vec3,    // velocity/derivative (Kv in paper)
    pub kr: Vec3,        // attitude proportional (KR in paper)
    pub k_omega: Vec3,   // angular velocity (Kω in paper)
}

/// Controller output
#[derive(Copy, Clone)]
pub struct ControlOutput {
    pub thrust: f32,
    pub torque: Vec3,
}

/// Lee Controller state
pub struct LeeController {
    pub gains: LeeGains,
    pub mass: f32,
    pub inertia: Vec3,
    pub gravity: f32,
}

impl LeeController {
    /// Create controller with Crazyflie default parameters
    pub fn new() -> Self {
        Self {
            gains: LeeGains {
                // From Crazyflie firmware controller_lee.c
                kp_pos: Vec3::new(7.0, 7.0, 7.0),
                kd_pos: Vec3::new(4.0, 4.0, 4.0),
                kr: Vec3::new(0.007, 0.007, 0.008),
                k_omega: Vec3::new(0.00115, 0.00115, 0.002),
            },
            mass: 0.032,  // Crazyflie 2.1 mass in kg
            inertia: Vec3::new(16.571710e-6, 16.655602e-6, 29.261652e-6),
            gravity: 9.81,
        }
    }
    
    /// Main control loop: compute thrust and torques
    /// 
    /// # Arguments
    /// * `pos` - current position
    /// * `vel` - current velocity
    /// * `quat` - current orientation quaternion (w, x, y, z)
    /// * `omega` - current angular velocity (body frame)
    /// * `pos_des` - desired position
    /// * `vel_des` - desired velocity
    /// * `acc_des` - desired acceleration (feedforward)
    /// * `jerk_des` - desired jerk (for omega_des computation)
    /// * `yaw_des` - desired yaw angle (radians)
    pub fn control(
        &self,
        pos: Vec3,
        vel: Vec3,
        quat: Quaternion,
        omega: Vec3,
        pos_des: Vec3,
        vel_des: Vec3,
        acc_des: Vec3,
        jerk_des: Vec3,
        yaw_des: f32,
    ) -> ControlOutput {
        
        // POSITION CONTROLLER
        
        // Position and velocity errors
        let e_pos = pos_des - pos;
        let e_vel = vel_des - vel;
        
        // Desired acceleration with gravity compensation
        let acc_with_gravity = Vec3::new(
            acc_des.x,
            acc_des.y,
            acc_des.z + self.gravity,
        );
        
        // F_d = a_d + Kv*e_v + Kp*e_p
        let f_d = acc_with_gravity + self.gains.kd_pos.eltmul(&e_vel) + self.gains.kp_pos.eltmul(&e_pos);
        
        // Current rotation matrix from quaternion
        let r = quat_to_rotmat(&quat);
        
        // z-axis of body frame in world coordinates
        let z_body = r.column(2);
        
        // Thrust = m * F_d · z_body
        let thrust = self.mass * f_d.dot(&z_body);
        
        // COMPUTE DESIRED ROTATION MATRIX
        
        let f_d_norm = f_d.magnitude();
        let z_des = if f_d_norm > 1e-6 {
            f_d.normalize()
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        
        // x_c: desired heading direction based on yaw
        let x_c = Vec3::new(cosf(yaw_des), sinf(yaw_des), 0.0);
        
        // y_des = normalize(z_des × x_c)
        let z_cross_x = z_des.cross(&x_c);
        let z_cross_x_norm = z_cross_x.magnitude();
        let y_des = if z_cross_x_norm > 1e-6 {
            z_cross_x.normalize()
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        
        // x_des = y_des × z_des
        let x_des = y_des.cross(&z_des);
        
        // Desired rotation matrix
        let r_des = Mat3::from_columns(&x_des, &y_des, &z_des);
        
        // ATTITUDE CONTROLLER
        
        // Rotation error: e_R = 0.5 * vee(R_d^T * R - R^T * R_d)
        let r_des_t_r = r_des.transpose().mul_mat(&r);
        let r_t_r_des = r.transpose().mul_mat(&r_des);
        let e_r_mat = r_des_t_r - r_t_r_des;
        let e_rot = vee(&e_r_mat).scale(0.5);
        
        // COMPUTE DESIRED ANGULAR VELOCITY
        
        let omega_des = if fabsf(thrust) > 1e-6 {
            let z_dot_jerk = z_des.dot(&jerk_des);
            let hw = (jerk_des - z_des.scale(z_dot_jerk)).scale(self.mass / thrust);
            Vec3::new(-hw.dot(&y_des), hw.dot(&x_des), 0.0)
        } else {
            Vec3::zero()
        };
        
        // Transform omega_des to body frame
        let omega_r = r.transpose().mul_mat(&r_des).mul_vec(&omega_des);
        
        // Angular velocity error
        let e_omega = omega - omega_r;
        
        // MOMENT COMPUTATION
        // M = -K_R * e_R - K_ω * e_ω + ω × (J * ω)
        
        let term1 = -self.gains.kr.eltmul(&e_rot);
        let term2 = -self.gains.k_omega.eltmul(&e_omega);
        let j_omega = self.inertia.eltmul(&omega);
        let term3 = omega.cross(&j_omega);
        
        let torque = term1 + term2 + term3;
        
        ControlOutput { thrust, torque }
    }
}
