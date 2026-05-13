#include "Sub.h"

#define NETCAGE_SWAY_VELOCITY_MS       0.10f
#define NETCAGE_CLIMB_RATE_CMS         2.0f
#define NETCAGE_FORWARD_CMD_MAX        0.35f
#define NETCAGE_LATERAL_CMD_MAX        0.35f
#define NETCAGE_TARGET_DIST_MIN_M      0.05f
#define NETCAGE_TARGET_DIST_MAX_M      20.0f

#define NETCAGE_DIST_P                 0.80f
#define NETCAGE_DIST_I                 0.00f
#define NETCAGE_DIST_D                 0.10f
#define NETCAGE_DIST_IMAX              0.20f

#define NETCAGE_SWAY_P                 2.00f
#define NETCAGE_SWAY_I                 0.20f
#define NETCAGE_SWAY_D                 0.05f
#define NETCAGE_SWAY_IMAX              0.20f

#define NETCAGE_YAW_P                  5000.0f
#define NETCAGE_YAW_I                  0.0f
#define NETCAGE_YAW_D                  200.0f
#define NETCAGE_YAW_IMAX               500.0f
#define NETCAGE_YAW_RATE_MAX_CDS       3000.0f

ModeNetCageInspection::ModeNetCageInspection() :
    netcage_dist_pid(NETCAGE_DIST_P, NETCAGE_DIST_I, NETCAGE_DIST_D, 0.0f,
                     NETCAGE_DIST_IMAX, 0.0f, 2.0f, 2.0f),
    netcage_sway_pid(NETCAGE_SWAY_P, NETCAGE_SWAY_I, NETCAGE_SWAY_D, 0.0f,
                     NETCAGE_SWAY_IMAX, 0.0f, 2.0f, 2.0f),
    netcage_yaw_pid(NETCAGE_YAW_P, NETCAGE_YAW_I, NETCAGE_YAW_D, 0.0f,
                    NETCAGE_YAW_IMAX, 0.0f, 2.0f, 2.0f)
{
}

bool ModeNetCageInspection::init(bool ignore_checks) {
    if (!sub.control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    sub.inertial_doppler.set_sensor_to_body_rot(Rotation::ROTATION_PITCH_270);
    position_control->init_z_controller();

    netcage_target_distance_m = constrain_float(g2.zz_desired_m, NETCAGE_TARGET_DIST_MIN_M, NETCAGE_TARGET_DIST_MAX_M);
    netcage_tangent_speed_ms = NETCAGE_SWAY_VELOCITY_MS;
    netcage_climb_rate_ms = NETCAGE_CLIMB_RATE_CMS * 0.01f;
    initial_z_m = inertial_nav.get_position_z_up_cm() * 0.01f;
    netcage_target_z_m = initial_z_m;

    netcage_dist_pid.reset_filter();
    netcage_dist_pid.reset_I();
    netcage_sway_pid.reset_filter();
    netcage_sway_pid.reset_I();
    netcage_yaw_pid.reset_filter();
    netcage_yaw_pid.reset_I();

    return true;
}

void ModeNetCageInspection::run()
{
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0.5f, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->relax_z_controller(motors.get_throttle_hover());
        netcage_dist_pid.reset_filter();
        netcage_dist_pid.reset_I();
        netcage_sway_pid.reset_filter();
        netcage_sway_pid.reset_I();
        netcage_yaw_pid.reset_filter();
        netcage_yaw_pid.reset_I();
        motors.set_forward(0.0f);
        motors.set_lateral(0.0f);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector3f vel_body_mps;
    uint32_t dvl_t_ms = 0;
    float dvl_quality = 0.0f;
    DVL_LockState dvl_lock = DVL_LockState::NO_LOCK;
    const bool dvl_vel_ok = sub.inertial_doppler.get_velocity_body(vel_body_mps, dvl_t_ms, dvl_quality, dvl_lock);

    netcage_target_distance_m = constrain_float(g2.zz_desired_m, NETCAGE_TARGET_DIST_MIN_M, NETCAGE_TARGET_DIST_MAX_M);

    float forward_cmd = 0.0f;
    const float plane_distance_m = sub.inertial_doppler.get_distance_m();
    const Vector3f plane_normal_body = sub.inertial_doppler.get_normal_body();
    const bool plane_ok = !plane_normal_body.is_zero() && plane_distance_m > 0.0f;

    float target_yaw_rate_cds = 0.0f;
    if (plane_ok) {
        const float yaw_error_rad = atan2f(plane_normal_body.y, plane_normal_body.x);
        target_yaw_rate_cds = netcage_yaw_pid.update_error(yaw_error_rad, G_Dt);
        target_yaw_rate_cds = constrain_float(target_yaw_rate_cds, -NETCAGE_YAW_RATE_MAX_CDS, NETCAGE_YAW_RATE_MAX_CDS);
    } else {
        netcage_yaw_pid.reset_filter();
        netcage_yaw_pid.reset_I();
    }

    // Keep roll and pitch level; yaw rate drives the DVL plane normal onto body +X.
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, target_yaw_rate_cds);

    if (plane_ok) {
        forward_cmd = -netcage_dist_pid.update_all(netcage_target_distance_m, plane_distance_m, G_Dt);
        forward_cmd = constrain_float(forward_cmd, -NETCAGE_FORWARD_CMD_MAX, NETCAGE_FORWARD_CMD_MAX);
    } else {
        netcage_dist_pid.reset_filter();
        netcage_dist_pid.reset_I();
    }

    float lateral_cmd = 0.0f;
    if (dvl_vel_ok) {
        lateral_cmd = netcage_sway_pid.update_all(netcage_tangent_speed_ms, vel_body_mps.y, G_Dt);
        lateral_cmd = constrain_float(lateral_cmd, -NETCAGE_LATERAL_CMD_MAX, NETCAGE_LATERAL_CMD_MAX);
    } else {
        netcage_sway_pid.reset_filter();
        netcage_sway_pid.reset_I();
    }

    motors.set_forward(forward_cmd);
    motors.set_lateral(lateral_cmd);

    position_control->set_pos_target_z_from_climb_rate_cm(netcage_climb_rate_ms * 100.0f);
    position_control->update_z_controller();

}
