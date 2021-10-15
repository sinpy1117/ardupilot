#include "Copter.h"

#if MODE_FIXLINE_ENABLED == ENABLED

int count = 0;
/*
 * 固定航线模式初始化
 */
bool ModeFixLine::init(bool ignore_checks)
{
    path_num = 0;
    generate_path();  // 生成航线

    pos_control_start();  // 开始位置控制

    return true;
}

// 生成固定航线
void ModeFixLine::generate_path()
{
    //float radius_cm = 1000.0;
    float length = 1000.0;
    float width = 2000.0;
    wp_nav->get_wp_stopping_point(path[0]);
    
/*
    path[1] = Vector3f(path[0].x + 1.0f * radius_cm , path[0].y, 250);
    path[2] = Vector3f(path[0].x - cosf(radians(36.0f)) * radius_cm, path[0].y - sinf(radians(36.0f)) * radius_cm, 250);
    path[3] = Vector3f(path[0].x + sinf(radians(18.0f)) * radius_cm, path[0].y + cosf(radians(18.0f)) * radius_cm, 250);
    path[4] = Vector3f(path[0].x + sinf(radians(18.0f)) * radius_cm, path[0].y - cosf(radians(18.0f)) * radius_cm, 250);
    path[5] = Vector3f(path[0].x - cosf(radians(36.0f)) * radius_cm, path[0].y + sinf(radians(36.0f)) * radius_cm, 250);
    path[6] = path[1];*/

    path[1] = Vector3f(path[0].x + width * 0.5, path[0].y, 250);
    path[2] = Vector3f(path[0].x + width * 0.5, path[0].y - length * 0.5, 250);
    path[3] = Vector3f(path[0].x - width * 0.5, path[0].y - length * 0.5, 250);
    path[4] = Vector3f(path[0].x - width * 0.5, path[0].y + length * 0.5, 250);
    path[5] = Vector3f(path[0].x + width * 0.5, path[0].y + length * 0.5, 250);
    path[6] = path[1];
    
}

// 开始位置控制
void ModeFixLine::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used cosf    
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// 此模式的周期调用
void ModeFixLine::run()
{
    Location loc{};
    if (!ahrs.get_position(loc)) {
        return;
    }
    Vector3f neddis; //当前点与home点的距离（NED）


    if (path_num < 6) {  // 航线尚未走完
        if (wp_nav->reached_wp_destination()) {  // 到达某个端点
            neddis = loc.get_distance_NED(ahrs.get_home());
            gcs().send_text(MAV_SEVERITY_ERROR, "diatance NED: %.1f , %.1f , %.1f",neddis.x, neddis.y, neddis.z);
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    } else if ((path_num == 6) && wp_nav->reached_wp_destination()) {  // 航线运行完成，自动进入land模式
        gcs().send_text(MAV_SEVERITY_INFO, "Fix line finished, now go into land mode");
        copter.set_mode(Mode::Number::LAND, ModeReason::MISSION_END);  // 切换到land模式
    }

    pos_control_run();
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeFixLine::pos_control_run()  // 此函数直接从mode_guided.cpp中复制过来，不需要该其中的内容
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

#endif