#ifndef DRONE_DEFAULT_PROPS_H
#define DRONE_DEFAULT_PROPS_H

#include "drone_model.hpp"
#include "drone_guidance_settings.hpp"
#include "drone_mpc_settings.hpp"

// TODO load from XML without ROS?

DroneProps<double> getDroneProps() {
    DroneProps<double> props;

    props.dry_mass = 1.67;
    props.dry_mass_inv = 1 / props.dry_mass;
    props.total_CM = 0.215;
    props.dry_Inertia[0] = 0.0644;
    props.dry_Inertia[1] = 0.0644;
    props.dry_Inertia[2] = 0.0128;
    props.inertia << props.dry_Inertia[0], props.dry_Inertia[1], props.dry_Inertia[2];
    props.inertia_inv = props.inertia.cwiseInverse();
    props.min_propeller_speed = 50;
    props.max_propeller_speed = 80;
    props.max_propeller_delta = 40;
    props.max_servo1_angle = 15 * M_PI / 180.0;
    props.max_servo2_angle = 15 * M_PI / 180.0;
    props.max_servo_rate = 100 * M_PI / 180.0;
    props.thrust_scaling = 1;
    props.torque_scaling = 1;
    props.servo1_offset = 0;
    props.servo2_offset = 0;

    return props;
};

ControlMPCSettings<double> getMPCSettings() {
    ControlMPCSettings<double> settings;

    settings.x_cost = 20;
    settings.dx_cost = 0.1;
    settings.z_cost = 20;
    settings.dz_cost = 30;
    settings.att_cost = 0.5;
    settings.datt_cost = 1;
    settings.droll_cost = 1;
    settings.servo_cost = 5;
    settings.thrust_cost = 0.001;
    settings.torque_cost = 0.001;
    settings.min_z = 0;
    settings.max_attitude_angle = M_PI / 2;
    settings.max_dx = 1.2;
    settings.min_dz = -1;
    settings.max_dz = 1.5;
    settings.max_datt = 0.6;
    settings.scaling_x = 2;
    settings.scaling_z = 4;
    settings.weight_scaling = 64;
    settings.period = 0.02;
    settings.horizon_length = 1;
    settings.max_sqp_iter = 1;
    settings.max_qp_iter = 25;
    settings.max_line_search_iter = 4;

    return settings;
}

GuidanceSettings<double> getGuidanceSettings() {
    GuidanceSettings<double> settings;

    settings.horizontal_slack = 50;
    settings.min_z = 0;
    settings.min_dz = -0.7;
    settings.max_dz = 20;
    settings.max_attitude_angle = 30 * M_PI / 180.0;
    settings.descent_min_propeller_speed = 50;
    settings.max_horizon_length = 30;
    settings.max_sqp_iter = 4;
    settings.max_qp_iter = 300;
    settings.max_line_search_iter = 5;
    settings.target_apogee_vec[0] = 0.5;
    settings.target_apogee_vec[1] = 0;
    settings.target_apogee_vec[2] = 5;
    settings.target_land_vec[0] = 0;
    settings.target_land_vec[1] = 1;
    settings.target_land_vec[2] = 0;

    return settings;
};

#endif //DRONE_DEFAULT_PROPS_H
