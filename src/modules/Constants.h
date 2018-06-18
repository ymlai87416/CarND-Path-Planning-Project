//
// Created by tom on 6/17/18.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#define DECLARE_CONSTANT(name, type, value, description)  const type name = value;

DECLARE_CONSTANT(sensor_fusion_range, float, 1000, "Sensor fusion detection range.")

DECLARE_CONSTANT(update_interval, float, 0.02, "Update interval in seconds for the main loop.")

DECLARE_CONSTANT(spline_time_step, float, 0.6, "time step between each coordinate point to draw a spline.")
DECLARE_CONSTANT(spline_total_step, int, 3, "No of coordinate point, exclude original position, to draw a spline.")

DECLARE_CONSTANT(trajectory_discretize_timestep, float, 0.02, "Time step for trajectory discretization")

DECLARE_CONSTANT(vehicle_speed_limit, float, 22.13, "Speed limit of the car in mph.")
DECLARE_CONSTANT(vehicle_max_acceleration, float, 10, "Maximum acceleration in m/s^2.")
DECLARE_CONSTANT(vehicle_max_deceleration, float, -10, "Maximum deceleration in m/s^2.")
DECLARE_CONSTANT(vehicle_max_jerk, float, 10, "Maximum jerk in m/s^3.")
DECLARE_CONSTANT(vehicle_normal_acceleration, float, 5, "Normal acceleration/deceleration in m/s.")

DECLARE_CONSTANT(prediction_time_length, float, 1.0, "Prediction ahead time length in seconds.")

DECLARE_CONSTANT(planning_run_period, float, 0.02, "Running period for planning module.")
DECLARE_CONSTANT(planning_time_length, float, 1.0, "Planning ahead time length in seconds.")
DECLARE_CONSTANT(planning_change_lane_max_time, float, 3.0, "Maximum time for changing lane in seconds.")
DECLARE_CONSTANT(planning_safe_follow_distance, float, 30, "Minimum safe following distance in m.")

DECLARE_CONSTANT(planning_cost_acceleration_weight, float, 1, "Weight for acceleration limit cost function.")
DECLARE_CONSTANT(planning_cost_off_lane_weight, float, 1, "Weight for off lane cost function.")
DECLARE_CONSTANT(planning_cost_speed_limit_weight, float, 1, "Weight for speed limit cost function.")
DECLARE_CONSTANT(planning_cost_lane_center_weight, float, 1, "Weight for lane center cost function.")
DECLARE_CONSTANT(planning_cost_distance_to_obstacle, float, 1, "Weight for distance to obstacle along the trajectory.")
DECLARE_CONSTANT(planning_cost_inefficiency_cost, float, 1, "Weight for reaching the target speed.")
DECLARE_CONSTANT(planning_collison_radius, float, 4, "Collision radius in m.")


#endif //PATH_PLANNING_CONSTANTS_H
