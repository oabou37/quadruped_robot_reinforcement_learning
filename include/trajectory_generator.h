#ifndef __TRAJECGEN_H__
#define __TRAJECGEN_H__

#include <vector>
#include <cmath>
#include <iostream>

/**
 * @brief Computes the rotation angles required for angular body motions
 *
 * This function calculates how the robot body should rotate
 * during in-place turning or while walking along a curved path.
 */
std::vector<std::vector<double>> Robot_angular_motion_endpoints(
    double L1, double L, double W, double SL);

/**
 * @brief Finds the intersection point(s) between two circles
 *
 * This function is used for geometric inverse kinematics,
 * ensuring that the foot placement remains physically reachable.
 */
std::vector<double> Circle_intersection_points(
    double h, double k, double R1, double R2);

/**
 * @brief Generates target XYZ positions for the robot’s four feet
 *
 * This is the main trajectory generation function.
 * It computes where each foot should be placed at the end
 * of the current gait phase.
 */
std::vector<std::vector<double>> Robot_end_points(
    std::vector<double> req_vel, 
    std::vector<std::vector<double>> mtn_angles, 
    int swing, 
    double SL, 
    double H
);

#endif // __TRAJECGEN_H__
