#ifndef __TRAJECGEN_H__
#define __TRAJECGEN_H__

#include <vector>
#include <cmath>
#include <iostream>

/**
 * @brief Computes the rotation angles required for angular body motions
 *
 * This function calculates the polar coordinates (Radius, Angle) 
 * of the feet relative to the robot center.
 */
std::vector<std::vector<double>> Robot_angular_motion_endpoints(
    double L1, double L, double W, double SL);

/**
 * @brief Generates target XYZ positions for the robot’s four feet
 *
 * This is the main trajectory generation function.
 * It computes where each foot should be placed at the end
 * of the current gait phase based on velocity commands.
 */
std::vector<std::vector<double>> Robot_end_points(
    std::vector<double> req_vel, 
    std::vector<std::vector<double>> mtn_angles, 
    int swing, 
    double SL, 
    double H
);

#endif // __TRAJECGEN_H__
