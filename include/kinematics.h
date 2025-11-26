#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "ros/ros.h"
#include <vector>
#include <cmath>

using namespace std;

// Link lengths based on URDF analysis
extern double L1;  // Hip offset (rolling joint to pitching joint)
extern double L2;  // Upper leg length (pitching joint to knee joint)
extern double L3;  // Lower leg length (knee joint to foot)
extern double body_length;  // Distance between front and back legs
extern double body_width;   // Distance between left and right legs

// Forward Kinematics - Joint angles to Cartesian position
vector<double> Front_Left_Leg_FK(vector<double> joint_angles);
vector<double> Front_Right_Leg_FK(vector<double> joint_angles);
vector<double> Back_Left_Leg_FK(vector<double> joint_angles);
vector<double> Back_Right_Leg_FK(vector<double> joint_angles);

// Inverse Kinematics - Cartesian position to joint angles
vector<double> Front_Left_Leg_IK(vector<double> xyz_position);
vector<double> Front_Right_Leg_IK(vector<double> xyz_position);
vector<double> Back_Left_Leg_IK(vector<double> xyz_position);
vector<double> Back_Right_Leg_IK(vector<double> xyz_position);

#endif
