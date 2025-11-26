#include "Kinematics.h"

// Link lengths extracted from URDF
// Hip offset: distance from rolling joint to pitching joint
double L1 = 0.056;  // 56mm in z-direction from URDF

// Upper leg length: distance from pitching joint to knee joint
double L2 = 0.118;  // 118mm (0.118m in URDF joint origin)

// Lower leg length: estimated from URDF inertial origins
double L3 = 0.25;   // Approximate length of lower leg

// Body dimensions
double body_length = 0.242;  // Distance front to back: 0.046 + 0.196 = 0.242m
double body_width = 0.15;    // Distance left to right: 0.075 * 2 = 0.15m


// ==================== FRONT LEFT LEG ====================

vector<double> Front_Left_Leg_IK(vector<double> xyz_position) {
    double P1, P2, P3, P4;
    vector<double> joint_angles = {0, 0, 0};
    
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    // Calculate intermediate values
    P1 = sqrt(pow(x, 2) + pow(y, 2) - pow(L1, 2));
    P2 = sqrt(pow(P1, 2) + pow(z, 2));
    P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    // Joint angle calculations
    joint_angles[0] = atan2(y, x) - atan2(L1, P1);  // Rolling joint
    joint_angles[1] = atan2(-z, P1) + acos(P3);      // Pitching joint
    joint_angles[2] = -acos(P4);                      // Knee joint
    
    return joint_angles;
}

vector<double> Front_Left_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position = {0, 0, 0};
    
    double theta1 = joint_angles[0];  // Rolling
    double theta2 = joint_angles[1];  // Pitching
    double theta3 = joint_angles[2];  // Knee
    
    // Forward kinematics equations
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) - 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) + 
                      L1 * cos(theta1);
    
    xyz_position[2] = -L2 * sin(theta2) - L3 * sin(theta2 + theta3);
    
    return xyz_position;
}


// ==================== FRONT RIGHT LEG ====================

vector<double> Front_Right_Leg_IK(vector<double> xyz_position) {
    double P1, P2, P3, P4;
    vector<double> joint_angles = {0, 0, 0};
    
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    // Calculate intermediate values
    P1 = sqrt(pow(x, 2) + pow(y, 2) - pow(L1, 2));
    P2 = sqrt(pow(P1, 2) + pow(z, 2));
    P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    // Joint angle calculations (mirrored for right side)
    joint_angles[0] = atan2(-y, x) - atan2(L1, P1);  // Rolling joint
    joint_angles[1] = -(atan2(-z, P1) + acos(P3));   // Pitching joint
    joint_angles[2] = acos(P4);                       // Knee joint
    
    return joint_angles;
}

vector<double> Front_Right_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position = {0, 0, 0};
    
    double theta1 = joint_angles[0];  // Rolling
    double theta2 = joint_angles[1];  // Pitching
    double theta3 = joint_angles[2];  // Knee
    
    // Forward kinematics equations
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) + 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) - 
                      L1 * cos(theta1);
    
    xyz_position[2] = L2 * sin(theta2) + L3 * sin(theta2 + theta3);
    
    return xyz_position;
}


// ==================== BACK LEFT LEG ====================

vector<double> Back_Left_Leg_IK(vector<double> xyz_position) {
    double P1, P2, P3, P4;
    vector<double> joint_angles = {0, 0, 0};
    
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    // Calculate intermediate values
    P1 = sqrt(pow(x, 2) + pow(y, 2) - pow(L1, 2));
    P2 = sqrt(pow(P1, 2) + pow(z, 2));
    P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    // Joint angle calculations
    joint_angles[0] = atan2(-y, x) - atan2(L1, P1);  // Rolling joint
    joint_angles[1] = atan2(-z, P1) + acos(P3);      // Pitching joint
    joint_angles[2] = -acos(P4);                      // Knee joint
    
    return joint_angles;
}

vector<double> Back_Left_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position = {0, 0, 0};
    
    double theta1 = joint_angles[0];  // Rolling
    double theta2 = joint_angles[1];  // Pitching
    double theta3 = joint_angles[2];  // Knee
    
    // Forward kinematics equations
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) + 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) - 
                      L1 * cos(theta1);
    
    xyz_position[2] = -L2 * sin(theta2) - L3 * sin(theta2 + theta3);
    
    return xyz_position;
}


// ==================== BACK RIGHT LEG ====================

vector<double> Back_Right_Leg_IK(vector<double> xyz_position) {
    double P1, P2, P3, P4;
    vector<double> joint_angles = {0, 0, 0};
    
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    // Calculate intermediate values
    P1 = sqrt(pow(x, 2) + pow(y, 2) - pow(L1, 2));
    P2 = sqrt(pow(P1, 2) + pow(z, 2));
    P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    // Joint angle calculations (mirrored for right side)
    joint_angles[0] = atan2(y, x) - atan2(L1, P1);   // Rolling joint
    joint_angles[1] = -(atan2(-z, P1) + acos(P3));   // Pitching joint
    joint_angles[2] = acos(P4);                       // Knee joint
    
    return joint_angles;
}

vector<double> Back_Right_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position = {0, 0, 0};
    
    double theta1 = joint_angles[0];  // Rolling
    double theta2 = joint_angles[1];  // Pitching
    double theta3 = joint_angles[2];  // Knee
    
    // Forward kinematics equations
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) - 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) + 
                      L1 * cos(theta1);
    
    xyz_position[2] = L2 * sin(theta2) + L3 * sin(theta2 + theta3);
    
    return xyz_position;
}
