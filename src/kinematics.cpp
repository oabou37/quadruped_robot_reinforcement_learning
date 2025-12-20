#include "Kinematics.h"
#include <cmath>
#include <algorithm>

using namespace std;

// ============================================================================
// DIMENSIONS PHYSIQUES (Extraites de testudog.urdf)
// ============================================================================
double L1 = 0.056;  // Hip offset
double L2 = 0.118;  // Upper leg
double L3 = 0.250;  // Lower leg
double body_length = 0.242;
double body_width = 0.15;

// ============================================================================
// LIMITES ARTICULAIRES (Extraites de l'URDF)
// ============================================================================
// Ces limites sont CRITIQUES pour éviter d'envoyer des commandes impossibles
const double ROLL_MIN  = -0.785;  // -45°
const double ROLL_MAX  =  0.785;  //  45°
const double PITCH_MIN = -1.571;  // -90°
const double PITCH_MAX =  1.571;  //  90°
const double KNEE_MIN  = -2.094;  // -120°
const double KNEE_MAX  =  2.094;  //  120°

// ============================================================================
// UTILITAIRES DE SÉCURITÉ
// ============================================================================

/**
 * @brief Clamp une valeur pour le domaine de acos [-1, 1]
 */
inline double safe_acos(double val) {
    return acos(std::max(-1.0, std::min(1.0, val)));
}

/**
 * @brief Applique les limites articulaires depuis l'URDF
 */
inline void apply_joint_limits(vector<double>& angles) {
    angles[0] = std::max(ROLL_MIN, std::min(ROLL_MAX, angles[0]));
    angles[1] = std::max(PITCH_MIN, std::min(PITCH_MAX, angles[1]));
    angles[2] = std::max(KNEE_MIN, std::min(KNEE_MAX, angles[2]));
}

// ============================================================================
// CONVENTION Z : Z NÉGATIF = VERS LE BAS (Sol)
// ============================================================================

// ==================== PATTE AVANT GAUCHE (FRONT LEFT) ====================

vector<double> Front_Left_Leg_IK(vector<double> xyz_position) {
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    // === VALIDATION 1 : Singularité centrale ===
    double xy_dist_sq = pow(x, 2) + pow(y, 2);
    if (xy_dist_sq < pow(L1, 2) + 1e-6) {
        return {NAN, NAN, NAN};
    }
    
    // === CALCUL INTERMÉDIAIRE ===
    double P1 = sqrt(xy_dist_sq - pow(L1, 2));
    double P2 = sqrt(pow(P1, 2) + pow(z, 2));
    
    // === VALIDATION 2 : Portée ===
    double max_reach = L2 + L3;
    double min_reach = fabs(L2 - L3);
    
    if (P2 > max_reach - 1e-3 || P2 < min_reach + 1e-3) {
        return {NAN, NAN, NAN};
    }
    
    // === CALCUL ANGLES ===
    double P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    double P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    vector<double> joint_angles(3);
    joint_angles[0] = atan2(y, x) - atan2(L1, P1);
    joint_angles[1] = atan2(-z, P1) + safe_acos(P3);
    joint_angles[2] = -safe_acos(P4);
    
    apply_joint_limits(joint_angles);
    return joint_angles;
}

vector<double> Front_Left_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position(3);
    
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2];
    
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) - 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) + 
                      L1 * cos(theta1);
    
    xyz_position[2] = -L2 * sin(theta2) - L3 * sin(theta2 + theta3);
    
    return xyz_position;
}

// ==================== PATTE AVANT DROITE (FRONT RIGHT) ====================

vector<double> Front_Right_Leg_IK(vector<double> xyz_position) {
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    double xy_dist_sq = pow(x, 2) + pow(y, 2);
    if (xy_dist_sq < pow(L1, 2) + 1e-6) {
        return {NAN, NAN, NAN};
    }
    
    double P1 = sqrt(xy_dist_sq - pow(L1, 2));
    double P2 = sqrt(pow(P1, 2) + pow(z, 2));
    
    double max_reach = L2 + L3;
    double min_reach = fabs(L2 - L3);
    
    if (P2 > max_reach - 1e-3 || P2 < min_reach + 1e-3) {
        return {NAN, NAN, NAN};
    }
    
    double P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    double P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    vector<double> joint_angles(3);
    joint_angles[0] = atan2(-y, x) - atan2(L1, P1);
    joint_angles[1] = atan2(-z, P1) + safe_acos(P3);
    joint_angles[2] = -safe_acos(P4);
    
    apply_joint_limits(joint_angles);
    return joint_angles;
}

vector<double> Front_Right_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position(3);
    
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2];
    
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) + 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) - 
                      L1 * cos(theta1);
    
    xyz_position[2] = -L2 * sin(theta2) - L3 * sin(theta2 + theta3);
    
    return xyz_position;
}

// ==================== PATTE ARRIÈRE GAUCHE (BACK LEFT) ====================

vector<double> Back_Left_Leg_IK(vector<double> xyz_position) {
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    double xy_dist_sq = pow(x, 2) + pow(y, 2);
    if (xy_dist_sq < pow(L1, 2) + 1e-6) {
        return {NAN, NAN, NAN};
    }
    
    double P1 = sqrt(xy_dist_sq - pow(L1, 2));
    double P2 = sqrt(pow(P1, 2) + pow(z, 2));
    
    double max_reach = L2 + L3;
    double min_reach = fabs(L2 - L3);
    
    if (P2 > max_reach - 1e-3 || P2 < min_reach + 1e-3) {
        return {NAN, NAN, NAN};
    }
    
    double P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    double P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    vector<double> joint_angles(3);
    joint_angles[0] = atan2(-y, x) - atan2(L1, P1);
    joint_angles[1] = atan2(-z, P1) + safe_acos(P3);
    joint_angles[2] = -safe_acos(P4);
    
    apply_joint_limits(joint_angles);
    return joint_angles;
}

vector<double> Back_Left_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position(3);
    
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2];
    
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) + 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) - 
                      L1 * cos(theta1);
    
    xyz_position[2] = -L2 * sin(theta2) - L3 * sin(theta2 + theta3);
    
    return xyz_position;
}

// ==================== PATTE ARRIÈRE DROITE (BACK RIGHT) ====================

vector<double> Back_Right_Leg_IK(vector<double> xyz_position) {
    double x = xyz_position[0];
    double y = xyz_position[1];
    double z = xyz_position[2];
    
    double xy_dist_sq = pow(x, 2) + pow(y, 2);
    if (xy_dist_sq < pow(L1, 2) + 1e-6) {
        return {NAN, NAN, NAN};
    }
    
    double P1 = sqrt(xy_dist_sq - pow(L1, 2));
    double P2 = sqrt(pow(P1, 2) + pow(z, 2));
    
    double max_reach = L2 + L3;
    double min_reach = fabs(L2 - L3);
    
    if (P2 > max_reach - 1e-3 || P2 < min_reach + 1e-3) {
        return {NAN, NAN, NAN};
    }
    
    double P3 = (pow(P2, 2) + pow(L2, 2) - pow(L3, 2)) / (2 * P2 * L2);
    double P4 = -(pow(L2, 2) + pow(L3, 2) - pow(P2, 2)) / (2 * L3 * L2);
    
    vector<double> joint_angles(3);
    joint_angles[0] = atan2(y, x) - atan2(L1, P1);
    joint_angles[1] = atan2(-z, P1) + safe_acos(P3);
    joint_angles[2] = -safe_acos(P4);
    
    apply_joint_limits(joint_angles);
    return joint_angles;
}

vector<double> Back_Right_Leg_FK(vector<double> joint_angles) {
    vector<double> xyz_position(3);
    
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2];
    
    xyz_position[0] = L2 * cos(theta1) * cos(theta2) + 
                      L3 * cos(theta1) * cos(theta2 + theta3) - 
                      L1 * sin(theta1);
    
    xyz_position[1] = L2 * sin(theta1) * cos(theta2) + 
                      L3 * sin(theta1) * cos(theta2 + theta3) + 
                      L1 * cos(theta1);
    
    xyz_position[2] = -L2 * sin(theta2) - L3 * sin(theta2 + theta3);
    
    return xyz_position;
}
