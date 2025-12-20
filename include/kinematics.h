#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <cmath>

// Dimensions globales (définies dans Kinematics.cpp)
extern double L1;
extern double L2;
extern double L3;
extern double body_length;
extern double body_width;

/**
 * @brief Cinématique Inverse (IK) - Modèle Z-Up
 * @param x, y, z : Position du pied relative à l'épaule
 * @param side_sign : 1 pour Gauche, -1 pour Droite
 * @return vector<double> {Roll, Pitch, Knee} en radians
 */
std::vector<double> Leg_IK(double x, double y, double z, int side_sign);

/**
 * @brief Cinématique Directe (FK) - Modèle Z-Up
 * @return vector<double> {x, y, z}
 */
std::vector<double> Leg_FK(std::vector<double> angles, int side_sign);

// Wrappers spécifiques pour chaque patte
std::vector<double> Front_Left_Leg_IK(std::vector<double> xyz);
std::vector<double> Front_Right_Leg_IK(std::vector<double> xyz);
std::vector<double> Back_Left_Leg_IK(std::vector<double> xyz);
std::vector<double> Back_Right_Leg_IK(std::vector<double> xyz);

std::vector<double> Front_Left_Leg_FK(std::vector<double> angles);
std::vector<double> Front_Right_Leg_FK(std::vector<double> angles);
std::vector<double> Back_Left_Leg_FK(std::vector<double> angles);
std::vector<double> Back_Right_Leg_FK(std::vector<double> angles);

#endif // KINEMATICS_H
