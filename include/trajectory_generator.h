#ifndef __TRAJECGEN_H__
#define __TRAJECGEN_H__

#include <vector>
#include <cmath>
#include <iostream>

// Suppression de "using namespace std;" pour éviter les conflits
// Les variables globales (T, blending_matrix) sont déplacées dans la classe principale

/**
 * @brief Calcule les angles de rotation nécessaires pour les mouvements angulaires
 */
std::vector<std::vector<double>> Robot_angular_motion_endpoints(
    double L1, double L, double W, double SL);

/**
 * @brief Trouve les points d'intersection entre deux cercles (Cinématique inverse géométrique)
 */
std::vector<double> Circle_intersection_points(
    double h, double k, double R1, double R2);

/**
 * @brief Génère les positions cibles XYZ pour les 4 pieds du robot
 */
std::vector<std::vector<double>> Robot_end_points(
    std::vector<double> req_vel, 
    std::vector<std::vector<double>> mtn_angles, 
    int swing, 
    double SL, 
    double H
);

#endif // __TRAJECGEN_H__
