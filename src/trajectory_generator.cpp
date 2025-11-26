#include "trajectory_generator.h"
#include "Kinematics.h"
#include <iostream>

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// Temps pour un demi-cycle de marche (calculé dynamiquement selon vitesse)
double T = 0;

// Matrice de blending pour interpolation parabolique (trajectoires lisses)
// Transforme 3 points de contrôle en courbe parabolique
vector<vector<double>> blending_matrix = {{1, -3, 2},
                                          {0,  4, -4},
                                          {0, -1, 2}};


// ============================================================================
// FONCTION 1 : Calcul des points d'intersection de deux cercles
// ============================================================================
/**
 * @brief Trouve les 2 points d'intersection entre deux cercles
 * 
 * Résout le système géométrique pour trouver où deux cercles se croisent.
 * Utilisé pour déterminer les positions de pieds atteignables.
 * 
 * @param h  : Décalage en X du centre du cercle 1 par rapport à l'origine
 * @param k  : Décalage en Y du centre du cercle 1 par rapport à l'origine
 * @param R1 : Rayon du cercle 1 (distance depuis origine)
 * @param R2 : Rayon du cercle 2 (longueur du pas / 2)
 * 
 * @return Vecteur [x1, x2, y1, y2] avec les coordonnées des 2 intersections
 *         - (x1, y1) : premier point d'intersection
 *         - (x2, y2) : second point d'intersection
 */
vector<double> Circle_intersection_points(double h, double k, double R1, double R2) {
    vector<double> intersection_pnts = {0, 0, 0, 0};
    
    // Calcul du discriminant pour vérifier l'existence des intersections
    double discriminant = (-pow((R1-R2), 2) + pow(h, 2) + pow(k, 2)) * 
                          (pow((R1+R2), 2) - pow(h, 2) - pow(k, 2));
    
    if (discriminant < 0) {
        std::cerr << "Warning: No circle intersection found!" << std::endl;
        return intersection_pnts;
    }
    
    double sqrt_term = sqrt(discriminant);
    double denom = 2 * (pow(h, 2) + pow(k, 2));
    
    // Coordonnées X des deux intersections (relatives au centre h)
    intersection_pnts[0] = ((pow(R1, 2)*h - pow(R2, 2)*h + h*pow(k, 2) + 
                            k*sqrt_term + pow(h, 3)) / denom) - h;
    
    intersection_pnts[1] = ((pow(R1, 2)*h - pow(R2, 2)*h + h*pow(k, 2) - 
                            k*sqrt_term + pow(h, 3)) / denom) - h;
    
    // Coordonnées Y des deux intersections (relatives au centre k)
    intersection_pnts[2] = ((pow(R1, 2)*k - pow(R2, 2)*k + k*pow(h, 2) + 
                            h*sqrt_term + pow(k, 3)) / denom) - k;
    
    intersection_pnts[3] = ((pow(R1, 2)*k - pow(R2, 2)*k + k*pow(h, 2) - 
                            h*sqrt_term + pow(k, 3)) / denom) - k;
    
    return intersection_pnts;
}


// ============================================================================
// FONCTION 2 : Calcul des angles de mouvement angulaire pour chaque patte
// ============================================================================
/**
 * @brief Détermine les angles de déplacement des pattes pour rotation sur place
 * 
 * Calcule comment chaque pied doit se déplacer pour faire tourner le robot
 * autour de son centre. Utilise la géométrie pour trouver les trajectoires
 * circulaires optimales.
 * 
 * @param L1 : Décalage de la hanche (offset du rolling joint) [m]
 * @param L  : Longueur du corps (distance avant-arrière) [m]  
 * @param W  : Largeur du corps (distance gauche-droite) [m]
 * @param SL : Longueur maximale du pas [m]
 * 
 * @return Matrice 2x4 des angles [right_turn, left_turn] pour [FL, FR, BL, BR]
 *         - right_turn_angles[i] : angle pour rotation droite de la patte i
 *         - left_turn_angles[i]  : angle pour rotation gauche de la patte i
 */
vector<vector<double>> Robot_angular_motion_endpoints(double L1, double L, double W, double SL) {
    
    vector<double> right_turn_angles = {0, 0, 0, 0};
    vector<double> left_turn_angles = {0, 0, 0, 0};
    
    // Rayon de rotation (distance centre robot → pied)
    double Rr = sqrt(pow((L1 + W/2), 2) + pow(L/2, 2));
    
    double h{}, k{};
    
    // ========== FRONT LEFT (FL) ==========
    h = W/2 + L1;   // Position X du pied FL
    k = L/2;        // Position Y du pied FL
    vector<double> FL_intersections = Circle_intersection_points(h, k, Rr, SL/2);
    right_turn_angles[0] = atan2(FL_intersections[2], FL_intersections[1]);
    left_turn_angles[0] = atan2(FL_intersections[3], FL_intersections[0]);
    
    // ========== FRONT RIGHT (FR) ==========
    h = -W/2 - L1;  // Position X du pied FR (négatif = côté droit)
    k = L/2;        // Position Y du pied FR
    vector<double> FR_intersections = Circle_intersection_points(h, k, Rr, SL/2);
    right_turn_angles[1] = atan2(FR_intersections[2], FR_intersections[1]);
    left_turn_angles[1] = atan2(FR_intersections[3], FR_intersections[0]);
    
    // ========== BACK LEFT (BL/RL) ==========
    h = W/2 + L1;   // Position X du pied BL
    k = -L/2;       // Position Y du pied BL (négatif = arrière)
    vector<double> RL_intersections = Circle_intersection_points(h, k, Rr, SL/2);
    right_turn_angles[2] = atan2(RL_intersections[2], RL_intersections[1]);
    left_turn_angles[2] = atan2(RL_intersections[3], RL_intersections[0]);
    
    // ========== BACK RIGHT (BR/RR) ==========
    h = -W/2 - L1;  // Position X du pied BR
    k = -L/2;       // Position Y du pied BR
    vector<double> RR_intersections = Circle_intersection_points(h, k, Rr, SL/2);
    right_turn_angles[3] = atan2(RR_intersections[2], RR_intersections[1]);
    left_turn_angles[3] = atan2(RR_intersections[3], RR_intersections[0]);
    
    vector<vector<double>> output_angles = {right_turn_angles, left_turn_angles};
    return output_angles;
}


// ============================================================================
// FONCTION 3 : Calcul des positions cibles (endpoints) pour les 4 pieds
// ============================================================================
/**
 * @brief Génère les positions XYZ cibles pour les 4 pieds du robot
 * 
 * FONCTION PRINCIPALE du générateur de trajectoire.
 * Combine mouvements linéaires et angulaires pour calculer où placer
 * chaque pied au prochain pas. Adapte automatiquement selon vitesse demandée.
 * 
 * @param req_vel     : Vitesse commandée [linéaire_x, angulaire_z]
 * @param mtn_angles  : Angles pré-calculés pour rotations (de Robot_angular_motion_endpoints)
 * @param swing       : Phase actuelle (1 = FL+BR en swing, 0 = FR+BL en swing)
 * @param SL          : Longueur du pas actuelle [m]
 * @param H           : Hauteur désirée du robot [m]
 * 
 * @return Matrice 4x3 des positions [x, y, z] pour [FL, FR, BL, BR]
 */
vector<vector<double>> Robot_end_points(vector<double> req_vel, 
                                        vector<vector<double>> mtn_angles, 
                                        int swing, 
                                        double SL, 
                                        double H) {
    
    // Offset vertical pour éviter frottement au sol
    double z_offset = 0.03;
    
    // ========== CAS SPÉCIAL : ROBOT IMMOBILE ==========
    if (req_vel[0] == 0 && req_vel[1] == 0) {
        // Position de repos : pieds sous le corps
        vector<double> FL_end_point = {H, L1, z_offset};
        vector<double> FR_end_point = {H, -L1, z_offset};
        vector<double> BL_end_point = {H, L1, z_offset};
        vector<double> BR_end_point = {H, -L1, z_offset};
        
        vector<vector<double>> out = {FL_end_point, FR_end_point, BL_end_point, BR_end_point};
        T = 0.1;  // Temps minimal pour retour à position de repos
        return out;
    }
    
    // ========== CALCUL DES PARAMÈTRES DE MOUVEMENT ==========
    
    // Rayon de rotation du robot (distance centre → pied)
    double Rr = sqrt(pow((L1 + body_width/2), 2) + pow(body_length/2, 2));
    
    // Calcul de l'angle de rotation pour un pas de longueur SL
    double x = 1 - pow(SL, 2) / (2 * pow(Rr, 2));
    double theta = atan(sqrt(1 - pow(x, 2)) / x);
    
    // Vitesse angulaire effective (rad/s)
    double angular_velocity = abs((req_vel[1] * SL) / theta);
    
    double y{}, z{};  // Composantes de vitesse temporaires
    
    
    // ========== FRONT LEFT (FL) ==========
    vector<double> FL_end_point = {0, 0, 0};
    double FL_theta{};
    
    // Sélection angle selon phase et direction de rotation
    if (swing) {
        FL_theta = (req_vel[1] > 0) ? mtn_angles[1][0] : mtn_angles[0][0];
    } else {
        FL_theta = (req_vel[1] > 0) ? mtn_angles[0][0] : mtn_angles[1][0];
    }
    
    // Calcul vitesse effective combinant rotation + translation
    y = angular_velocity * cos(FL_theta);
    z = angular_velocity * sin(FL_theta);
    z += (swing) ? req_vel[0] : -req_vel[0];  // Swing avance, stance recule
    
    FL_theta = atan2(z, y);
    FL_end_point[0] = H;
    FL_end_point[1] = cos(FL_theta) * SL/2 + L1;  // L1 = offset de hanche
    FL_end_point[2] = sin(FL_theta) * SL/2 + z_offset;
    
    
    // ========== BACK RIGHT (BR/RR) ==========
    vector<double> BR_end_point = {0, 0, 0};
    double BR_theta{};
    
    if (swing) {
        BR_theta = (req_vel[1] > 0) ? mtn_angles[1][3] : mtn_angles[0][3];
    } else {
        BR_theta = (req_vel[1] > 0) ? mtn_angles[0][3] : mtn_angles[1][3];
    }
    
    y = angular_velocity * cos(BR_theta);
    z = angular_velocity * sin(BR_theta);
    z += (swing) ? req_vel[0] : -req_vel[0];
    
    BR_theta = atan2(z, y);
    BR_end_point[0] = H;
    BR_end_point[1] = cos(BR_theta) * SL/2 - L1;  // -L1 pour côté droit
    BR_end_point[2] = sin(BR_theta) * SL/2 + z_offset;
    
    
    // ========== FRONT RIGHT (FR) ==========
    vector<double> FR_end_point = {0, 0, 0};
    double FR_theta{};
    
    if (swing) {
        FR_theta = (req_vel[1] > 0) ? mtn_angles[0][1] : mtn_angles[1][1];
    } else {
        FR_theta = (req_vel[1] > 0) ? mtn_angles[1][1] : mtn_angles[0][1];
    }
    
    y = angular_velocity * cos(FR_theta);
    z = angular_velocity * sin(FR_theta);
    z += (swing) ? -req_vel[0] : req_vel[0];  // Inverse pour côté droit
    
    FR_theta = atan2(z, y);
    FR_end_point[0] = H;
    FR_end_point[1] = cos(FR_theta) * SL/2 - L1;
    FR_end_point[2] = sin(FR_theta) * SL/2 + z_offset;
    
    
    // ========== BACK LEFT (BL/RL) ==========
    vector<double> BL_end_point = {0, 0, 0};
    double BL_theta{};
    
    if (swing) {
        BL_theta = (req_vel[1] > 0) ? mtn_angles[0][2] : mtn_angles[1][2];
    } else {
        BL_theta = (req_vel[1] > 0) ? mtn_angles[1][2] : mtn_angles[0][2];
    }
    
    y = angular_velocity * cos(BL_theta);
    z = angular_velocity * sin(BL_theta);
    z += (swing) ? -req_vel[0] : req_vel[0];
    
    BL_theta = atan2(z, y);
    BL_end_point[0] = H;
    BL_end_point[1] = cos(BL_theta) * SL/2 + L1;
    BL_end_point[2] = sin(BL_theta) * SL/2 + z_offset;
    
    
    // ========== CALCUL DU TEMPS DE CYCLE ==========
    // T = distance / vitesse (temps pour parcourir un pas)
    double velocity = sqrt(pow(y, 2) + pow(z, 2));
    T = SL / velocity;
    
    
    // ========== RETOUR DES RÉSULTATS ==========
    vector<vector<double>> out = {FL_end_point, FR_end_point, BL_end_point, BR_end_point};
    return out;
}
