#include "trajecgen.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

// ============================================================================
// PARAMÈTRES PHYSIQUES & SÉCURITÉ
// ============================================================================
// Limites mécaniques strictes (Doivent matcher l'URDF/Kinematics)
const double L2_MAX = 0.118; // Cuisse
const double L3_MAX = 0.120; // Tibia
const double MIN_REACH = 0.010; // Rayon min (pour éviter singularité au centre)

// ============================================================================
// 1. CALCUL DE LA GÉOMÉTRIE STATIQUE (POLAIRE)
// ============================================================================
// Cette fonction définit la "forme" du robot au repos en coordonnées polaires.
vector<vector<double>> Robot_angular_motion_endpoints(double L1, double L, double W, double SL) {
    vector<vector<double>> angles(4, vector<double>(2));
    
    // Dimensions effectives depuis le centre de masse (CoM)
    double half_L = L / 2.0;
    double half_W_eff = W / 2.0 + L1; // Largeur + déport hanche

    // 1. Calcul du Rayon (R) - Constant pour toutes les pattes si symétrique
    double R = std::hypot(half_L, half_W_eff); // hypot(x,y) = sqrt(x²+y²)

    // 2. Calcul des Angles (Alpha) - Repère ROS (X devant, Y gauche)
    // FL (Front Left)  : (+, +)
    angles[0] = {R, atan2(half_W_eff, half_L)};
    // FR (Front Right) : (+, -)
    angles[1] = {R, atan2(-half_W_eff, half_L)};
    // RL (Rear Left)   : (-, +)
    angles[2] = {R, atan2(half_W_eff, -half_L)};
    // RR (Rear Right)  : (-, -)
    angles[3] = {R, atan2(-half_W_eff, -half_L)};

    return angles;
}

// ============================================================================
// 2. GÉNÉRATION DES CIBLES DE TRAJECTOIRE (TARGETS)
// ============================================================================
vector<vector<double>> Robot_end_points(
    vector<double> req_vel, 
    vector<vector<double>> mtn_angles, 
    int swing, 
    double SL, 
    double H
) {
    vector<vector<double>> targets(4, vector<double>(3));

    // Entrées de commande
    double cmd_linear_x = req_vel[0];  // Vitesse linéaire demandée
    double cmd_angular_z = req_vel[1]; // Vitesse angulaire demandée

    // --- A. Calcul de la zone de travail valide (Safety Bubble) ---
    // Portée horizontale maximale possible à la hauteur H
    double max_extension = L2_MAX + L3_MAX;
    double max_reach_2d = 0.0;

    if (max_extension > H) {
        max_reach_2d = sqrt(pow(max_extension, 2) - pow(H, 2));
        // Marge de sécurité (2mm) pour ne pas tendre la jambe à 100% (singularité)
        max_reach_2d -= 0.002; 
    } else {
        max_reach_2d = MIN_REACH; // Robot trop bas, quasi impossible de bouger
    }

    // Détermination du sens de marche (1: Avant, -1: Arrière)
    double direction_sign = (cmd_linear_x >= 0) ? 1.0 : -1.0;

    // Facteur d'échelle pour la rotation (Conversion rad/s -> déplacement par pas)
    // Empirique : permet d'équilibrer l'amplitude du pas linéaire (SL) et rotatif.
    const double ROTATION_SCALE = 0.15; 

    for(int i=0; i<4; i++) {
        // --- B. Reconstruction de la Position Neutre ---
        // On utilise les données polaires (mtn_angles) pour retrouver le Cartésien (X, Y).
        // C'est ici que se fait le lien mathématique "propre" avec la fonction précédente.
        double R = mtn_angles[i][0];
        double Alpha = mtn_angles[i][1];

        double neutral_x = R * cos(Alpha);
        double neutral_y = R * sin(Alpha);

        // --- C. Gestion de la Phase (Swing vs Stance) ---
        // Trot Diagonal : (FL+RR) vs (FR+RL)
        bool is_swing_leg = false;
        if (swing == 1 && (i == 0 || i == 3)) is_swing_leg = true;
        if (swing == 0 && (i == 1 || i == 2)) is_swing_leg = true;

        // Le signe s'inverse entre Swing (on avance la patte) et Stance (on recule la patte)
        double phase_sign = is_swing_leg ? 1.0 : -1.0;

        // --- D. Calcul des Déplacements (Cinématique) ---

        // 1. Déplacement Linéaire (X)
        // Amplitude totale du pas = SL. On va de -SL/2 à +SL/2.
        double delta_lin_x = phase_sign * (SL / 2.0) * direction_sign;
        double delta_lin_y = 0.0; // Pas de pas latéral (Crab walk) implémenté ici

        // 2. Déplacement Angulaire (Yaw)
        // Cinématique différentielle : V = Omega x R
        // dx = -y * omega, dy = x * omega
        // On applique le ROTATION_SCALE pour transformer la vitesse en amplitude de pas.
        double rot_val = cmd_angular_z * ROTATION_SCALE;
        
        // Note: neutral_y et neutral_x sont les coordonnées locales de la patte
        double delta_rot_x = -neutral_y * rot_val;
        double delta_rot_y =  neutral_x * rot_val;

        // En phase Stance, le sol doit défiler dans le sens inverse du mouvement souhaité
        if (!is_swing_leg) {
            delta_rot_x = -delta_rot_x;
            delta_rot_y = -delta_rot_y;
        }

        // --- E. Somme et Application ---
        double target_dx = delta_lin_x + delta_rot_x;
        double target_dy = delta_lin_y + delta_rot_y;

        // --- F. Clamping (Sécurité de portée) ---
        // On vérifie si le point cible (neutral + delta) est accessible
        double final_x = neutral_x + target_dx;
        double final_y = neutral_y + target_dy;

        // Distance du point cible par rapport à l'épaule (Origine 0,0 locale de la patte ?)
        // ATTENTION: Dans ce modèle simplifié, l'épaule est le point de pivot.
        // Le "neutral point" calculé plus haut est la position par défaut du pied.
        // La portée se mesure depuis l'origine de la hanche (environ neutral_x, neutral_y modifiés par L1).
        // SIMPLIFICATION ROBUSTE : On clamp le *déplacement* par rapport au neutre 
        // pour ne pas demander un pas plus grand que la capacité physique autour du neutre.
        
        double dist_from_neutral = std::hypot(target_dx, target_dy);
        double max_allowed_offset = max_reach_2d * 0.4; // On autorise ~40% de la jambe en amplitude de pas

        if (dist_from_neutral > max_allowed_offset) {
            double scale = max_allowed_offset / dist_from_neutral;
            target_dx *= scale;
            target_dy *= scale;
        }

        // --- G. Sortie Finale ---
        targets[i][0] = neutral_x + target_dx;
        targets[i][1] = neutral_y + target_dy;
        targets[i][2] = -H; // Z constant (la courbe de levée est gérée par le contrôleur)
    }

    return targets;
}
