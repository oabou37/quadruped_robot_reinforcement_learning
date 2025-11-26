#ifndef __TRAJECGEN_H__
#define __TRAJECGEN_H__

#include <vector>
#include <cmath>

using namespace std;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// T : Durée d'un demi-cycle de marche (temps pour une phase swing ou stance)
// Plus T est petit, plus le robot marche vite
extern double T;

// blending_matrix : Matrice pour interpolation parabolique des trajectoires
// Utilisée pour créer des trajectoires lisses (pas de saccades)
// Format 3x3 pour interpolation quadratique entre 3 points de contrôle
extern vector<vector<double>> blending_matrix;


// ============================================================================
// FONCTION 1 : Calcul des angles de rotation du corps
// ============================================================================
/**
 * @brief Calcule les angles de rotation nécessaires pour les mouvements angulaires
 * 
 * Cette fonction détermine comment le corps du robot doit pivoter lorsqu'il 
 * effectue des rotations sur place ou pendant la marche en virage.
 * 
 * @param L1 : Décalage de la hanche (distance rolling→pitching joint) [m]
 * @param L  : Longueur du corps (distance avant-arrière) [m]
 * @param W  : Largeur du corps (distance gauche-droite) [m]
 * @param SL : Longueur du pas maximale [m]
 * 
 * @return Matrice des angles de rotation pour chaque patte [4x2]
 *         Format: [[FL_angles], [FR_angles], [BL_angles], [BR_angles]]
 *         Chaque ligne contient les angles pour mouvements linéaires et rotatifs
 * 
 * Utilité : Permet au robot de tourner sur place ou de combiner marche + rotation
 */
vector<vector<double>> Robot_angular_motion_endpoints(double L1, double L, double W, double SL);


// ============================================================================
// FONCTION 2 : Calcul des points d'intersection de cercles
// ============================================================================
/**
 * @brief Trouve les points d'intersection entre deux cercles
 * 
 * Cette fonction calcule géométriquement où placer les pieds du robot
 * en tenant compte des contraintes physiques (longueur des jambes).
 * 
 * @param h  : Coordonnée X du centre du premier cercle
 * @param k  : Coordonnée Y du centre du premier cercle
 * @param R1 : Rayon du premier cercle (portée maximale de la jambe)
 * @param R2 : Rayon du second cercle
 * 
 * @return Vecteur contenant les coordonnées [x, y] du point d'intersection
 * 
 * Utilité : Résolution du problème de cinématique inverse de manière géométrique
 *          Évite les singularités et garantit que les pieds restent atteignables
 */
vector<double> Circle_intersection_points(double h, double k, double R1, double R2);


// ============================================================================
// FONCTION 3 : Calcul des positions cibles des pieds (FONCTION PRINCIPALE)
// ============================================================================
/**
 * @brief Génère les positions cibles XYZ pour les 4 pieds du robot
 * 
 * C'est la fonction MAÎTRESSE qui coordonne toute la marche du robot.
 * Elle calcule où chaque pied doit se poser pour le prochain demi-cycle.
 * 
 * @param req_vel      : Vitesse commandée [linéaire_x, angulaire_z]
 *                       - req_vel[0] : vitesse avant/arrière (m/s)
 *                       - req_vel[1] : vitesse de rotation (rad/s)
 * 
 * @param mtn_angles   : Angles pré-calculés pour mouvements angulaires
 *                       (résultat de Robot_angular_motion_endpoints)
 * 
 * @param swing        : Indique quelle paire de pattes est en phase swing
 *                       - swing = 1 : FL et RR en l'air (phase swing)
 *                                     FR et RL au sol (phase stance)
 *                       - swing = 0 : FR et RL en l'air
 *                                     FL et RR au sol
 * 
 * @param SL           : Longueur du pas actuelle [m]
 *                       (adaptée automatiquement selon la vitesse)
 * 
 * @param H            : Hauteur désirée du corps par rapport au sol [m]
 * 
 * @return Matrice 4x3 des positions cibles [x, y, z] pour chaque pied
 *         Format: [[FL_xyz], [FR_xyz], [BL_xyz], [BR_xyz]]
 *         - Pattes en swing : nouvelle position après le pas
 *         - Pattes en stance : position ajustée pour stabilité
 * 
 * Utilité : 
 * - Implémente le pattern de marche "trot diagonal" (2 pattes opposées ensemble)
 * - Combine mouvement linéaire + rotation
 * - Adapte la longueur des pas selon la vitesse
 * - Maintient l'équilibre du robot
 * - Fournit les targets pour la cinématique inverse (IK)
 */
vector<vector<double>> Robot_end_points(
    vector<double> req_vel, 
    vector<vector<double>> mtn_angles, 
    int swing, 
    double SL, 
    double H
);


// ============================================================================
// PRINCIPE DE FONCTIONNEMENT GLOBAL
// ============================================================================
/*
 * PIPELINE DE GÉNÉRATION DE TRAJECTOIRE :
 * 
 * 1. INITIALISATION (une seule fois au démarrage) :
 *    - Calculer Robot_angular_motion_endpoints() → angles de référence
 *    - Initialiser blending_matrix pour interpolation parabolique
 * 
 * 2. BOUCLE DE MARCHE (à chaque cycle) :
 *    a) Lire la vitesse demandée (/cmd_vel)
 *    b) Calculer Robot_end_points() → positions cibles XYZ des 4 pieds
 *    c) Pour chaque pied :
 *       - Si en swing : trajectoire parabolique (lever, avancer, poser)
 *       - Si en stance : trajectoire linéaire (glisser vers l'arrière)
 *    d) À chaque instant t :
 *       - Interpoler la position actuelle sur la trajectoire
 *       - Appliquer IK pour convertir XYZ → angles articulaires
 *       - Envoyer les commandes aux moteurs
 *    e) Alterner swing (1→0→1→0...)
 * 
 * 3. COORDINATION DES PATTES (Trot diagonal) :
 *    Cycle 1: FL+RR en l'air, FR+RL au sol
 *    Cycle 2: FR+RL en l'air, FL+RR au sol
 *    → Garantit toujours 2 pattes au sol = stabilité
 * 
 * 4. INTERPOLATION PARABOLIQUE :
 *    - Point de départ : position actuelle du pied
 *    - Point intermédiaire : mi-chemin + hauteur de levée (SH)
 *    - Point d'arrivée : position cible calculée
 *    - blending_matrix transforme ces 3 points en trajectoire lisse
 */

#endif // __TRAJECGEN_H__
