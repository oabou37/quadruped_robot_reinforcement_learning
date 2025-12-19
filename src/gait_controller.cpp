#include "Kinematics.h"
#include "trajecgen.h" 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>

using namespace std::chrono_literals;

class QuadrupedGaitController : public rclcpp::Node
{
public:
    QuadrupedGaitController() : Node("quadruped_gait_controller"), 
                                swing_phase_(1), 
                                is_initialized_(false),
                                current_cycle_time_(0.0)
    {
        // === PARAMÈTRES CORRIGÉS ===
        T_duration_ = 0.25; // 0.25 secondes par phase
        dt_ = 0.02;         // 50Hz
        
        // 1. CORRECTION HAUTEUR : On baisse le robot pour rester dans l'espace de travail
        // Si les jambes font 0.30m (0.15+0.15), 0.33m est impossible. 0.27m est sûr.
        H_ = 0.27; 
        
        SH_ = 0.05; // Hauteur de levée du pied (Step Height)
        SL_ = 0.0;  // Longueur de pas (dynamique)

        // Initialisation variables
        req_vel_ = {0.0, 0.0};
        
        // Positions XYZ [FL, FR, RL, RR]
        current_xyz_pos_.resize(4, {0.0, 0.0, 0.0});
        start_xyz_pos_.resize(4, {0.0, 0.0, 0.0});
        target_xyz_pos_.resize(4, {0.0, 0.0, 0.0});

        jnt_command_msg_.data.resize(12, 0.0);

        // Publishers & Subscribers
        jnt_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_commands", 10);
        
        req_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&QuadrupedGaitController::req_vel_callback, this, std::placeholders::_1));

        jnt_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&QuadrupedGaitController::joint_state_callback, this, std::placeholders::_1));

        // Calcul initial des angles de mouvement
        // Note: Assurez-vous que L1, body_length, etc. sont bien définis dans Kinematics.h
        Robot_angular_mtn_angles_ = Robot_angular_motion_endpoints(L1, body_length, body_width, SL_);

        // Timer de contrôle 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), 
            std::bind(&QuadrupedGaitController::control_tick, this));

        RCLCPP_INFO(this->get_logger(), "Quadruped Gait Controller Started (Corrected). Waiting for JointStates...");
    }

private:
    // --- Variables d'état ---
    bool is_initialized_;
    double current_cycle_time_;
    double T_duration_;
    double dt_;
    int swing_phase_;

    // Paramètres
    double SL_;
    double SH_;
    double H_;

    // Données
    std::vector<double> req_vel_;
    std::vector<std::vector<double>> Robot_angular_mtn_angles_;

    // Positions [Patte][X,Y,Z]
    std::vector<std::vector<double>> current_xyz_pos_;
    std::vector<std::vector<double>> start_xyz_pos_;
    std::vector<std::vector<double>> target_xyz_pos_;

    // ROS
    std_msgs::msg::Float64MultiArray jnt_command_msg_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jnt_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr req_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jnt_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callbacks ---

    void req_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        req_vel_[0] = msg->linear.x;
        req_vel_[1] = msg->angular.z;
        // SL adapté à la vitesse
        SL_ = std::min(0.2, std::sqrt(std::pow(req_vel_[0], 2) + std::pow(req_vel_[1], 2)) * 0.25);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 1. Mapping sécurisé (À ADAPTER SELON VOTRE URDF)
        std::map<std::string, double> jnt_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            jnt_map[msg->name[i]] = msg->position[i];
        }

        try {
            // Remplacez les chaînes ci-dessous par les vrais noms de vos joints
            std::vector<double> FL_jnt = {jnt_map.at("FL_roll"), jnt_map.at("FL_pitch"), jnt_map.at("FL_knee")};
            std::vector<double> FR_jnt = {jnt_map.at("FR_roll"), jnt_map.at("FR_pitch"), jnt_map.at("FR_knee")};
            std::vector<double> RL_jnt = {jnt_map.at("RL_roll"), jnt_map.at("RL_pitch"), jnt_map.at("RL_knee")};
            std::vector<double> RR_jnt = {jnt_map.at("RR_roll"), jnt_map.at("RR_pitch"), jnt_map.at("RR_knee")};

            current_xyz_pos_[0] = Front_Left_Leg_FK(FL_jnt);
            current_xyz_pos_[1] = Front_Right_Leg_FK(FR_jnt);
            current_xyz_pos_[2] = Back_Left_Leg_FK(RL_jnt);
            current_xyz_pos_[3] = Back_Right_Leg_FK(RR_jnt);

            if (!is_initialized_) {
                start_xyz_pos_ = current_xyz_pos_;
                target_xyz_pos_ = current_xyz_pos_; 
                is_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Initialized. H_body set to %.2f m", H_);
            }

        } catch (const std::out_of_range& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Joint mapping error. Check URDF names vs Code.");
        }
    }

    // --- Boucle Principale ---
    void control_tick()
    {
        if (!is_initialized_) return;

        current_cycle_time_ += dt_;

        // Fin de cycle : Changement de phase
        if (current_cycle_time_ >= T_duration_) {
            current_cycle_time_ = 0.0;
            swing_phase_ = 1 - swing_phase_;
            
            // Le point de départ du nouveau mouvement est la position actuelle
            start_xyz_pos_ = current_xyz_pos_;

            // Calcul des cibles pour la fin du cycle
            target_xyz_pos_ = Robot_end_points(req_vel_, Robot_angular_mtn_angles_, swing_phase_, SL_, H_);
        }

        double u = current_cycle_time_ / T_duration_;
        u = std::max(0.0, std::min(1.0, u));

        // Génération trajectoires
        process_leg(0, u, swing_phase_ == 1); // FL
        process_leg(3, u, swing_phase_ == 1); // RR
        process_leg(1, u, swing_phase_ == 0); // FR
        process_leg(2, u, swing_phase_ == 0); // RL

        jnt_pub_->publish(jnt_command_msg_);
    }

    // --- Cœur Mathématique (Corrigé) ---
    void process_leg(int leg_index, double u, bool is_swinging)
    {
        std::vector<double> next_pos(3);
        
        if (is_swinging) {
            // 2. CORRECTION TRAJECTOIRE : Méthode de Lagrange (Parabole exacte)
            // Plus de matrices compliquées, juste des poids polynomiaux
            
            // Calcul des poids pour t=0 (start), t=0.5 (mid), t=1 (end)
            double w_start = 2 * u * u - 3 * u + 1;
            double w_mid   = -4 * u * u + 4 * u;
            double w_end   = 2 * u * u - u;
            
            // Définition du point milieu (Sommet de la parabole)
            // X, Y = Milieu géométrique
            // Z = Hauteur moyenne + Step Height (Levée du pied)
            std::vector<double> mid_point = {
                (start_xyz_pos_[leg_index][0] + target_xyz_pos_[leg_index][0]) / 2.0,
                (start_xyz_pos_[leg_index][1] + target_xyz_pos_[leg_index][1]) / 2.0,
                (start_xyz_pos_[leg_index][2] + target_xyz_pos_[leg_index][2]) / 2.0 + SH_
            };

            // Application de la formule
            for(int i=0; i<3; i++) {
                next_pos[i] = w_start * start_xyz_pos_[leg_index][i] + 
                              w_mid   * mid_point[i] + 
                              w_end   * target_xyz_pos_[leg_index][i];
            }

        } else {
            // Interpolation Linéaire (Stance) - Resté inchangé car correct
            for (int i = 0; i < 3; ++i) {
                next_pos[i] = start_xyz_pos_[leg_index][i] * (1.0 - u) + target_xyz_pos_[leg_index][i] * u;
            }
        }

        // IK et Remplissage Message
        std::vector<double> joint_angles;
        
        if (leg_index == 0) joint_angles = Front_Left_Leg_IK(next_pos);
        else if (leg_index == 1) joint_angles = Front_Right_Leg_IK(next_pos);
        else if (leg_index == 2) joint_angles = Back_Left_Leg_IK(next_pos);
        else if (leg_index == 3) joint_angles = Back_Right_Leg_IK(next_pos);

        // Attention à l'ordre dans le tableau (Dépend de votre driver)
        int offset = 0;
        if(leg_index == 1) offset = 0;      // FR
        else if(leg_index == 0) offset = 3; // FL
        else if(leg_index == 3) offset = 6; // RR
        else if(leg_index == 2) offset = 9; // RL

        // Si l'IK échoue (retourne nan ou vide), on garde la dernière position pour éviter le crash
        if (joint_angles.size() == 3 && !std::isnan(joint_angles[0])) {
            jnt_command_msg_.data[offset + 0] = joint_angles[0];
            jnt_command_msg_.data[offset + 1] = joint_angles[1];
            jnt_command_msg_.data[offset + 2] = joint_angles[2];
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrupedGaitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
