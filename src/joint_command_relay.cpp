#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <vector>
#include <string>

/**
 * @brief Nœud de relais : Convertit /joint_commands → /joint_states
 * 
 * Ce nœud est essentiel pour la visualisation dans RViz.
 * Le gait_controller publie des commandes sous forme de tableau de 12 floats,
 * mais RViz a besoin d'un message JointState avec noms + positions.
 */
class JointCommandRelay : public rclcpp::Node
{
public:
    JointCommandRelay() : Node("joint_command_relay")
    {
        // === NOMS DES JOINTS (EXACTEMENT COMME DANS L'URDF) ===
        joint_names_ = {
            // Front Left (indices 0-2)
            "front_left_rolling_joint",
            "front_left_pitching_joint",
            "front_left_knee_joint",
            
            // Front Right (indices 3-5)
            "front_right_rolling_joint",
            "front_right_pitching_joint",
            "front_right_knee_joint",
            
            // Back Left (indices 6-8)
            "back_left_rolling_joint",
            "back_left_pitching_joint",
            "back_left_knee_joint",
            
            // Back Right (indices 9-11)
            "back_right_rolling_joint",
            "back_right_pitching_joint",
            "back_right_knee_joint"
        };

        // Subscriber : Écoute les commandes du contrôleur
        command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_commands", 10,
            std::bind(&JointCommandRelay::command_callback, this, std::placeholders::_1));

        // Publisher : Envoie les états pour RViz
        state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        RCLCPP_INFO(this->get_logger(), "Joint Command Relay started.");
        RCLCPP_INFO(this->get_logger(), "Listening: /joint_commands → Publishing: /joint_states");
    }

private:
    std::vector<std::string> joint_names_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;

    void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // === VALIDATION ===
        if (msg->data.size() != 12) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "⚠ Received %zu joint commands, expected 12!", msg->data.size());
            return;
        }

        // === CONSTRUCTION DU MESSAGE JOINT_STATE ===
        sensor_msgs::msg::JointState joint_state_msg;
        
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.header.frame_id = ""; // Pas de frame nécessaire
        
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = msg->data;
        
        // Velocity et Effort vides (non utilisés pour visualisation)
        joint_state_msg.velocity.resize(12, 0.0);
        joint_state_msg.effort.resize(12, 0.0);

        // === PUBLICATION ===
        state_pub_->publish(joint_state_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointCommandRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
