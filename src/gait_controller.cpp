#include "Kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_generator.h"
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class QuadrupedGaitController : public rclcpp::Node
{
public:
    QuadrupedGaitController() : Node("quadruped_gait_controller"), swing_(1)
    {
        // Initialize variables
        req_vel_ = {0.0, 0.0};
        FL_current_xyz_st_ = {0.0, 0.0, 0.0};
        RL_current_xyz_st_ = {0.0, 0.0, 0.0};
        FR_current_xyz_st_ = {0.0, 0.0, 0.0};
        RR_current_xyz_st_ = {0.0, 0.0, 0.0};
        
        // Initialize joint positions message
        jnt_set_st_.data.resize(12, 0.0);
        
        RCLCPP_INFO(this->get_logger(), "Starting the gait Controller....");
        
        // Create publisher for joint commands
        jnt_st_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "joint_commands", 10);
        
        // Create subscribers
        req_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&QuadrupedGaitController::req_vel_callback, this, std::placeholders::_1));
        
        crnt_jnt_st_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&QuadrupedGaitController::joint_current_act_state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Topic subscription and publication successful");
        
        // One time run functions
        Robot_angular_mtn_angles_ = Robot_angular_motion_endpoints(L1, body_length, body_width, SL_);
        RCLCPP_INFO(this->get_logger(), "Twist angle estimation successful");
        
        // Create timer for main control loop (50 Hz)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&QuadrupedGaitController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Gait controller initialized. Use Ctrl-C to quit");
    }

private:
    // Member variables
    std::vector<double> req_vel_;
    std::vector<double> FL_current_xyz_st_;
    std::vector<double> RL_current_xyz_st_;
    std::vector<double> FR_current_xyz_st_;
    std::vector<double> RR_current_xyz_st_;
    
    std_msgs::msg::Float64MultiArray jnt_set_st_;
    
    double SL_ = 0.2;     // step length
    double SH_ = 0.03;    // step height
    double Height_ = 0.33; // standing height
    
    int swing_;
    std::vector<std::vector<double>> Robot_angular_mtn_angles_;
    
    // ROS2 publishers, subscribers, and timer
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jnt_st_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr req_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr crnt_jnt_st_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Callback functions
    void req_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        req_vel_[0] = msg->linear.x;
        req_vel_[1] = msg->angular.z;
        
        // Changes step length depending on velocity to keep robot stable
        SL_ = std::min(0.2, std::sqrt(std::pow(req_vel_[0], 2) + 
                                      std::pow(req_vel_[1], 2)) * 0.1);
    }
    
    void joint_current_act_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 12) {
            RCLCPP_WARN(this->get_logger(), "Received incomplete joint state");
            return;
        }
        
        std::vector<double> FL_current_jnt_st = {0, 0, 0};
        std::vector<double> RL_current_jnt_st = {0, 0, 0};
        std::vector<double> FR_current_jnt_st = {0, 0, 0};
        std::vector<double> RR_current_jnt_st = {0, 0, 0};
        
        // Joint mapping - adapt according to your robot's joint order
        // Assuming joint order: [front_right_rolling, front_right_pitching, front_right_knee,
        //                        front_left_rolling, front_left_pitching, front_left_knee,
        //                        back_right_rolling, back_right_pitching, back_right_knee,
        //                        back_left_rolling, back_left_pitching, back_left_knee]
        
        // Front Left
        FL_current_jnt_st[0] = msg->position[3];  // rolling
        FL_current_jnt_st[1] = msg->position[4];  // pitching
        FL_current_jnt_st[2] = msg->position[5];  // knee
        
        // Front Right
        FR_current_jnt_st[0] = msg->position[0];
        FR_current_jnt_st[1] = msg->position[1];
        FR_current_jnt_st[2] = msg->position[2];
        
        // Back Left
        RL_current_jnt_st[0] = msg->position[9];
        RL_current_jnt_st[1] = msg->position[10];
        RL_current_jnt_st[2] = msg->position[11];
        
        // Back Right
        RR_current_jnt_st[0] = msg->position[6];
        RR_current_jnt_st[1] = msg->position[7];
        RR_current_jnt_st[2] = msg->position[8];
        
        // Convert joint states to xyz positions using FK
        FL_current_xyz_st_ = Front_Left_Leg_FK(FL_current_jnt_st);
        RL_current_xyz_st_ = Back_Left_Leg_FK(RL_current_jnt_st);
        FR_current_xyz_st_ = Front_Right_Leg_FK(FR_current_jnt_st);
        RR_current_xyz_st_ = Back_Right_Leg_FK(RR_current_jnt_st);
    }
    
    // Matrix multiplication helper
    std::vector<std::vector<double>> Multiply(
        const std::vector<std::vector<double>>& a,
        const std::vector<std::vector<double>>& b)
    {
        const int n = a.size();
        const int m = a[0].size();
        const int p = b[0].size();
        std::vector<std::vector<double>> c(n, std::vector<double>(p, 0));
        
        for (int j = 0; j < p; ++j) {
            for (int k = 0; k < m; ++k) {
                for (int i = 0; i < n; ++i) {
                    c[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        return c;
    }
    
    // Trajectory execution function
    void Trajectory_exec(std::vector<std::vector<double>> Trajectory_end_points, int swing)
    {
        auto start_time = this->now();
        rclcpp::Rate loop_rate(500);
        
        if (swing) {
            // FL and RR swing phase (diagonal gait)
            auto FL_1 = FL_current_xyz_st_;
            auto FL_3 = Trajectory_end_points[0];
            std::vector<double> FL_2 = {
                ((FL_1[0] + FL_3[0]) / 2) - SH_,
                (FL_1[1] + FL_3[1]) / 2,
                (FL_1[2] + FL_3[2]) / 2
            };
            
            std::vector<std::vector<double>> FL_x = {
                {FL_1[0], FL_2[0], FL_3[0]},
                {FL_1[1], FL_2[1], FL_3[1]},
                {FL_1[2], FL_2[2], FL_3[2]}
            };
            auto FL_XB = Multiply(FL_x, blending_matrix);
            
            auto RR_1 = RR_current_xyz_st_;
            auto RR_3 = Trajectory_end_points[3];
            std::vector<double> RR_2 = {
                ((RR_1[0] + RR_3[0]) / 2) - SH_,
                (RR_1[1] + RR_3[1]) / 2,
                (RR_1[2] + RR_3[2]) / 2
            };
            
            std::vector<std::vector<double>> RR_x = {
                {RR_1[0], RR_2[0], RR_3[0]},
                {RR_1[1], RR_2[1], RR_3[1]},
                {RR_1[2], RR_2[2], RR_3[2]}
            };
            auto RR_XB = Multiply(RR_x, blending_matrix);
            
            auto FR_1 = FR_current_xyz_st_;
            auto FR_2 = Trajectory_end_points[1];
            
            auto RL_1 = RL_current_xyz_st_;
            auto RL_2 = Trajectory_end_points[2];
            
            while (((this->now() - start_time).seconds() < T) && rclcpp::ok()) {
                double u = (this->now() - start_time).seconds() / T;
                
                std::vector<double> FL_req_pos(3), FR_req_pos(3), RL_req_pos(3), RR_req_pos(3);
                std::vector<std::vector<double>> U_mat = {{1}, {u}, {std::pow(u, 2)}};
                
                // Front Left (swing)
                auto FL_XBU = Multiply(FL_XB, U_mat);
                FL_req_pos = {FL_XBU[0][0], FL_XBU[1][0], FL_XBU[2][0]};
                auto FL_req_jnt = Front_Left_Leg_IK(FL_req_pos);
                jnt_set_st_.data[3] = FL_req_jnt[0];
                jnt_set_st_.data[4] = FL_req_jnt[1];
                jnt_set_st_.data[5] = FL_req_jnt[2];
                
                // Rear Right (swing)
                auto RR_XBU = Multiply(RR_XB, U_mat);
                RR_req_pos = {RR_XBU[0][0], RR_XBU[1][0], RR_XBU[2][0]};
                auto RR_req_jnt = Back_Right_Leg_IK(RR_req_pos);
                jnt_set_st_.data[6] = RR_req_jnt[0];
                jnt_set_st_.data[7] = RR_req_jnt[1];
                jnt_set_st_.data[8] = RR_req_jnt[2];
                
                // Front Right (stance - linear interpolation)
                for (int i = 0; i < 3; ++i)
                    FR_req_pos[i] = FR_1[i] * (1 - u) + FR_2[i] * u;
                auto FR_req_jnt = Front_Right_Leg_IK(FR_req_pos);
                jnt_set_st_.data[0] = FR_req_jnt[0];
                jnt_set_st_.data[1] = FR_req_jnt[1];
                jnt_set_st_.data[2] = FR_req_jnt[2];
                
                // Rear Left (stance - linear interpolation)
                for (int i = 0; i < 3; ++i)
                    RL_req_pos[i] = RL_1[i] * (1 - u) + RL_2[i] * u;
                auto RL_req_jnt = Back_Left_Leg_IK(RL_req_pos);
                jnt_set_st_.data[9] = RL_req_jnt[0];
                jnt_set_st_.data[10] = RL_req_jnt[1];
                jnt_set_st_.data[11] = RL_req_jnt[2];
                
                jnt_st_pub_->publish(jnt_set_st_);
                loop_rate.sleep();
            }
        } else {
            // FR and RL swing phase
            auto FR_1 = FR_current_xyz_st_;
            auto FR_3 = Trajectory_end_points[1];
            std::vector<double> FR_2 = {
                ((FR_1[0] + FR_3[0]) / 2) - SH_,
                (FR_1[1] + FR_3[1]) / 2,
                (FR_1[2] + FR_3[2]) / 2
            };
            
            std::vector<std::vector<double>> FR_x = {
                {FR_1[0], FR_2[0], FR_3[0]},
                {FR_1[1], FR_2[1], FR_3[1]},
                {FR_1[2], FR_2[2], FR_3[2]}
            };
            auto FR_XB = Multiply(FR_x, blending_matrix);
            
            auto RL_1 = RL_current_xyz_st_;
            auto RL_3 = Trajectory_end_points[2];
            std::vector<double> RL_2 = {
                ((RL_1[0] + RL_3[0]) / 2) - SH_,
                (RL_1[1] + RL_3[1]) / 2,
                (RL_1[2] + RL_3[2]) / 2
            };
            
            std::vector<std::vector<double>> RL_x = {
                {RL_1[0], RL_2[0], RL_3[0]},
                {RL_1[1], RL_2[1], RL_3[1]},
                {RL_1[2], RL_2[2], RL_3[2]}
            };
            auto RL_XB = Multiply(RL_x, blending_matrix);
            
            auto FL_1 = FL_current_xyz_st_;
            auto FL_2 = Trajectory_end_points[0];
            
            auto RR_1 = RR_current_xyz_st_;
            auto RR_2 = Trajectory_end_points[3];
            
            while (((this->now() - start_time).seconds() < T) && rclcpp::ok()) {
                double u = (this->now() - start_time).seconds() / T;
                
                std::vector<double> FL_req_pos(3), FR_req_pos(3), RL_req_pos(3), RR_req_pos(3);
                std::vector<std::vector<double>> U_mat = {{1}, {u}, {std::pow(u, 2)}};
                
                // Front Right (swing)
                auto FR_XBU = Multiply(FR_XB, U_mat);
                FR_req_pos = {FR_XBU[0][0], FR_XBU[1][0], FR_XBU[2][0]};
                auto FR_req_jnt = Front_Right_Leg_IK(FR_req_pos);
                jnt_set_st_.data[0] = FR_req_jnt[0];
                jnt_set_st_.data[1] = FR_req_jnt[1];
                jnt_set_st_.data[2] = FR_req_jnt[2];
                
                // Rear Left (swing)
                auto RL_XBU = Multiply(RL_XB, U_mat);
                RL_req_pos = {RL_XBU[0][0], RL_XBU[1][0], RL_XBU[2][0]};
                auto RL_req_jnt = Back_Left_Leg_IK(RL_req_pos);
                jnt_set_st_.data[9] = RL_req_jnt[0];
                jnt_set_st_.data[10] = RL_req_jnt[1];
                jnt_set_st_.data[11] = RL_req_jnt[2];
                
                // Front Left (stance)
                for (int i = 0; i < 3; ++i)
                    FL_req_pos[i] = FL_1[i] * (1 - u) + FL_2[i] * u;
                auto FL_req_jnt = Front_Left_Leg_IK(FL_req_pos);
                jnt_set_st_.data[3] = FL_req_jnt[0];
                jnt_set_st_.data[4] = FL_req_jnt[1];
                jnt_set_st_.data[5] = FL_req_jnt[2];
                
                // Rear Right (stance)
                for (int i = 0; i < 3; ++i)
                    RR_req_pos[i] = RR_1[i] * (1 - u) + RR_2[i] * u;
                auto RR_req_jnt = Back_Right_Leg_IK(RR_req_pos);
                jnt_set_st_.data[6] = RR_req_jnt[0];
                jnt_set_st_.data[7] = RR_req_jnt[1];
                jnt_set_st_.data[8] = RR_req_jnt[2];
                
                jnt_st_pub_->publish(jnt_set_st_);
                loop_rate.sleep();
            }
        }
    }
    
    // Main control loop
    void control_loop()
    {
        std::vector<std::vector<double>> Trajectory_end_points = 
            Robot_end_points(req_vel_, Robot_angular_mtn_angles_, swing_, SL_, Height_);
        
        Trajectory_exec(Trajectory_end_points, swing_);
        swing_ = 1 - swing_;
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
