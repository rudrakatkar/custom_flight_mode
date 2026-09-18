#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include "trajectory.hpp"
#include "cbf_filter.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <array>
#include <string>

// Base mode handling shared state, CBF, and sync
class DrawBaseMode : public px4_ros2::ModeBase {
public:
    DrawBaseMode(rclcpp::Node& node, const std::string& mode_name, int drone_id, const std::string& peer1_ns, const std::string& peer2_ns) 
        : ModeBase(node, mode_name), drone_id_(drone_id), cbf_(2.5, 1.0) 
    {
        trajectory_setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        
        peer1_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/" + peer1_ns + "/fmu/out/vehicle_odometry", rclcpp::QoS(1).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                peer1_pos_ = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
                peer1_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
                has_peer1_ = true;
            });
            
        peer2_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/" + peer2_ns + "/fmu/out/vehicle_odometry", rclcpp::QoS(1).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                peer2_pos_ = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
                peer2_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
                has_peer2_ = true;
            });
    }

    void onActivate() override {
        start_time_ = node().get_clock()->now();
    }

    void onDeactivate() override {}

    void updateSetpoint(float /* dt_s */) override {
        double t = (node().get_clock()->now() - start_time_).seconds();
        
        Eigen::Vector3d nom_pos, nom_vel;
        calculateNominalTrajectory(t, nom_pos, nom_vel);

        std::vector<Eigen::Vector3d> peer_p, peer_u;
        if (has_peer1_) {
            peer_p.push_back(peer1_pos_);
            peer_u.push_back(peer1_vel_);
        }
        if (has_peer2_) {
            peer_p.push_back(peer2_pos_);
            peer_u.push_back(peer2_vel_);
        }
        
        // Simplified: assume tracking is tight, use nominal position for CBF filter base
        Eigen::Vector3d p_i = nom_pos; 

        // Apply CBF
        Eigen::Vector3d u_safe = cbf_.filter(p_i, nom_vel, peer_p, peer_u);

        px4_ros2::TrajectorySetpoint setpoint;
        setpoint.withPosition(Eigen::Vector3f(nom_pos.x(), nom_pos.y(), nom_pos.z()));
        setpoint.withVelocity(Eigen::Vector3f(u_safe.x(), u_safe.y(), u_safe.z()));
        trajectory_setpoint_->update(setpoint);
    }

protected:
    virtual void calculateNominalTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) = 0;
    
    int drone_id_;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
    rclcpp::Time start_time_;
    CBFSafetyFilter cbf_;
    const double T_draw = 15.0; // Draw duration
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr peer1_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr peer2_sub_;
    
    Eigen::Vector3d peer1_pos_{0,0,0}, peer1_vel_{0,0,0};
    Eigen::Vector3d peer2_pos_{0,0,0}, peer2_vel_{0,0,0};
    bool has_peer1_ = false, has_peer2_ = false;
};

// Drone 1
class DrawStraightLineMode : public DrawBaseMode {
public:
    DrawStraightLineMode(rclcpp::Node& node) : DrawBaseMode(node, "draw_straight_line", 1, "px4_2", "px4_3") {}
protected:
    void calculateNominalTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) override {
        Eigen::Vector3d start(-10, 0, -50);
        Eigen::Vector3d end(10, 0, -50);
        TrajectoryGenerator::getStraightLine(t, T_draw, start, end, pos, vel);
    }
};

// Drone 2
class DrawCurveMode : public DrawBaseMode {
public:
    DrawCurveMode(rclcpp::Node& node) : DrawBaseMode(node, "draw_curve", 2, "px4_1", "px4_3") {}
protected:
    void calculateNominalTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) override {
        Eigen::Vector3d center(5, 0, -50);
        TrajectoryGenerator::getSemiCircle(t, T_draw, center, 5.0, -M_PI/2, M_PI/2, pos, vel);
    }
};

// Drone 3
class DrawDiagonalMode : public DrawBaseMode {
public:
    DrawDiagonalMode(rclcpp::Node& node) : DrawBaseMode(node, "draw_diagonal", 3, "px4_1", "px4_2") {}
protected:
    void calculateNominalTrajectory(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) override {
        Eigen::Vector3d start(0, 0, -50);
        Eigen::Vector3d end(-10, 10, -50);
        TrajectoryGenerator::getStraightLine(t, T_draw, start, end, pos, vel);
    }
};
