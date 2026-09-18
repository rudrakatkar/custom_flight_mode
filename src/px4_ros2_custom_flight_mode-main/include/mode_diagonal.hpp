#pragma once
/**
 * mode_diagonal.hpp — Drone 2: "draw_diagonal"
 *
 * Draws the diagonal leg of the letter R.
 * Spawn: Gazebo (E=0, N=7.5).  After takeoff the drone is at the midpoint of
 * the vertical stroke in world coordinates.
 * Leg: local NED (0,0) → (-7.5, +7.5, 0)  i.e. South-East along the leg.
 *
 *  Letter R world frame (NED X=North, Y=East):
 *
 *    (N=15)  ← Drone 3 arc start
 *      │╲
 *      │  ╲  ← arc
 *    (N=7.5) ← Drone 2 diagonal start ← this drone
 *      │  ╲
 *    (N=0)  (E=7.5) ← diagonal end
 */

#include <Eigen/Core>
#include <vector>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include "repulsion.hpp"

using namespace std::chrono_literals;  // NOLINT

static const std::string kNameDiagonal = "draw_diagonal";

class DrawDiagonalMode : public px4_ros2::ModeBase
{
public:
    explicit DrawDiagonalMode(rclcpp::Node & node,
        const std::string & ns1, const std::string & ns3)
    : ModeBase(node, Settings{kNameDiagonal}), _node{node}
    {
        _goto = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        _odom = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        // Three-way ready barrier
        _ready_pub = _node.create_publisher<std_msgs::msg::Bool>("/draw_r_ready_d2", 10);
        _sub_ready1 = _node.create_subscription<std_msgs::msg::Bool>(
            "/draw_r_ready_d1", 10,
            [this](const std_msgs::msg::Bool::SharedPtr) { _d1_ready = true; });
        _sub_ready3 = _node.create_subscription<std_msgs::msg::Bool>(
            "/draw_r_ready_d3", 10,
            [this](const std_msgs::msg::Bool::SharedPtr) { _d3_ready = true; });

        // Neighbour positions for CBF
        _sub_pos1 = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + ns1 + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                _pos1 = {m->x, m->y, m->z}; _has1 = true; });
        _sub_pos3 = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + ns3 + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                _pos3 = {m->x, m->y, m->z}; _has3 = true; });
    }

    ~DrawDiagonalMode() override = default;

    void onActivate() override
    {
        _state    = State::ClimbToAltitude;
        _start    = _odom->positionNed();
        _d1_ready = false; _d3_ready = false;
        RCLCPP_INFO(_node.get_logger(),
            "[D2/draw_diagonal] Activated at (%.1f, %.1f, %.1f) — climbing to 50m",
            _start.x(), _start.y(), _start.z());
    }

    void onDeactivate() override {}

    void updateSetpoint(float /*dt_s*/) override
    {
        const Eigen::Vector3f pos = _odom->positionNed();

        std::vector<Eigen::Vector3f> nbrs;
        if (_has1) { nbrs.push_back(_pos1); }
        if (_has3) { nbrs.push_back(_pos3); }

        switch (_state) {
            // ── 1. Climb to 50 m AGL (NED Z = -50) ────────────────────────
            case State::ClimbToAltitude: {
                Eigen::Vector3f climb_wp = pos;
                climb_wp.z() = -kR_altitude_m;
                _goto->update(climb_wp);

                if (std::abs(pos.z() - (-kR_altitude_m)) < 1.0f &&
                    _odom->velocityNed().norm() < 0.5f) {
                    _start = _odom->positionNed();
                    _state = State::WaitingForAllReady;
                    RCLCPP_INFO(_node.get_logger(),
                        "[D2] Reached 50m (z=%.1f) — syncing with swarm...", pos.z());
                }
                break;
            }

            // ── 2. Three-way sync barrier ─────────────────────────────────
            case State::WaitingForAllReady: {
                _goto->update(_start);
                std_msgs::msg::Bool msg; msg.data = true;
                _ready_pub->publish(msg);
                if (_d1_ready && _d3_ready) {
                    _state = State::DrawDiagonal;
                    RCLCPP_INFO(_node.get_logger(),
                        "[D2] All drones ready — drawing diagonal!");
                }
                break;
            }

            // ── 3. Draw diagonal leg ──────────────────────────────────────
            //  From local (0,0,0) → (-kR_height_m/2, +kR_width_m, 0) in NED
            //  i.e. 7.5 m South and 7.5 m East — the lower-right leg of R.
            case State::DrawDiagonal: {
                const Eigen::Vector3f target =
                    _start + Eigen::Vector3f{-kR_height_m / 2.f, kR_width_m, 0.f};

                const Eigen::Vector3f to_tgt = target - pos;
                const float dist = to_tgt.norm();
                Eigen::Vector3f v_nom = Eigen::Vector3f::Zero();
                if (dist > 0.1f) { v_nom = to_tgt.normalized() * kR_speed_ms; }

                const Eigen::Vector3f v_safe = cbfSafetyFilter(pos, v_nom, nbrs);

                // Lookahead waypoint along safe direction
                Eigen::Vector3f wp = pos + v_safe * kR_lookahead;
                // Clamp so we don't over-shoot the endpoint
                wp.x() = std::max(wp.x(), target.x());
                wp.y() = std::min(wp.y(), target.y());
                wp.z() = _start.z();

                // Heading: face the direction of travel (South-East ≈ NED yaw π/4)
                _goto->update(wp, static_cast<float>(M_PI / 4.0), kR_speed_ms);

                if (posReached(target)) {
                    RCLCPP_INFO(_node.get_logger(), "[D2] Diagonal complete");
                    _state = State::Done;
                }
                break;
            }

            case State::Done:
                completed(px4_ros2::Result::Success);
                break;
        }
    }

private:
    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto;
    std::shared_ptr<px4_ros2::OdometryLocalPosition>       _odom;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  _ready_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_ready1, _sub_ready3;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _sub_pos1, _sub_pos3;

    Eigen::Vector3f _pos1, _pos3;
    bool _has1{false}, _has3{false};
    bool _d1_ready{false}, _d3_ready{false};
    Eigen::Vector3f _start;

    enum class State { ClimbToAltitude, WaitingForAllReady, DrawDiagonal, Done } _state{State::ClimbToAltitude};

    bool posReached(const Eigen::Vector3f & tgt) const
    {
        return (_odom->positionNed() - tgt).norm() < 0.5f &&
               _odom->velocityNed().norm() < 0.3f;
    }
};
