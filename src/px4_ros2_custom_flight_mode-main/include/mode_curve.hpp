#pragma once
/**
 * mode_curve.hpp — Drone 3: "draw_curve"
 *
 * Draws the curved bump of the letter R using a sequence of Bézier-derived
 * arc waypoints.  The arc is a semicircle of radius r = kR_height_m/4.
 *
 * Spawn: Gazebo (E=0, N=15).  After takeoff the drone is at the TOP of the
 * vertical stroke in world coordinates.
 *
 * Arc geometry in the drone's LOCAL NED frame (origin = its takeoff position):
 *
 *   Centre  = ( -r,  0 )   with r = kR_height_m/4 = 3.75 m
 *   θ = 0   → ( 0,   0 )   ← start (top of vertical stroke)
 *   θ = π/2 → ( -r,  r )   ← rightmost point of the bump
 *   θ = π   → ( -2r, 0 )   ← end   (midpoint of vertical stroke)
 *
 *   p(θ) = centre + r·(cos θ, sin θ)
 *        = ( -r + r·cosθ,  r·sinθ )   θ ∈ [0, π]
 *
 * 9 discrete waypoints (including start & end) trace this arc.
 * The MulticopterGotoSetpointType navigates sequentially between them, with
 * the CBF safety filter applied at every control tick.
 */

#include <Eigen/Core>
#include <vector>
#include <string>
#include <array>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include "repulsion.hpp"
#include "mode.hpp"   // kR_height_m, kR_width_m, kR_speed_ms, kR_altitude_m, kR_lookahead

using namespace std::chrono_literals;  // NOLINT

static const std::string kNameCurve = "draw_curve";

// Number of arc waypoints (including start and end)
static constexpr int kArcWaypoints = 9;

class DrawCurveMode : public px4_ros2::ModeBase
{
public:
    explicit DrawCurveMode(rclcpp::Node & node,
        const std::string & ns1, const std::string & ns2)
    : ModeBase(node, Settings{kNameCurve}), _node{node}
    {
        _goto = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        _odom = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        // Pre-compute the arc waypoints in local NED offsets from start position.
        // r = half of R bump height = kR_height_m / 4
        const float r = kR_height_m / 4.0f;
        for (int i = 0; i < kArcWaypoints; ++i) {
            const float theta = static_cast<float>(i) * static_cast<float>(M_PI) /
                                static_cast<float>(kArcWaypoints - 1);
            // Local NED offsets:  X = North component (negative = South)
            //                     Y = East component
            _arc_offsets[i] = {
                -r + r * std::cos(theta),   // NED-X (North/South)
                 r * std::sin(theta),        // NED-Y (East)
                 0.f
            };
        }

        // Three-way ready barrier
        _ready_pub = _node.create_publisher<std_msgs::msg::Bool>("/draw_r_ready_d3", 10);
        _sub_ready1 = _node.create_subscription<std_msgs::msg::Bool>(
            "/draw_r_ready_d1", 10,
            [this](const std_msgs::msg::Bool::SharedPtr) { _d1_ready = true; });
        _sub_ready2 = _node.create_subscription<std_msgs::msg::Bool>(
            "/draw_r_ready_d2", 10,
            [this](const std_msgs::msg::Bool::SharedPtr) { _d2_ready = true; });

        // Neighbour positions for CBF
        _sub_pos1 = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + ns1 + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                _pos1 = {m->x, m->y, m->z}; _has1 = true; });
        _sub_pos2 = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + ns2 + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                _pos2 = {m->x, m->y, m->z}; _has2 = true; });
    }

    ~DrawCurveMode() override = default;

    void onActivate() override
    {
        _state     = State::ClimbToAltitude;
        _start     = _odom->positionNed();
        _wp_idx    = 0;
        _d1_ready  = false; _d2_ready = false;
        RCLCPP_INFO(_node.get_logger(),
            "[D3/draw_curve] Activated at (%.1f, %.1f, %.1f) — climbing to 50m",
            _start.x(), _start.y(), _start.z());
    }

    void onDeactivate() override {}

    void updateSetpoint(float /*dt_s*/) override
    {
        const Eigen::Vector3f pos = _odom->positionNed();

        std::vector<Eigen::Vector3f> nbrs;
        if (_has1) { nbrs.push_back(_pos1); }
        if (_has2) { nbrs.push_back(_pos2); }

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
                        "[D3] Reached 50m (z=%.1f) — syncing with swarm...", pos.z());
                }
                break;
            }

            // ── 2. Three-way sync barrier ─────────────────────────────────
            case State::WaitingForAllReady: {
                _goto->update(_start);
                std_msgs::msg::Bool msg; msg.data = true;
                _ready_pub->publish(msg);
                if (_d1_ready && _d2_ready) {
                    _state  = State::DrawArc;
                    _wp_idx = 1;   // 0 is the current position; start from 1
                    RCLCPP_INFO(_node.get_logger(),
                        "[D3] All drones ready — drawing arc (%d waypoints)!", kArcWaypoints);
                }
                break;
            }

            // ── 3. Traverse arc waypoints sequentially ────────────────────
            case State::DrawArc: {
                if (_wp_idx >= kArcWaypoints) {
                    RCLCPP_INFO(_node.get_logger(), "[D3] Arc complete");
                    _state = State::Done;
                    break;
                }

                // Current waypoint in world frame
                const Eigen::Vector3f wp_world = _start + _arc_offsets[_wp_idx];

                // Nominal velocity toward current waypoint
                const Eigen::Vector3f to_wp = wp_world - pos;
                const float dist = to_wp.norm();
                Eigen::Vector3f v_nom = Eigen::Vector3f::Zero();
                if (dist > 0.1f) { v_nom = to_wp.normalized() * kR_speed_ms; }

                // CBF safety filter
                const Eigen::Vector3f v_safe = cbfSafetyFilter(pos, v_nom, nbrs);

                // Lookahead point along safe direction
                Eigen::Vector3f send_wp = pos + v_safe * kR_lookahead;
                send_wp.z() = _start.z();  // hold altitude

                // Heading: tangent to the arc (perpendicular to the radius)
                // At waypoint i, the radius points from centre to waypoint.
                // centre_offset = (-r, 0), waypoint_offset = _arc_offsets[_wp_idx]
                const float r = kR_height_m / 4.0f;
                const Eigen::Vector3f centre_offset{-r, 0.f, 0.f};
                Eigen::Vector3f radius = _arc_offsets[_wp_idx] - centre_offset;
                // Tangent (90° CCW in NED XY plane) = (-radius.y, radius.x)
                Eigen::Vector3f tangent{-radius.y(), radius.x(), 0.f};
                float heading = std::atan2(tangent.y(), tangent.x());

                _goto->update(send_wp, heading, kR_speed_ms);

                // Advance to the next waypoint when this one is reached
                if (posReached(wp_world)) {
                    RCLCPP_INFO(_node.get_logger(),
                        "[D3] Arc waypoint %d/%d reached", _wp_idx + 1, kArcWaypoints);
                    ++_wp_idx;
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_ready1, _sub_ready2;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _sub_pos1, _sub_pos2;

    Eigen::Vector3f _pos1, _pos2;
    bool _has1{false}, _has2{false};
    bool _d1_ready{false}, _d2_ready{false};
    Eigen::Vector3f _start;

    std::array<Eigen::Vector3f, kArcWaypoints> _arc_offsets;
    int _wp_idx{0};

    enum class State { ClimbToAltitude, WaitingForAllReady, DrawArc, Done } _state{State::ClimbToAltitude};

    bool posReached(const Eigen::Vector3f & tgt) const
    {
        return (_odom->positionNed() - tgt).norm() < 0.5f &&
               _odom->velocityNed().norm() < 0.35f;
    }
};
