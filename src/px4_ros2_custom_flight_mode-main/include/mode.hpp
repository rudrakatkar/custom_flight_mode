#pragma once
/**
 * mode.hpp — Drone 1: "draw_straight_line"
 *
 * Draws the vertical stroke of the letter R.
 * Spawn: Gazebo (E=0, N=0).  Stroke: local NED +X for kR_height_m metres.
 *
 * Three-way synchronisation:
 *   1. onActivate() publishes /draw_r_trigger → arms Drones 2 & 3
 *   2. All three drones hover at their start positions and publish
 *      /draw_r_ready_dN until they have received the other two.
 *   3. Drawing begins simultaneously.
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
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include "repulsion.hpp"   // cbfSafetyFilter

using namespace std::chrono_literals;  // NOLINT

// ── Shared geometry ──────────────────────────────────────────────────────────
static constexpr float kR_height_m   = 15.0f;  // vertical stroke (m, NED-X / North)
static constexpr float kR_width_m    =  7.5f;  // horizontal extent (m, NED-Y / East)
static constexpr float kR_speed_ms   =  2.0f;  // drawing speed (m/s)
static constexpr float kR_altitude_m = 50.0f;  // takeoff altitude AGL (m)
static constexpr float kR_lookahead  =  2.5f;  // CBF lookahead horizon (s)

// ── QGC-visible mode name ────────────────────────────────────────────────────
static const std::string kNameStraightLine = "draw_straight_line";

// ============================================================
//  DrawLeftEdgeMode — Drone 1
// ============================================================
class DrawLeftEdgeMode : public px4_ros2::ModeBase
{
public:
    explicit DrawLeftEdgeMode(rclcpp::Node & node,
        const std::string & ns2, const std::string & ns3)
    : ModeBase(node, Settings{kNameStraightLine}),
      _node{node}
    {
        _goto = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        _odom = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        // Trigger arms Drones 2 & 3
        _trigger_pub = _node.create_publisher<std_msgs::msg::Empty>("/draw_r_trigger", 10);

        // Three-way ready barrier
        _ready_pub = _node.create_publisher<std_msgs::msg::Bool>("/draw_r_ready_d1", 10);
        _sub_ready2 = _node.create_subscription<std_msgs::msg::Bool>(
            "/draw_r_ready_d2", 10,
            [this](const std_msgs::msg::Bool::SharedPtr) { _d2_ready = true; });
        _sub_ready3 = _node.create_subscription<std_msgs::msg::Bool>(
            "/draw_r_ready_d3", 10,
            [this](const std_msgs::msg::Bool::SharedPtr) { _d3_ready = true; });

        // Neighbour positions for CBF
        _sub_pos2 = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + ns2 + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                _pos2 = {m->x, m->y, m->z}; _has2 = true; });
        _sub_pos3 = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + ns3 + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
                _pos3 = {m->x, m->y, m->z}; _has3 = true; });
    }

    ~DrawLeftEdgeMode() override = default;

    void onActivate() override
    {
        _state     = State::ClimbToAltitude;
        _start     = _odom->positionNed();
        _d2_ready  = false;
        _d3_ready  = false;

        // Fire trigger so Drones 2 & 3 start arming immediately
        _trigger_pub->publish(std_msgs::msg::Empty{});
        RCLCPP_INFO(_node.get_logger(),
            "[D1/draw_straight_line] Activated at (%.1f, %.1f, %.1f) — climbing to 50m",
            _start.x(), _start.y(), _start.z());
    }

    void onDeactivate() override {}

    void updateSetpoint(float /*dt_s*/) override
    {
        const Eigen::Vector3f pos = _odom->positionNed();

        std::vector<Eigen::Vector3f> nbrs;
        if (_has2) { nbrs.push_back(_pos2); }
        if (_has3) { nbrs.push_back(_pos3); }

        switch (_state) {
            // ── 1. Climb to 50 m AGL (NED Z = -50) ────────────────────────
            case State::ClimbToAltitude: {
                // Keep re-publishing trigger so late drones always receive it
                _trigger_pub->publish(std_msgs::msg::Empty{});

                Eigen::Vector3f climb_wp = pos;
                climb_wp.z() = -kR_altitude_m;  // NED: negative = up
                _goto->update(climb_wp);

                if (std::abs(pos.z() - (-kR_altitude_m)) < 1.0f &&
                    _odom->velocityNed().norm() < 0.5f) {
                    // Capture start position at correct altitude
                    _start = _odom->positionNed();
                    _state = State::WaitingForAllReady;
                    RCLCPP_INFO(_node.get_logger(),
                        "[D1] Reached 50m (z=%.1f) — syncing with swarm...", pos.z());
                }
                break;
            }

            // ── 2. Three-way sync barrier ─────────────────────────────────
            case State::WaitingForAllReady: {
                _goto->update(_start);                              // hover at 50m
                // Keep re-publishing trigger + ready
                _trigger_pub->publish(std_msgs::msg::Empty{});
                std_msgs::msg::Bool msg; msg.data = true;
                _ready_pub->publish(msg);
                if (_d2_ready && _d3_ready) {
                    _state = State::DrawVertical;
                    RCLCPP_INFO(_node.get_logger(),
                        "[D1] All drones ready — drawing vertical stroke!");
                }
                break;
            }

            // ── 3. Draw vertical stroke (NED +X = North) ──────────────────
            case State::DrawVertical: {
                const Eigen::Vector3f target =
                    _start + Eigen::Vector3f{kR_height_m, 0.f, 0.f};

                // Nominal velocity toward target
                const Eigen::Vector3f to_target = target - pos;
                const float dist = to_target.norm();
                Eigen::Vector3f v_nom = Eigen::Vector3f::Zero();
                if (dist > 0.1f) { v_nom = to_target.normalized() * kR_speed_ms; }

                // CBF safety filter
                const Eigen::Vector3f v_safe = cbfSafetyFilter(pos, v_nom, nbrs);

                // Lookahead waypoint — keeps goto controller chasing the safe direction
                Eigen::Vector3f wp = pos + v_safe * kR_lookahead;
                wp.x() = std::min(wp.x(), target.x());   // don't overshoot North
                wp.y() = _start.y();                      // stay on East = 0
                wp.z() = _start.z();                      // hold altitude

                _goto->update(wp, 0.f /*heading*/, kR_speed_ms);

                if (posReached(target)) {
                    RCLCPP_INFO(_node.get_logger(), "[D1] Vertical stroke complete");
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

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _trigger_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  _ready_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_ready2, _sub_ready3;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _sub_pos2, _sub_pos3;

    Eigen::Vector3f _pos2, _pos3;
    bool _has2{false}, _has3{false};
    bool _d2_ready{false}, _d3_ready{false};
    Eigen::Vector3f _start;

    enum class State { ClimbToAltitude, WaitingForAllReady, DrawVertical, Done } _state{State::ClimbToAltitude};

    bool posReached(const Eigen::Vector3f & tgt) const
    {
        return (_odom->positionNed() - tgt).norm() < 0.5f &&
               _odom->velocityNed().norm() < 0.3f;
    }
};

// ============================================================
//  DrawModeExecutor<ModeT> — shared executor template
//  Trigger → Arm → TakeOff(50 m) → CustomMode → RTL → Disarm
// ============================================================
template<typename ModeT>
class DrawModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
    explicit DrawModeExecutor(ModeT & mode, rclcpp::Node & node,
        bool require_trigger = true)
    : ModeExecutorBase(
        {px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately},
        mode),
      _node(node),
      _require_trigger(require_trigger)
    {
        if (_require_trigger) {
            _trigger_sub = _node.create_subscription<std_msgs::msg::Empty>(
                "/draw_r_trigger", 10,
                [this](const std_msgs::msg::Empty::SharedPtr) {
                    if (_state == State::WaitingForTrigger) {
                        RCLCPP_INFO(_node.get_logger(), "Trigger received — arming...");
                        runState(State::Arming, px4_ros2::Result::Success);
                    }
                });
        }
    }

    enum class State {
        Reset, WaitingForTrigger, Arming,
        TakingOff, DrawMode, RTL, WaitUntilDisarmed
    };

    void onActivate() override
    {
        if (_require_trigger) {
            _state = State::WaitingForTrigger;
            RCLCPP_INFO(_node.get_logger(), "Executor active — waiting for trigger...");
        } else {
            runState(State::Arming, px4_ros2::Result::Success);
        }
    }

    void onDeactivate(DeactivateReason /*reason*/) override {}

    void runState(State s, px4_ros2::Result prev)
    {
        if (prev != px4_ros2::Result::Success) {
            RCLCPP_ERROR(_node.get_logger(), "State %i failed: %s",
                static_cast<int>(s), resultToString(prev));
            return;
        }
        _state = s;
        switch (s) {
            case State::Reset:
            case State::WaitingForTrigger: break;

            case State::Arming:
                arm([this](px4_ros2::Result r) { runState(State::TakingOff, r); });
                break;

            case State::TakingOff:
                takeoff([this](px4_ros2::Result r) { runState(State::DrawMode, r); },
                    kR_altitude_m);
                break;

            case State::DrawMode:
                scheduleMode(ownedMode().id(),
                    [this](px4_ros2::Result r) { runState(State::RTL, r); });
                break;

            case State::RTL:
                rtl([this](px4_ros2::Result r) { runState(State::WaitUntilDisarmed, r); });
                break;

            case State::WaitUntilDisarmed:
                waitUntilDisarmed([this](px4_ros2::Result r) {
                    RCLCPP_INFO(_node.get_logger(),
                        "Mission complete (%s)", resultToString(r));
                });
                break;
        }
    }

private:
    rclcpp::Node & _node;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _trigger_sub;
    State _state{State::Reset};
    bool  _require_trigger;
};