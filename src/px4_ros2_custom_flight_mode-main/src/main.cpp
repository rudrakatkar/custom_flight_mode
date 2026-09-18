/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Three-drone swarm that draws the letter R at 50 m AGL.
 *
 *  Drone 1  →  /px4_1  →  QGC mode: "draw_straight_line"  (vertical stroke)
 *  Drone 2  →  /px4_2  →  QGC mode: "draw_diagonal"       (diagonal leg)
 *  Drone 3  →  /px4_3  →  QGC mode: "draw_curve"          (bump arc)
 *
 * Spawn positions (Gazebo ENU: x=East, y=North):
 *  Drone 1: (0, 0)    — bottom of vertical stroke
 *  Drone 2: (0, 7.5)  — midpoint of vertical stroke (diagonal start)
 *  Drone 3: (0, 15)   — top of vertical stroke (arc start)
 *
 * Collision avoidance: CBF-QP safety filter (repulsion.hpp).
 * Synchronisation: three-way ready barrier on /draw_r_ready_dN topics.
 ****************************************************************************/

#include <string>

#include "rclcpp/rclcpp.hpp"

#include <mode.hpp>          // DrawLeftEdgeMode + DrawModeExecutor  (Drone 1)
#include <mode_curve.hpp>    // DrawCurveMode                        (Drone 3)
#include <mode_diagonal.hpp> // DrawDiagonalMode                     (Drone 2)

// Helper: create NodeOptions with namespace remapping
static rclcpp::NodeOptions makeOptions(const std::string & ns)
{
    return rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__ns:=/" + ns});
}

// ── Drone 1 — draw_straight_line ─────────────────────────────────────────────
using DrawLeftEdgeExecutor = DrawModeExecutor<DrawLeftEdgeMode>;

class Drone1Node : public rclcpp::Node
{
public:
    Drone1Node(const std::string & name, bool dbg,
        const rclcpp::NodeOptions & opts,
        const std::string & ns2, const std::string & ns3)
    : rclcpp::Node(name, opts)
    {
        if (dbg) {
            auto ret = rcutils_logging_set_logger_level(
                get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            (void)ret;
        }
        _mode     = std::make_unique<DrawLeftEdgeMode>(*this, ns2, ns3);
        _executor = std::make_unique<DrawLeftEdgeExecutor>(
            *_mode, *this, /*require_trigger=*/false);  // Drone 1 starts immediately
        if (!_executor->doRegister()) {
            throw std::runtime_error("Drone1: registration failed");
        }
    }

private:
    std::unique_ptr<DrawLeftEdgeMode>     _mode;
    std::unique_ptr<DrawLeftEdgeExecutor> _executor;
};

// ── Drone 2 — draw_diagonal ───────────────────────────────────────────────────
using DrawDiagonalExecutor = DrawModeExecutor<DrawDiagonalMode>;

class Drone2Node : public rclcpp::Node
{
public:
    Drone2Node(const std::string & name, bool dbg,
        const rclcpp::NodeOptions & opts,
        const std::string & ns1, const std::string & ns3)
    : rclcpp::Node(name, opts)
    {
        if (dbg) {
            auto ret = rcutils_logging_set_logger_level(
                get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            (void)ret;
        }
        _mode     = std::make_unique<DrawDiagonalMode>(*this, ns1, ns3);
        _executor = std::make_unique<DrawDiagonalExecutor>(
            *_mode, *this, /*require_trigger=*/true);   // waits for /draw_r_trigger
        if (!_executor->doRegister()) {
            throw std::runtime_error("Drone2: registration failed");
        }
    }

private:
    std::unique_ptr<DrawDiagonalMode>     _mode;
    std::unique_ptr<DrawDiagonalExecutor> _executor;
};

// ── Drone 3 — draw_curve ──────────────────────────────────────────────────────
using DrawCurveExecutor = DrawModeExecutor<DrawCurveMode>;

class Drone3Node : public rclcpp::Node
{
public:
    Drone3Node(const std::string & name, bool dbg,
        const rclcpp::NodeOptions & opts,
        const std::string & ns1, const std::string & ns2)
    : rclcpp::Node(name, opts)
    {
        if (dbg) {
            auto ret = rcutils_logging_set_logger_level(
                get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            (void)ret;
        }
        _mode     = std::make_unique<DrawCurveMode>(*this, ns1, ns2);
        _executor = std::make_unique<DrawCurveExecutor>(
            *_mode, *this, /*require_trigger=*/true);   // waits for /draw_r_trigger
        if (!_executor->doRegister()) {
            throw std::runtime_error("Drone3: registration failed");
        }
    }

private:
    std::unique_ptr<DrawCurveMode>     _mode;
    std::unique_ptr<DrawCurveExecutor> _executor;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Parameter node for namespace configuration
    auto param_node = std::make_shared<rclcpp::Node>("draw_r_params");
    param_node->declare_parameter<std::string>("drone1_ns", "px4_1");
    param_node->declare_parameter<std::string>("drone2_ns", "px4_2");
    param_node->declare_parameter<std::string>("drone3_ns", "px4_3");

    const std::string ns1 = param_node->get_parameter("drone1_ns").as_string();
    const std::string ns2 = param_node->get_parameter("drone2_ns").as_string();
    const std::string ns3 = param_node->get_parameter("drone3_ns").as_string();

    RCLCPP_INFO(param_node->get_logger(),
        "\n╔══════════════════════════════════════════════════╗"
        "\n║  Three-Drone Letter-R Swarm  (CBF-QP Safety)    ║"
        "\n╠══════════════════════════════════════════════════╣"
        "\n║  Drone1 [draw_straight_line]  → /%s             ║"
        "\n║  Drone2 [draw_diagonal]       → /%s             ║"
        "\n║  Drone3 [draw_curve]          → /%s             ║"
        "\n║  Altitude: 50 m AGL  |  d_safe: 4 m            ║"
        "\n╚══════════════════════════════════════════════════╝",
        ns1.c_str(), ns2.c_str(), ns3.c_str());

    static constexpr bool kDebug = true;

    auto drone1 = std::make_shared<Drone1Node>(
        "draw_mode_d1", kDebug, makeOptions(ns1), ns2, ns3);
    auto drone2 = std::make_shared<Drone2Node>(
        "draw_mode_d2", kDebug, makeOptions(ns2), ns1, ns3);
    auto drone3 = std::make_shared<Drone3Node>(
        "draw_mode_d3", kDebug, makeOptions(ns3), ns1, ns2);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(param_node);
    executor.add_node(drone1);
    executor.add_node(drone2);
    executor.add_node(drone3);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
