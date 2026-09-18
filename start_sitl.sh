#!/bin/bash
# ============================================================
# Three-Drone R-Drawing SITL Launch Script
# PX4 v1.16 + Gazebo Harmonic + ROS 2 Jazzy
# ============================================================
# Usage:  ./start_sitl.sh          (start everything)
#         ./start_sitl.sh stop     (kill everything)
#         ./start_sitl.sh ros      (launch ROS 2 node only)
#
# Spawn positions (Gazebo ENU: x=East, y=North):
#   Drone 1  (draw_straight_line)  →  E=0, N=0    (bottom of vertical stroke)
#   Drone 2  (draw_diagonal)       →  E=0, N=7.5  (midpoint — diagonal start)
#   Drone 3  (draw_curve)          →  E=0, N=15   (top — arc start)
# ============================================================

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="$HOME/custom_flightmode"
LOG_DIR="$WS_DIR/sitl_logs"

# Spawn positions: "East,North" in Gazebo ENU world frame
DRONE1_POSE="0,0,0,0,0,0"
DRONE2_POSE="0,7.5,0,0,0,0"
DRONE3_POSE="0,15,0,0,0,0"

RED='\033[0;31m'; GREEN='\033[0;32m'
YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

print_header() {
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  Three-Drone R-Drawing SITL  (PX4 v1.16 + Jazzy)   ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

kill_all() {
    echo -e "${YELLOW}Stopping all PX4/DDS/ROS processes...${NC}"
    pkill -f "px4 -i"         2>/dev/null || true
    pkill -f "MicroXRCEAgent"  2>/dev/null || true
    pkill -f "draw_r"          2>/dev/null || true
    pkill -f "gz sim"          2>/dev/null || true
    pkill -f "ruby.*gz"        2>/dev/null || true
    sleep 2
    echo -e "${GREEN}Done.${NC}"
}

launch_ros() {
    echo -e "${GREEN}Launching ROS 2 swarm nodes...${NC}"
    source /opt/ros/humble/setup.bash
    source "$WS_DIR/install/setup.bash"
    
    # Launch nodes in background
    ROS_DOMAIN_ID=0 ROS_NAMESPACE=/px4_1 ros2 run px4_swarm_drawer swarm_mode_node 1 &
    ROS_DOMAIN_ID=0 ROS_NAMESPACE=/px4_2 ros2 run px4_swarm_drawer swarm_mode_node 2 &
    ROS_DOMAIN_ID=0 ROS_NAMESPACE=/px4_3 ros2 run px4_swarm_drawer swarm_mode_node 3 &
    
    # Wait for all background processes
    wait
    exit 0
}

if [ "$1" == "stop" ]; then kill_all; exit 0; fi
if [ "$1" == "ros"  ]; then launch_ros; fi

print_header
kill_all
mkdir -p "$LOG_DIR"

# ── Prerequisite checks ───────────────────────────────────────
if [ ! -d "$PX4_DIR/build/px4_sitl_default" ]; then
    echo -e "${RED}ERROR: PX4 not built. Run:${NC}"
    echo "  cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo -e "${RED}ERROR: ROS 2 workspace not built. Run:${NC}"
    echo "  cd $WS_DIR && source /opt/ros/jazzy/setup.bash && colcon build"
    exit 1
fi

cd "$PX4_DIR"

# ── PX4 Instance 1 — Drone 1 (draw_straight_line) ───────────
echo -e "${GREEN}[1/4] Starting PX4 instance 1 (draw_straight_line)...${NC}"
PX4_GZ_WORLD=baylands \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="$DRONE1_POSE" \
PX4_UXRCE_DDS_NS=px4_1 \
    ./build/px4_sitl_default/bin/px4 -i 1 \
    > "$LOG_DIR/px4_d1.log" 2>&1 &
PID_D1=$!
echo "  PID=$PID_D1  Log: $LOG_DIR/px4_d1.log"
echo -e "${YELLOW}  Waiting 20s for Gazebo + Drone 1 to initialize...${NC}"
sleep 20

# ── PX4 Instance 2 — Drone 2 (draw_diagonal) ────────────────
echo -e "${GREEN}[2/4] Starting PX4 instance 2 (draw_diagonal)...${NC}"
PX4_GZ_WORLD=baylands \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="$DRONE2_POSE" \
PX4_UXRCE_DDS_NS=px4_2 \
    ./build/px4_sitl_default/bin/px4 -i 2 \
    > "$LOG_DIR/px4_d2.log" 2>&1 &
PID_D2=$!
echo "  PID=$PID_D2  Log: $LOG_DIR/px4_d2.log"
sleep 10

# ── PX4 Instance 3 — Drone 3 (draw_curve) ───────────────────
echo -e "${GREEN}[3/4] Starting PX4 instance 3 (draw_curve)...${NC}"
PX4_GZ_WORLD=baylands \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="$DRONE3_POSE" \
PX4_UXRCE_DDS_NS=px4_3 \
    ./build/px4_sitl_default/bin/px4 -i 3 \
    > "$LOG_DIR/px4_d3.log" 2>&1 &
PID_D3=$!
echo "  PID=$PID_D3  Log: $LOG_DIR/px4_d3.log"
sleep 10

# ── Micro XRCE-DDS Agent (single, serves all 3 instances) ───
# PX4 v1.16 SITL instances auto-connect to ports 8888, 8889, 8890.
# The agent listens on 8888 and PX4 uses RTPS_LOCALHOST_ONLY=0
# to bridge all three. Alternatively run with -p 0 for auto.
echo -e "${GREEN}[4/4] Starting Micro XRCE-DDS Agent...${NC}"
MicroXRCEAgent udp4 -p 8888 \
    > "$LOG_DIR/dds_agent.log" 2>&1 &
PID_DDS=$!
echo "  PID=$PID_DDS  Log: $LOG_DIR/dds_agent.log"
sleep 3

# ── Summary ──────────────────────────────────────────────────
echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════════════╗"
echo -e "║  SITL environment ready                              ║"
echo -e "╠══════════════════════════════════════════════════════╣"
printf "║  Drone1 (draw_straight_line)  PID=%-6s  N= 0.0 m  ║\n" "$PID_D1"
printf "║  Drone2 (draw_diagonal)       PID=%-6s  N= 7.5 m  ║\n" "$PID_D2"
printf "║  Drone3 (draw_curve)          PID=%-6s  N=15.0 m  ║\n" "$PID_D3"
printf "║  DDS Agent                    PID=%-6s  port 8888  ║\n" "$PID_DDS"
echo -e "╚══════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}  Logs : $LOG_DIR/${NC}"
echo -e "${YELLOW}  Stop : ./start_sitl.sh stop${NC}"
echo ""
echo -e "${GREEN}  Step 1 — Open QGroundControl and connect all 3 vehicles.${NC}"
echo -e "${GREEN}  Step 2 — Launch the ROS 2 mission node:${NC}"
echo ""
echo -e "    ./start_sitl.sh ros"
echo ""
echo -e "  Or manually:"
echo -e "    source /opt/ros/jazzy/setup.bash"
echo -e "    source $WS_DIR/install/setup.bash"
echo -e "    ros2 run draw_flight_mode draw_r"
echo ""
echo -e "${CYAN}  QGC mode verification:${NC}"
echo -e "    Select Drone 1 → Flight Mode list should show 'draw_straight_line'"
echo -e "    Select Drone 2 → Flight Mode list should show 'draw_diagonal'"
echo -e "    Select Drone 3 → Flight Mode list should show 'draw_curve'"
