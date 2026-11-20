# ros2-doctor-mcp

[![CI](https://github.com/darshmenon/ros2_doctor_mcp/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/darshmenon/ros2_doctor_mcp/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

An MCP server that answers **"why isn't my robot moving?"** for ROS 2. Most ROS MCP servers *control* a robot (publish, call services, send goals). This one only *diagnoses*: it is read-only, never publishes, and never changes anything, so it is safe to point an agent at a running robot.

It wraps existing read-only doctors and returns one normalised result, `{ok, findings: [{level, code, msg, hint}]}`:

| Tool | Wraps | Answers |
|---|---|---|
| `diagnose_nav2` | [nav2_doctor](https://github.com/darshmenon/nav2_doctor) | stale TF, dead odom or scan, inactive Nav2 servers, no `cmd_vel`, flapping goals, no progress |
| `nav2_ready` | `nav2_doctor --ready` | pre-flight: can this stack navigate right now? |
| `diagnose_arm` | [armdoc](https://github.com/darshmenon/armdoc) | inactive controller, silent `/joint_states`, joint at a limit, stalled trajectory, missing `move_group` |
| `arm_ready` | `armdoc --ready` | pre-flight for a ros2_control arm |
| `diagnose_slam` | [slam_doctor](https://github.com/darshmenon/slam_doctor) | SLAM health: map stalls, lost tracking, odometry drift, no loop closures (slam_toolbox, Cartographer, RTAB-Map, ORB-SLAM3) |
| `analyze_force` | [forcetrace](https://github.com/darshmenon/forcetrace) | offline: did a peg-in-hole or press run jam or insert, peak and steady force, time in contact |
| `check_reach` | [reachlens](https://github.com/darshmenon/reachlens) | offline: can a UR3 or UR5e reach this tool-down pose? IK error, manipulability, joint-limit margin |
| `score_map` | [nav_explore_eval](https://github.com/darshmenon/nav_explore_eval) | offline: coverage, wall recall and precision of a saved map against ground truth |
| `ros_graph` | `ros2 node list`, `ros2 topic list -t` | what is visible on a domain; start here when a doctor says nothing is visible |
| `analyze_trajectory` | [trajlens](https://github.com/darshmenon/trajlens) | offline: jerk spikes, limit violations, slow profiles in a recorded trajectory (CSV, JSON, rosbag2); no robot needed |

Every tool takes `ns` (namespace), `seconds` (watch time, clamped to 1 to 120) and `domain_id` (sets `ROS_DOMAIN_ID` for that call only). Each finding carries a `hint` with a suggested next step.

## Install

```bash
# a ROS 2 install with the doctors you want
cd ~/ros2_ws/src && git clone https://github.com/darshmenon/nav2_doctor.git && git clone https://github.com/darshmenon/armdoc.git
cd .. && colcon build --packages-select nav2_doctor armdoc

pip install git+https://github.com/darshmenon/ros2_doctor_mcp.git
```

Run the server from a shell where ROS 2 and your workspace are sourced.

## Use with Claude Code

```bash
claude mcp add ros2-doctor -- ros2-doctor-mcp
```

or in `.mcp.json`:

```json
{"mcpServers": {"ros2-doctor": {"command": "ros2-doctor-mcp"}}}
```

Then ask: *"Nav2 is running on domain 77 but the robot won't move. Why?"*

## Configuration

The doctors are found on `PATH` (`nav2_doctor`, `armdoc`), else through `ros2 run`. Override either with an environment variable:

```bash
ROS2_DOCTOR_NAV2_CMD="python3 -m nav2_doctor.doctor"
ROS2_DOCTOR_ARM_CMD="ros2 run armdoc armdoc"
```

## Not included

`nav2_fault_inject` and `armchaos` deliberately break a running robot, so they stay out of a read-only server. `ctrlbench` is a long benchmark, not a quick question.

## Safety

- The wrapped doctors only subscribe to topics and query lifecycle or controller state.
- The server runs them as subprocesses with a hard timeout and a clamped watch time. There is no tool that publishes, calls a mutating service, or sends a goal.
- Pairs well with a control-oriented ROS MCP server: let that one move the robot, and use this one to find out why it did not.

## Tests

```bash
pip install ".[test]" && pytest -q
```

The unit tests stub the subprocess, so they need neither ROS nor a robot. The Nav2 and arm tools were also run end to end through the MCP protocol against live Gazebo sims (see the changelog).

## Roadmap

- fault injectors (nav2_fault_inject, armchaos) are deliberately not exposed: this server stays read-only
- merged node logs tied to a finding
- multi-robot: one call per namespace
- a PyPI release

License: Apache-2.0.
