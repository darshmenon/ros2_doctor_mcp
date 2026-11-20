# ros2-doctor-mcp

[![CI](https://github.com/darshmenon/ros2_doctor_mcp/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/darshmenon/ros2_doctor_mcp/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

An MCP server that answers **"why isn't my robot moving?"** for ROS 2. Most ROS MCP servers *control* a robot (publish, call services, send goals). This one only *diagnoses*: it is read-only, never publishes, and never changes anything, so it is safe to point an agent at a running robot.

It wraps two existing read-only doctors and returns one normalised result, `{ok, findings: [{level, code, msg, hint}]}`:

| Tool | Wraps | Answers |
|---|---|---|
| `diagnose_nav2` | [nav2_doctor](https://github.com/darshmenon/nav2_doctor) | stale TF, dead odom or scan, inactive Nav2 servers, no `cmd_vel`, flapping goals, no progress |
| `nav2_ready` | `nav2_doctor --ready` | pre-flight: can this stack navigate right now? |
| `diagnose_arm` | [armdoc](https://github.com/darshmenon/armdoc) | inactive controller, silent `/joint_states`, joint at a limit, stalled trajectory, missing `move_group` |
| `arm_ready` | `armdoc --ready` | pre-flight for a ros2_control arm |
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

## Safety

- The wrapped doctors only subscribe to topics and query lifecycle or controller state.
- The server runs them as subprocesses with a hard timeout and a clamped watch time. There is no tool that publishes, calls a mutating service, or sends a goal.
- Pairs well with a control-oriented ROS MCP server: let that one move the robot, and use this one to find out why it did not.

## Tests

```bash
pip install ".[test]" && pytest -q
```

The unit tests stub the subprocess, so they need neither ROS nor a robot. The tool path was also run end to end through the MCP protocol against a live Nav2 Gazebo sim (see the changelog); the arm tools have only been unit-tested through MCP so far.

## Roadmap

- forcetrace, reachlens and nav_explore_eval as extra tools
- merged node logs tied to a finding
- multi-robot: one call per namespace
- a PyPI release

License: Apache-2.0.
