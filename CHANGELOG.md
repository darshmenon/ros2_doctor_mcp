# Changelog

## Unreleased
- New `diagnose_slam` (slam_doctor) and `analyze_force` (forcetrace; `analyze_force` run through MCP on a bundled Gazebo UR3 peg run). Findings are sorted errors first.
- `diagnose_arm` and `arm_ready` run through MCP on a live UR3 Gazebo sim: healthy reads healthy (no false positives); with the arm controller deactivated they return `CTRL_INACTIVE` and `NO_ACTIVE_ARM_CTRL`. That run exposed a noise bug in armdoc, now fixed there.
- New `ros_graph` (nodes and topics on a domain) and `analyze_trajectory` (offline trajlens check). `analyze_trajectory` was run through MCP on the bundled UR3 jerky example; `ros_graph` through MCP on an empty domain.

## 0.1.0
- First version: `diagnose_nav2`, `nav2_ready`, `diagnose_arm`, `arm_ready`, wrapping nav2_doctor and armdoc (read-only), with one normalised result shape and per-call `domain_id`.
- Tested end to end through the MCP stdio protocol against a live rosnav Gazebo sim whose Nav2 never came up (no `base_link` to `odom` transform): `diagnose_nav2` returned `TF_NONE`, `ODOM_NONE` and `SCAN_NONE` with hints. The arm tools are covered by unit tests with a stubbed subprocess only; not yet run live through MCP.
