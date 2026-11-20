# Changelog

## Unreleased
- New `ros_graph` (nodes and topics on a domain) and `analyze_trajectory` (offline trajlens check). `analyze_trajectory` was run through MCP on the bundled UR3 jerky example; `ros_graph` through MCP on an empty domain.

## 0.1.0
- First version: `diagnose_nav2`, `nav2_ready`, `diagnose_arm`, `arm_ready`, wrapping nav2_doctor and armdoc (read-only), with one normalised result shape and per-call `domain_id`.
- Tested end to end through the MCP stdio protocol against a live rosnav Gazebo sim whose Nav2 never came up (no `base_link` to `odom` transform): `diagnose_nav2` returned `TF_NONE`, `ODOM_NONE` and `SCAN_NONE` with hints. The arm tools are covered by unit tests with a stubbed subprocess only; not yet run live through MCP.
