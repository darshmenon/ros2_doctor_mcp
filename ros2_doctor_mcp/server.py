"""MCP server: ask 'why isn't my robot moving?' and get a diagnosis. Read-only, never publishes."""
from mcp.server.fastmcp import FastMCP

from . import core

mcp = FastMCP("ros2-doctor")


@mcp.tool()
def diagnose_robot(ns: str = "", seconds: float = 8.0, domain_id: int | None = None) -> dict:
    """Start here. Looks at the ROS graph, detects which stacks are present (Nav2, a ros2_control or MoveIt
    arm, SLAM), runs every doctor that applies in parallel and returns one merged verdict, errors first, each
    finding tagged with its source. Good for mobile manipulators. Read-only."""
    return core.diagnose_all(ns=ns, seconds=seconds, domain_id=domain_id)


@mcp.tool()
def diagnose_nav2(ns: str = "", seconds: float = 8.0, domain_id: int | None = None) -> dict:
    """Watch a running Nav2 stack and name why the robot is not moving (stale TF, dead odom/scan,
    inactive lifecycle nodes, no cmd_vel, flapping goals ...). Read-only. Returns ok and findings with hints."""
    return core.run("nav2", ns=ns, seconds=seconds, domain_id=domain_id)


@mcp.tool()
def nav2_ready(ns: str = "", seconds: float = 10.0, domain_id: int | None = None) -> dict:
    """Pre-flight check for Nav2: ok is true only if every server is active and the streams are alive."""
    return core.run("nav2", ns=ns, seconds=seconds, ready=True, domain_id=domain_id)


@mcp.tool()
def diagnose_arm(ns: str = "", moveit: bool = False, seconds: float = 8.0, domain_id: int | None = None) -> dict:
    """Watch a ros2_control / MoveIt 2 arm and name why it is not moving (inactive controller, silent
    joint_states, joint at a limit, stalled trajectory, missing move_group ...). Read-only."""
    return core.run("arm", ns=ns, moveit=moveit, seconds=seconds, domain_id=domain_id)


@mcp.tool()
def arm_ready(ns: str = "", moveit: bool = False, seconds: float = 10.0, domain_id: int | None = None) -> dict:
    """Pre-flight check for a ros2_control arm: ok only if it can move now."""
    return core.run("arm", ns=ns, moveit=moveit, seconds=seconds, ready=True, domain_id=domain_id)


@mcp.tool()
def diagnose_slam(ns: str = "", seconds: float = 8.0, domain_id: int | None = None) -> dict:
    """Watch a running SLAM (slam_toolbox, Cartographer, RTAB-Map, ORB-SLAM3) and report map stalls, lost
    tracking, odometry drift and missing loop closures. Read-only. Uses slam_doctor."""
    return core.run("slam", ns=ns, seconds=seconds, domain_id=domain_id)


@mcp.tool()
def analyze_force(path: str, cols: str | None = None) -> dict:
    """Offline verdict on a recorded contact-force run (peg-in-hole, press, push; CSV): jammed or not, inserted
    or not, peak and steady force, time in contact. Needs no robot. cols maps CSV columns, e.g. t=time,f_axial=Fz."""
    return core.analyze_force(path, cols=cols)


@mcp.tool()
def check_reach(x: float, y: float, z: float, arm: str = "ur3", tool_length: float = 0.0) -> dict:
    """Can a UR arm (ur3 or ur5e) reach a tool-down pose at x, y, z metres in the base frame? Returns reachable,
    the IK error, manipulability and joint-limit margin. Offline, no robot needed (MuJoCo IK via reachlens)."""
    return core.check_reach(x, y, z, arm=arm, tool_length=tool_length)


@mcp.tool()
def score_map(map_yaml: str, truth_yaml: str, align: bool = False) -> dict:
    """Score a saved occupancy map (map_saver .yaml) against a ground-truth map: coverage, wall recall and
    precision, false-free area. align=true searches for the best translation first. Offline (nav_explore_eval)."""
    return core.score_map(map_yaml, truth_yaml, align=align)


@mcp.tool()
def ros_graph(domain_id: int | None = None) -> dict:
    """List the nodes and topics visible on a ROS domain (read-only). Use it first when a doctor reports
    that nothing is visible, to find the right domain or namespace."""
    return core.graph(domain_id=domain_id)


@mcp.tool()
def analyze_trajectory(path: str, urdf: str | None = None, rate: float | None = None) -> dict:
    """Offline quality check of a recorded joint trajectory (CSV, JSON or rosbag2) with trajlens: jerk spikes,
    limit violations, slow profiles, gaps. Needs no running robot. urdf adds joint limits."""
    return core.analyze_trajectory(path, urdf=urdf, rate=rate)


def main():
    mcp.run()


if __name__ == "__main__":
    main()
