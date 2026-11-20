"""MCP server: ask 'why isn't my robot moving?' and get a diagnosis. Read-only, never publishes."""
from mcp.server.fastmcp import FastMCP

from . import core

mcp = FastMCP("ros2-doctor")


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
