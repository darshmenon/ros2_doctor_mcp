import json
import subprocess
from types import SimpleNamespace

from ros2_doctor_mcp import core


def fake(stdout, code=0):
    return lambda *a, **k: SimpleNamespace(stdout=stdout, stderr="", returncode=code)


def test_nav2_json_is_normalised():
    out = json.dumps({"ok": False, "findings": [{"level": "ERROR", "code": "ODOM_STALE", "msg": "m", "hint": "h"}]})
    r = core.run("nav2", runner=fake("log line\n" + out + "\n"))
    assert r["ok"] is False and r["findings"][0] == {"level": "ERROR", "code": "ODOM_STALE", "msg": "m", "hint": "h"}


def test_arm_list_json_is_normalised_and_ok_when_no_error():
    r = core.run("arm", runner=fake(json.dumps([{"level": "WARN", "code": "NEAR_LIMIT", "msg": "x"}])))
    assert r["ok"] is True and r["findings"][0]["hint"] is None


def test_garbage_output_is_an_error_not_a_crash():
    r = core.run("nav2", runner=fake("not json", code=2))
    assert r["ok"] is None and r["exit_code"] == 2


def test_missing_binary_and_timeout_are_reported():
    def nope(*a, **k):
        raise FileNotFoundError()

    def slow(*a, **k):
        raise subprocess.TimeoutExpired("x", 1)
    assert "could not start" in core.run("arm", runner=nope)["error"]
    assert "in time" in core.run("arm", runner=slow)["error"]


def test_args_clamp_seconds_and_flags(monkeypatch):
    monkeypatch.setenv("ROS2_DOCTOR_ARM_CMD", "armdoc")
    a = core.build_args("arm", ns="/ur3", seconds=9999, moveit=True)
    assert a[:3] == ["armdoc", "--ns", "/ur3"] and "--moveit" in a
    assert a[a.index("--once") + 1] == str(core.MAX_SECONDS)
    assert "--ready" in core.build_args("arm", ready=True)
    assert "--moveit" not in core.build_args("nav2", moveit=True)


def test_domain_id_goes_to_the_environment():
    seen = {}

    def runner(args, **k):
        seen.update(k["env"])
        return SimpleNamespace(stdout="[]", stderr="", returncode=0)
    r = core.run("arm", domain_id=77, runner=runner)
    assert seen["ROS_DOMAIN_ID"] == "77" and r["domain_id"] == "77"


def test_graph_lists_nodes_and_topics_and_hints_when_empty():
    def runner(args, **k):
        out = "/a\n/b\n" if args[1] == "node" else "/tf [tf2_msgs/msg/TFMessage]\n"
        return SimpleNamespace(stdout=out, stderr="", returncode=0)
    g = core.graph(domain_id=5, runner=runner)
    assert g["nodes"] == ["/a", "/b"] and g["topics"] == ["/tf [tf2_msgs/msg/TFMessage]"] and g["hint"] is None
    empty = core.graph(runner=lambda *a, **k: SimpleNamespace(stdout="", stderr="", returncode=0))
    assert "ROS_DOMAIN_ID" in empty["hint"]
    only_defaults = core.graph(runner=lambda a, **k: SimpleNamespace(stdout="/rosout [x]\n" if a[1] == "topic" else "", stderr="", returncode=0))
    assert only_defaults["hint"]


def test_analyze_trajectory_reports_missing_file_and_normalises(tmp_path):
    assert "no such file" in core.analyze_trajectory(str(tmp_path / "nope.csv"))["error"]
    f = tmp_path / "t.csv"
    f.write_text("t,j1\n0,0\n")

    def runner(args, **k):
        out = args[args.index("--json") + 1]
        json.dump({"samples": 3, "duration_s": 1.0, "findings": [
            {"level": "WARN", "code": "JERK_SPIKE", "message": "m", "hint": "h"}]}, open(out, "w"))
        return SimpleNamespace(stdout="", stderr="", returncode=0)
    r = core.analyze_trajectory(str(f), urdf="u.urdf", runner=runner)
    assert r["ok"] is True and r["findings"][0]["msg"] == "m" and r["metrics"] == {"samples": 3, "duration_s": 1.0}
