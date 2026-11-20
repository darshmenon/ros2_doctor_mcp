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


def test_slam_command_and_flags(monkeypatch):
    monkeypatch.setenv("ROS2_DOCTOR_SLAM_CMD", "slam_doctor")
    a = core.build_args("slam", ns="robot1", seconds=5)
    assert a[:3] == ["slam_doctor", "--ns", "robot1"] and "--once" in a and "--moveit" not in a


def test_analyze_force_flags_a_jam_and_normalises(tmp_path):
    assert "no such file" in core.analyze_force(str(tmp_path / "x.csv"))["error"]
    f = tmp_path / "r.csv"
    f.write_text("t,Fz\n0,0\n")

    def runner(args, **k):
        json.dump({"samples": 9, "jammed": True, "jam_t": 3.5, "inserted": False, "peak_force_n": 22.0, "extra": 1},
                  open(args[args.index("--json") + 1], "w"))
        return SimpleNamespace(stdout="", stderr="", returncode=0)
    r = core.analyze_force(str(f), cols="t=t,f_axial=Fz", runner=runner)
    assert r["ok"] is False and {x["code"] for x in r["findings"]} == {"JAMMED", "NOT_INSERTED"}
    assert r["metrics"]["peak_force_n"] == 22.0 and "extra" not in r["metrics"]


def test_errors_come_first():
    doc = [{"level": "WARN", "code": "A", "msg": ""}, {"level": "ERROR", "code": "B", "msg": ""},
           {"level": "WARN", "code": "C", "msg": ""}]
    assert [f["code"] for f in core.parse("arm", json.dumps(doc))["findings"]] == ["B", "A", "C"]


def test_check_reach_parses_reachable_and_unreachable():
    ok = "REACHABLE  pos_err=0.03 mm  ori_err=0.00 deg\nmanipulability=0.0148  cond=13.1  limit_margin=0.41\nq = [-0.04, -1.5, 1.8, -1.9, -1.57, -1.6]\n"
    r = core.check_reach(0.3, 0.1, 0.25, runner=fake(ok))
    assert r["ok"] is True and r["cond"] == 13.1 and len(r["joint_angles_rad"]) == 6
    bad = core.check_reach(1.5, 0, 0.2, runner=fake("UNREACHABLE  pos_err=764.27 mm  ori_err=180.00 deg\n"))
    assert bad["ok"] is False and bad["pos_err_mm"] == 764.27 and "joint_angles_rad" not in bad
    assert core.check_reach(0, 0, 0, runner=fake("garbage"))["ok"] is None


def test_score_map_reads_json_and_checks_files(tmp_path):
    assert "no such file" in core.score_map(str(tmp_path / "a.yaml"), str(tmp_path / "b.yaml"))["error"]
    a = tmp_path / "a.yaml"
    a.write_text("x")
    r = core.score_map(str(a), str(a), align=True, runner=fake('log\n{"coverage_pct": 91.5, "wall_f1_pct": 80.0}'))
    assert r["ok"] is True and r["metrics"]["coverage_pct"] == 91.5


def test_detect_picks_doctors_from_node_names():
    d = core.detect(["/controller_server", "/bt_navigator", "/controller_manager", "/move_group", "/slam_toolbox"])
    assert d == {"nav2": True, "arm": True, "slam": True, "moveit": True}
    assert core.detect(["/foo"]) == {"nav2": False, "arm": False, "slam": False, "moveit": False}


def make_runner(nodes, nav2_findings=(), arm_findings=()):
    def runner(args, **k):
        if args[:2] == ["ros2", "node"]:
            return SimpleNamespace(stdout="\n".join(nodes), stderr="", returncode=0)
        if args[:2] == ["ros2", "topic"]:
            return SimpleNamespace(stdout="/tf [x]", stderr="", returncode=0)
        cmd = " ".join(args)
        doc = list(arm_findings) if "armdoc" in cmd else list(nav2_findings)
        return SimpleNamespace(stdout=json.dumps(doc), stderr="", returncode=0)
    return runner


def test_diagnose_all_merges_and_tags_sources(monkeypatch):
    monkeypatch.setenv("ROS2_DOCTOR_NAV2_CMD", "nav2_doctor")
    monkeypatch.setenv("ROS2_DOCTOR_ARM_CMD", "armdoc")
    r = core.diagnose_all(runner=make_runner(
        ["/controller_server", "/controller_manager", "/move_group"],
        nav2_findings=[{"level": "WARN", "code": "AMCL_UNCERTAIN", "msg": "a"}],
        arm_findings=[{"level": "ERROR", "code": "CTRL_INACTIVE", "msg": "b"}]))
    assert r["ok"] is False and r["stacks"] == ["nav2", "arm", "moveit"]
    assert [(f["source"], f["code"]) for f in r["findings"]] == [("arm", "CTRL_INACTIVE"), ("nav2", "AMCL_UNCERTAIN")]
    assert r["summary"].startswith("1 error(s), 1 warning(s)")


def test_diagnose_all_with_nothing_found_explains_why():
    r = core.diagnose_all(runner=make_runner(["/talker"]))
    assert r["ok"] is None and "domain_id" in r["error"] and r["stacks"] == []
