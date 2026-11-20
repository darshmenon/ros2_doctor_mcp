"""Run the read-only robot doctors as subprocesses and normalise their JSON into one shape."""
import json
import os
import re
import shlex
import shutil
import subprocess

DEFAULT_CMDS = {
    "nav2": ("ROS2_DOCTOR_NAV2_CMD", "nav2_doctor", ["ros2", "run", "nav2_doctor", "nav2_doctor"]),
    "arm": ("ROS2_DOCTOR_ARM_CMD", "armdoc", ["ros2", "run", "armdoc", "armdoc"]),
    "slam": ("ROS2_DOCTOR_SLAM_CMD", "slam_doctor", ["ros2", "run", "slam_doctor", "slam_doctor"]),
}
MAX_SECONDS = 120.0
LEVEL_RANK = {"ERROR": 0, "WARN": 1, "INFO": 2, "OK": 3}


def base_cmd(kind):
    env, exe, fallback = DEFAULT_CMDS[kind]
    if os.environ.get(env):
        return shlex.split(os.environ[env])
    return [exe] if shutil.which(exe) else list(fallback)


def build_args(kind, ns="", seconds=8.0, ready=False, moveit=False):
    seconds = max(1.0, min(float(seconds), MAX_SECONDS))
    args = base_cmd(kind)
    if ns:
        args += ["--ns", ns]
    args += ["--hints", "--json"]
    args += ["--ready", str(seconds)] if ready else ["--once", str(seconds)]
    if moveit and kind == "arm":
        args.append("--moveit")
    return args


def parse(kind, stdout):
    """Last JSON document on stdout -> {'ok': bool, 'findings': [{'level','code','msg','hint'}]}."""
    doc = None
    for line in stdout.splitlines():
        line = line.strip()
        if line[:1] in "{[":
            try:
                doc = json.loads(line)
            except ValueError:
                continue
    if doc is None:
        return None
    findings = doc["findings"] if isinstance(doc, dict) else doc
    out = [{"level": f.get("level"), "code": f.get("code"), "msg": f.get("msg"), "hint": f.get("hint")}
           for f in findings]
    out.sort(key=lambda f: LEVEL_RANK.get(f["level"], 9))   # causes (errors) before noise
    return {"ok": not any(f["level"] == "ERROR" for f in out), "findings": out}


def run(kind, ns="", seconds=8.0, ready=False, moveit=False, domain_id=None, runner=subprocess.run):
    args = build_args(kind, ns, seconds, ready, moveit)
    env = dict(os.environ)
    if domain_id is not None:
        env["ROS_DOMAIN_ID"] = str(int(domain_id))
    try:
        p = runner(args, capture_output=True, text=True, env=env, timeout=float(seconds) + 60)
    except FileNotFoundError:
        return {"ok": None, "error": f"could not start {args[0]!r}: is ROS 2 sourced and the doctor installed?"}
    except subprocess.TimeoutExpired:
        return {"ok": None, "error": "the doctor did not finish in time"}
    res = parse(kind, p.stdout)
    if res is None:
        return {"ok": None, "error": "no JSON from the doctor", "exit_code": p.returncode,
                "stderr": p.stderr[-500:]}
    res["domain_id"] = env.get("ROS_DOMAIN_ID")
    return res


# ---------- read-only graph listing ----------
def _run_text(args, env, runner, timeout=25):
    p = runner(args, capture_output=True, text=True, env=env, timeout=timeout)
    return [ln.strip() for ln in p.stdout.splitlines() if ln.strip()]


def graph(domain_id=None, runner=subprocess.run):
    """Nodes and topics visible on a ROS domain. Use it when a doctor says nothing is visible."""
    env = dict(os.environ)
    if domain_id is not None:
        env["ROS_DOMAIN_ID"] = str(int(domain_id))
    try:
        nodes = _run_text(["ros2", "node", "list"], env, runner)
        topics = _run_text(["ros2", "topic", "list", "-t"], env, runner)
    except FileNotFoundError:
        return {"error": "ros2 not found: source your ROS 2 install first"}
    except subprocess.TimeoutExpired:
        return {"error": "ros2 did not answer in time"}
    return {"domain_id": env.get("ROS_DOMAIN_ID"), "nodes": nodes, "topics": topics,
            "hint": None if nodes else "no nodes visible: check ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY and the RMW"}


# ---------- offline trajectory analysis (trajlens) ----------
def analyze_trajectory(path, urdf=None, rate=None, runner=subprocess.run):
    """Quality findings for a recorded joint trajectory file (CSV, JSON, rosbag2). Offline, reads only."""
    if not os.path.exists(path):
        return {"ok": None, "error": f"no such file: {path}"}
    import tempfile
    base = shlex.split(os.environ["ROS2_DOCTOR_TRAJLENS_CMD"]) if os.environ.get("ROS2_DOCTOR_TRAJLENS_CMD") \
        else (["trajlens"] if shutil.which("trajlens") else ["python3", "-m", "trajlens"])
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "r.json")
        args = base + ["analyze", path, "--json", out]
        if urdf:
            args += ["--urdf", urdf]
        if rate:
            args += ["--rate", str(float(rate))]
        try:
            runner(args, capture_output=True, text=True, timeout=120)
            doc = json.load(open(out))
        except (FileNotFoundError, ValueError, OSError):
            return {"ok": None, "error": "trajlens failed or is not installed (pip install trajlens)"}
        except subprocess.TimeoutExpired:
            return {"ok": None, "error": "trajlens did not finish in time"}
    findings = [{"level": f.get("level"), "code": f.get("code"), "msg": f.get("message"), "hint": f.get("hint")}
                for f in doc.get("findings", [])]
    keep = ("samples", "duration_s", "moving_time_s", "path_length_rad", "max_speed")
    return {"ok": not any(f["level"] == "ERROR" for f in findings), "findings": findings,
            "metrics": {k: doc[k] for k in keep if k in doc}}


# ---------- offline contact-force analysis (forcetrace) ----------
FORCE_KEYS = ("samples", "duration_s", "peak_force_n", "peak_axial_n", "peak_lateral_n", "time_in_contact_s",
              "impulse_ns", "first_contact_t", "inserted", "time_to_insert_s", "jammed", "jam_t",
              "steady_force_n", "overshoot_pct", "settling_time_s")


def analyze_force(path, cols=None, runner=subprocess.run):
    """Contact-force verdict for a recorded peg-in-hole / press / push run (CSV). Offline, reads only."""
    if not os.path.exists(path):
        return {"ok": None, "error": f"no such file: {path}"}
    import tempfile
    base = shlex.split(os.environ["ROS2_DOCTOR_FORCETRACE_CMD"]) if os.environ.get("ROS2_DOCTOR_FORCETRACE_CMD") \
        else (["forcetrace"] if shutil.which("forcetrace") else ["python3", "-m", "forcetrace"])
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "r.json")
        args = base + ["analyze", path, "--json", out] + (["--cols", cols] if cols else [])
        try:
            runner(args, capture_output=True, text=True, timeout=120)
            doc = json.load(open(out))
        except (FileNotFoundError, ValueError, OSError):
            return {"ok": None, "error": "forcetrace failed or is not installed (pip install forcetrace)"}
        except subprocess.TimeoutExpired:
            return {"ok": None, "error": "forcetrace did not finish in time"}
    findings = []
    if doc.get("jammed"):
        findings.append({"level": "ERROR", "code": "JAMMED", "hint": "reduce lateral force or add a search pattern (spiral)",
                         "msg": f"the run jammed at t={doc.get('jam_t')}s"})
    if doc.get("inserted") is False:
        findings.append({"level": "WARN", "code": "NOT_INSERTED", "hint": "check the alignment offset and insertion depth",
                         "msg": "the peg never reached the insertion depth"})
    return {"ok": not doc.get("jammed"), "findings": findings, "metrics": {k: doc[k] for k in FORCE_KEYS if k in doc}}


# ---------- reachability (reachlens) ----------
def _tool(env, exe, module_cmd):
    if os.environ.get(env):
        return shlex.split(os.environ[env])
    return [exe] if shutil.which(exe) else module_cmd


def check_reach(x, y, z, arm="ur3", tool_length=0.0, runner=subprocess.run):
    """Can a UR arm reach a tool-down pose? IK in MuJoCo through reachlens. Offline, no robot needed."""
    args = _tool("ROS2_DOCTOR_REACHLENS_CMD", "reachlens", ["python3", "-m", "reachlens"]) + [
        "check", "--arm", arm, "--tool-length", str(float(tool_length)), "--pose", str(float(x)), str(float(y)), str(float(z))]
    try:
        p = runner(args, capture_output=True, text=True, timeout=120)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return {"ok": None, "error": "reachlens did not run (pip install reachlens, needs MuJoCo)"}
    m = re.search(r"(UN)?REACHABLE\s+pos_err=([\d.]+) mm\s+ori_err=([\d.]+) deg", p.stdout)
    if not m:
        return {"ok": None, "error": "could not read reachlens output", "stderr": p.stderr[-300:]}
    res = {"ok": m.group(1) is None, "reachable": m.group(1) is None, "pos_err_mm": float(m.group(2)),
           "ori_err_deg": float(m.group(3))}
    extra = re.search(r"manipulability=([\d.eE+-]+)\s+cond=([\d.eE+-]+)\s+limit_margin=([\d.eE+-]+)", p.stdout)
    if extra:
        res.update(manipulability=float(extra.group(1)), cond=float(extra.group(2)), limit_margin=float(extra.group(3)))
    q = re.search(r"q = \[([^\]]*)\]", p.stdout)
    if q and res["reachable"]:
        res["joint_angles_rad"] = [float(v) for v in q.group(1).split(",")]
    return res


# ---------- map scoring (nav_explore_eval) ----------
def score_map(map_yaml, truth_yaml, align=False, runner=subprocess.run):
    """Score a saved occupancy map against a ground-truth map: coverage, wall recall/precision, false-free area."""
    for f in (map_yaml, truth_yaml):
        if not os.path.exists(f):
            return {"ok": None, "error": f"no such file: {f}"}
    args = _tool("ROS2_DOCTOR_NAVEVAL_CMD", "nav_explore_eval", ["python3", "-m", "nav_explore_eval.cli"]) + [
        "score", map_yaml, truth_yaml, "--json"] + (["--align"] if align else [])
    try:
        p = runner(args, capture_output=True, text=True, timeout=300)
        doc = json.loads(p.stdout[p.stdout.index("{"):])
    except (FileNotFoundError, ValueError, subprocess.TimeoutExpired):
        return {"ok": None, "error": "nav_explore_eval failed or is not installed"}
    return {"ok": True, "metrics": doc}
