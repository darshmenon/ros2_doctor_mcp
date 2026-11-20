"""Run the read-only robot doctors as subprocesses and normalise their JSON into one shape."""
import json
import os
import shlex
import shutil
import subprocess

DEFAULT_CMDS = {
    "nav2": ("ROS2_DOCTOR_NAV2_CMD", "nav2_doctor", ["ros2", "run", "nav2_doctor", "nav2_doctor"]),
    "arm": ("ROS2_DOCTOR_ARM_CMD", "armdoc", ["ros2", "run", "armdoc", "armdoc"]),
}
MAX_SECONDS = 120.0


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
