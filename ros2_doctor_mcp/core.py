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
