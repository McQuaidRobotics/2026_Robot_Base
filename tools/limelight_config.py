"""Back up and restore Limelight configs over the Limelight REST API (port 5807).

  python tools/limelight_config.py pull                      # all robot cams -> limelight-configs/
  python tools/limelight_config.py pull limelight-intake
  python tools/limelight_config.py push limelight-intake     # restore saved config to same cam
  python tools/limelight_config.py push 10.31.73.15 --from limelight-intake   # clone onto a replacement cam

Saved per camera: pipeline_0..9.json (every GUI pipeline setting, including the
robot-space camera pose from the 3D tab), cal_file.json (custom calibration), and
hwreport.json / status.json (read-only, for reference).
push uses /upload-pipeline, which writes to the Limelight's disk, so it survives
power cycles (unlike the NetworkTables *_set values). Each pushed pipeline is
read back and compared to confirm it saved.
"""

import argparse
import json
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

# Names from SubsystemConstants.LimeLightVision
CAMS = ["limelight", "limelight-intake", "limelight-back", "limelight-left"]
PIPELINES = range(10)
TIMEOUT = 5


def url(host, path):
    if "." not in host and ":" not in host:
        host += ".local"
    return f"http://{host}:5807{path}"


def request(host, path, method="GET", body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url(host, path), data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        text = r.read().decode()
    return json.loads(text) if text.strip() else None


def try_get(host, path):
    try:
        return request(host, path)
    except (urllib.error.URLError, ValueError, TimeoutError) as e:
        print(f"  skip {path}: {e}")
        return None


def folder_name(host):
    return re.sub(r"\.local$", "", host).replace(":", "_")


def pull(host, root):
    out = root / folder_name(host)
    print(f"pull {host} -> {out}")
    request(host, "/status")  # fail fast if the camera isn't reachable
    out.mkdir(parents=True, exist_ok=True)
    files = {f"pipeline_{i}.json": f"/pipeline-atindex?index={i}" for i in PIPELINES}
    files |= {"cal_file.json": "/cal-file", "hwreport.json": "/hwreport", "status.json": "/status"}
    for name, path in files.items():
        data = try_get(host, path)
        if data is not None:
            (out / name).write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
            print(f"  saved {name}")


def push(host, src):
    print(f"push {src} -> {host}")
    if not src.is_dir():
        sys.exit(f"no saved config at {src}")
    ok = True
    for i in PIPELINES:
        f = src / f"pipeline_{i}.json"
        if not f.exists():
            continue
        pipe = json.loads(f.read_text())
        request(host, f"/upload-pipeline?index={i}", "POST", pipe)
        if request(host, f"/pipeline-atindex?index={i}") == pipe:
            print(f"  pipeline {i}: saved + verified")
        else:
            ok = False
            print(f"  pipeline {i}: UPLOADED BUT READBACK DIFFERS - check the web UI")
    cal = src / "cal_file.json"
    if cal.exists():
        request(host, "/cal-file", "POST", json.loads(cal.read_text()))
        print("  calibration: saved")
    request(host, "/reload-pipeline", "POST")
    return ok


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=["pull", "push"])
    p.add_argument("hosts", nargs="*", default=CAMS, help="hostnames or IPs (default: all robot cams)")
    p.add_argument("--dir", type=Path, default=Path(__file__).resolve().parent.parent / "limelight-configs")
    p.add_argument("--from", dest="src", help="push: saved folder to use instead of the host's own")
    a = p.parse_args(argv)
    if a.src and len(a.hosts) != 1:
        p.error("--from needs exactly one host")

    failed = []
    for host in a.hosts:
        try:
            if a.action == "pull":
                pull(host, a.dir)
            elif not push(host, a.dir / (a.src or folder_name(host))):
                failed.append(host)
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            print(f"  FAILED {host}: {e}")
            failed.append(host)
    if failed:
        sys.exit(f"failed: {', '.join(failed)}")


if __name__ == "__main__":
    main()
