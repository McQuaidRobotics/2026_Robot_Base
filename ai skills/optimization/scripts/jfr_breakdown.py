"""Break down where one thread spends its time in a JFR recording.

Usage:
  python jfr_breakdown.py <recording.jfr> [--thread main] [--jfr <path to jfr exe>] [--top 60]

On a roboRIO the robot loop thread is "main"; in desktop simulation it is "robot main".
Samples whose top frame is the loop's idle wait (waitForNotifierAlarm / startCompetition)
count as idle, so busy/(busy+idle) approximates how much of each 20 ms loop is used.
Prints self-time (top frame) and inclusive counts; inclusive is what you usually want.
"""
import argparse, collections, os, re, shutil, subprocess, sys

ap = argparse.ArgumentParser()
ap.add_argument("recording")
ap.add_argument("--thread", default="main")
ap.add_argument("--jfr", default=None, help="jfr executable (default: WPILib JDK or PATH)")
ap.add_argument("--top", type=int, default=60)
ap.add_argument("--filter", default=None, help="only show inclusive frames containing this substring")
a = ap.parse_args()

jfr = a.jfr or next((p for p in (
    r"C:\Users\Public\wpilib\2026\jdk\bin\jfr.exe",
    r"C:\Users\Public\wpilib\2025\jdk\bin\jfr.exe",
    os.path.expanduser("~/wpilib/2026/jdk/bin/jfr"),
    os.path.expanduser("~/wpilib/2025/jdk/bin/jfr"),
) if os.path.exists(p)), None) or shutil.which("jfr")
if not jfr:
    sys.exit("jfr executable not found; pass --jfr")

txt = subprocess.run(
    [jfr, "print", "--events", "jdk.ExecutionSample,jdk.NativeMethodSample", "--stack-depth", "80", a.recording],
    capture_output=True, text=True, encoding="utf-8", errors="replace").stdout

IDLE = ("waitForNotifierAlarm", "LoggedRobot.startCompetition", "TimedRobot.startCompetition")
incl, top, threads = collections.Counter(), collections.Counter(), collections.Counter()
busy = idle = 0
for e in re.split(r"\n(?=jdk\.\w+ \{)", txt):
    m = re.search(r'sampledThread = "([^"]*)"', e)
    if not m or "stackTrace = [" not in e:
        continue
    threads[m.group(1)] += 1
    if m.group(1) != a.thread:
        continue
    fr = [re.sub(r"\s+line:.*", "", l.strip())
          for l in e.split("stackTrace = [")[1].split("]")[0].splitlines() if "line:" in l]
    if not fr:
        continue
    if any(k in fr[0] for k in IDLE):
        idle += 1
        continue
    busy += 1
    top[fr[0]] += 1
    for f in set(fr):
        incl[f] += 1

print("threads by sample count:", ", ".join(f"{t}={c}" for t, c in threads.most_common(8)))
if busy + idle == 0:
    sys.exit(f'no samples for thread "{a.thread}" (try --thread "robot main" for simulation)')
print(f"\n[{a.thread}] busy={busy} idle={idle} -> busy {100*busy/(busy+idle):.0f}% of loop time")
print("\n--- self (top frame)")
for f, c in top.most_common(20):
    print(f"{c:5d} {f}")

# GC pauses stop every thread, so a loop near its budget overruns on any long pause.
gc = subprocess.run([jfr, "print", "--events", "jdk.GarbageCollection", a.recording],
                    capture_output=True, text=True, encoding="utf-8", errors="replace").stdout
pauses = []
for m in re.finditer(r"longestPause = ([\d.]+) (ms|s|us)", gc):
    v = float(m.group(1)) * {"ms": 1, "s": 1000, "us": 0.001}[m.group(2)]
    pauses.append(v)
if pauses:
    print(f"\nGC: {len(pauses)} collections, avg pause {sum(pauses)/len(pauses):.1f} ms, "
          f"max {max(pauses):.1f} ms (pauses > ~5 ms on a busy loop cause overruns)")

print("\n--- inclusive")
shown = 0
for f, c in incl.most_common():
    if a.filter and a.filter not in f:
        continue
    print(f"{c:5d} {100*c/busy:5.1f}%  {f}")
    shown += 1
    if shown >= a.top:
        break
