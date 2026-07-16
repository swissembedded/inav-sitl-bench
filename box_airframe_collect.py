# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Box-side collector for the airframe video batch: gathers the per-flight
work dirs, runs the SAME automated gate as the local runner
(_run_airframe_videos.verify - no hand review), renders only PASS flights
(parallel), and writes the summary. Usage inside the render container:

    python3 box_airframe_collect.py /workall /renders
"""
import concurrent.futures
import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _run_airframe_videos import MANEUVERS, MODELS, TITLE, verify


def render(tag, man, renders):
    subprocess.run([sys.executable, "animate_jsbsim.py", tag, "--title",
                    f"{tag.split('_')[0]} under the real FC: "
                    f"{TITLE.get(man, man)}"], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    shutil.copy(f"docs/videos/jsbsim_{tag}.mp4", f"{renders}/videos/")


def main():
    workall, renders = sys.argv[1], sys.argv[2]
    os.makedirs(f"{renders}/videos", exist_ok=True)
    results = []
    jobs = []
    for model in MODELS:
        for man in MANEUVERS:
            tag = f"{model}_{man}"
            src = f"{workall}/af-{model}-{man}/jsbsim_log_{man}.csv"
            if not os.path.exists(src):
                results.append((tag, False, ["no log (flight died)"]))
                continue
            shutil.copy(src, f"jsbsim_log_{tag}.csv")
            par = f"{workall}/af-{model}-{man}/jsbsim_params_{man}.txt"
            if os.path.exists(par):
                shutil.copy(par, f"jsbsim_params_{tag}.txt")
            try:
                ok, fails = verify(tag, man)
            except Exception as e:
                ok, fails = False, [f"verify crashed: {e}"]
            results.append((tag, ok, fails))
            if ok:
                jobs.append((tag, man))
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as ex:
        futs = {ex.submit(render, tag, man, renders): tag for tag, man in jobs}
        for f in concurrent.futures.as_completed(futs):
            tag = futs[f]
            try:
                f.result()
            except Exception as e:
                for i, (t, ok, fails) in enumerate(results):
                    if t == tag:
                        results[i] = (t, False, [f"render failed: {e}"])
    lines = ["=== SUMMARY ==="]
    for tag, ok, fails in sorted(results):
        lines.append(f"  {tag:32s} "
                     f"{'PASS' if ok else 'FAIL: ' + '; '.join(fails)}")
    npass = sum(1 for _, ok, _ in results if ok)
    lines.append(f"{npass}/{len(results)} PASS (only PASS rendered)")
    out = "\n".join(lines)
    print(out, flush=True)
    with open(f"{renders}/SUMMARY.txt", "w") as fh:
        fh.write(out + "\n")
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
