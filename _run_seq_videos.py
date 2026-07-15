import csv, os, shutil, subprocess, sys
CONTAINER = os.environ.get("INAV_SITL_CONTAINER", "inav-sitl")
ROUTINES = ["immelmann", "wargo_vol9_easy_routine", "wargo_addiction_xl",
            "veloxity_3d_demo", "wargo_vol8_immelmann_inverted", "wargo_gyro_knife_pass"]
def run(*a, **kw): subprocess.run(a, check=kw.pop("check", True), **kw)
for name in ROUTINES:
    print(f"=== {name} ===", flush=True)
    if os.path.exists("fcdata/eeprom.bin"): os.remove("fcdata/eeprom.bin")
    run("podman", "restart", CONTAINER); run("python", "-c", "import time; time.sleep(3)")
    run("python", "bench.py", "provision")
    run("podman", "restart", CONTAINER); run("python", "-c", "import time; time.sleep(3)")
    run("python", "figure_script.py", f"examples/{name}.json")
    run("python", "jsbsim_fly.py", "--flip-ele", "--lockstep", "seq")
    # trim the level tail: keep ~8 s after the last non-level/action frame
    rows = list(csv.DictReader(open("jsbsim_log_seq.csv")))
    hdr = rows[0].keys(); last = 0
    for i, r in enumerate(rows):
        if abs(float(r['js_roll'])) > 8 or abs(float(r['js_pitch'])) > 12 or float(r['ias']) < 45:
            last = i
    cut = min(len(rows), last + 8 * 60)
    man = f"seq_{name}"
    with open(f"jsbsim_log_{man}.csv", "w", newline="", encoding="utf-8") as w:
        out = csv.DictWriter(w, fieldnames=hdr); out.writeheader()
        for r in rows[:cut]: out.writerow(r)
    if os.path.exists("jsbsim_params_seq.txt"):
        shutil.copy("jsbsim_params_seq.txt", f"jsbsim_params_{man}.txt")
    run("python", "animate_jsbsim.py", man, "--title", f"Figure sequencer: {name}")
    print(f"=== {name} done ===", flush=True)
print("ALL DONE", flush=True)
