# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Measure the GPS innovation distribution BEFORE arming the disabled gate.

INAV's position estimator carries a disabled GPS acceptance gate
(src/main/navigation/navigation_pos_estimator.c:730):

    //const float gpsWeightScaler = scaleRangef(bellCurve(gpsPosResidualMag,
    //        INAV_GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
    const float gpsWeightScaler = 1.0f;

bellCurve(x, w) = exp(-x^2 / (2 w^2)) (common/maths.c), the residual is the
XY INNOVATION gpsPosResidualMag = |GPS pos - estimated pos| in cm, and
INAV_GPS_ACCEPTANCE_EPE = 500 cm. The live in-tree example of the same
pattern is the surface-offset gate (navigation_pos_estimator_agl.c:149,
width 75/50 cm, the same 0.1..1.0 scaleRangef mapping). Method mandate:
measure the residual distribution of HEALTHY flights before choosing any
width - this script is that measurement.

Input: jsbsim_fly.py flight CSVs. Reported residual channels (magnitudes):

  xy_truth     |injected GPS xy - plant truth xy| [m]. The error of the
               injection chain itself; with truth GPS (--gps) this is the
               measurement floor (lat_e7 quantisation + log round-off).
               With --gps-falsevalid it is the size of the lie.
  xy_inertial  |injected GPS xy - inertial reference| [m], the innovation
               PROXY. The reference anchors on the GPS fix and propagates
               with the plant-truth displacement (an ideal INS), re-anchored
               every --anchor-s seconds. Default 2 s: the estimator pulls
               position at w_xy_gps_p = 1.0 (tau ~ 1 s), so a reference of
               mean age ~1 s sees the same innovation scale. The proxy
               brackets the true innovation within a factor ~2; the armed-
               gate reflight is the end proof, not this analysis.
  z            |injected GPS alt - plant truth alt| [m]. Context only: the
               gate at :730 scales the XY weights, the Z path has no
               bellCurve gate.
  vel          |injected GPS vn/ve - truth velocity| [m/s]. Context: the
               decaying stale velocity is the false-valid signature. Truth
               velocity is a +-0.1 s central difference of the logged x/y.

Freeze phases (settle/cal/armL/armH) hold the plant at its initial
condition while JSBSim still reports the trim velocity - their vel channel
is meaningless by construction. They are excluded from the overall numbers
via --skip-phases (default: exactly those phases, like the show gate).

Old logs without inj_gps_x/y (vn/ve) columns fall back to the channels
that exist, and say so. Output per CSV: per-phase percentile table, the
implied bellCurve weight of each overall percentile at --epe, a gate view
(fraction of frames the in-tree scaler keeps >= target weight, longest dip
below 0.5, first crossing below 0.15 = detection), and a PNG (residual
timeline + histogram). A combined recommendation block prints the minimum
width that keeps the healthy flights at weight >= --target-weight for
--coverage percent of frames. Files whose injected GPS leaves the truth by
more than 2 m (false-valid runs) are excluded from that recommendation
automatically - the healthy width must come from healthy flights only.

    python gnss_residual_analysis.py <flight.csv> [more.csv ...]
        [--epe 500] [--target-weight 0.9] [--coverage 99]
        [--anchor-s 2.0] [--skip-phases settle,cal,armL,armH]
        [--min-fix 0] [--xy-scale 1.0] [--vel-scale 1.0]
        [--png out.png] [--no-png]
"""
import argparse
import csv
import math
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:            # stats still run, plotting is skipped
    HAVE_MPL = False

EPE_DEFAULT = 500.0            # INAV_GPS_ACCEPTANCE_EPE [cm]
SCALER_LO = 0.1                # in-tree scaleRangef(bell, 0,1, 0.1,1.0)
SKIP_DEFAULT = "settle,cal,armL,armH"
FALSEVALID_LEAVE_M = 2.0       # xy_truth beyond this = GPS no longer truth
PCTS = (50.0, 90.0, 99.0)
CH_COLOR = {"xy_truth": "#1f77b4", "xy_inertial": "#d62728",
            "z": "#2ca02c", "vel": "#9467bd"}
CH_UNIT = {"xy_truth": "m", "xy_inertial": "m", "z": "m", "vel": "m/s"}


# --- the firmware's math, verbatim in python ------------------------------

def bell_curve(r_cm, width_cm):
    """maths.c bellCurve: exp(-x^2 / (2 w^2)) - gaussian, sigma = width."""
    return math.exp(-(r_cm * r_cm) / (2.0 * width_cm * width_cm))


def in_tree_scaler(r_cm, width_cm, lo=SCALER_LO):
    """The disabled line at navigation_pos_estimator.c:730."""
    return lo + (1.0 - lo) * bell_curve(r_cm, width_cm)


def width_for(r_cm, weight, lo=SCALER_LO):
    """Smallest width whose scaler still yields `weight` at residual r."""
    b = (weight - lo) / (1.0 - lo)
    if b <= 0.0:
        return 0.0
    if b >= 1.0:
        return float("inf")
    return r_cm / math.sqrt(2.0 * math.log(1.0 / b))


def radius_for(weight, width_cm, lo=SCALER_LO):
    """Residual at which the scaler drops to `weight` (inverse of width_for)."""
    b = (weight - lo) / (1.0 - lo)
    if b <= 0.0:
        return float("inf")
    if b >= 1.0:
        return 0.0
    return width_cm * math.sqrt(2.0 * math.log(1.0 / b))


# --- small helpers --------------------------------------------------------

def percentile(sorted_vals, pct):
    """Linear-interpolated percentile on an ascending list."""
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * pct / 100.0
    lo = int(math.floor(k))
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def _f(row, key):
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def load_frames(path):
    """Parse the flight CSV into per-frame dicts; absent columns become None."""
    rows = list(csv.DictReader(open(path, newline="")))
    frames = []
    for r in rows:
        t = _f(r, "t")
        if t is None:
            continue
        frames.append({
            "t": t, "phase": r.get("phase", ""),
            "x": _f(r, "x"), "y": _f(r, "y"), "alt": _f(r, "alt"),
            "fix": _f(r, "gps_fix"),
            "gz": _f(r, "inj_gps_alt"),        # [cm] per jsbsim_fly.py
            "gx": _f(r, "inj_gps_x"), "gy": _f(r, "inj_gps_y"),
            "gvn": _f(r, "inj_gps_vn"), "gve": _f(r, "inj_gps_ve"),
        })
    header = rows[0].keys() if rows else []
    present = [c for c in ("inj_gps_alt", "inj_gps_x", "inj_gps_y",
                           "inj_gps_vn", "inj_gps_ve") if c in header]
    return frames, present


def build_channels(frames, args):
    """name -> list of (t, phase, |residual|); plus notes about what is missing."""
    ch = {}
    notes = []
    n = len(frames)
    mean_dt = ((frames[-1]["t"] - frames[0]["t"]) / (n - 1)) if n > 1 else 0.01
    if mean_dt <= 0.0:
        mean_dt = 0.01          # t has 0.01 s resolution; guard a broken log

    def ok(fr):                 # optional FC-side fix filter (readback, 10 Hz)
        return args.min_fix <= 0 or (fr["fix"] is not None
                                     and fr["fix"] >= args.min_fix)

    have_xy = any(fr["gx"] is not None and fr["gy"] is not None
                  for fr in frames)
    have_vel = any(fr["gvn"] is not None and fr["gve"] is not None
                   for fr in frames)
    have_z = any(fr["gz"] is not None for fr in frames)

    if have_xy:
        s = args.xy_scale
        vals = [(fr["t"], fr["phase"],
                 math.hypot(fr["gx"] * s - fr["x"], fr["gy"] * s - fr["y"]))
                for fr in frames if ok(fr) and None not in
                (fr["gx"], fr["gy"], fr["x"], fr["y"])]
        ch["xy_truth"] = vals
        # unit sanity: the injection work logs metres in the truth frame
        # (measured: inj_gps_x 349.57 vs x 349.6); warn if it looks like cm
        med_g = percentile(sorted(abs(fr["gx"]) for fr in frames
                                  if fr["gx"] is not None), 50.0)
        med_x = percentile(sorted(abs(fr["x"]) for fr in frames
                                  if fr["x"] is not None), 50.0)
        if med_x > 2.0 and med_g > 30.0 * med_x and s == 1.0:
            notes.append("WARNING: inj_gps_x is ~%.0fx the truth x - "
                         "centimetres? rerun with --xy-scale 0.01"
                         % (med_g / med_x))

        # inertial reference: anchor on GPS, add the truth displacement
        # since the anchor (ideal INS), re-anchor every --anchor-s
        vals = []
        anchor = None
        for fr in frames:
            if (not ok(fr)) or None in (fr["gx"], fr["gy"], fr["x"], fr["y"]):
                anchor = None   # validity gap -> fresh anchor
                continue
            if anchor is None or fr["t"] - anchor["t"] >= args.anchor_s:
                anchor = fr
            rx = (fr["gx"] - anchor["gx"]) * s - (fr["x"] - anchor["x"])
            ry = (fr["gy"] - anchor["gy"]) * s - (fr["y"] - anchor["y"])
            vals.append((fr["t"], fr["phase"], math.hypot(rx, ry)))
        ch["xy_inertial"] = vals
    else:
        notes.append("inj_gps_x/y not in this log - no XY residuals; the "
                     "gate decision needs a flight with the injection "
                     "columns (z reported below is a proxy only)")

    if have_z:
        ch["z"] = [(fr["t"], fr["phase"], abs(fr["gz"] / 100.0 - fr["alt"]))
                   for fr in frames if ok(fr) and None not in
                   (fr["gz"], fr["alt"])]
    else:
        notes.append("inj_gps_alt empty - no GPS injected in this flight?")

    if have_vel:
        w = max(1, int(round(0.1 / mean_dt)))   # +-0.1 s central difference
        vs = args.vel_scale
        vals = []
        for i, fr in enumerate(frames):
            if (not ok(fr)) or None in (fr["gvn"], fr["gve"]):
                continue
            j0, j1 = max(0, i - w), min(n - 1, i + w)
            a, b = frames[j0], frames[j1]
            if None in (a["x"], a["y"], b["x"], b["y"]):
                continue
            dt = b["t"] - a["t"]
            if dt <= 0.0:
                continue
            vtn = (b["x"] - a["x"]) / dt        # x = north, y = east [m]
            vte = (b["y"] - a["y"]) / dt
            vals.append((fr["t"], fr["phase"],
                         math.hypot(fr["gvn"] * vs - vtn,
                                    fr["gve"] * vs - vte)))
        ch["vel"] = vals
    elif have_xy:
        notes.append("inj_gps_vn/ve not in this log - velocity residual "
                     "skipped")

    return ch, notes, mean_dt


# --- reporting ------------------------------------------------------------

def stats_line(vals):
    """(n, p50, p90, p99, max) of a value list."""
    if not vals:
        return 0, float("nan"), float("nan"), float("nan"), float("nan")
    sv = sorted(vals)
    return (len(sv), percentile(sv, 50.0), percentile(sv, 90.0),
            percentile(sv, 99.0), sv[-1])


def phase_table(channels, skip):
    """Per-phase percentile table, phases in first-appearance order."""
    order = []
    for name in channels:
        for t, ph, v in channels[name]:
            if ph not in order:
                order.append(ph)
    print("    phase         channel          n      p50      p90      p99      max")
    for ph in order:
        first = True
        for name, series in channels.items():
            vals = [v for t, p, v in series if p == ph]
            if not vals:
                continue
            n, p50, p90, p99, mx = stats_line(vals)
            tag = (ph + ("*" if ph in skip else "")) if first else ""
            print("    %-13s %-12s %7d %8.3f %8.3f %8.3f %8.3f  [%s]"
                  % (tag, name, n, p50, p90, p99, mx, CH_UNIT[name]))
            first = False
    if any(ph in skip for ph in order):
        print("    (* = prep phase, excluded from the overall numbers below)")


def overall_vals(series, skip):
    return [v for t, ph, v in series if ph not in skip]


def weight_table(name, vals_m, epe):
    """Percentiles of one channel -> implied bellCurve weight at `epe`."""
    sv = sorted(vals_m)
    print("    %s overall vs bellCurve width %.0f cm:" % (name, epe))
    print("        pct    residual      bellCurve   in-tree scaler (0.1..1.0)")
    for label, val in [("p%.0f" % p, percentile(sv, p)) for p in PCTS] + \
                      [("max", sv[-1] if sv else float("nan"))]:
        r_cm = val * 100.0
        print("        %-5s %7.3f m      %7.4f      %7.4f"
              % (label, val, bell_curve(r_cm, epe),
                 in_tree_scaler(r_cm, epe)))


def gate_view(name, series, skip, args, mean_dt):
    """What the armed gate would have done to this flight at --epe."""
    seq = [(t, v) for t, ph, v in series if ph not in skip]
    if not seq:
        print("    gate view: no frames outside the prep phases")
        return
    w = args.epe
    n_hi = sum(1 for t, v in seq if in_tree_scaler(v * 100.0, w)
               >= args.target_weight)
    frac = 100.0 * n_hi / len(seq)
    # longest contiguous dip below scaler 0.5 (frame count * mean dt: the
    # 0.01 s t resolution makes per-frame t diffs useless at 1 kHz)
    r_half = radius_for(0.5, w) / 100.0
    r_low = radius_for(0.15, w) / 100.0
    longest = run = 0
    first_half = first_low = None
    for t, v in seq:
        if v >= r_half:
            run += 1
            longest = max(longest, run)
            if first_half is None:
                first_half = t
        else:
            run = 0
        if first_low is None and v >= r_low:
            first_low = t
    print("    gate view at width %.0f cm (in-tree scaler), channel %s:"
          % (w, name))
    ok = "met" if frac >= args.coverage else "MISSED"
    print("        scaler >= %.2f for %6.2f %% of frames   "
          "(target %.1f %% -> %s)"
          % (args.target_weight, frac, args.coverage, ok))
    print("        longest stretch below 0.50: %6.2f s   "
          "(residual >= %.2f m)" % (longest * mean_dt, r_half))
    print("        first crossing below 0.50:  %s"
          % ("t=%.2f s" % first_half if first_half is not None else "never"))
    print("        first crossing below 0.15:  %s   (residual >= %.2f m)"
          % ("t=%.2f s" % first_low if first_low is not None else "never",
             r_low))
    return first_low


def falsevalid_leave(channels):
    """First time the injected GPS leaves the truth by > 2 m, if ever."""
    for t, ph, v in channels.get("xy_truth", []):
        if v > FALSEVALID_LEAVE_M:
            return t
    return None


# --- plotting -------------------------------------------------------------

def plot_file(path, channels, skip, args, chosen):
    if not HAVE_MPL:
        print("    (matplotlib not available - PNG skipped)")
        return
    out = args.png or (os.path.splitext(path)[0] + "_gnss_residuals.png")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8))
    floor = 1e-3                                 # log plot floor: 1 mm
    for name, series in channels.items():
        if not series:
            continue
        ax1.plot([t for t, p, v in series],
                 [max(v, floor) for t, p, v in series],
                 color=CH_COLOR[name], lw=0.7, alpha=0.8,
                 label="%s [%s]" % (name, CH_UNIT[name]))
    ax1.set_yscale("log")
    ax1.axhline(args.epe / 100.0, color="gray", ls="--", lw=1.0,
                label="width %.0f cm" % args.epe)
    ax1.axhline(radius_for(args.target_weight, args.epe) / 100.0,
                color="gray", ls=":", lw=1.0,
                label="scaler %.2f radius" % args.target_weight)
    # phase boundaries; labels only when they stay readable
    any_series = next(iter(channels.values()))
    marks = [(t, p) for i, (t, p, v) in enumerate(any_series)
             if i == 0 or p != any_series[i - 1][1]]
    for t, p in marks:
        ax1.axvline(t, color="0.85", lw=0.6, zorder=0)
    if len(marks) <= 28:
        top = ax1.get_ylim()[1]
        for t, p in marks:
            ax1.text(t, top, " " + p, rotation=90, va="top", ha="left",
                     fontsize=6, color="0.4")
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("|residual| (clipped at 1 mm)")
    ax1.set_title("%s - GPS residual timeline" % os.path.basename(path))
    ax1.legend(fontsize=7, loc="upper right")
    ax1.grid(alpha=0.3)

    vals = overall_vals(channels[chosen], skip)
    if vals:
        top = max(max(vals), args.epe / 100.0) * 1.5
        bins = [10.0 ** (math.log10(floor) + i / 60.0
                         * (math.log10(top) - math.log10(floor)))
                for i in range(61)]
        ax2.hist([max(v, floor) for v in vals], bins=bins,
                 color=CH_COLOR[chosen], alpha=0.75)
        ax2.set_xscale("log")
        sv = sorted(vals)
        for p in PCTS:
            v = max(percentile(sv, p), floor)
            ax2.axvline(v, color="k", ls=":", lw=0.9)
            ax2.text(v, ax2.get_ylim()[1] * 0.95, " p%.0f" % p,
                     rotation=90, va="top", fontsize=7)
        ax2.axvline(args.epe / 100.0, color="gray", ls="--", lw=1.0)
        ax2.set_title("%s outside prep phases (dashed: width %.0f cm)"
                      % (chosen, args.epe))
        ax2.set_xlabel("|residual| [%s]" % CH_UNIT[chosen])
        ax2.set_ylabel("frames")
        ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("    wrote %s" % out)


# --- main -----------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="GPS innovation statistics for the disabled bellCurve "
                    "gate (navigation_pos_estimator.c:730). Feed --gps "
                    "truth-baseline flights for the width recommendation; "
                    "false-valid flights are reported but excluded from it.")
    ap.add_argument("csvs", nargs="+", metavar="flight.csv")
    ap.add_argument("--epe", type=float, default=EPE_DEFAULT,
                    help="bellCurve width to evaluate [cm] "
                         "(default %.0f = INAV_GPS_ACCEPTANCE_EPE)"
                         % EPE_DEFAULT)
    ap.add_argument("--target-weight", type=float, default=0.9,
                    help="healthy flights must keep the in-tree scaler "
                         "above this (default 0.9)")
    ap.add_argument("--coverage", type=float, default=99.0,
                    help="...for this percentage of frames (default 99)")
    ap.add_argument("--anchor-s", type=float, default=2.0,
                    help="inertial reference re-anchor interval [s] "
                         "(default 2.0 ~ 2x the w_xy_gps_p=1.0 pull time)")
    ap.add_argument("--skip-phases", default=SKIP_DEFAULT,
                    help="comma list excluded from overall numbers "
                         "(default %s)" % SKIP_DEFAULT)
    ap.add_argument("--min-fix", type=int, default=0,
                    help="drop frames whose gps_fix readback is below this "
                         "(default 0 = keep all)")
    ap.add_argument("--xy-scale", type=float, default=1.0,
                    help="multiply inj_gps_x/y into metres (0.01 for cm logs)")
    ap.add_argument("--vel-scale", type=float, default=1.0,
                    help="multiply inj_gps_vn/ve into m/s (0.01 for cm/s logs)")
    ap.add_argument("--png", default=None,
                    help="PNG path (single CSV only; default <csv>_gnss_"
                         "residuals.png)")
    ap.add_argument("--no-png", action="store_true")
    args = ap.parse_args()
    if args.png and len(args.csvs) > 1:
        ap.error("--png only makes sense with a single CSV")
    skip = set(p for p in args.skip_phases.split(",") if p)

    rec_pool = {"xy_inertial": [], "xy_truth": [], "z": []}
    rec_files = {"xy_inertial": [], "xy_truth": [], "z": []}

    for path in args.csvs:
        frames, present = load_frames(path)
        print("=== %s ===" % path)
        if not frames:
            print("    empty log, skipped")
            continue
        channels, notes, mean_dt = build_channels(frames, args)
        print("    rows %d, t %.1f..%.1f s, mean dt %.1f ms, "
              "injection columns: %s"
              % (len(frames), frames[0]["t"], frames[-1]["t"],
                 mean_dt * 1000.0, ", ".join(present) if present else "none"))
        for note in notes:
            print("    note: %s" % note)
        if not channels:
            print("    nothing to analyse in this log")
            continue
        phase_table(channels, skip)

        chosen = next(n for n in ("xy_inertial", "xy_truth", "z")
                      if n in channels)
        for name in ("xy_inertial", "xy_truth") if "xy_truth" in channels \
                else (chosen,):
            vals = overall_vals(channels[name], skip)
            if vals:
                weight_table(name, vals, args.epe)
        first_low = gate_view(chosen, channels[chosen], skip, args, mean_dt)

        leave = falsevalid_leave(channels)
        if leave is not None:
            print("    injected GPS leaves the truth (> %.0f m) first at "
                  "t=%.2f s -> NOT a healthy baseline, excluded from the "
                  "width recommendation" % (FALSEVALID_LEAVE_M, leave))
            if first_low is not None:
                print("    detection latency at width %.0f cm: %.2f s "
                      "(leave %.2f -> scaler<=0.15 at %.2f)"
                      % (args.epe, first_low - leave, leave, first_low))
            elif chosen.startswith("xy"):
                print("    the gate would NEVER have pushed the scaler to "
                      "0.15 at width %.0f cm - the lie stays inside the "
                      "acceptance radius" % args.epe)
        else:
            vals = overall_vals(channels[chosen], skip)
            if vals:
                rec_pool[chosen].extend(vals)
                rec_files[chosen].append(os.path.basename(path))

        if not args.no_png:
            plot_file(path, channels, skip, args, chosen)
        print()

    # --- combined recommendation over the healthy flights -----------------
    chosen = next((n for n in ("xy_inertial", "xy_truth", "z")
                   if rec_pool[n]), None)
    print("=== RECOMMENDATION (healthy-flight gate width) ===")
    if chosen is None:
        print("    no healthy baseline frames in the given files - fly the "
              "--gps truth baselines first (docs/gnss_measurement_plan.md)")
        return
    if chosen == "z":
        print("    WARNING: only the Z residual is available; the gate at "
              "navigation_pos_estimator.c:730 acts on XY. Add the "
              "inj_gps_x/y columns before deciding anything.")
    vals = sorted(rec_pool[chosen])
    r_cov = percentile(vals, args.coverage) * 100.0     # [cm]
    w_min = width_for(r_cov, args.target_weight)
    w_raw = width_for(r_cov, args.target_weight, lo=0.0)
    print("    channel %s, %d frames from: %s"
          % (chosen, len(vals), ", ".join(rec_files[chosen])))
    print("    p%.0f residual: %.3f m = %.1f cm"
          % (args.coverage, r_cov / 100.0, r_cov))
    print("    at width %.0f cm that residual weighs: bellCurve %.4f, "
          "in-tree scaler %.4f"
          % (args.epe, bell_curve(r_cov, args.epe),
             in_tree_scaler(r_cov, args.epe)))
    print("    minimum width for in-tree scaler >= %.2f at p%.0f: %.0f cm"
          % (args.target_weight, args.coverage, w_min))
    print("    (raw bellCurve >= %.2f would need %.0f cm)"
          % (args.target_weight, w_raw))
    if w_min <= args.epe:
        print("    -> the in-tree default %.0f cm already satisfies the "
              "target; keep INAV_GPS_ACCEPTANCE_EPE unchanged" % args.epe)
    else:
        print("    -> the in-tree default %.0f cm is TOO NARROW for these "
              "flights; the gate needs a width >= %.0f cm" % (args.epe, w_min))
    print("    note: the bench GPS is noiseless - real receivers add their "
          "own metres of noise, so this measurement can only RAISE the "
          "width above %.0f cm, never justify shrinking it." % EPE_DEFAULT)


if __name__ == "__main__":
    main()
