#!/usr/bin/env python3
"""Compare damping using the actual firmware PID and an illustrative roll plant."""
import csv
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CASES = [("P only", 0.0, "#d97706"), ("Current demo PD", 0.003, "#2563eb"),
         ("More damping", 0.008, "#059669")]


def run(binary, kp, kd, substeps=10):
    result = subprocess.run([str(binary), str(kp), str(kd), str(substeps)],
                            check=True, capture_output=True, text=True)
    return [{k: float(v) for k, v in row.items()}
            for row in csv.DictReader(io.StringIO(result.stdout))]


def recovery(rows, start, end, tolerance=1.0):
    window = [r for r in rows if start <= r["time_s"] < end]
    last_out = max((i for i, r in enumerate(window) if abs(r["angle_deg"]) > tolerance), default=-1)
    if last_out == len(window)-1 or window[-1]["time_s"]-window[last_out+1]["time_s"] < 0.5:
        return None
    return round(window[last_out+1]["time_s"]-start, 3)


def main():
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        raise SystemExit("A C++ compiler is needed. On macOS install the Xcode Command Line Tools.")
    output = ROOT / "simulation" / "output"
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="drone-simulation-") as temporary:
        # Keep compilation and Matplotlib caches outside the repository.
        os.environ.setdefault("MPLCONFIGDIR", str(Path(temporary)/"matplotlib"))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        binary = Path(temporary)/"closed_loop"
        subprocess.run([compiler, "-std=c++17", "-O2", "-I", str(ROOT/"include"),
                        str(ROOT/"simulation"/"closed_loop.cpp"), "-o", str(binary)], check=True)
        cases = [(name, run(binary, 0.02, kd), color) for name, kd, color in CASES]
        # Physical/numerical checks independent of specific plotted values.
        free = run(binary, 0, 0)
        assert max(abs(r["angle_deg"]-20) for r in free if r["time_s"]<4) < 1e-5
        assert free[-1]["angle_deg"]>20, "Positive gust must produce positive rotation"
        for _, rows, _ in cases:
            assert all(math.isfinite(v) for r in rows for v in r.values())
            assert all(abs(r["correction"])<=1.000001 and abs(r["torque_nm"])<=0.080001 for r in rows)
            assert rows[0]["correction"]<0, "Initial correction must oppose tilt"
        refined=run(binary, 0.02, 0.003, 20)
        difference=max(abs(a["angle_deg"]-b["angle_deg"]) for a,b in zip(cases[1][1],refined))
        assert difference<0.15, f"Integration refinement changed angles by {difference} degrees"
        assert abs(cases[1][1][-1]["angle_deg"])<1, "Demo must recover by end of run"
        metrics={}
        fig, axes = plt.subplots(3,1,figsize=(11,9),sharex=True)
        for name,rows,color in cases:
            slug=name.lower().replace(" ","_")
            with (output/f"closed_loop_{slug}.csv").open("w",newline="") as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
            t=[r["time_s"] for r in rows]
            for ax,key in zip(axes,["angle_deg","correction","torque_nm"]):
                ax.plot(t,[r[key] for r in rows],color=color,label=name,linewidth=1.7)
            metrics[name]={"initial_settling_within_1deg_s":recovery(rows,0,4),
                           "post_gust_settling_within_1deg_s":recovery(rows,4.2,10.01),
                           "peak_absolute_angle_after_gust_deg":round(max(abs(r["angle_deg"]) for r in rows if r["time_s"]>=4),3),
                           "final_angle_deg":round(rows[-1]["angle_deg"],4)}
        axes[0].axhline(0,color="#475569",linestyle="--",linewidth=1,label="Target: level")
        axes[0].set_ylabel("Roll angle (degrees)")
        axes[0].legend(ncol=2,loc="upper right")
        axes[1].set_ylabel("Correction (unitless)")
        axes[1].set_ylim(-1.05,1.05)
        axes[2].plot([r["time_s"] for r in cases[0][1]],[r["gust_nm"] for r in cases[0][1]],
                     color="#9333ea",linestyle="--",label="External gust torque")
        axes[2].set_ylabel("Torque (N m)"); axes[2].legend(loc="upper right")
        axes[2].set_xlabel("Time (seconds)")
        for ax in axes:
            ax.axvspan(4,4.2,color="#9333ea",alpha=0.10)
            ax.grid(alpha=0.2); ax.set_xlim(0,10)
        fig.suptitle("Virtual roll stabilization: initial 20-degree tilt + gust at 4 seconds",fontsize=15)
        fig.text(0.5,0.015,"Illustrative plant, ideal angle/rate sensing. Gains are not validated for flight.",ha="center",fontsize=10)
        fig.tight_layout(rect=(0,0.035,1,0.96))
        fig.savefig(output/"closed_loop_comparison.png",dpi=170)
        plt.close(fig)
        (output/"closed_loop_metrics.json").write_text(json.dumps(metrics,indent=2)+"\n")
        print(json.dumps(metrics,indent=2))
        print(f"Simulation checks passed; integration refinement difference: {difference:.4f} deg")
        print(f"Plot: {output/'closed_loop_comparison.png'}")
        print("Settling is time to enter and remain within +/-1 degree for the rest of each observation window.")
        print("null means not settled within that window. These are model results, not hardware validation.")

if __name__=="__main__":
    main()
