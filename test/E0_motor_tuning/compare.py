"""Compare completed E0 runs using the final second of each speed step.

Example: python compare.py --out results/comparison results/<run1> results/<run2>
This only reads saved data; it never opens a serial port.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    records, traces = [], []
    for folder in args.runs:
        meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
        if meta["status"] != "complete":
            raise ValueError(f"Incomplete run: {folder}")
        with (folder / "samples.csv").open() as file:
            rows = list(csv.DictReader(file))
        label = ", ".join(f"{v:g}" for v in meta["pid"])
        key = f"m{meta['motor']}"
        for trial in meta["trials"]:
            step = [r for r in rows if int(r["trial"]) == trial["trial"] and r["phase"] == "step"]
            tail = [float(r[key]) for r in step
                    if float(r["time_s"]) - trial["step_time_s"] >= meta["duration"] - 1]
            rest = [float(r[key]) for r in rows if int(r["trial"]) == trial["trial"]
                    and r["phase"] == "rest"
                    and float(r["time_s"]) - trial["stop_time_s"] >= meta["rest"] - 1]
            target = trial["speed"]
            records.append(dict(run=folder.name, pid=label, motor=meta["motor"], target=target,
                                duration_s=meta["duration"],
                                mean=statistics.mean(tail), std=statistics.pstdev(tail),
                                rmse=math.sqrt(statistics.mean((v-target)**2 for v in tail)),
                                peak_to_peak=max(tail)-min(tail),
                                step_peak=max(float(r[key]) for r in step),
                                rest_rms=math.sqrt(statistics.mean(v*v for v in rest))))
            if target == 20:
                data = [r for r in rows if int(r["trial"]) == trial["trial"]
                        and r["phase"] in ["baseline", "step", "rest"]]
                traces.append((label, folder.name, meta, trial, data, key))
    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "comparison.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    lines = ["# E0 PID comparison", "", "Final-second statistics; driver speed units, host receive timestamps.", "",
             "| Run | P, I, D | Target | Duration (s) | Mean | Std dev | RMSE | Peak-to-peak | Whole-step peak | Rest RMS |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in records:
        lines.append(f"| {r['run']} | {r['pid']} | {r['target']} | {r['duration_s']} | " +
                     " | ".join(f"{r[k]:.2f}" for k in ["mean", "std", "rmse", "peak_to_peak", "step_peak", "rest_rms"]) + " |")
    lines += ["", "RMSE includes both bias and variation. Rest RMS uses the final second after commanding zero.",
              "Single-run differences may include load/supply changes and sample timing; these are not calibrated vehicle tests."]
    (args.out / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if traces:
        fig, axes = plt.subplots(len(traces), 1, sharex=True, sharey=True,
                                 figsize=(11, 2.5 * len(traces)), squeeze=False)
        for ax, (label, name, meta, trial, data, key) in zip(axes[:, 0], traces):
            origin = trial["step_time_s"]
            x = [float(r["time_s"]) - origin for r in data]
            ax.plot(x, [float(r[key]) for r in data], ".-", markersize=3)
            stop = trial["stop_time_s"] - origin
            ax.step([min(x), 0, stop, max(x)], [0, 20, 0, 0], where="post", linestyle="--", color="orange")
            ax.set_title(f"PID {label} | {name}")
            ax.set_ylabel("Driver speed")
            ax.grid(alpha=0.3)
        axes[-1, 0].set_xlabel("Seconds from command (shared axes across all runs)")
        fig.tight_layout()
        fig.savefig(args.out / "low_speed_comparison.png", dpi=140)
        plt.close(fig)
    print(args.out)


if __name__ == "__main__":
    main()
