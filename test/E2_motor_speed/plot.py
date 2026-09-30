"""Plot the fixed E2 command-to-speed mapping from summary.csv (no serial access)."""

import argparse
import csv
from pathlib import Path


def plot_run(folder):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with (folder / "summary.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    x = [int(r["command"]) for r in rows]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for motor in ["m2", "m4"]:
        y = [float(r[motor + "_estimated_mm_s"]) for r in rows]
        spread = [float(r[motor + "_estimated_mm_s_std"]) for r in rows]
        axes[0].errorbar(
            x,
            y,
            yerr=spread,
            fmt="o-",
            markersize=3,
            capsize=2,
            label=motor.upper() + " mean +/- velocity SD",
        )
        axes[1].plot(
            x,
            [float(r[motor + "_counts_s"]) for r in rows],
            "o-",
            markersize=3,
            label=motor.upper(),
        )
    axes[0].plot(x, x, "k--", alpha=0.5, label="y=x nominal reference")
    axes[0].set_ylabel("Encoder-derived speed estimate (mm/s, uncalibrated)")
    axes[1].set_ylabel("Measured encoder speed (counts/s)")
    for ax in axes:
        ax.set_xlabel("Commanded speed (driver units)")
        ax.axhline(0, color="gray", linewidth=0.5)
        ax.axvline(0, color="gray", linewidth=0.5)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    # Keep degraded captures visibly distinct; plot.py can also be run offline.
    report = (folder / "summary.md").read_text(encoding="utf-8")
    status = "DEGRADED" if "INCOMPLETE OR DEGRADED" in report else "complete"
    fig.suptitle(f"E2: {min(x)}..{max(x)} | saved PID 3/0.375/0.5 (assumed) | {status}")
    fig.tight_layout()
    fig.savefig(folder / "summary.png", dpi=160)
    plt.close(fig)
    print(folder / "summary.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    plot_run(parser.parse_args().run)
