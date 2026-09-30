"""Plot counts/s versus command from an E2 run's summary.csv and save summary.png beside it."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path, help="results folder containing summary.csv")
    ap.add_argument(
        "--label", default="command", help="x-axis label, e.g. 'PWM command'"
    )
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()

    with (a.run / "summary.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    x = [int(r["command"]) for r in rows]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for motor, color in (("m2", "tab:blue"), ("m4", "tab:orange")):
        y = [float(r[f"{motor}_counts_s"]) for r in rows]
        ax.plot(x, y, "o-", ms=3, color=color, label=motor.upper())
    ax.axhline(0, color="gray", lw=0.6)
    ax.axvline(0, color="gray", lw=0.6)
    ax.set_xlabel(a.label)
    ax.set_ylabel("measured speed (encoder counts/s, 1–2 s window)")
    ax.set_title(f"E2 motor response: {a.run.name}")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out = a.run / "summary.png"
    fig.savefig(out, dpi=150)
    print(out)
    if a.show:
        plt.show()


if __name__ == "__main__":
    main()
