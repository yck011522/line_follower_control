"""Plot generation script for E12 Dual BLDC Motor, Line Sensor & NFC Reader tests.

Parses raw test logs from test/E12_motor_line_nfc/results/ and outputs high-resolution
analytical plots to test/E12_motor_line_nfc/images/.
"""

import pathlib
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
E12_DIR = ROOT / "test" / "E12_motor_line_nfc"
RESULTS_DIR = E12_DIR / "results"
IMAGES_DIR = E12_DIR / "images"
IMAGES_DIR.mkdir(parents=True, exist_ok=True)

TELEMETRY_RE = re.compile(
    r"^P=(-?\d+),L=(-?\d+),R=(-?\d+),LINE=([01\-]{8}),NFC=([0-9A-Fa-f:]{11}|NONE),AGE=([0-9\.\-]+s),RATE=([0-9\.]+Hz)$"
)

SENSOR_WEIGHTS = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]


def parse_log(filepath):
    """Parse a single raw log file into structured motion telemetry arrays."""
    records = []
    active_motion = False
    target_speed = 0.0

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if "OK S=" in line:
                m_speed = re.search(r"OK S=([0-9\.\-]+)", line)
                if m_speed:
                    target_speed = float(m_speed.group(1))
                active_motion = True
                continue

            m = TELEMETRY_RE.match(line)
            if not m:
                continue

            p = int(m.group(1))
            l = int(m.group(2))
            r = int(m.group(3))
            mask_str = m.group(4)
            rate = float(m.group(7).replace("Hz", ""))

            # Calculate line centroid
            active_count = 0
            weighted_sum = 0.0
            if mask_str not in ("11111111", "00000000", "--------"):
                for i in range(8):
                    if mask_str[7 - i] == "0":  # bit i (0 is rightmost, 7 is leftmost)
                        weighted_sum += SENSOR_WEIGHTS[i]
                        active_count += 1
            norm_error = (weighted_sum / active_count / 3.5) if active_count > 0 else 0.0

            records.append({
                "p": p,
                "l": l,
                "r": r,
                "line": mask_str,
                "active_line": active_count > 0,
                "norm_error": norm_error,
                "rate": rate,
                "active_motion": active_motion,
            })

    # Filter to active run window: from start of active_motion until 3 frames after stop
    motion_records = [r for r in records if r["active_motion"]]
    # Trim leading trailing stationary states
    start_idx = 0
    end_idx = len(motion_records)
    for i, r in enumerate(motion_records):
        if r["l"] != 0 or r["r"] != 0 or r["active_line"]:
            start_idx = max(0, i - 1)
            break
    for i in range(len(motion_records) - 1, -1, -1):
        r = motion_records[i]
        if r["l"] != 0 or r["r"] != 0:
            end_idx = min(len(motion_records), i + 3)
            break

    trimmed = motion_records[start_idx:end_idx]
    return target_speed, trimmed


def plot_speed_comparison(runs):
    """Plot multi-speed wheel velocity tracking and line error comparison."""
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), sharex=False)
    axes = axes.flatten()

    for idx, (label, target_speed, data) in enumerate(runs):
        ax = axes[idx]
        t = [i * 0.1 for i in range(len(data))]
        l_spd = [d["l"] for d in data]
        r_spd = [d["r"] for d in data]
        err = [d["norm_error"] for d in data]

        # Plot wheel speeds
        ax.plot(t, l_spd, "b-", linewidth=2.0, label="Left Wheel (L, mm/s)")
        ax.plot(t, r_spd, "r-", linewidth=2.0, label="Right Wheel (R, mm/s)")
        ax.axhline(target_speed, color="gray", linestyle="--", alpha=0.6, label=f"Target ({target_speed:.0f} mm/s)")
        ax.axhline(0, color="black", linestyle=":", linewidth=0.8, alpha=0.5)

        # Plot secondary axis for line centroid error
        ax2 = ax.twinx()
        ax2.plot(t, err, "g--", linewidth=1.3, alpha=0.75, label="Line Error (norm)")
        ax2.set_ylim(-1.2, 1.2)
        ax2.set_ylabel("Line Error", color="green", fontsize=9)
        ax2.tick_params(axis="y", labelcolor="green")

        ax.set_title(f"{label} (Target S{target_speed:.0f} mm/s)", fontsize=11, fontweight="bold")
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel("Speed (mm/s)", fontsize=9)
        ax.grid(True, linestyle=":", alpha=0.5)

        # Combine legends on first subplot
        if idx == 0:
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    plt.suptitle("E12 Line-Following Closed-Loop Steering Dynamics Across Speeds (15 to 100 mm/s)", fontsize=14, fontweight="bold", y=0.995)
    plt.tight_layout()
    out_path = IMAGES_DIR / "e12_speed_comparison.png"
    plt.savefig(out_path, dpi=180)
    plt.close()
    print(f"Generated: {out_path}")


def plot_detailed_dynamics(run_s25, run_s100):
    """Plot detailed telemetry breakdown (Speeds, Error, PWM) for S25 and S100."""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex="col")

    runs_data = [("S25 (25 mm/s)", run_s25), ("S100 (100 mm/s)", run_s100)]

    for col, (title, (target_speed, data)) in enumerate(runs_data):
        t = [i * 0.1 for i in range(len(data))]
        l_spd = [d["l"] for d in data]
        r_spd = [d["r"] for d in data]
        err = [d["norm_error"] for d in data]
        pwm = [d["p"] for d in data]

        # Row 1: Wheel Speeds
        ax0 = axes[0, col]
        ax0.plot(t, l_spd, "b-", linewidth=2.0, label="Left Wheel (Inner)")
        ax0.plot(t, r_spd, "r-", linewidth=2.0, label="Right Wheel (Outer)")
        ax0.axhline(target_speed, color="black", linestyle="--", alpha=0.5, label=f"Target ({target_speed:.0f} mm/s)")
        ax0.axhline(-target_speed, color="red", linestyle=":", alpha=0.4, label=f"Min Limit (-{target_speed:.0f} mm/s)")
        ax0.axhline(0, color="gray", linestyle="-", linewidth=0.8, alpha=0.5)
        ax0.set_title(f"{title} - Closed-Loop Velocities", fontsize=11, fontweight="bold")
        ax0.set_ylabel("Speed (mm/s)")
        ax0.grid(True, linestyle=":", alpha=0.5)
        ax0.legend(loc="upper right", fontsize=8)

        # Row 2: Normalized Line Error
        ax1 = axes[1, col]
        ax1.plot(t, err, "g-", linewidth=2.0, label="Centroid Error")
        ax1.axhline(0, color="gray", linestyle="-", linewidth=0.8, alpha=0.5)
        ax1.axhline(0.57, color="orange", linestyle=":", alpha=0.7, label="Outer Sensors")
        ax1.axhline(-0.57, color="orange", linestyle=":", alpha=0.7)
        ax1.set_ylabel("Normalized Line Error [-1..1]")
        ax1.set_ylim(-1.15, 1.15)
        ax1.grid(True, linestyle=":", alpha=0.5)
        ax1.legend(loc="upper right", fontsize=8)

        # Row 3: PWM Output
        ax2 = axes[2, col]
        ax2.plot(t, pwm, "m-", linewidth=2.0, label="Avg Motor PWM (P)")
        ax2.axhline(0, color="gray", linestyle="-", linewidth=0.8, alpha=0.5)
        ax2.set_xlabel("Time (seconds)")
        ax2.set_ylabel("Commanded PWM (-255..+255)")
        ax2.grid(True, linestyle=":", alpha=0.5)
        ax2.legend(loc="upper right", fontsize=8)

    plt.suptitle("E12 Dynamic Response: Velocities, Line Tracking Error, and Motor PWM", fontsize=14, fontweight="bold", y=0.995)
    plt.tight_layout()
    out_path = IMAGES_DIR / "e12_detailed_dynamics.png"
    plt.savefig(out_path, dpi=180)
    plt.close()
    print(f"Generated: {out_path}")


def plot_reverse_constraint_verification(all_runs):
    """Plot Inner Wheel vs Outer Wheel Speed during steering to verify v_inner >= -v_outer."""
    plt.figure(figsize=(8, 7))

    v_outer_all = []
    v_inner_all = []
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    labels = ["S15", "S25", "S40", "S70", "S100"]

    for idx, (label, target_speed, data) in enumerate(all_runs):
        vo_list = []
        vi_list = []
        for d in data:
            if not d["active_line"] or (d["l"] == 0 and d["r"] == 0):
                continue
            # When steering (outer > inner)
            if d["r"] > d["l"]:
                v_outer = d["r"]
                v_inner = d["l"]
            else:
                v_outer = d["l"]
                v_inner = d["r"]

            if v_outer > 5:  # Only active forward motion
                vo_list.append(v_outer)
                vi_list.append(v_inner)

        plt.scatter(vo_list, vi_list, color=colors[idx % len(colors)], label=label, alpha=0.75, s=28)

    # Reference boundaries
    v_range = np.linspace(0, 110, 200)
    plt.plot(v_range, -v_range, "r--", linewidth=2.2, label=r"Constraint Limit: $v_{inner} = -v_{outer}$ (In-Place Pivot)")
    plt.plot(v_range, np.zeros_like(v_range), "k:", linewidth=1.5, label=r"Zero Inner Speed: $v_{inner} = 0$ (Pivot on Inner Wheel)")
    plt.plot(v_range, v_range, "g-.", linewidth=1.2, alpha=0.6, label=r"Straight Motion: $v_{inner} = v_{outer}$")

    plt.fill_between(v_range, -v_range, -120, color="red", alpha=0.1, label="Disallowed Region ($v_{inner} < -v_{outer}$)")

    plt.title(r"Inner Wheel Reverse Speed Constraint Verification ($v_{inner} \geq -v_{outer}$)", fontsize=12, fontweight="bold")
    plt.xlabel("Outer Wheel Speed (mm/s)", fontsize=10)
    plt.ylabel("Inner Wheel Speed (mm/s)", fontsize=10)
    plt.xlim(0, 115)
    plt.ylim(-115, 115)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="upper left", fontsize=8.5)

    plt.tight_layout()
    out_path = IMAGES_DIR / "e12_inner_wheel_reverse_constraint.png"
    plt.savefig(out_path, dpi=180)
    plt.close()
    print(f"Generated: {out_path}")


def main():
    log_files = [
        ("S15", RESULTS_DIR / "line_follow_S15_20261013_221028_COM17_raw.log"),
        ("S25", RESULTS_DIR / "line_follow_S25_20261013_221054_COM17_raw.log"),
        ("S40 (Run 1)", RESULTS_DIR / "line_follow_S40_20261013_221122_COM17_raw.log"),
        ("S40 (Run 2)", RESULTS_DIR / "line_follow_S40_20261013_221202_COM17_raw.log"),
        ("S70", RESULTS_DIR / "line_follow_S70_20261013_221238_COM17_raw.log"),
        ("S100", RESULTS_DIR / "line_follow_S100_20261013_221319_COM17_raw.log"),
    ]

    parsed_runs = []
    for label, path in log_files:
        speed, data = parse_log(path)
        parsed_runs.append((label, speed, data))

    # Generate the 3 figures
    plot_speed_comparison(parsed_runs)

    # For detailed dynamics: S25 and S100
    s25_run = (parsed_runs[1][1], parsed_runs[1][2])
    s100_run = (parsed_runs[5][1], parsed_runs[5][2])
    plot_detailed_dynamics(s25_run, s100_run)

    # For reverse constraint: select S15, S25, S40 (Run 1), S70, S100
    constraint_runs = [
        parsed_runs[0],
        parsed_runs[1],
        parsed_runs[2],
        parsed_runs[4],
        parsed_runs[5]
    ]
    plot_reverse_constraint_verification(constraint_runs)


if __name__ == "__main__":
    main()
