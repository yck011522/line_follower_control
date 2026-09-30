"""Summarize a captured E2 serial log; no upload, motor commands, or serial access."""

import argparse
import csv
import io
import math
from pathlib import Path
from statistics import mean, pstdev


def delta(a, b):
    return ((int(b) - int(a) + 2**31) % 2**32) - 2**31


def analyze(log_path, counts_per_rev):
    text = log_path.read_text(encoding="utf-8")
    lines = text.splitlines()
    headers = [i for i, line in enumerate(lines) if line.startswith("step,command,")]
    if len(headers) != 1:
        raise ValueError("Expected exactly one sweep CSV header in the log")
    records = [
        line for line in lines[headers[0] :] if line and not line.startswith("#")
    ]
    data = list(csv.DictReader(io.StringIO("\n".join(records))))
    start = next((s for s in lines if s.startswith("# START ")), "")
    fields = dict(p.split("=", 1) for p in start.split() if "=" in p)
    mode = fields.get("mode", "speed")
    if "min" in fields:
        expected = list(
            range(int(fields["min"]), int(fields["max"]) + 1, int(fields["inc"]))
        )
    else:
        expected = list(range(-240, 241, 20))
    label = "PWM" if mode == "pwm" else "speed"
    groups = {speed: [] for speed in expected}
    for row in data:
        speed = int(row["command"])
        if speed not in groups or int(row["step"]) != expected.index(speed):
            raise ValueError("Unexpected command/step in log")
        if row["phase"] == "measure" and 1000 <= int(row["step_ms"]) < 2000:
            groups[speed].append(row)
    summaries = []
    for speed, rows in groups.items():
        if len(rows) < 2:
            raise ValueError(f"Insufficient measurement data at command {speed}")
        result = {"command": speed, "measurement_samples": len(rows)}
        for motor, timestamp in [("m2", "t2_us"), ("m4", "t4_us")]:
            elapsed = (
                (int(rows[-1][timestamp]) - int(rows[0][timestamp])) % 2**32
            ) / 1e6
            if elapsed <= 0:
                raise ValueError("Invalid timestamp window")
            cps = delta(rows[0][motor + "_count"], rows[-1][motor + "_count"]) / elapsed
            # Divide the measured second into two halves to flag continuing settling.
            mid = len(rows) // 2
            halves = []
            for segment in (rows[:mid], rows[mid:]):
                dt = (
                    (int(segment[-1][timestamp]) - int(segment[0][timestamp])) % 2**32
                ) / 1e6
                halves.append(
                    delta(segment[0][motor + "_count"], segment[-1][motor + "_count"])
                    / dt
                    if dt
                    else float("nan")
                )
            recent = [int(r[motor + "_recent"]) for r in rows]
            result.update(
                {
                    motor + "_window_s": elapsed,
                    motor + "_counts_s": cps,
                    motor + "_estimated_mm_s": cps * math.pi * 65 / counts_per_rev,
                    motor + "_recent_mean": mean(recent),
                    motor + "_recent_std": pstdev(recent),
                    motor + "_half_window_change_counts_s": halves[1] - halves[0],
                }
            )
        summaries.append(result)
    end = next((s for s in lines if s.startswith("# END ")), "# END missing")
    clean = all(
        token in end.split()
        for token in [
            "reason=complete",
            "dropped=0",
            "missed_ticks=0",
            "errors=0",
            "stop_ack=1",
            "release_ack=1",
        ]
    )
    with log_path.with_name("samples.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=data[0].keys())
        writer.writeheader()
        writer.writerows(data)
    with log_path.with_name("summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    report = [
        f"# E2 motor {label} sweep",
        "",
        f"Capture status: {'complete, no reported drops/errors/missed ticks' if clean else 'INCOMPLETE OR DEGRADED; inspect end record'}.",
        "",
        "Both motors received the same raw command. Only samples from 1–2 seconds after each command are summarized.",
        f"The mm/s columns are estimates using 65 mm diameter and {counts_per_rev:g} counts/output revolution; this conversion is NOT calibrated.",
        "Counts/s comes from cumulative differences over actual device time. The command unit and physical forward direction remain unverified.",
        "One second is an allowed settling period, not proof of settling; half-window drift and recent-count variability are retained in summary.csv.",
        "",
        "| Command | M2 counts/s | M4 counts/s | M2 estimated mm/s | M4 estimated mm/s |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ]
    for r in summaries:
        report.append(
            f"| {r['command']} | {r['m2_counts_s']:.1f} | {r['m4_counts_s']:.1f} | {r['m2_estimated_mm_s']:.1f} | {r['m4_estimated_mm_s']:.1f} |"
        )
    report += ["", "End record:", "", "```text", end, "```", ""]
    log_path.with_name("summary.md").write_text("\n".join(report), encoding="utf-8")
    print("\n".join(report))
    return clean


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--counts-per-rev",
        type=float,
        default=500 * 4 * 22.569,
        help="Provisional default: 500 encoder cycles x4 quadrature x22.569 gearing",
    )
    args = parser.parse_args()
    if not math.isfinite(args.counts_per_rev) or args.counts_per_rev <= 0:
        parser.error("counts-per-rev must be positive and finite")
    raise SystemExit(0 if analyze(args.log, args.counts_per_rev) else 1)
