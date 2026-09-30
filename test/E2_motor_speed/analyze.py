"""Analyze E2 schema-2 logs. Encoder-based speed; no motor-driver UART required."""
import argparse
import csv
import io
import math
from pathlib import Path
from statistics import mean, pstdev


def delta(a, b):
    """Signed 32-bit counter difference, including positive/negative wraparound."""
    return ((int(b) - int(a) + 2**31) % 2**32) - 2**31


def fields(line):
    return dict(token.split("=", 1) for token in line.split() if "=" in token)


def analyze(log_path):
    lines = log_path.read_text(encoding="utf-8-sig").splitlines()
    starts = [i for i, line in enumerate(lines) if line.startswith("# START ")]
    if len(starts) != 1:
        raise ValueError("Expected exactly one E2 START record")
    start = fields(lines[starts[0]])
    if start.get("schema") != "2":
        raise ValueError("Requires new E2 schema=2 firmware; historical result files are unchanged")
    expected_start = dict(min="-100", max="100", inc="10", steps="21",
                          baseline_ms="1000", settle_ms="1000", measure_ms="2000", rest_ms="1000")
    if any(start.get(k) != v for k, v in expected_start.items()):
        raise ValueError("Unexpected sweep definition")
    configs = [fields(line) for line in lines[:starts[0]] if line.startswith("# SETTINGS ")]
    if not configs:
        raise ValueError("Missing settings record")
    config = configs[-1]
    expected_config = dict(type="1", deadzone="1650", lines="2000", ratio="23",
                           diameter_mm="65.0", pid="3,0.375,0.5", pid_source="stored_unverified")
    if any(config.get(k) != v for k, v in expected_config.items()):
        raise ValueError("Unexpected motor configuration")
    # Nominal conversion only: 2000 counts/motor rev x 23 gear ratio.
    # Keep counts/s alongside this estimate until a physical revolution is calibrated.
    scale = math.pi * float(config["diameter_mm"]) / (int(config["lines"]) * int(config["ratio"]))
    ends = [i for i in range(starts[0] + 1, len(lines)) if lines[i].startswith("# END ")]
    end_index = ends[0] if ends else len(lines)
    records = [s for s in lines[starts[0]+1:end_index] if s and not s.startswith("#")]
    if not records or not records[0].startswith("step,command,"):
        raise ValueError("Missing CSV header")
    data = list(csv.DictReader(io.StringIO("\n".join(records))))
    if not data or any(None in r or None in r.values() for r in data):
        raise ValueError("Empty or malformed sample rows")
    # Always retain the samples before rejecting a partial sweep.
    with log_path.with_name("samples.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(data[0]))
        writer.writeheader()
        writer.writerows(data)
    targets = list(range(-100, 101, 10))
    groups = {target: [] for target in targets}
    previous_step, previous_ms = -1, -1
    for row in data:
        command, step, elapsed = int(row["command"]), int(row["step"]), int(row["step_ms"])
        if command not in groups or step != targets.index(command):
            raise ValueError("Unexpected command/step")
        if step < previous_step or (step == previous_step and elapsed <= previous_ms):
            raise ValueError("Samples out of order or repeated")
        previous_step, previous_ms = step, elapsed
        phase = "baseline" if elapsed < 1000 else "settle" if elapsed < 2000 else "measure" if elapsed < 4000 else "rest"
        if not 0 <= elapsed < 5000 or row["phase"] != phase:
            raise ValueError("Invalid phase/timing")
        if int(row["applied"]) != (command if phase in ("settle", "measure") else 0):
            raise ValueError("Unexpected applied command")
        groups[command].append(row)
    summaries = []
    for target, trial in groups.items():
        for phase, low, high in [("baseline", 0, 1000), ("settle", 1000, 2000),
                                 ("measure", 2000, 4000), ("rest", 4000, 5000)]:
            segment = [r for r in trial if r["phase"] == phase]
            if len(segment) < 2 or int(segment[0]["step_ms"]) > low+100 or int(segment[-1]["step_ms"]) < high-100:
                raise ValueError(f"Incomplete {phase} coverage at {target}")
        rows = [r for r in trial if r["phase"] == "measure"]
        result = dict(command=target, measurement_samples=len(rows))
        for motor, timestamp in [("m2", "t2_us"), ("m4", "t4_us")]:
            velocities = []
            for a, b in zip(rows, rows[1:]):
                dt = (int(b[timestamp])-int(a[timestamp])) % 2**32 / 1e6
                if not 0 < dt <= 0.1:
                    raise ValueError("Invalid timestamp or excessive measurement gap")
                velocities.append(delta(a[motor+"_count"], b[motor+"_count"])/dt)
            window = (int(rows[-1][timestamp])-int(rows[0][timestamp])) % 2**32 / 1e6
            cps = delta(rows[0][motor+"_count"], rows[-1][motor+"_count"])/window
            result.update({motor+"_window_s": window, motor+"_counts_s": cps,
                           motor+"_counts_s_std": pstdev(velocities),
                           motor+"_estimated_mm_s": cps*scale,
                           motor+"_estimated_mm_s_std": pstdev(velocities)*scale,
                           motor+"_estimated_error": cps*scale-target,
                           motor+"_recent_mean": mean(int(r[motor+"_recent"]) for r in rows)})
        summaries.append(result)
    end = fields(lines[end_index]) if ends else {}
    clean = all(end.get(k) == v for k, v in dict(reason="complete", dropped="0", missed_ticks="0",
                errors="0", stop_ack="1", release_ack="1").items()) and end.get("rows") == str(len(data))
    with log_path.with_name("summary.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    report = ["# E2 commanded versus encoder-measured speed", "",
              "Capture status: " + ("complete" if clean else "INCOMPLETE OR DEGRADED"), "",
              "PID 3/0.375/0.5 is assumed stored, not read back over I2C.",
              "Measurements use 2-4 s of each trial (1-3 s after applying the target).",
              "Nominal mm/s = counts/s * pi * 65 / (2000 * 23); physical scaling is NOT calibrated.",
              "No extra quadrature factor is applied to the 2000 setting. Command units/signs remain subject to calibration.",
              "Variation is standard deviation of sample-to-sample encoder velocities, not uncertainty of the mean.", "",
              "| Command | M2 counts/s | M4 counts/s | M2 estimated mm/s | M4 estimated mm/s |",
              "| --- | --- | --- | --- | --- |"]
    for row in summaries:
        report.append(f"| {row['command']} | {row['m2_counts_s']:.2f} | {row['m4_counts_s']:.2f} | {row['m2_estimated_mm_s']:.2f} | {row['m4_estimated_mm_s']:.2f} |")
    report += ["", "End record: " + (lines[end_index] if ends else "missing"), ""]
    log_path.with_name("summary.md").write_text("\n".join(report), encoding="utf-8")
    print(f"Analyzed {len(data)} samples: {'complete' if clean else 'degraded'}")
    return clean


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    raise SystemExit(0 if analyze(parser.parse_args().log) else 1)
