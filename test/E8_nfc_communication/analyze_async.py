"""Summarize measured real-device logs; this does not emulate or test hardware."""
import collections
import json
import pathlib
import statistics
import sys
import argparse
import zipfile


def percentile(values, fraction):
    """Use the nearest-rank empirical quantile; return None for an empty population."""
    import math
    return sorted(values)[max(0, math.ceil(len(values) * fraction) - 1)] if values else None


def summarize(path, raw_text=None, raw_location=None):
    """Parse summary and retained per-request samples from one completed or partial capture."""
    summary = None
    rows = []
    options = {}
    metadata = {}
    histogram = []
    text = path.read_text(errors="replace") if raw_text is None else raw_text
    for line in text.splitlines():
        if line.startswith("SUMMARY "):
            summary = json.loads(line.removeprefix("SUMMARY "))
        elif line.startswith("METADATA "):
            metadata = json.loads(line.removeprefix("METADATA "))
        elif line.startswith("SERVICE_HIST_4US,"):
            histogram = [int(value) for value in line.split(",")[1:]]
        elif line.startswith("OPTIONS "):
            options.update({key: int(value) for key, value in (part.split("=") for part in line.split()[1:])})
        elif line and line[0].isdigit() and "," in line:
            rows.append([int(value) for value in line.split(",")])
    if not summary:
        return {"file": path.name, "status": "no summary"}
    summary.update(options)
    latencies = [row[0] for row in rows if row[2] == 1]
    failure_details = collections.Counter(tuple(row[2:]) for row in rows if row[2] != 1)
    summary.update({"file": path.name, "raw_location": raw_location or str(path), "metadata": metadata,
                    "uid_per_second": round(summary["uid_reads"] * 1e6 / summary["elapsed_us"], 2),
                    "uid_latency_mean_us": round(statistics.mean(latencies), 2) if latencies else None,
                    "uid_latency_p50_us": percentile(latencies, 0.5),
                    "uid_latency_p95_us": percentile(latencies, 0.95),
                    "uid_latency_p99_us": percentile(latencies, 0.99),
                    "uid_latency_max_us": max(latencies) if latencies else None,
                    "first_uid_latency_us": latencies[0] if latencies else None,
                    "service_time_fraction": summary["service_total_us"] / summary["elapsed_us"],
                    "failure_details": [{"fields": list(key), "count": count} for key, count in failure_details.most_common(5)]})
    for fraction in (0.5, 0.95, 0.99):
        threshold = sum(histogram) * fraction
        cumulative = 0
        for bucket, count in enumerate(histogram):
            cumulative += count
            if cumulative >= threshold:
                upper = summary["max_service_us"] if bucket == 255 else min(bucket * 4 + 3, summary["max_service_us"])
                summary[f"service_p{int(fraction * 100)}_upper_us"] = upper
                break
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    paths = args.paths
    if paths:
        summaries = [summarize(path) for path in paths]
    else:
        folder = pathlib.Path(__file__).resolve().parents[2] / "src/e8_nfc_fm17550_i2c/results"
        summaries = [summarize(path, raw_location=f"raw/{path.name}") for path in sorted((folder / "raw").glob("*.log"))]
        archive = folder / "archive/iterations.zip"
        if archive.exists():
            with zipfile.ZipFile(archive) as bundle:
                for name in sorted(bundle.namelist()):
                    if name.endswith(".log"):
                        summaries.append(summarize(pathlib.Path(name), bundle.read(name).decode(errors="replace"),
                                                   f"archive/iterations.zip#{name}"))
        # Initial communication logs use a different format; their raw bytes remain in the manifest.
        summaries = sorted((entry for entry in summaries if "status" not in entry), key=lambda entry: entry["file"])
    if args.output:
        args.output.write_text(json.dumps(summaries, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {len(summaries)} summaries to {args.output}")
    else:
        for summary in summaries:
            print(json.dumps(summary))
