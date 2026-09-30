"""Run one E2 sweep over serial and write serial.log. Requires pyserial.

Opening the port may reboot the board, so a handshake first waits for a
'# STATUS' reply with running=0 configured=1 before sending 'start'.
"""

import argparse, re, shutil, sys, time
from datetime import datetime, timezone
from pathlib import Path

import serial

HERE = Path(__file__).parent


def read_lines(port, seconds):
    end = time.monotonic() + seconds
    buf = b""
    while time.monotonic() < end:
        buf += port.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield line.decode("ascii", "replace").strip()


def handshake(port, timeout=20.0):
    deadline = time.monotonic() + timeout
    port.write(b"!\n")  # cancel any sweep still running
    while time.monotonic() < deadline:
        port.write(b"status\n")
        for line in read_lines(port, 0.7):
            m = re.match(r"# STATUS configured=(\d) running=(\d)", line)
            if not m:
                continue
            configured, running = m.groups()
            if running == "1":
                port.write(b"stop\n")
            elif configured == "0":
                port.write(b"config\n")
                time.sleep(1.5)
            else:
                return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--command", choices=["start", "pwm"], default="start")
    ap.add_argument("--range", nargs=3, type=int, metavar=("MIN", "MAX", "INC"))
    ap.add_argument("--deadzone", type=int, help="send 'deadzone N' during handshake")
    a = ap.parse_args()
    out = a.out or HERE / "results" / datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ"
    )
    out.mkdir(parents=True, exist_ok=True)

    with serial.Serial(a.port, 115200, timeout=0.05, dsrdtr=False) as port:
        port.dtr = True
        port.rts = False
        time.sleep(1.5)  # allow a possible reboot to finish
        port.reset_input_buffer()
        if not handshake(port):
            sys.exit("Handshake failed: no idle, configured STATUS reply")
        port.reset_input_buffer()
        if a.deadzone is not None:
            port.write(f"deadzone {a.deadzone}\n".encode())
            time.sleep(1.0)
            port.reset_input_buffer()
        cmd = a.command + (" %d %d %d" % tuple(a.range) if a.range else "")
        port.write(cmd.encode() + b"\n")
        lines, ended = [], False
        deadline = time.monotonic() + 100
        while not ended and time.monotonic() < deadline:
            for line in read_lines(port, 0.5):
                if re.match(r"(#|step,|\d)", line):
                    lines.append(line)
                    print(line) if line.startswith("#") else None
                    if line.startswith("# END"):
                        ended = True
                        break
    (out / "serial.log").write_text("\n".join(lines) + "\n")
    meta = HERE / "results" / "20260930T043851Z" / "metadata.json"
    if meta.exists() and not (out / "metadata.json").exists():
        shutil.copy(meta, out / "metadata.json")
    print(f"{len(lines)} lines -> {out}")
    sys.exit(0 if ended else 1)


if __name__ == "__main__":
    main()
