"""Modem-only bounded AUTO track experiment with raw, incomplete-run evidence."""
import argparse
import datetime
import json
from pathlib import Path
import re
import struct
import subprocess
import time
import serial

parser = argparse.ArgumentParser()
parser.add_argument('--run', action='store_true', help='Explicitly command AUTO forward on the prepared track')
parser.add_argument('--seconds', type=float, default=5)
arguments = parser.parse_args()
if not arguments.run or not 0 < arguments.seconds <= 30:
    parser.error('Use --run and a duration in (0, 30] seconds')
directory = Path(__file__).parent / 'results' / datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ_track')
directory.mkdir(parents=True)
metadata = {'complete': False, 'modem': 'COM11', 'seconds': arguments.seconds, 'phases': []}
metadata['environment'] = 'e16_car_1'
metadata['git_revision'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
metadata['git_status'] = subprocess.check_output(['git', 'status', '--short'], text=True)
metadata['speed_scale'] = 'Encoder nominal mm/s: 66.5 mm wheel, 150:1 gear, 9 pulses/rotor revolution'
connection = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=1)
connection.dtr = True
connection.rts = False
connection.port = 'COM11'
sequence = 0
buffer = b''
last_robot = None
last_robot_time = 0
raw = (directory / 'serial.log').open('w', encoding='utf-8')


def send_world(movement):
    """Use the unchanged wire packet, with every other car commanded stopped."""
    global sequence
    sequence = (sequence + 1) & 65535
    connection.write(b'W' + struct.pack('<HB4B', sequence, 0, movement, 0, 0, 0).hex().encode() + b'\n')


def phase(name, movement, seconds):
    """Stop on stale feedback or safety states during the bounded AUTO phase."""
    global buffer, last_robot, last_robot_time
    records = []
    metadata['phases'].append({'name': name, 'movement': movement, 'robots': records})
    deadline = time.monotonic() + seconds
    next_world = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if movement and (now - last_robot_time > .5 or last_robot[7] in (11, 12, 14) or last_robot[8] == 10):
            raise RuntimeError('AUTO stopped: stale feedback, recovery, battery protection or line loss')
        if now >= next_world:
            send_world(movement)
            next_world = now + .02
        buffer += connection.read(connection.in_waiting)
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            text = line.decode('utf-8', errors='replace').strip()
            raw.write(f'{now:.6f} {name} {text}\n')
            if re.fullmatch('[0-9a-fA-F]{42}', text):
                robot = struct.unpack('<BHHB4sHBBBhhH', bytes.fromhex(text))
                if robot[0] == 1:
                    last_robot = robot
                    last_robot_time = now
                    records.append([now, *robot[:4], robot[4].hex(), *robot[5:]])
        time.sleep(.002)
    raw.flush()
    print(name, 'packets', len(records), 'latest', records[-1:] , flush=True)
    return records


try:
    connection.open()
    initial = phase('preflight_stopped', 0, 5)
    if not initial or time.monotonic() - last_robot_time > .5:
        raise RuntimeError('No fresh car telemetry; no motion started')
    if not last_robot[6] or last_robot[4] == bytes(4):
        raise RuntimeError('Preflight needs visible line and a retained NFC UID; no motion started')
    if last_robot[11] < 10000 or last_robot[7] != 1:
        raise RuntimeError('Preflight needs safe battery and stopped MANUAL; no motion started')
    phase('auto_center', 3, arguments.seconds)
    final = phase('final_stop', 0, 2)
    if not final or any(abs(value) > 5 for value in final[-1][10:12]):
        raise RuntimeError('Final stopped wheel speeds not confirmed')
    metadata['complete'] = True
except BaseException as error:
    metadata['error'] = repr(error)
    raise
finally:
    if connection.is_open:
        try:
            # A zero command also clears the modem mailbox before releasing USB.
            for _ in range(10):
                send_world(0)
                time.sleep(.02)
        except Exception as error:
            metadata['cleanup_error'] = repr(error)
        connection.close()
    raw.close()
    (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print('Evidence:', directory, flush=True)
