"""Bounded real-hardware E16 smoke check; never flashes or resets either device."""
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
parser.add_argument('--run', action='store_true', help='Explicitly authorize lifted-wheel movement')
arguments = parser.parse_args()
if not arguments.run:
    parser.error('--run required; lift the wheels before running')
directory = Path(__file__).parent / 'results' / datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
directory.mkdir(parents=True)
metadata = {'complete': False, 'car': 'COM13', 'modem': 'COM11', 'phases': []}
metadata['git_revision'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
metadata['git_status'] = subprocess.check_output(['git', 'status', '--short'], text=True)
metadata['environment'] = 'e16_car_1'
metadata['speed_scale'] = 'Encoder nominal mm/s: 66.5 mm wheel, 150:1 gear, 9 pulses/rotor revolution'
ports = {}
buffers = {}
sequence = 0
raw = (directory / 'serial.log').open('w', encoding='utf-8')


def open_port(name):
    """Set physical line levels before opening, without pulsing RTS."""
    connection = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=1)
    connection.dtr = True
    connection.rts = False
    connection.port = name
    connection.open()
    ports[name] = connection
    buffers[name] = b''
    return connection


def send_world(movement):
    """Send the existing seven-byte World State through the unchanged modem."""
    global sequence
    sequence = (sequence + 1) & 65535
    command = b'W' + struct.pack('<HB4B', sequence, 0, movement, 0, 0, 0).hex().encode() + b'\n'
    ports['COM11'].write(command)


def phase(name, movement, duration):
    """Bound command duration, collect original serial lines and decoded telemetry."""
    record = {'name': name, 'movement': movement, 'robots': [], 'status': []}
    metadata['phases'].append(record)
    deadline = time.monotonic() + duration
    next_world = next_status = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if movement is not None and now >= next_world:
            send_world(movement)
            next_world = now + .02
        if now >= next_status:
            ports['COM13'].write(b'STATUS\n')
            next_status = now + .2
        for name_port, connection in ports.items():
            buffers[name_port] += connection.read(connection.in_waiting)
            while b'\n' in buffers[name_port]:
                line, buffers[name_port] = buffers[name_port].split(b'\n', 1)
                text = line.decode('utf-8', errors='replace').strip()
                raw.write(f'{now:.6f} {name} {name_port} {text}\n')
                if name_port == 'COM11' and re.fullmatch('[0-9a-fA-F]{42}', text):
                    packet = struct.unpack('<BHHB4sHBBBhhH', bytes.fromhex(text))
                    record['robots'].append([now, *packet[:4], packet[4].hex(), *packet[5:]])
                if '# STATUS:' in text:
                    record['status'].append(text)
        time.sleep(.002)
    raw.flush()
    print(name, 'packets=', len(record['robots']), 'status=', record['status'][-1:] , flush=True)
    return record


try:
    open_port('COM13')
    open_port('COM11')
    ports['COM13'].write(b'STOP\nHELP\n')
    initial = phase('connect_stopped', 0, 12)
    if not initial['robots']:
        raise RuntimeError('No Robot State via COM11; refusing movement')
    for name, movement in [('manual_forward', 2), ('manual_reverse', 4),
                           ('manual_left', 8), ('manual_right', 16),
                           ('manual_both_turns', 24), ('manual_both_directions', 6),
                           ('auto_center', 3), ('auto_left', 11),
                           ('auto_right', 19), ('auto_both_turns', 27),
                           ('auto_forward_disabled', 9)]:
        phase(name, movement, 2)
        phase(name + '_stop', 0, .8)
    phase('timeout_start', 2, 1.5)
    phase('world_timeout', None, 2.5)
    phase('resume_reverse', 4, 2)
    phase('final_stop', 0, 2)
    metadata['complete'] = True
except BaseException as error:
    metadata['error'] = repr(error)
    raise
finally:
    # Stop through both paths and let the existing car timeout cover modem expiry.
    if 'COM11' in ports:
        try:
            for _ in range(5):
                send_world(0)
                time.sleep(.02)
        except Exception as error:
            metadata['modem_cleanup_error'] = repr(error)
    if 'COM13' in ports:
        try:
            ports['COM13'].write(b'STOP\n')
        except Exception as error:
            metadata['car_cleanup_error'] = repr(error)
    for connection in ports.values():
        connection.close()
    raw.close()
    (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print('Evidence:', directory, flush=True)
