#!/usr/bin/env python3
"""Bench run of an LC29H GNSS module through an RP2040 (README.md): the PC plays the phone and records the USB lines.

One run = one test step, in its own folder under --out (keep runs outside the repository):
    usb.log      every line from the RP2040 over USB, prefixed with the PC time:   <unix time>\t<line>
    ntrip.log    corrections sent: <unix time>\t<frame bytes>  (and connection events)
    meta.yaml    step name, firmware, arguments, start/end

    python3 gnss_bench.py --port /dev/ttyACM0 --step "epochs rtcm on" --firmware rp_lc29h_debug --duration 300 \\
        --out ~/gnss_runs [--no-ntrip] [--sentences 7]

    python3 gnss_bench.py --ntrip-check 15      only connect to the caster and list the RTCM messages received

NTRIP settings: ~/.config/<robot>/ntrip.yaml (host, port, mountpoint, user, password); never in a repository.
Corrections are framed as the Android app does: "$=" + 2-byte little-endian length + the bytes of one socket
read, at most MAX_FRAME. Board-specific additions (e.g. CAN on a robot) use run()'s hooks.
"""
import argparse
import base64
import datetime
import os
import socket
import sys
import threading
import time

import serial
import yaml

MAX_FRAME = 4094


class Logger:
    def __init__(self, path):
        self.f = open(path, 'w', buffering=1)
        self.lock = threading.Lock()

    def write(self, text, t=None):
        with self.lock:
            self.f.write('%.6f\t%s\n' % (time.time() if t is None else t, text))


def ntrip_connect(cfg):
    """NTRIP v1 request as the app makes it; returns (socket, status, bytes already received after the header)."""
    s = socket.create_connection((cfg['host'], int(cfg['port'])), timeout=10)
    req = 'GET /%s HTTP/1.0\r\nUser-Agent: NTRIP gnss_bench\r\nAccept: */*\r\nConnection: close\r\n' % cfg['mountpoint']
    if cfg.get('user'):
        req += 'Authorization: Basic %s\r\n' % base64.b64encode(('%s:%s' % (cfg['user'], cfg.get('password', ''))).encode()).decode()
    s.sendall((req + '\r\n').encode())
    s.settimeout(20)
    head = s.recv(1024)
    status = head.split(b'\r\n')[0]
    if b'200' not in status:
        raise ConnectionError('caster refused: %r' % head[:80])
    return s, status, (head.split(b'\r\n\r\n', 1)[1] if b'\r\n\r\n' in head else b'')


def rtcm_messages(buf):
    """Complete RTCM 3 messages in buf -> ([(type, length)], rest). Frame: 0xD3, 6 bits 0, 10-bit length, payload, CRC24."""
    out = []
    while True:
        i = buf.find(b'\xd3')
        if i < 0:
            return out, b''
        buf = buf[i:]
        if len(buf) < 6:
            return out, buf
        n = ((buf[1] & 0x03) << 8) | buf[2]
        if len(buf) < 3 + n + 3:
            return out, buf
        out.append(((buf[3] << 4) | (buf[4] >> 4), n))
        buf = buf[3 + n + 3:]


def ntrip_check(cfg, seconds):
    s, status, data = ntrip_connect(cfg)
    print('caster %s:%s/%s -> %s' % (cfg['host'], cfg['port'], cfg['mountpoint'], status.decode(errors='replace')))
    counts, total, end, buf = {}, 0, time.time() + seconds, data
    while time.time() < end:
        chunk = s.recv(MAX_FRAME)
        if not chunk:
            break
        total += len(chunk)
        msgs, buf = rtcm_messages(buf + chunk)
        for t, n in msgs:
            counts[t] = counts.get(t, 0) + 1
    s.close()
    print('%d bytes in %.0f s (%.0f B/s); RTCM 3 messages (type: count):' % (total, seconds, total / seconds))
    for t in sorted(counts):
        print('  %4d: %d' % (t, counts[t]))


def ntrip_thread(cfg, ser, ser_lock, log, stop):
    """NTRIP v1 client: socket bytes -> '$=' frames on the serial port; reconnects on errors."""
    while not stop.is_set():
        try:
            s, status, rest = ntrip_connect(cfg)
            log.write('connected: %r' % status)
            while not stop.is_set():
                data = rest or s.recv(MAX_FRAME)
                rest = b''
                if not data:
                    raise ConnectionError('caster closed the connection')
                for i in range(0, len(data), MAX_FRAME):
                    chunk = data[i:i + MAX_FRAME]
                    frame = b'$=' + len(chunk).to_bytes(2, 'little') + chunk
                    with ser_lock:
                        ser.write(frame)
                    log.write(str(len(chunk)))
        except Exception as e:                                   # noqa: BLE001 - keep the run going
            log.write('error: %s' % e)
            time.sleep(2)


def usb_thread(ser, log, stop):
    buf = b''
    while not stop.is_set():
        data = ser.read(4096)
        t = time.time()
        if not data:
            continue
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            log.write(line.rstrip(b'\r').decode(errors='replace'), t)


def add_arguments(ap):
    ap.add_argument('--port', help='RP2040 USB serial port, e.g. /dev/ttyACM0')
    ap.add_argument('--step', help='test step name, e.g. "epochs rtcm on"')
    ap.add_argument('--firmware', help='firmware on the board, e.g. rp_lc29h_debug, rp_lc29h_sniffer')
    ap.add_argument('--ntrip-check', type=float, metavar='SECONDS', help='only test the caster connection')
    ap.add_argument('--duration', type=float, default=300.0, help='seconds')
    ap.add_argument('--no-ntrip', action='store_true', help='no corrections (test step without RTCM)')
    ap.add_argument('--ntrip-config', default=os.path.expanduser('~/.config/mx01/ntrip.yaml'))
    ap.add_argument('--sentences', type=int, choices=range(1, 8), metavar='1..7',
                    help='rp_lc29h_debug: epoch sentence set (GNSS_SENTENCE_* bits: 1 GGA, 2 PQTMTAR, 4 PQTMEPE)')
    ap.add_argument('--out', help='folder for the runs (outside the repository); required for a run')


def run(a, on_start=None, on_stop=None):
    """One run. on_start(out, meta) runs before the serial port is opened, on_stop(out, meta) after it is closed."""
    if a.ntrip_check:
        return ntrip_check(yaml.safe_load(open(a.ntrip_config)), a.ntrip_check)
    if not (a.port and a.step and a.firmware and a.out):
        raise SystemExit('--port, --step, --firmware and --out are required for a run')

    name = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_') + a.step.replace(' ', '_')
    out = os.path.join(os.path.expanduser(a.out), name)
    os.makedirs(out)
    meta = {'step': a.step, 'firmware': a.firmware, 'port': a.port, 'duration_s': a.duration,
            'ntrip': not a.no_ntrip, 'sentences': a.sentences, 'start': time.time()}
    print('run folder:', out)
    if on_start:
        on_start(out, meta)

    ser = serial.Serial(a.port, 115200, timeout=0.2)
    ser_lock = threading.Lock()
    stop = threading.Event()
    usb_log = Logger(os.path.join(out, 'usb.log'))
    usb_log.write('#step %s' % a.step)
    if a.sentences:
        ser.write(b'$#%d' % a.sentences)
    threads = [threading.Thread(target=usb_thread, args=(ser, usb_log, stop), daemon=True)]
    if not a.no_ntrip:
        cfg = yaml.safe_load(open(a.ntrip_config))
        threads.append(threading.Thread(target=ntrip_thread, args=(cfg, ser, ser_lock, Logger(os.path.join(out, 'ntrip.log')), stop),
                                        daemon=True))
    for t in threads:
        t.start()
    try:
        end = time.time() + a.duration
        while time.time() < end:
            time.sleep(1)
            print('\r%4.0f s left' % (end - time.time()), end='', flush=True)
        print()
    except KeyboardInterrupt:
        print('\nstopped early')
    stop.set()
    for t in threads:
        t.join(timeout=3)
    ser.close()
    meta['end'] = time.time()

    if on_stop:
        on_stop(out, meta)
    yaml.safe_dump(meta, open(os.path.join(out, 'meta.yaml'), 'w'), sort_keys=False)
    print('done:', out)


def main():
    ap = argparse.ArgumentParser()
    add_arguments(ap)
    return run(ap.parse_args())


if __name__ == '__main__':
    sys.exit(main())
