#!/usr/bin/env python3
"""Synthetic runs with known losses: gnss_bench_analyze.py must attribute every one.   python3 test_gnss_bench_analyze.py"""
import os
import tempfile

import yaml

import gnss_bench_analyze as A

FAILS = []


def check(name, got, want):
    if got != want:
        FAILS.append('%s: got %r, want %r' % (name, got, want))


def nmea(body):
    cs = 0
    for c in body:
        cs ^= ord(c)
    return '$%s*%02X' % (body, cs)


def epoch_run(d):
    """600 fixes at 10 Hz, sequence starting at 200 (wraps), as rp_lc29h_debug prints them (done = 0)."""
    gga_lost = {50, 51, 120, 300, 301, 450, 500, 510, 560, 590}    # epoch of PQTMTAR + PQTMEPE only, no time
    overwritten = {70, 200, 333, 444, 555}                          # closed, never taken
    timeout = {10, 20, 30}
    t0, utc0 = 1000.0, 57600000
    usb = ['%.6f\t#step synthetic' % t0]
    for k in range(600):
        seq, utc, t = (200 + k) % 256, utc0 + 100 * k, t0 + 0.1 * k
        lost_gga = k in gga_lost
        sent, flags = (6, 0x0e) if lost_gga else (7, 0x1f)
        if k in overwritten:
            continue
        reason = 4 if k in timeout else 1
        usb.append('%.6f\tE,%d,%d,%x,%x,%d,%d,%d,%d,%d,%d,0' % (t + 0.08, seq, 0 if lost_gga else utc, sent, flags, reason,
                                                                int(t * 1000) % 10**7, 50, 65535 if lost_gga else 5, 50, 1))
    for j, t in enumerate([t0 + 5 * i for i in range(13)]):
        usb.append('%.6f\tDU,%d,%d,%d,0,120,15' % (t, 1000 * j, 20000 * j, 2 * j))
        usb.append('%.6f\tDC,%d,%d,0,%d,0,%d' % (t, 1000 * j, 50 * j, 2 * j, j))
        usb.append('%.6f\tDL,%d,0,%d,0,450,10200' % (t, 1000 * j, 900 * j))
    for j in range(60):                                                 # S lines: one state mismatch at the end
        usb.append('%.6f\tS,%d,%d,1f,4,41.1,-8.5,133.20,0.009,90.50,%d,%d,0,%d'
                   % (t0 + j, 1000 * j, j % 256, 10 * j, j // 20, 1 if j == 59 else 0))
    open(os.path.join(d, 'usb.log'), 'w').write('\n'.join(usb) + '\n')
    yaml.safe_dump({'step': 'synthetic', 'firmware': 'rp_lc29h_debug'}, open(os.path.join(d, 'meta.yaml'), 'w'))


def sniffer_run(d):
    """300 fixes: GGA, EPE at +5 ms, 6 GSV lines, PQTMTAR at +50 ms; 7 PQTMTAR dropped, 3 GGA corrupted, 2 GGA dropped."""
    usb = ['0.0\tI,rp_lc29h_debug,sniffer']
    tar_dropped, gga_bad, gga_dropped = {10, 20, 30, 40, 50, 60, 70}, {100, 110, 120}, {200, 250}
    for k in range(300):
        t = 4294000000 + 100000 * k                                 # us, wraps past 2^32 during the run
        utc = '%06d.%d00' % (160000 + k // 10, k % 10)
        lines = []
        if k not in gga_dropped:
            g = nmea('GNGGA,%s,4110.6,N,00835.7,W,4,30,0.5,90,M,50,M,1,0' % utc)
            lines.append((0, g[:-2] + '00' if k in gga_bad else g))
        lines.append((5000, nmea('PQTMEPE,2,0.004,0.003,0.005,0.005,0.007')))
        lines += [(10000 + 3000 * i, nmea('GPGSV,6,%d,24,01,40,083,46' % (i + 1))) for i in range(6)]
        if k not in tar_dropped:
            lines.append((50000, nmea('PQTMTAR,1,%s,4,,0.86,1.1,,50.9,0.25,,0.01,21' % utc)))
        usb += ['0.0\tN,%d,%s' % ((t + dt) % 2 ** 32, line) for dt, line in lines]
    usb.append('0.0\tD,5000000,123456,3000,0,0,0,800,0,30,28000')
    open(os.path.join(d, 'usb.log'), 'w').write('\n'.join(usb) + '\n')
    yaml.safe_dump({'step': 'sniffer synthetic', 'firmware': 'rp_lc29h_sniffer'}, open(os.path.join(d, 'meta.yaml'), 'w'))


with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
    epoch_run(d1)
    r = A.analyze(d1)
    check('stages', r['stages'], {'sent': 595})
    check('fixes expected', r['fixes_expected_from_utc'], 600)
    check('epochs closed', r['epochs_closed_by_rp2040'], 600)
    check('epochs not reported', r['epochs_not_reported'], 5)
    check('close reasons', r['close_reasons'], {'complete': 592, 'timeout': 3})
    check('GGA missing %', r['sentences_missing_pct_of_reported']['GGA'], round(100 * 10 / 595, 2))
    check('UART overrun over run', r['rp2040_counters_over_run']['uart_overrun'], 24)
    check('overwritten counter', r['rp2040_counters_over_run']['overwritten_before_taken'], 12)
    check('rtcm bytes', r['rp2040_counters_over_run']['rtcm_bytes'], 10800)
    check('EPE after GGA p50', r['arrival_after_gga_ms']['PQTMEPE']['p50'], 5.0)
    check('parse max', r['rp2040_maxima_over_run']['parse_max_us'], 450)
    check('newest epoch checks', r['newest_epoch_checks'],
          {'checks': 590, 'skipped_newer_epoch': 2, 'epoch_mismatch': 0, 'state_mismatch': 1})

    sniffer_run(d2)
    s = A.analyze(d2)
    check('sniffer bursts', s['bursts'], 298)                       # each dropped GGA merges two fixes
    check('sniffer merged fixes', s['bursts_with_duplicate_sentences'], 2)
    check('sniffer checksum failed', s['checksum_failed_per_type'], {'GNGGA': 3})
    check('sniffer TAR after GGA p50', s['arrival_after_gga_ms']['PQTMTAR']['p50'], 50.0)
    check('sniffer EPE after GGA p50', s['arrival_after_gga_ms']['PQTMEPE']['p50'], 5.0)

print('\n'.join(FAILS) if FAILS else 'ALL PASSED')
raise SystemExit(1 if FAILS else 0)
