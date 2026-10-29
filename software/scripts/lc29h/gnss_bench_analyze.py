#!/usr/bin/env python3
"""Analysis of gnss_bench.py runs (README.md): what does the module send, and what does the driver make of it?

    python3 gnss_bench_analyze.py <run> [<run> ...]

Per run, from what its usb.log contains:
  epoch lines (E, DU/DS/DB/DC/DL from LC29HDriver's debug formats): every epoch the driver closed, its
    sentences, close reason and timing; fixes expected from the UTC times; the driver counters.
    Writes epochs.csv (one row per epoch) and report.yaml.
  sniffer lines (N/D, rp_lc29h_debug in sniffer mode): what the module sends per fix: sentences, checksum
    errors, bytes, arrival offsets. Writes fixes.csv and report.yaml.
Several runs: also a table of the main numbers per run (the test matrix).
Board-specific stages (e.g. CAN on a robot) extend analyze() through its `extend` argument.
"""
import os
import re
import sys

import numpy as np
import pandas as pd
import yaml

GGA, TAR, EPE = 1, 2, 4                       # GnssEpoch::sentences_
CLOSE = {1: 'complete', 2: 'next_gga', 3: 'repeat', 4: 'timeout'}
FIX_PERIOD_MS = 100
BAUD_BYTES_S = 460800 / 10


def read_usb(run):
    rows = []
    for line in open(os.path.join(run, 'usb.log'), errors='replace'):
        t, _, text = line.rstrip('\n').partition('\t')
        rows.append((float(t), text))
    return rows


def unwrap(values, modulo):
    out, base, prev = [], 0, None
    for v in values:
        if prev is not None and v < prev:
            base += modulo
        out.append(v + base)
        prev = v
    return np.array(out)


def counters(rows, prefix, names):
    """D lines of one kind -> DataFrame (first value is the RP2040 time, the rest cumulative or maxima)."""
    data = [[float(x) for x in text.split(',')[1:]] for _, text in rows if text.startswith(prefix + ',')]
    return pd.DataFrame(data, columns=names) if data else pd.DataFrame(columns=names)


def delta(df, col):
    return int(df[col].iloc[-1] - df[col].iloc[0]) if len(df) > 1 else None


def pct(n, d):
    return round(100.0 * n / d, 2) if d else None


# ------------------------------------------------------------------------------------------------ epoch run
def read_epochs(rows):
    """E lines -> DataFrame. The last two fields come from the firmware: take_after_close, done_after_take (-1: not sent)."""
    E = [text.split(',')[1:] for _, text in rows if text.startswith('E,')]
    e = pd.DataFrame(E, columns=['seq', 'utc_ms', 'sentences', 'flags', 'reason', 'opened', 'tar_after', 'epe_after',
                                 'close_dt', 'take_dt', 'done_dt']).astype(
        {'seq': int, 'utc_ms': int, 'reason': int, 'opened': int, 'tar_after': int, 'epe_after': int,
         'close_dt': int, 'take_dt': int, 'done_dt': int})
    e['sentences'] = e.sentences.apply(lambda x: int(x, 16))
    e['flags'] = e['flags'].apply(lambda x: int(x, 16))
    e['t_pc'] = [t for t, text in rows if text.startswith('E,')]
    e['seq_u'] = unwrap(e.seq.values, 256)
    e['close_reason'] = e.reason.map(CLOSE)
    e['stage'] = np.where(e.done_dt >= 0, 'sent', 'not sent (robot not connected)')
    return e


def epoch_run(run, rows, meta):
    e = read_epochs(rows)
    timed = e[(e.sentences & GGA) > 0]
    fixes_expected = int((timed.utc_ms.max() - timed.utc_ms.min()) // FIX_PERIOD_MS + 1) if len(timed) else 0
    closed = int(e.seq_u.max() - e.seq_u.min() + 1)

    du = counters(rows, 'DU', ['t', 'bytes', 'overrun', 'framing_break', 'ring_max', 'uart_irq_max_us'])
    ds = counters(rows, 'DS', ['t', 'lines', 'ok_gga', 'ok_tar', 'ok_epe', 'ok_other', 'long_lines'])
    db = counters(rows, 'DB', ['t', 'bad_gga', 'bad_tar', 'bad_epe', 'bad_other'])
    dc = counters(rows, 'DC', ['t', 'complete', 'next_gga', 'repeat', 'timeout', 'overwritten'])
    dl = counters(rows, 'DL', ['t', 'rx_dropped', 'rtcm_bytes', 'rtcm_dropped'])

    rep = {
        'step': meta.get('step'), 'firmware': meta.get('firmware'),
        'duration_s': round(e.t_pc.iloc[-1] - e.t_pc.iloc[0], 1),
        'fixes_expected_from_utc': fixes_expected,
        'epochs_closed_by_rp2040': closed,
        'epochs_reported': int(len(e)),
        'epochs_not_reported': closed - len(e),
        'stages': None,                                                  # filled in by analyze()
        'sentences_missing_pct_of_reported': {'GGA': pct(((e.sentences & GGA) == 0).sum(), len(e)),
                                              'PQTMTAR': pct(((e.sentences & TAR) == 0).sum(), len(e)),
                                              'PQTMEPE': pct(((e.sentences & EPE) == 0).sum(), len(e))},
        'close_reasons': e.close_reason.value_counts().to_dict(),
        'arrival_after_gga_ms': {k: {'p50': float(np.median(v)), 'p95': float(np.percentile(v, 95)), 'max': int(v.max())}
                                 for k, v in (('PQTMTAR', e.tar_after[(e.tar_after != 65535) & ((e.sentences & GGA) > 0)]),
                                              ('PQTMEPE', e.epe_after[(e.epe_after != 65535) & ((e.sentences & GGA) > 0)]))
                                 if len(v)},
        'latency_ms': {'close_after_open_p95': float(np.percentile(e.close_dt, 95)),
                       'take_after_close_p95': float(np.percentile(e.take_dt, 95)),
                       'done_after_take_p95': float(np.percentile(e.done_dt[e.done_dt >= 0], 95)) if (e.done_dt >= 0).any() else None},
        'rp2040_counters_over_run': {
            'uart_bytes': delta(du, 'bytes'), 'uart_overrun': delta(du, 'overrun'),
            'uart_framing_break': delta(du, 'framing_break'),
            'lines': delta(ds, 'lines'), 'long_lines': delta(ds, 'long_lines'),
            'ok': {k: delta(ds, 'ok_' + k) for k in ('gga', 'tar', 'epe', 'other')},
            'checksum_failed': {k: delta(db, 'bad_' + k) for k in ('gga', 'tar', 'epe', 'other')},
            'closed': {k: delta(dc, k) for k in ('complete', 'next_gga', 'repeat', 'timeout')},
            'overwritten_before_taken': delta(dc, 'overwritten'),
            'rx_ring_dropped': delta(dl, 'rx_dropped'), 'rtcm_bytes': delta(dl, 'rtcm_bytes'),
            'rtcm_dropped': delta(dl, 'rtcm_dropped')},
        'rp2040_maxima_over_run': {
            'ring_max_pending_bytes': int(du.ring_max.max()) if len(du) else None,
            'uart_irq_max_us': int(du.uart_irq_max_us.max()) if len(du) else None},
    }
    return e, rep


# ------------------------------------------------------------------------------------------------ sniffer run
def nmea_ok(line):
    m = re.match(r'^\$([^*]*)\*([0-9A-Fa-f]{2})$', line)
    if not m:
        return False
    cs = 0
    for c in m.group(1):
        cs ^= ord(c)
    return cs == int(m.group(2), 16)


def sniffer_run(run, rows, meta):
    lines = []
    for _, text in rows:
        if text.startswith('N,'):
            _, t_us, line = text.split(',', 2)
            lines.append((int(t_us), line))
    t = unwrap([x[0] for x in lines], 2 ** 32) / 1000.0                  # ms
    df = pd.DataFrame({'t_ms': t, 'line': [x[1] for x in lines]})
    df['type'] = df.line.str.extract(r'^\$([A-Z0-9]+)')[0].fillna('?')
    df['ok'] = df.line.apply(nmea_ok)
    df['bytes'] = df.line.str.len() + 2
    # A fix = from one GGA line (any checksum) to the next; a gap > 80 ms also starts one (fix period 100 ms),
    # so a fix whose GGA was lost does not swallow the next one
    df['burst'] = ((df.type == 'GNGGA') | (df.t_ms.diff().fillna(1e9) > 80)).cumsum()
    fixes = []
    for b, g in df.groupby('burst'):
        gga = g[(g.type == 'GNGGA') & g.ok]
        t0 = gga.t_ms.iloc[0] if len(gga) else g.t_ms.iloc[0]
        row = {'burst': b, 't_ms': g.t_ms.iloc[0], 'lines': len(g), 'bytes': int(g.bytes.sum()),
               'duration_ms': g.t_ms.iloc[-1] - g.t_ms.iloc[0], 'bad_checksum': int((~g.ok).sum()),
               'utc': gga.line.iloc[0].split(',')[1] if len(gga) else None,
               'duplicate_sentences': int(g[g.ok & g.type.isin(['GNGGA', 'PQTMTAR', 'PQTMEPE'])].type.duplicated().sum())}
        for name in ('GNGGA', 'PQTMTAR', 'PQTMEPE'):
            s = g[(g.type == name) & g.ok]
            row['has_' + name] = len(s) > 0
            row['after_gga_ms_' + name] = s.t_ms.iloc[0] - t0 if len(s) else np.nan
        fixes.append(row)
    f = pd.DataFrame(fixes)
    f.to_csv(os.path.join(run, 'fixes.csv'), index=False)
    d = counters(rows, 'D', ['t_us', 'bytes', 'lines', 'overrun', 'framing_break', 'ring_overflow', 'ring_max', 'too_long',
                             'rtcm_frames', 'rtcm_bytes'])
    span_s = (df.t_ms.iloc[-1] - df.t_ms.iloc[0]) / 1000.0
    missing = lambda s: round(100.0 * (~s).mean(), 2)
    return {
        'step': meta.get('step'), 'firmware': meta.get('firmware'), 'duration_s': round(span_s, 1),
        'bursts': int(len(f)), 'bursts_per_s': round(len(f) / span_s, 2),
        'bursts_with_duplicate_sentences': int((f.duplicate_sentences > 0).sum()),
        'sentences_per_s': {k: round(v / span_s, 2) for k, v in df[df.ok].type.value_counts().items()},
        'checksum_failed_per_type': df[~df.ok].type.value_counts().to_dict(),
        'missing_pct_of_bursts': {k: missing(f['has_' + k]) for k in ('GNGGA', 'PQTMTAR', 'PQTMEPE')},
        'arrival_after_gga_ms': {k: {'p50': round(float(f['after_gga_ms_' + k].median()), 1),
                                     'p95': round(float(f['after_gga_ms_' + k].quantile(.95)), 1)}
                                 for k in ('PQTMTAR', 'PQTMEPE')},
        'burst_bytes': {'p50': int(f.bytes.median()), 'max': int(f.bytes.max())},
        'burst_duration_ms_p95': round(float(f.duration_ms.quantile(.95)), 1),
        'uart_load_pct': round(100.0 * df.bytes.sum() / span_s / BAUD_BYTES_S, 1),
        'sniffer_counters_over_run': {c: delta(d, c) for c in ('overrun', 'framing_break', 'ring_overflow', 'too_long',
                                                               'rtcm_frames', 'rtcm_bytes')},
        'sniffer_ring_max_bytes': int(d.ring_max.max()) if len(d) else None,
    }


def native(o):
    """numpy / pandas numbers and keys -> plain Python, for YAML."""
    if isinstance(o, dict):
        return {native(k): native(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [native(v) for v in o]
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return None if np.isnan(o) else float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def analyze(run, extend=None):
    """extend(run, rows, meta, epochs, report): optional, adds board-specific stages and counters to an epoch run."""
    rows = read_usb(run)
    meta = yaml.safe_load(open(os.path.join(run, 'meta.yaml'))) if os.path.exists(os.path.join(run, 'meta.yaml')) else {}
    if any(text.startswith('E,') for _, text in rows):
        e, rep = epoch_run(run, rows, meta)
        if extend:
            extend(run, rows, meta, e, rep)
        rep['stages'] = e.stage.value_counts().to_dict()
        e.to_csv(os.path.join(run, 'epochs.csv'), index=False)
    elif any(text.startswith('N,') for _, text in rows):
        rep = sniffer_run(run, rows, meta)
    else:
        raise SystemExit('%s: no E (epoch) or N (sniffer) lines in usb.log' % run)
    rep = native(rep)
    yaml.safe_dump(rep, open(os.path.join(run, 'report.yaml'), 'w'), sort_keys=False)
    return rep


def print_matrix(reps, delivered=lambda r: r['stages'].get('sent', 0)):
    print('| run | step | firmware | fixes | lost | GGA miss % | PQTMTAR miss % | PQTMEPE miss % | UART overrun |')
    print('|---|---|---|---|---|---|---|---|---|')
    for run, r in reps.items():
        if 'fixes_expected_from_utc' in r:
            m = r['sentences_missing_pct_of_reported']
            print('| %s | %s | %s | %d | %d | %s | %s | %s | %s |' % (os.path.basename(run), r['step'], r['firmware'],
                  r['fixes_expected_from_utc'], r['fixes_expected_from_utc'] - delivered(r), m['GGA'], m['PQTMTAR'],
                  m['PQTMEPE'], r['rp2040_counters_over_run']['uart_overrun']))
        else:
            m = r['missing_pct_of_bursts']
            print('| %s | %s | %s | %d | - | %s | %s | %s | %s |' % (os.path.basename(run), r['step'], r['firmware'],
                  r['bursts'], m['GNGGA'], m['PQTMTAR'], m['PQTMEPE'], r['sniffer_counters_over_run']['overrun']))


def main():
    reps = {}
    for run in sys.argv[1:]:
        reps[run] = analyze(run)
        print('== %s\n%s' % (run, yaml.safe_dump(reps[run], sort_keys=False)))
    if len(reps) > 1:
        print_matrix(reps)


if __name__ == '__main__':
    main()
