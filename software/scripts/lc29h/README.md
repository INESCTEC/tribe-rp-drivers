# LC29H GNSS bench tools

Tools to see what an LC29H module sends and what `LC29HDriver` makes of it, with only the board, the module and
a PC. No robot, no CAN. They found the GNSS losses of 2026-10-23 (UART RX FIFO off, blocking correction writes,
a single epoch slot).

| Piece | What it is |
|---|---|
| `examples/src/rp_lc29h_debug.cpp` | Debug firmware. **Epoch mode** (`rp_lc29h_debug`): the driver's epochs and counters as `E` and `D*` lines. **Sniffer mode** (`rp_lc29h_sniffer`): every raw module line with its time, no driver |
| `gnss_bench.py` | The PC plays the phone. It is an NTRIP client to the caster, sends the corrections to the board over USB, and records every USB line |
| `gnss_bench_analyze.py` | Turns a run into `report.yaml` + `epochs.csv` / `fixes.csv` |
| `test_gnss_bench_analyze.py` | Synthetic runs with known losses: the analyzer must find every one |

Sniffer mode vs epoch mode separates "the module doesn't send it" from "the driver loses it".

## Build and flash
`software/scripts/build.sh` builds `examples/rp_lc29h_debug.uf2` and `rp_lc29h_sniffer.uf2`. Flash with BOOTSEL +
copy to `RPI-RP2`. The default UART is uart0, pins TX 0 / RX 1, 460800 baud. Change them with `UART_ID`,
`UART_TX_PIN`, `UART_RX_PIN`, `UART_BAUDRATE`.

The example only starts USB. A board that needs more before USB works (e.g. USB-C power delivery) builds
`rp_lc29h_debug.cpp` from its own repository, with a file that defines `lc29h_debug_board_start()`.

## Run
Python 3 with `pyserial`, `pyyaml`, `pandas`, `numpy`.

```bash
# NTRIP settings, never in a repository: ~/.config/mx01/ntrip.yaml with host, port, mountpoint, user, password
python3 gnss_bench.py --ntrip-check 15                       # caster reachable? RTCM message types
python3 gnss_bench.py --port /dev/ttyACM0 --step "epochs rtcm on" --firmware rp_lc29h_debug \
    --duration 300 --out ~/gnss_runs [--no-ntrip] [--sentences 7]
python3 gnss_bench_analyze.py ~/gnss_runs/<run> [~/gnss_runs/<run> ...]   # several runs: also a matrix
python3 test_gnss_bench_analyze.py
```

`--out` is required and should point outside the repository: runs are data, not code. `--sentences` sets the
epoch sentence set of `rp_lc29h_debug` (bits: 1 GGA, 2 PQTMTAR, 4 PQTMEPE; default 3). It must match what the
module is configured to send. A sentence in the set that never arrives makes every epoch close by timeout.

## Lines on USB
Epoch mode (formats from `LC29HDriver::formatEpochDebugLine()` / `formatStatsDebugLines()`, library
`rp_agrolib_lc29h_debug`):

- `E,seq,utc_ms,sentences,flags,close_reason,opened_ms,tar_after_ms,epe_after_ms,close_after_open_ms,take_after_close_ms,done_after_take_ms`.
  One per epoch. `sentences`/`flags` are hex. `close_reason`: 1 complete, 2 next GGA, 3 repeated sentence,
  4 timeout. 65535 = not received. The last two fields come from the firmware (here `done` = 0: printed; a
  firmware that sends epochs elsewhere reports when, or -1 if not sent).
- Every 5 s (counters cumulative, maxima since the previous report):

| Line | Fields |
|---|---|
| `DU` | t, rx_bytes, uart_overrun, uart_framing_break, rx_ring_max_pending, irq_max_us |
| `DS` | t, lines, ok_gga, ok_tar, ok_epe, ok_other, long_lines |
| `DB` | t, checksum failed: gga, tar, epe, other |
| `DC` | t, closed: complete, next_gga, repeat, timeout; overwritten (never taken) |
| `DL` | t, rx_dropped (RX ring full), rtcm_bytes, rtcm_dropped (TX ring full) |

Sniffer mode: `N,<t_us>,<NMEA line>` (time of the line's `\n`), and every 5 s
`D,t_us,bytes,lines,uart_overrun,uart_framing_break,ring_overflow,ring_max,too_long,rtcm_frames,rtcm_bytes`.

## Reading the report
- `fixes_expected_from_utc` vs `epochs_closed_by_rp2040`: fixes the driver never saw. The bytes were lost
  before parsing: check `uart_overrun` and `rx_ring_dropped`.
- `epochs_not_reported`: closed but never taken (`overwritten_before_taken`). The firmware takes too slowly.
- `sentences_missing_pct_of_reported` and `close_reasons`: `timeout` everywhere means the sentence set doesn't
  match the module configuration.
- `rtcm_dropped` > 0: corrections came in faster than the UART could send them (never expected).

Board-specific stages (e.g. CAN and a ROS bridge on a robot) extend the scripts: `gnss_bench.run()` takes start
and stop hooks, and `gnss_bench_analyze.analyze()` takes an `extend` function.
