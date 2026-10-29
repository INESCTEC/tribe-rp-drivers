# GNSS LC29H (Quectel)

Driver for the Quectel LC29H GNSS module (tested with the LC29H EA, dual-antenna RTK with heading), and the tools
to configure the module and check that everything works.

1. [Overview](#1-overview)
2. [Example: epochs and RTK corrections](#2-example-epochs-and-rtk-corrections)
3. [Epochs](#3-epochs)
4. [RTK corrections](#4-rtk-corrections)
5. [Simple use without epochs](#5-simple-use-without-epochs)
6. [Counters](#6-counters)
7. [Module configuration](#7-module-configuration)
8. [Debug tools](#8-debug-tools)
9. [Testing](#9-testing)
10. [Reference](#10-reference)

---

## 1. Overview

The module prints NMEA sentences on a UART, several per fix (10 fixes/s on the LC29H EA), and takes RTK
corrections (RTCM 3) on the same UART. The driver:

- **owns the UART** (`begin()`): it sets it up with the FIFOs on, keeps the received bytes in its own ring and
  owns the UART interrupt;
- **parses the sentences** as they arrive and groups those of the same fix into an **epoch** (position, heading,
  position error), kept in a queue until the firmware takes it;
- **sends corrections** without blocking: `writeCorrections()` queues them and the UART interrupt sends them;
- **counts** everything that can go wrong (bytes lost, checksum errors, epochs incomplete or dropped).

| File | What |
|---|---|
| `software/drivers/include/rp_agrolib_lc29h.h` | API (every method is documented there) |
| `software/drivers/src/rp_agrolib_lc29h.cpp` | implementation |
| `software/examples/src/rp_lc29h_debug.cpp` | debug firmware (section 8) |
| `software/scripts/lc29h/` | PC tools: bench run, analysis, its test (sections 8, 9) |

Two libraries are built from the same source:

| Library | Use |
|---|---|
| `rp_agrolib_lc29h` | production |
| `rp_agrolib_lc29h_debug` | debugging only: adds timing fields and the debug line formats (`LC29H_DEBUG_TIMING`, section 8) |

---

## 2. Example: epochs and RTK corrections

A firmware that takes the module's fixes as epochs and passes RTK corrections to the module. Here the corrections
come from USB (e.g. a PC or phone running an NTRIP client); from a radio or another UART it is the same call.

```cpp
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "rp_agrolib_lc29h.h"

#define GNSS_RX_PIN 1
#define GNSS_TX_PIN 0
#define GNSS_BAUDRATE 460800

static LC29HDriver gnss(uart0);
static struct repeating_timer parse_timer;

// Timer interrupt, every 10 ms: parse the bytes received since the last call and close the epochs of the
// fixes that are complete. Closed epochs wait in the driver's queue.
static bool parse_cb(struct repeating_timer *) {
    gnss.processStreamEpochs(to_ms_since_boot(get_absolute_time()));
    return true;
}

// Main loop: take the oldest closed epoch, if any (see "Why interrupts are disabled" below)
static bool take_epoch(GnssEpoch *e) {
    uint32_t irq = save_and_disable_interrupts();
    bool got = gnss.takeEpoch(e);
    restore_interrupts(irq);
    return got;
}

// Main loop: pass whatever correction bytes arrived on USB to the module. writeCorrections() only copies
// them into the driver's TX ring and returns; the UART interrupt sends them
static void forward_corrections() {
    static uint8_t buf[512];
    uint16_t n = 0;
    int c;
    while (n < sizeof(buf) && (c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        buf[n++] = (uint8_t) c;
    }
    if (n > 0) {
        gnss.writeCorrections(buf, n);
    }
}

int main() {
    stdio_init_all();

    // Settings first, while no interrupt uses the driver yet
    gnss.setEpochSentences(GNSS_SENTENCE_DEFAULT);  // GGA + PQTMTAR; GNSS_SENTENCE_ALL if the module sends $PQTMEPE
    if (!gnss.begin(GNSS_RX_PIN, GNSS_TX_PIN, GNSS_BAUDRATE)) {
        printf("UART0 interrupt already in use\n");
    }
    add_repeating_timer_ms(-10, parse_cb, NULL, &parse_timer);

    while (true) {
        GnssEpoch e;
        while (take_epoch(&e)) {
            if (e.flags_ & GNSS_VALID_POSITION) {
                printf("fix %u utc %lu ms: %.8f %.8f quality %u\n", e.seq_, (unsigned long) e.utc_ms_, e.latitude_,
                       e.longitude_, e.fix_quality_);
            }
            if (e.flags_ & GNSS_VALID_HEADING) {
                printf("  heading %.2f deg, quality %u\n", e.heading_, e.heading_quality_);
            }
        }
        forward_corrections();
    }
}
```

Link `rp_agrolib_lc29h` (it brings `rp_agrolib_uart` and `pico_stdlib`).

### Why interrupts are disabled around `takeEpoch()`
The epoch queue is used from two places at once:
- **`processStreamEpochs()` runs in the timer interrupt** and adds epochs to the queue: it writes an epoch into
  the queue's array and increments its count (or, when the queue is full, drops the oldest: moves the head and
  decrements the count).
- **`takeEpoch()` runs in the main loop** and removes the oldest epoch: it copies it out (a `GnssEpoch` is about
  100 bytes, copied in many steps), then moves the head and decrements the count.

An interrupt can stop the main loop between any two instructions. If the timer interrupt runs while
`takeEpoch()` is half way through:
- the epoch being copied can be overwritten mid-copy when the queue is full, and the main loop gets half of one
  fix and half of another;
- both sides update the count with a read-modify-write (`count = count - 1` in the main loop, `count = count + 1`
  in the interrupt). If the interrupt runs between the main loop's read and its write, the main loop writes back a
  stale value: the interrupt's change is lost and the queue's count no longer matches its contents (an epoch
  vanishes or an old one comes back).

`save_and_disable_interrupts()` stops interrupts on this core for the few microseconds `takeEpoch()` takes, so
the copy and the count update happen as one step. `restore_interrupts()` puts back the previous state (it does not
blindly enable: if interrupts were already off, they stay off). An interrupt that becomes due meanwhile is not
lost: it runs right after `restore_interrupts()`.

Keep the section short: the UART interrupt waits too, and the FIFO gives it ~700 µs of margin at 460800 baud.
Copy the epoch out and do the work (printing, sending) after `restore_interrupts()`, as above.

When it is **not** needed:
- if `processStreamEpochs()` is called from the main loop instead of an interrupt (then nothing runs at the same
  time as `takeEpoch()`);
- for `writeCorrections()`: it protects its own data inside the driver;
- for `stats()`: each counter is a 32-bit value, read in one instruction.

The same applies to settings: `setEpochSentences()` and `setEpochTimeoutMs()` change what the interrupt reads, so
call them before starting the timer (as above), or with interrupts off.

**Two cores:** `save_and_disable_interrupts()` only acts on the core that calls it. Run `processStreamEpochs()`
and `takeEpoch()` on the same core, or protect them with a `critical_section` (`pico/critical_section.h`), which
works across cores.

### Rules
- **Call `begin()` once**, before anything else. It returns `false` if the UART's interrupt already has a
  handler; the firmware must not install its own (`uart_enable_interrupt()`) on that UART. One driver per UART.
- **Parse regularly**, every ~10 ms. At 460800 baud and 10 Hz the module sends ~3 KB/s; the 1000-byte RX ring
  holds ~0.3 s.
- **Use one parse call:** `processStreamEpochs()` or `processStream()`, not both (each one consumes the ring).
- **Nothing else may write to this UART's TX**, or its bytes get mixed into the corrections.

---

## 3. Epochs

An epoch is one fix: the sentences the module prints for it, parsed into one `GnssEpoch`.

| Sentence | Gives | `GnssEpoch` fields |
|---|---|---|
| `$GNGGA` | time, position, fix quality, satellites | `utc_ms_`, `latitude_`, `longitude_`, `fix_quality_`, `num_satellites_` |
| `$PQTMTAR` | heading from the two antennas | `heading_`, `heading_quality_`, `heading_acc_`, `baseline_m_` |
| `$PQTMEPE` | estimated position error | `epe_north_`, `epe_east_`, `epe_down_` |

- **Only use a field if its flag is set** in `flags_` (`GNSS_VALID_POSITION`, `_HEADING`, `_HEADING_ACC`, `_EPE`,
  `_TIME`). For example, position is valid only with a fix (quality ≥ 1); heading only with heading quality ≥ 1.
- `sentences_` says which sentences arrived (`GNSS_SENTENCE_GGA`, `_PQTMTAR`, `_PQTMEPE`).
- `seq_` numbers the closed epochs (+1 each, wraps at 255): a gap means an epoch was dropped.
- Fix quality (`fix_quality_`, `heading_quality_`): 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float.

### How an epoch closes
A GGA opens the epoch; the other sentences join the open one. It closes, with `close_reason_`:

| `close_reason_` | When |
|---|---|
| `GNSS_CLOSE_COMPLETE` | every sentence of the set arrived (normal) |
| `GNSS_CLOSE_NEXT_GGA` | the next fix's GGA arrived first |
| `GNSS_CLOSE_REPEAT` | a second copy of a sentence arrived (e.g. this fix's GGA was lost) |
| `GNSS_CLOSE_TIMEOUT` | the timeout passed since it opened (default 90 ms) |

Closed epochs wait in a queue of 8 for `takeEpoch()` (oldest first). If the queue is full, the oldest is dropped
and counted (`stats().overwritten`).

### The sentence set
`setEpochSentences(mask)` says which sentences make up an epoch. GGA is always included.

| Set | Use |
|---|---|
| `GNSS_SENTENCE_DEFAULT` (GGA + PQTMTAR), **default** | position + heading; the sentences used before PQTMEPE |
| `GNSS_SENTENCE_ALL` (+ PQTMEPE) | also the position error. The module must be configured to send `$PQTMEPE` (section 7) |
| `GNSS_SENTENCE_GGA` | position only, no wait for the heading |

The set must match what the module sends:

| Set (driver) | Module sends | Epoch closes | Result |
|---|---|---|---|
| default | GGA + TAR | at TAR, ~50 ms after GGA | position + heading |
| default | GGA + TAR + EPE | at TAR | EPE ignored (counted in `stats()`, never in the epoch) |
| ALL | GGA + TAR + EPE | at the last of the three | position + heading + EPE |
| ALL | GGA + TAR (no EPE) | **timeout, 90 ms after GGA** | works, ~40 ms later; every close is `TIMEOUT`: the mismatch shows in the counters |
| GGA only | anything | at GGA | position only |

### The timeout
`setEpochTimeoutMs(ms)`, default `GNSS_EPOCH_TIMEOUT_MS` (90 ms). It must be **above** the delay between GGA and
the last sentence of the set (PQTMTAR arrives ~50 ms after GGA on the LC29H EA) and **below** the fix period
(100 ms at 10 Hz). Change it only if the module runs at another rate.

---

## 4. RTK corrections

```cpp
gnss.writeCorrections(frame, len);   // len up to 65535; returns at once
```

After `begin()`, the frame is copied into the driver's TX ring (`LC29H_TX_RING_SIZE`, 8 KB) and the UART
interrupt sends it (~22 ms per 1 KB at 460800 baud) while the program goes on. If the ring has no room, the
**whole frame is dropped** and counted (`stats().rtcm_dropped`): a lost correction is replaced by the next one
(~1 s later), and at ~1 KB/s in and ~46 KB/s out this is never expected.

Without `begin()`, `writeCorrections()` behaves as before: it writes the frame and waits until it is sent.

---

## 5. Simple use without epochs

```cpp
RtkState rtk = {};
gnss.processStream(&rtk);   // after begin(), every ~10 ms
```
Fills `RtkState` from the newest GGA, PQTMTAR and VTG (speed). Each field holds the newest sentence **of its
type**: right after a GGA, `heading_` may still be from the previous fix (PQTMTAR arrives ~50 ms later). Use
epochs when position and heading must belong to the same fix.

---

## 6. Counters

`stats()` returns `LC29HStats`, cumulative since start (32-bit reads are atomic on the RP2040).

| Counter | Means | Should be |
|---|---|---|
| `rx_bytes` | bytes received from the module | ~3 KB/s at 10 Hz |
| `uart_overrun` | bytes lost in the UART (its FIFO was full: interrupts blocked too long) | 0 |
| `uart_framing_break` | framing/break errors (wrong baud rate, wiring) | 0 |
| `rx_dropped` | bytes dropped: RX ring full (parse calls too rare) | 0 |
| `lines`, `long_lines` | lines assembled; lines too long (garbage) | `long_lines` 0 |
| `ok_*`, `bad_*` | sentences with a valid / failed checksum, by type | `bad_*` 0 |
| `closed[GNSS_CLOSE_*]` | epochs closed, by reason | mostly `COMPLETE` |
| `overwritten` | epochs dropped before `takeEpoch()` (queue full) | 0 |
| `rtcm_bytes`, `rtcm_dropped` | correction bytes queued; frames dropped (TX ring full) | `rtcm_dropped` 0 |

---

## 7. Module configuration

The driver does not configure the module: the module's settings are saved in its own flash, and are set once
with a script, over a serial port connected to the module (default 460800 baud).

The sentence set (section 3) must match what the module sends. Two configurations:

| Configuration | For | Commands |
|---|---|---|
| **default** | `GNSS_SENTENCE_DEFAULT` | `$PQTMRESTOREPAR*13` (factory settings), `$PQTMCFGMSGRATE,W,PQTMTAR,1,1*09` (heading on every fix), `$PQTMSAVEPAR*5A` (save) |
| **with EPE** | `GNSS_SENTENCE_ALL` | the above, plus `$PQTMCFGMSGRATE,W,PQTMEPE,1,2*1D` (`$PQTMEPE` version 2 on every fix) before `$PQTMSAVEPAR*5A` |

Useful extras:
- `$PQTMCFGMSGRATE,W,PQTMEPE,0,2*1C` turns `$PQTMEPE` off explicitly (then `$PQTMSAVEPAR*5A`).
- Read a setting back: `$PQTMCFGMSGRATE,R,PQTMEPE,2*05` (answer `$PQTMCFGMSGRATE,OK,PQTMEPE,1,2*4E` when on),
  `$PQTMCFGMSGRATE,R,PQTMTAR,1*11`.
- The module answers every command (e.g. `$PQTMCFGMSGRATE,OK*29`); the scripts print every line received.
- A module set up "with EPE" keeps sending `$PQTMEPE` after a power cycle until it is reconfigured. If it still
  sends it after the default configuration, also send the "off" command above.

Scripts: `configure_lc29he_heading.py` in the tl.heart repository (`scripts/`). It asks for the serial port and
baud rate and sends the commands of one of the configurations above. Check what the module sends afterwards with
the debug firmware in sniffer mode (section 8).

---

## 8. Debug tools

Everything here is for testing and debugging; nothing of it is needed in production.

### Debug library
Link `rp_agrolib_lc29h_debug` instead of `rp_agrolib_lc29h`. It defines `LC29H_DEBUG_TIMING` for the library and
the firmware alike (it changes the struct layout: never define it only in the firmware). It adds:

- in `GnssEpoch`: `opened_ms_`, `closed_ms_`, `tar_after_ms_`, `epe_after_ms_` (when each sentence arrived);
- in `LC29HStats`: `irq_max_us` (longest UART interrupt), `rx_ring_max_pending` (fullest RX ring);
- `formatEpochDebugLine()` and `formatStatsDebugLines()`: the lines below, written into a buffer. The driver
  never prints; the firmware decides where the lines go.

### Debug firmware: `rp_lc29h_debug`
`software/examples/src/rp_lc29h_debug.cpp`, built as two examples:

| Example | Mode | Shows |
|---|---|---|
| `rp_lc29h_debug` | epoch | the driver: one `E` line per epoch, `D*` counters every 5 s |
| `rp_lc29h_sniffer` | sniffer (`LC29H_DEBUG_SNIFFER`) | the module: every line it sends, with the time of its end; no driver |

Comparing the two separates "the module does not send it" from "the driver loses it". Both take corrections from
USB, framed as the phone app does (`$=` + 2-byte little-endian length + payload), so a PC can play the phone.
The epoch mode also takes `$#<d>` (d = 1..7, `GNSS_SENTENCE_*` bits) to change the sentence set without
reflashing.

- **Default UART:** uart0, TX pin 0, RX pin 1, 460800 baud. Change with `UART_ID`, `UART_TX_PIN`, `UART_RX_PIN`,
  `UART_BAUDRATE`.
- **Boards:** the example only starts USB (`lc29h_debug_board_start()`, weak). A board that needs more before USB
  works (e.g. USB-C power delivery) builds the example from its own repository, with a file that defines
  `lc29h_debug_board_start()`.
- **Build:** `software/scripts/build.sh`, then flash `rp_lc29h_debug.uf2` / `rp_lc29h_sniffer.uf2` (BOOTSEL +
  copy to `RPI-RP2`).

### Lines on USB
Epoch mode:

`E,seq,utc_ms,sentences,flags,close_reason,opened_ms,tar_after_ms,epe_after_ms,close_after_open_ms,take_after_close_ms,done_after_take_ms`

One per epoch. `sentences`/`flags` in hex; 65535 = not received; times in the RP2040's ms (modulo 10⁷). The last
two fields come from the firmware: in the example `done` = 0 (printed); a firmware that sends epochs elsewhere
reports how long that took, or −1 if it did not send.

Every 5 s (counters cumulative, maxima since the previous report):

| Line | Fields |
|---|---|
| `DU` | t, rx_bytes, uart_overrun, uart_framing_break, rx_ring_max_pending, irq_max_us |
| `DS` | t, lines, ok_gga, ok_tar, ok_epe, ok_other, long_lines |
| `DB` | t, checksum failed: gga, tar, epe, other |
| `DC` | t, closed: complete, next_gga, repeat, timeout; overwritten |
| `DL` | t, rx_dropped, rtcm_bytes, rtcm_dropped |

Sniffer mode: `N,<t_us>,<NMEA line>`, and every 5 s
`D,t_us,bytes,lines,uart_overrun,uart_framing_break,ring_overflow,ring_max,too_long,rtcm_frames,rtcm_bytes`.

### PC tools: `software/scripts/lc29h/`
Python 3 with `pyserial`, `pyyaml`, `pandas`, `numpy`.

| Script | What |
|---|---|
| `gnss_bench.py` | the PC plays the phone: NTRIP client to the caster → corrections over USB; records every USB line (`usb.log`, `ntrip.log`, `meta.yaml`) |
| `gnss_bench_analyze.py` | one run → `report.yaml` + `epochs.csv` (epoch mode) or `fixes.csv` (sniffer mode); several runs → a table |
| `test_gnss_bench_analyze.py` | synthetic runs with known losses: the analyzer must find every one (`ALL PASSED`) |

NTRIP settings go in a file outside any repository, e.g. `~/.config/mx01/ntrip.yaml` (`host`, `port`,
`mountpoint`, `user`, `password`).

```bash
cd software/scripts/lc29h
python3 gnss_bench.py --ntrip-check 15        # caster reachable? RTCM message types received
python3 gnss_bench.py --port /dev/ttyACM0 --step "C all rtcm on" --firmware rp_lc29h_debug \
    --duration 300 --out ~/gnss_runs --sentences 7 [--no-ntrip]
python3 gnss_bench_analyze.py ~/gnss_runs/<run> [~/gnss_runs/<run> ...]
python3 test_gnss_bench_analyze.py
```

`--out` is required and must be outside the repository: runs are data, not code.

Board-specific stages (e.g. CAN and a ROS bridge on a robot) extend the scripts without copying them:
`gnss_bench.run(args, on_start, on_stop)` and `gnss_bench_analyze.analyze(run, extend)`.

---

## 9. Testing

Each step is one ~5 min run (`--duration 300`), module antennas outdoors, then `gnss_bench_analyze.py`. Module
configured "with EPE" (section 7) unless noted.

| Step | Firmware | Corrections | Checks | Pass when |
|---|---|---|---|---|
| A | `rp_lc29h_sniffer` | on | what the module sends | `sentences_per_s`: GNGGA, PQTMTAR, PQTMEPE ≈ 10; `missing_pct_of_bursts` ≈ 0; overrun 0 |
| B | `rp_lc29h_debug`, `--sentences 3` | on | default set | `close_reasons` all `complete`; no epoch with flag 0x08 (EPE); PQTMTAR `arrival_after_gga_ms` ≈ 50 |
| C | `rp_lc29h_debug`, `--sentences 7` | on | full set, corrections | all `complete`; criteria below; RTK float/fixed reached |
| D | `rp_lc29h_debug`, `--sentences 7`, module in the **default** configuration | on | mismatch shows | every close `timeout`; `ok.epe` = 0 |
| E | `rp_lc29h_debug` | off (`--no-ntrip`) | without corrections | criteria below |

Criteria for an epoch-mode run (`report.yaml`):

| Key | Pass | If not |
|---|---|---|
| `epochs_closed_by_rp2040` = `fixes_expected_from_utc` | equal | fixes never parsed: see `uart_overrun`, `rx_ring_dropped` |
| `rp2040_counters_over_run.uart_overrun` | 0 | the UART interrupt is blocked too long (other interrupts, long interrupts-off sections) |
| `rp2040_counters_over_run.rx_ring_dropped` | 0 | parse calls too rare |
| `epochs_not_reported`, `overwritten_before_taken` | 0 | the firmware takes epochs too slowly |
| `checksum_failed` | 0 | noise, baud rate, wiring |
| `close_reasons` | `complete` only (a few `next_gga` / `repeat` at start-up) | `timeout` everywhere: sentence set ≠ module configuration |
| `rtcm_dropped` | 0 | corrections faster than the UART |
| `arrival_after_gga_ms.PQTMTAR.p95` | < timeout (90 ms) | raise the timeout (section 3) |
| `rp2040_maxima_over_run.uart_irq_max_us` | tens of µs | a few hundred µs is still within the FIFO's ~700 µs |

Reading other numbers:
- `sentences_missing_pct_of_reported`: per sentence, the share of epochs without it.
- `latency_ms.close_after_open_p95`: how long an epoch waits for its last sentence (~50 ms with PQTMTAR).
- In sniffer mode, `uart_load_pct` is how busy the UART is (~6 % at 10 Hz).

---

## 10. Reference

### Methods (details in the header)

| Method | |
|---|---|
| `LC29HDriver(uart_inst_t *uart)` | constructor |
| `bool begin(rx_pin, tx_pin, baud)` | UART (8N1, FIFOs), rings, interrupt |
| `uint8_t processStreamEpochs(now_ms)` | parse, close epochs; returns how many closed |
| `bool takeEpoch(GnssEpoch *)` | oldest closed epoch |
| `void setEpochSentences(mask)`, `epochSentences()` | sentence set (default GGA + PQTMTAR) |
| `void setEpochTimeoutMs(ms)` | epoch timeout (default 90 ms) |
| `void processStream(RtkState *)` | parse into `RtkState` (no epochs) |
| `void writeCorrections(data, size)` | queue corrections (non-blocking after `begin()`) |
| `const LC29HStats &stats()` | counters |
| `formatEpochDebugLine()`, `formatStatsDebugLines()` | debug lines (`rp_agrolib_lc29h_debug` only) |
| `getNMEA()` | deprecated: use `begin()` + `processStream()` |
| `getPQMT()`, `getPair()` | unchanged |

### Build-time options
They change the size of `LC29HDriver` or `GnssEpoch`: set them on the library target
(`target_compile_definitions(rp_agrolib_lc29h PUBLIC ...)`), never only in the firmware.

| Define | Default | |
|---|---|---|
| `LC29H_RX_RING_SIZE` | 1000 | bytes of module output kept for the parser (~0.3 s at 10 Hz) |
| `LC29H_TX_RING_SIZE` | 8192 | bytes of corrections waiting for the UART (~8 s at ~1 KB/s) |
| `LC29H_DEBUG_TIMING` | off | debug fields and line formats; use the `rp_agrolib_lc29h_debug` library |

### Constants
`GNSS_EPOCH_TIMEOUT_MS` 90 (default timeout), `GNSS_EPOCH_QUEUE_MAX` 8 (queue).
