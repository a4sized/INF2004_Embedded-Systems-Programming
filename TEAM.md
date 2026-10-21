# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub ID | Code | Tests |
|---|---|---|---|---|---|
| 1 | Wi-Fi & MQTT Telemetry: Aggregates sensor data, publishes JSON telemetry, decodes MQTT commands, CSV logging, and heartbeat LED. |2501283|ray15rk| `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | *example:* Producer: makes a sample every 300 ms | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | *example:* Consumer: collects samples into statistics | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | *example:* Monitor: prints reports, pauses the LED | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | Buddy 2 (2501820) | Motor 2 encoder A input |
| GP3 | Buddy 2 (2501820) | Motor 2 encoder B input |
| GP4 | Buddy 3 (2501107) | Right IR sensor DO |
| GP5 | | |
| GP6 | Buddy 3 (2501107), bench only | IR calibration DO; Maker Pi Pico Grove 4, GP6 |
| GP7 | Buddy 3 (2501107) | Left IR sensor DO; GP6 bench is separate |
| GP8 | | |
| GP9 | | |
| GP10 | | |
| GP11 | | |
| GP12 | | |
| GP13 | | |
| GP14 | Buddy 2 (2501820) | Motor 1 encoder A input |
| GP15 | Buddy 2 (2501820) | Motor 1 encoder B input |
| GP16 | Buddy 3 (2501107) | Barcode IR sensor DO |
| GP17 | | |
| GP18 | | |
| GP19 |S1| Heartbeat status LED (LED + 330 Ω to GND) |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | | (ADC0 capable) |
| GP27 | | (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | | GP26–GP28 |
| PWM slices (list) | | | |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | led_task – 12 – any |
| 2 | 14–15 | *example:* producer_task – 14 – core 1 |
| 3 | 16–17 | *example:* consumer_task – 16 – core 0 |
| 4 | 18–19 | *example:* monitor_task – 18 – any |
| 5 | 20–21 | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | Any -> S1 | Shared data under mutex (S1) | mtx_telemetry | S1_TELEMETRY struct containing speed, distance, pitch, faults and barcode via s1_update_telemetry() | 2501283
| IF-02 | S1 -> Any | Event flag | flg_s1_cmd | S1_FLG_CMD_START, STOP, ESTOP, CLEAR, PING bits set upon remote MQTT command | 2501283
| IF-03 | IwIp -> S1 | Message Buffer | mbf_mqtt_rx | Incoming raw string payloads mapped to S1_CMD_TYPE by led_task via s1_mqtt_push_rx() | 2501283
| IF-04 | S1 -> IwIP | Message Buffer | mbf_mqtt_tx | Formatted JSON telemetry string (96 bytes max) queued every 1 second using s1_mqttt_pop_tx() | 2501283
| IF-05 | S4 -> S1 | Task Control | tid_ed | tl_sus_tsk / ts_rsm_tsk / tk_cng_pri | 2501283
#| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
#| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
#| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
#| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
#| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *example* |
| IF-06 | | | | | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | |
| Semaphore | 16 | 0 | 1 | | | |
| Event flag | 16 | 1 | | | 1 | |
| Mailbox | 12 | 0 | | 1 | | |
| Mutex | 12 | 1 | | 1 | | |
| Message buffer | 12 | 2 | | | 1 | |
| Fixed memory pool | 12 | 0 | 1 | | | |
| Variable memory pool | 12 | 0 | | | | |
| Cyclic handler | 12 | 1 | | | | |
| Alarm handler | 12 | 0 | | | 1 | |

(The kernel's USB and WiFi service tasks also use a few of these.)

## 7. Standalone Buddy 3 calibration bench (13 October 2026)

`bench/ir_calibration/` is a separate one-sensor application for Buddy 3
(2501107). Its isolated build replaces the example application only inside
an ignored build staging tree; the example code and its tables above remain
unchanged. The user confirmed DO on GP6 through Grove 4 of a Maker Pi Pico
carrier with the Pico W seated on it. The current bench reads GP6; GP7
remains the intended robot left IR allocation. AO is disconnected; no ADC
is used.

| Bench task | Priority | Core | Stack | Kernel objects |
|---|---|---|---|---|
| `ir_bench_task` | 16 | core 0 (`TP_PRC1`) | 2048 bytes | One task |

Only that task reads/configures GP6. It uses the existing kernel USB console
service and creates no subsystem communication objects. The bench profile
disables Wi-Fi/MQTT and example tasks. These choices do not settle robot
priorities or cross-buddy interfaces. See `testing/subsystem3/IR_CALIBRATION_TEST_LOG.md` for evidence and
pending hardware checks.

## 8. Buddy 3 three-sensor digital readout (13 October 2026)

`app_program/subsystem3/` contains a sensing-only application for GP7 left,
GP4 right and GP16 barcode DO. `build_sensing.py` stages only Buddy 3 sources
in a temporary build tree; it does not start the example consumer, producer
or monitor. The output UF2 is placed in `build_make/`. The profile has a USB
console and disables Wi-Fi, lwIP and MQTT.

The standalone sensing build stages a 5 ms kernel tick and wakes the
sensing task with a 5 ms cyclic handler. It prints raw GPIO levels about
every 40 samples while a USB terminal is open. The project-wide
`config/config.h` remains at 10 ms. Compilation does not prove the design's
5 ms sampling and jitter target on hardware. The USB line includes min/max
sample gaps in microseconds, count within 4000–6000 us, and count above
10 ms over a rolling window of up to 12,000 intervals. On 21 October 2026, the user
reported black=1 and white=0 on GP7, GP4 and GP16, so the standalone image
maps a high level to BLACK on each channel.
See `testing/subsystem3/IR_SENSING_TEST_LOG.md`.

On 21 October 2026, Buddy 3 added a time-qualified filter to the two line
levels in this standalone readout. It accepts a changed level only after a
second observation at least 10 ms later; the barcode level remains raw.
The USB line now shows each line sensor as `raw>stable`. This bench value is
not a calibrated moving-robot filter setting. No motion request is sent.
The later steering interface must give Motion Control the latest correction
for its own cycle, subject to the team's unfinished queue contract.
The 20 ms Motion State cycle is a design target and is not present in the
current Buddy 2 application. The earlier GP16 encoder-power output was
removed on this local branch; GP16 belongs to Buddy 3's barcode input.
The isolated Buddy 3 UF2 still excludes Buddy 2 code.

The user found a narrow sweet spot with both line sensors black when centered.
Left black/right white points to a left correction; right black/left white
points to a right correction. Both black is centered/no correction. Both
white for 30 ms confirms potential loss; a lost state clears after both
black persists for 20 ms. The isolated readout updates this latest decision
every 5 ms and prints it over USB. The intended handoff is one latest
decision at each future Motion State cycle, without queueing every sample.
Moving-robot validation of the narrow centered margin remains outstanding.
