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
| 4 | IMU motion and terrain monitoring. `imu_task` reads an LSM303D accelerometer/magnetometer through `lsm303d` every 20 ms, calibrates it, and works out tilt, turn rate, humps with their peak height, and collisions; `report_task` decides the motion state and reports it. `subsystem4.c/.h` hold the start-up, shared data and the links to other subsystems: pitch, motion state and hump state go to Subsystem 1 through its setters, the IMU fault is reported to Subsystem 1 as FAULT_IMU_SENS, and data between the two tasks is packed into 8-byte messages. All tuning values are in `s4_config.h`. On-target self-tests are in `subsystem4_test.c`. |2500787|DariusKong| `app_program/subsystem4/` | `testing/subsystem4/` |
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
| GP2 | | |
| GP3 | | |
| GP4 | | |
| GP5 | | |
| GP6 | | |
| GP7 | | |
| GP8 | | |
| GP9 | | |
| GP10 | | |
| GP11 | | |
| GP12 | | |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | | |
| GP17 | | |
| GP18 | | |
| GP19 |S1| Heartbeat status LED (LED + 330 Ω to GND) |
| GP20 | S4 | I2C0 SDA, parked and unused: the kernel configures I2C0 at boot when `DEVCNF_USE_IIC` = 1, and the default GP8/GP9 are the motor 1 pins |
| GP21 | S4 | I2C0 SCL, parked and unused (same reason) |
| GP22 | | |
| GP26 | S4 | IMU (GY-511 board, LSM303D chip), I2C1 SDA (Grove 6) |
| GP27 | S4 | IMU (GY-511 board, LSM303D chip), I2C1 SCL (Grove 6) |
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
| I2C0 | S4 (parked) | none: unused. Kernel I2C driver switched ON (`DEVCNF_USE_IIC` = 1) | GP20, GP21 |
| I2C1 | S4 | `imu_task` | GP26 (SDA), GP27 (SCL) |
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
| 4 | 11, 18 (11 is outside the 18–19 band: needs team agreement) | imu_task – 11 – any; report_task – 18 – any |
| 5 | 20–21 | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | Any -> S1 | Shared data under mutex (S1) | mtx_telemetry | S1_TELEMETRY_PKT updated via Gatekeeper setters | 2501283
| IF-02 | S1 -> Any | Event flag | flg_s1_cmd | S1_FLG_CMD_START, STOP, ESTOP, CLEAR, PING bits set upon remote MQTT command | 2501283
| IF-03 | IwIp -> S1 | Message Buffer | mbf_mqtt_rx | Incoming 1-byte binary command payloads mapped to S1_CMD_TYPE by led_task via s1_mqtt_push_rx() | 2501283
| IF-04 | S1 -> IwIP | Message Buffer | mbf_mqtt_tx | Packed 8-byte binary telemetry payload (S1_TELEMETRY_PKT) queued every 1 second using s1_mqtt_pop_tx() | 2501283
| IF-05 | S4 -> S1 | Task Control | tid_led | tk_sus_tsk / tk_rsm_tsk / tk_chg_pri | 2501283
#| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
#| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
#| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
#| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
#| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *example* |
| IF-06 | S2 -> S4 | Function calls (S2) | `get_left_encoder_ticks()`, `get_right_encoder_ticks()` | Signed ticks; S4 uses the mean of both wheels times `S4_CM_PER_TICK` (`s4_config.h`) as distance in cm. Ticks must count up when driving forward. | 2500787 |
| IF-07 | S4 -> S1 | Function calls (S1) | `s1_report_fault(FAULT_IMU_SENS)`, `s1_clear_fault(FAULT_IMU_SENS)` | One IMU bit (bit 4), set while the IMU is not answering or a collision is active; collision released after `S4_COLLISION_HOLD_MS` (3 s, 0 = until `s4_clear_collision()` is called) | 2500787 |
| IF-08 | S4 -> S1 | Function call (S1) | `s1_csv_log_event()` | Event names `HUMP`, `COLLISION`, `IMU_FAULT`, `RUN_SUMMARY`, with a short details string (heights in millimetres) | 2500787 |
| IF-09 | S4 -> S1 | Function calls (S1) | `s1_set_pitch()`, `s1_set_car_state()` | pitch (8 bits signed, degrees, clipped); car_state (3 bits) = Subsystem 4's motion state 0..6; hump_state (2 bits): 0 flat, 1 going up, 2 going down | pending 2501283 |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 2 | |
| Semaphore | 16 | 0 | 1 | | 0 | |
| Event flag | 16 | 1 | | | 1 | |
| Mailbox | 12 | 0 | | 1 | 0 | |
| Mutex | 12 | 1 | | 1 | 0 | |
| Message buffer | 12 | 2 | | | 1 | |
| Fixed memory pool | 12 | 0 | 1 | | 0 | |
| Variable memory pool | 12 | 0 | | | 0 | |
| Cyclic handler | 12 | 1 | | | 0 | |
| Alarm handler | 12 | 0 | | | 0 | |

(The kernel's USB and WiFi service tasks also use a few of these.)
