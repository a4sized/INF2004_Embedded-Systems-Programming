# Subsystem 1 — test log

**Subsystem:** Wi-Fi & MQTT Telemetry, Telemetry Gatekeeper, Centralized Fault Manager & Heartbeat LED  ·  **Student ID:** `2501283` (`ray15rk`)  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`, `scripts/telemetry_decoder.py`

See `testing/README.md` for how to run tests and record results.

## Test layout note

- **On-target automated firmware tests (`S1-U01`–`S1-U10`)** stay in `app_program/subsystem1/subsystem1_test.c` because `build_make/mtkernel_3/app_program/subdir.mk` compiles `app_program/*/*.c` and `app_program/app_main.c` calls `subsystem1_test()` when built with `TEST=1`.
- **Host/PC unit test (`S1-U11`)** lives in `testing/subsystem1/test_telemetry_decoder.py` and checks packet bit-layout parity, `systemFaults.h` fault mapping, and `scripts/telemetry_decoder.py` without Pico hardware.
- **Manual hardware/network tests (`S1-U12`–`S1-U14`)** require `NET=lwip MQTT=1` with `config/wifi_secrets.h`, Mosquitto (`mosquitto_local.conf -v`), and `scripts/telemetry_decoder.py` / MQTTX. *(Starter template's manual LED item `S1-U03` was assigned to `S1-U12` so IDs do not collide with `S1-U03` already compiled in `subsystem1_test.c`.)*

## Test cases

Define a test here before running it. Test IDs never change.

### On-target automatic tests (`make TEST=1` / `make NET=lwip MQTT=1 TEST=1`)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | `cyc_led`, `led_task` (`tid_led`), `mbf_mqtt_tx`, `mbf_mqtt_rx`, `mtx_telemetry`, `flg_s1_cmd`, and `flg_s1_isr_fault` are created and active | RTOS init, `IF-01`–`IF-05` | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | `led_task` blocks in `TTS_WAI` between 250 ms cyclic ticks (≥ 9 of 10 polls at 97 ms) | Non-busy cyclic execution | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | Packed wire structs (`sizeof(S1_TELEMETRY_PKT) == 8`, `sizeof(S1_CMD_PKT) == 1`) and core Gatekeeper setters (`s1_set_speed`, `s1_set_obstacle`, `s1_set_pitch`, `s1_report_fault`, `s1_get_telemetry`) | `IF-01`, 8-byte payload | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | Inbound `S1_CMD_PING` pushed via `s1_mqtt_push_rx()` is popped by `led_task`, sets `S1_FLG_CMD_PING` on `flg_s1_cmd`, updates `s1_last_cmd`, and increments `s1_rx_commands` | `IF-02`, `IF-03` | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | Invalid binary command (`cmd_id = 7`) pushed via `s1_mqtt_push_rx()` is rejected, increments `s1_rx_rejected`, sets `FAULT_CMD_REJECTED`, and clears cleanly afterward | `IF-03`, Fault Manager | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | `s1_csv_log_event()` logs faulted (`TEST_FAULT`), `BARCODE_DECODED`, and `NULL`-argument calls without crashing or blocking | `IF-08`, CSV logging | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | Extended Gatekeeper setters (`s1_set_car_state`, `s1_set_line_mask`, `s1_set_barcode_id`), bit-isolated fault set/clear (`FAULT_WIFI \| FAULT_CMD_REJECTED`), and `s1_get_telemetry(NULL)` guardrails | `IF-01`, Fault Manager | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | `s1_parse_command()` decodes 1-byte binary `S1_CMD_START`, ASCII digits `'1'`/`'5'`, case-insensitive/whitespace-trimmed words (`START`, `stop\n`, `Estop\r\n`, `cLeAr`, `PING`), rejects invalid inputs (`0`, `6`, `INVALID`, `""`, `NULL`), and `s1_mqtt_push_rx()` rejects `NULL`/zero-len | `IF-02`, `IF-03` | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | Non-blocking ISR fault reporting (`s1_report_fault_isr(FAULT_WIFI)`) sets `flg_s1_isr_fault`, drains via `led_task` into `fault_mask`, and clears cleanly | ISR safety, Fault Manager | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | 1 Hz telemetry TX path increments `seq` and `s1_tx_queued` after `4 × 250 ms` ticks; `s1_mqtt_pop_tx()` enforces NULL/short-buffer bounds and pops 8-byte `S1_TELEMETRY_PKT` (or 0 if already popped by MQTT client) | `IF-04`, 1 Hz telemetry | auto | `app_program/subsystem1/subsystem1_test.c` | `[TEST] S1-U10 PASS` |

### PC / host automatic test (`testing/subsystem1/test_telemetry_decoder.py`)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U11 | Ground Station decoder (`scripts/telemetry_decoder.py`) matches `S1_TELEMETRY_PKT` 64-bit bitfield order in `subsystem1.h`, `systemFaults.h` bit names, signed negative pitch, barcode `'0'`/`'A'`/`'Z'`, fault masks, non-8-byte rejection, and real C struct layout (when host `gcc` is available) | Wire format parity, Ground Station | auto (PC) | Run `python testing/subsystem1/test_telemetry_decoder.py` | `Ran 6 tests ... OK` (or `skipped=1` if host `gcc` unavailable in sandbox) |

### Manual hardware & networked tests (`make NET=lwip MQTT=1`)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U12 | Status LED on GP19 (+ 330 Ω resistor) toggles every 250 ms *(supersedes starter template's `S1-U03` manual LED label)* | GP19 heartbeat, 2 Hz indicator | manual | 1. Flash normal (`make`) or networked (`make NET=lwip MQTT=1`) image. 2. Observe GP19 LED (or measure with scope/logic analyzer) for 15 s. | Steady 2 Hz blink (250 ms ON / 250 ms OFF, 50% duty); record 15–30 s video clip |
| S1-U13 | CYW43 Wi-Fi association, Mosquitto broker connection, and 1 Hz 8-byte binary telemetry decode over the air | Wi-Fi / lwIP / MQTT / `IF-04` | manual | 1. Configure `config/wifi_secrets.h` and launch Mosquitto (`mosquitto -c mosquitto_local.conf -v`). 2. Flash `make NET=lwip MQTT=1`. 3. Run `python scripts/telemetry_decoder.py`. 4. Capture USB serial log + decoder output. | USB prints `wifi: link up`, `mqtt: connected`, `mqtt: subscribed to robot/inf2004team15/cmd`, `mqtt: TX` (8 bytes) every 1 s; decoder prints `[SEQ #...]` with `FAULT_WIFI` cleared; record video clip |
| S1-U14 | Remote MQTT command RX (`PING`, `START`, `STOP`, `ESTOP`, `"INVALID"`, `CLEAR`) and fault injection/clearing | `IF-02`, `IF-03`, Fault Manager | manual | 1. While `S1-U13` is active, publish `PING`, `STOP`, `INVALID`, `CLEAR`, and `ESTOP` to `robot/inf2004team15/cmd` using MQTTX or `mosquitto_pub`. 2. Observe USB console (`mqtt: RX`, `LOG_CSV`) and `telemetry_decoder.py`. | `PING` / `STOP` / `START` execute cleanly; `"INVALID"` logs `LOG_CSV,...,CMD_REJECTED` and shows `FAULT_CMD_REJECTED` in decoder; `ESTOP` raises `FAULT_ESTOP`; `CLEAR` clears `FAULT_CMD_REJECTED \| FAULT_ESTOP`; record video clip |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (PC)** = run on a laptop/desktop;
**manual** = a person observes the result (LED, scope, meter, Ground Station / MQTTX…).

## Results

Add a new row for every post-commit run (`git rev-parse --short HEAD`). Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
