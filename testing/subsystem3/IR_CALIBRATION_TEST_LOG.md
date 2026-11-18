# Buddy 3 standalone IR calibration evidence

Date: 13 October 2026, Asia/Singapore.

## Scope and configuration

New code in `bench/ir_calibration/`, built against the fresh professor
starter. No older reference code was migrated and no historical test
results were reused. The existing example subsystem3 is preserved.

User-confirmed hardware: Pico W and one module labelled VCC/GND/AO/DO.
Sensor model, rated supply range and equivalence to robot sensors remain
unconfirmed. Current bench wiring: 3V3(OUT) to VCC, GND to GND, GP6 to DO,
AO disconnected. No physical wiring inspection has occurred.

Image: `ir_calibration_smp1_usb_cdc_wifi_none.uf2`.
Build: SMP=1, CONSOLE=usb_cdc, WIFI=none, NET=none, MQTT=0,
CRASH_REPORT=0, TEST=0. Kernel tick remains 10 ms.
Bench task: priority 16, core 0, stack 2048 bytes. No ADC or motor control.
Training: 100 samples per surface, 95 agreeing samples required, opposite
black/white levels required; requested sample delay 20 ms.

## Fresh automated results

| Check | Result | What it establishes |
|---|---|---|
| `python3 bench/ir_calibration/check_style.py` | PASS, 8 C/header files | Defined BARR-C subset; not a full standards audit |
| `python3 bench/ir_calibration/test.py` | PASS with ASan and UBSan | Portable calibration behaviour under synthetic inputs |
| Both electrical polarities | PASS | Accepted 95–100 agreeing samples for each polarity |
| Noise boundary sweep | PASS | Rejected 6–94 high samples, with either surface noisy |
| Identical black/white levels | PASS | Rejected all-high and all-low pairs |
| Failed recalibration | PASS | Incomplete data clears a previously valid result |
| Transition accounting and full window | PASS | Alternation counted; extra sample rejected without count changes |
| `python3 bench/ir_calibration/build.py` | PASS | Fresh ARM ELF link and UF2 conversion |
| Application warnings | PASS | `-Wall -Wextra -Werror -Wconversion -Wshadow` |
| ELF symbols | PASS | Strong bench `usermain`, bench task; no example subsystem task symbols |

Environment observed: Pico SDK 2.3.0, Arm GNU toolchain 15.3.Rel1
(`arm-none-eabi-gcc` 15.3.1); host compiler Apple Clang 17.0.0. The toolchain
directory name still contains `15_2_Rel1`; the executable's reported
version above is authoritative for this build. The build log and ELF/UF2/map are under
`bench/ir_calibration/build/` (ignored). Source inspection confirms the
build uses a separate object tree and does not clean the original build.

Prepared UF2: 122368 bytes.
SHA-256: `6659fe6236fb7b9cff940a8c391083504124c4cc32dab7ca5e608f8c1533dc09`.
ELF sections: text 61073 bytes, initialized data 36 bytes, BSS 22144 bytes.

## Hardware results at initial preparation: NOT RUN

No device was flashed or connected through a serial session. No physical
polarity, contrast, sample timing, USB interaction, disconnect recovery,
line following, barcode decoding or robot integration result is claimed.

For the first bench run, record:

- Sensor model/board markings and confirmed supply rating.
- Firmware image/checksum and date.
- Wiring, surface material, height, angle and ambient lighting.
- The complete BLACK and WHITE summaries, including elapsed time.
- PASS polarity and repeated live readings on both surfaces.
- A deliberately poor threshold trial: ensure identical/noisy levels fail.
- Repeated calibration, reset and terminal close/reopen behaviour.

Keep failed captures in the log. Physical results should be added here and
to `PROJECT_CONTEXT.md` when observed. A bench pass does not establish
three-sensor mounting, final RTOS timing, or tracking performance.


## User-reported first run and corrected build

13 October 2026: the user supplied WHITE high=0/100, transitions=0,
elapsed=3000 ms and FAIL status=3. No BLACK summary was supplied; from
status 3, the application also classified that window as stable LOW.

The user confirmed a Pico W on a Maker Pi Pico carrier, with the sensor
connected via Grove 4, GP6. Initial firmware read GP7. The current firmware
reads GP6 and its startup banner states DO=GP6. This is a pin-configuration
correction based on the user's wiring report, not proof of sensor failure.

Host sanitizer tests, BARR-C subset checks and ARM ELF/UF2 build passed
again after the correction. Current UF2 SHA-256:
`3af8ac0e59f0ac90f7c0f59bd1398e0a9fa532bf69322e42a51aaff8f8ca9913`.
UF2 size remains 122368 bytes; ELF section sizes are unchanged. The earlier
checksum above identifies the original GP7 image only. Updated GP6 hardware
calibration remains pending; no agent flashing or serial session occurred.
