# Subsystem 3 — test log

**Subsystem:** IR line following, barcode decoding, and junction detection  ·  **Owner:** Rance (2501107)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

### Legacy starter test

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Starter consumer statistics (min ≤ max, both within 0–100) | Starter sample consumer | auto | Reserved for the starter example in `subsystem3_test.c`. | `[TEST] S3-U01 PASS` on the starter image |

### Robot track tests with video evidence

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-M01 | Line following, barcode decoding, junction detection, and left turn | Code 39 decode → navigation instruction | manual | Run the car on a marked track with a barcode mapped to left. Observe line tracking, check the displayed letter, and observe the response at the junction. | The car follows the line, the display shows the printed letter, and the car turns left at the detected junction and resumes line following. |
| S3-M02 | Line following, barcode decoding, junction detection, and right turn | Code 39 decode → navigation instruction | manual | Run the car on a marked track with a barcode mapped to right. Observe line tracking, check the displayed letter, and observe the response at the junction. | The car follows the line, the display shows the printed letter, and the car turns right at the detected junction and resumes line following. |
| S3-M03 | Line following, barcode decoding, junction detection, and straight travel | Code 39 decode → navigation instruction | manual | Run the car on a marked track with a barcode mapped to straight. Observe line tracking, check the displayed letter, and observe the response at the junction. | The car follows the line, the display shows the printed letter, and the car continues through the detected junction while following the line. |
| S3-M04 | Line following, barcode decoding, junction detection, and U-turn | Code 39 decode → navigation instruction | manual | Run the car on a marked track with a barcode mapped to U-turn. Observe line tracking, check the displayed letter, and observe the response at the junction. | The car follows the line, the display shows the printed letter, and the car makes a U-turn at the detected junction and resumes line following. |

### Pico W hardware tests with USB serial evidence

Use the networked team image built with
`python3 app_program/subsystem3/build_team_network.py`. Connect the USB
serial monitor before applying the inputs and save the Pico's output under
`logs/`. For the bench cases, keep the robot stationary with motion disabled.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-M05 | Read a valid printed barcode in both directions | GP6 barcode sensing and Code 39 decoding | Manual bench | Move a known `*LETTER*` print fully past the mounted barcode sensor in each direction, starting and ending over white. Record the printed letter and the direction of each pass. | Each pass produces one `BARCODE_DECODED` event with the correct letter. The two passes report `order=F` and `order=R`. |
| S3-M06 | Reject an incomplete scan and accept the next valid scan | GP6 barcode capture and timeout | Manual bench | Slide the opening marker and letter past GP6, then hold the white gap before the closing marker until the capture expires. Repeat with the full print. | The partial pass logs an incomplete `IR_BARCODE_WINDOW` without a `BARCODE_DECODED` event. The full pass logs the correct letter. |
| S3-M07 | Left and right sensor response on black and white | IR line sensor inputs | Manual bench | Hold black under the left sensor and white under the right, then swap them. Also test both over black and both over white. Hold each placement until a new `IR_BARCODE_SAMPLE` line appears. | The `L` and `R` readings change independently, and each reading matches its sensor's configured black/white polarity in all four placements. |
| S3-M08 | Line loss and reacquisition | Line tracking state and motion requests to Subsystem 2 | Manual track | Drive on a marked line with no barcode or junction. Start with one sensor reading black, then introduce a white gap long enough to enter search. Let the robot find the line again and save the event trace. | `IR_FOLLOW_MODE` goes from `TRACK` through `CORRECT` and `STOPPING` to `SEARCH` while the line is absent, then returns to `TRACK` when a sensor sees the line again. |

## Results

Add a row after each run with the date, tested firmware commit, tester,
result, and evidence. Retain earlier runs and record retests on new rows.
Only the four video-backed track runs have results below. The serial cases
are defined for future hardware runs.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S3-M01 | `ed835e5` | Rance (2501107) | PASS (reported) | [Left turn video](https://youtube.com/shorts/wQ1VXyUp1To?feature=share) | Line following, barcode decoding, junction detection, and left turn. |
| 2026-11-19 | S3-M02 | `ed835e5` | Rance (2501107) | PASS (reported) | [Right turn video](https://youtu.be/xYHqsB7dWBY) | Line following, barcode decoding, junction detection, and right turn. |
| 2026-11-19 | S3-M03 | `ed835e5` | Rance (2501107) | PASS (reported) | [Straight video](https://youtube.com/shorts/kDpTaxi1b3c) | Line following, barcode decoding, junction detection, and straight travel. |
| 2026-11-19 | S3-M04 | `ed835e5` | Rance (2501107) | PASS (reported) | [U-turn video](https://youtube.com/shorts/zCP5LPdWkfk?feature=share) | Line following, barcode decoding, junction detection, and U-turn. |

The four recordings were supplied by the tester. The firmware commits, printed letters, and clip timestamps are pending confirmation.
