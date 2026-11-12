# Subsystem 5 — test log

**Subsystem:** <name>  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | GP20 START button reads "released" at power-up (catches a stuck button, a wrong pin, a missing pull-up) | TEAM.md pin claim; start of the run | auto | `subsystem5_test.c` (`make TEST=1`) | `[TEST] S5-U01 PASS  GP20 start button released at power-up` |
| S5-U02 | Power-up is a STOP state; one GP20 press starts the run; a press during the run does nothing | start/stop behaviour, NFR6 | manual | 1. Flash, lift the wheels off the table. 2. Wait 10 s without touching anything. 3. Press and release GP20. 4. Press GP20 again while the wheels turn. | 1–2. `[S5] state -> IDLE`; the wheels never turn. 3. `[S5] GP20 pressed -> START` then `state -> LINE_FOLLOW`; the wheels turn and keep turning after the button is released. 4. `[S5] GP20 pressed -> START` but the state stays `LINE_FOLLOW` (a run is not stopped by the button). |
| S5-U03 | A press while in FAULT resets to the stopped state; the next press starts | fault recovery without MQTT | manual | 1. Drive, then stop the car for good: `REMOTE_STOP` from the dashboard (or build with `VC_BENCH_LINE_ONLY 0` and let the line be lost). 2. Press GP20 once. 3. Press GP20 again. | 1. `state -> FAULT`. 2. `[S5] GP20 pressed -> RESET`, `state -> IDLE`, wheels off. 3. `state -> LINE_FOLLOW`. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
