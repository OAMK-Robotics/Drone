# DJI Tello: audited evidence and one controlled motor-response test

Prepared from `code_csv_logs.zip` (158 source/CSV/log files, plus metadata) and `git_all_refs.bundle`, received 23 Nov 2026. Git bundle HEAD is `86942bb` on `feature/vision-navigation`; many local experiment scripts and captures are uncommitted. **No drone was flown while preparing this package.**

## Objective for Tuesday 24 Nov 2026

Obtain timestamped evidence of the aircraft's **Marker 23 image-space motion before, during, and after an actual short RIGHT (+10) RC command**, and, ONLY if gates and physical safety continue to pass, a matching LEFT (-10) RC command. Distinguish "a command was logged" from "a reproducible physical motor response was validated." The latter will need repeated trials and control for time-varying drift.

**NOT objectives tomorrow:** autonomous hover, path following, or guaranteed 20–30 cm flying height. These remain unfinished.

## What the archives establish

- **2026-11-20 paired RIGHT** (`captures/paired_axis_right_20261120_112827/measurements.csv`): both +10 and -10 pulses, 24/24 Marker 23 observations; successive medians for [neutral1, neutral2, after +, after -] were x `[158.25,156.00,152.50,151.12]`, y `[99.12,97.25,91.12,94.00]`. The Y reversal after negative command is a **candidate** for an axis response, not isolated causal evidence (the measurement windows have gaps; natural drift changes).
- **2026-11-20 paired FORWARD** (`captures/paired_axis_forward_20261120_113237/measurements.csv`): +10 and -10 pulses, 21/24 valid marker samples; window medians x `[137.0,137.12,133.25,133.25]`, y `[57.5,53.0,51.12,38.0]`. No clean sign reversal, so **forward motor mapping remains inconclusive**.
- **2026-11-20 continuous RIGHT** (`captures/streaming_axis_right_20261120_124334/observations.csv`): three individually logged +10 sends at 35.032, 35.184, and 35.339 s; Marker 23 tracked during the pulse but disappeared intermittently; neutral sent at ~35.490s, later reverse withheld. No continuous controlled post-pulse sequence for a causal comparison.
- **2026-11-20 streaming V2:** marker quality failed, no motor commands. **V3:** unstable post-descent SDK height, no motor commands. **V4:** M23 dropped after descent, no motor commands.
- **2026-11-23 V5:** four flight datasets (15:24, 15:26, 15:27, 15:29) with zero horizontal motor pulses. Causes: M25 lost in 2 runs; SDK height stayed at 50 cm in one; takeoff SDK height 80 cm prohibited the existing descent in another. The 15:24 run lacked `LANDING_ACK` after `KeyboardInterrupt`; visual physical landing must always be checked.
- **2026-11-23 drift captures:** 11:38 had 91 M23 / 88 M25 detections; 11:44 had 78 M23 / 116 M25. The low altitude target could not be reliably reached. Earlier velocity/quality/prediction analyses did NOT beat a no-motion baseline in prediction tests, and may not be used for automatic motor corrections.
- The `hover/junction_hover_controller.py` module gives **offline image-space suggestions**, not physical motor control; camera-relative 2-marker normalized pose is not verified metric displacement or a calibrated motor map.
- All archived paired/streaming V1–V6 **offline self-tests passed** here; that does **not** validate safe real flight.

## Why V7 is different (and why it can still fail)

`motor_response_v7.py` has **NO automatic descent**. Instead, after SDK takeoff ACK it records a brief neutral baseline and checks M23 tracking and recent SDK height, then sends **at most three +10 RC calls, 0.15 s apart**, continuously checking marker/height conditions. A reverse -10 pulse is conditional on six consecutive M23 observations; otherwise it is skipped. It saves continuous, time-stamped FRAME + PULSE_COMMAND + neutral + landing events and automatically prints a post-flight evidence summary.

Its provisional SDK post-takeoff acceptance band is **50–85 cm, not the original 20–30 cm goal**. Actual altitude is not independently measured; DJI's automatic takeoff can overshoot. Camera image centering does not locate the physical table edge. V7 MUST NOT be flown over an unprotected exposed 1×2 m table. Do not fake clearance consent. Get a physically protected, instructor-approved test area/recovery surface and human observer first. Otherwise, use offline modes only. No safe flight or motor response is guaranteed.

### Evidence-based gate replay (NOT a forecast)

Replaying V5's recorded camera/SDK samples at takeoff ACK +0.65...1.10 s, **3 of 4** data sequences meet V7's pre-pulse image/height gates at some point; the fourth has missing Marker 23. But V7 changes actual flight behavior by *skipping descent*, so these results are **hypothetical retrospective visibility checks**, NOT guaranteed flight behavior or a geofence test. See `replay_v7_gates.py`.

## Recommended sequence at school, one step at a time

1. In PowerShell, `Set-Location C:\python\Drone\Drone`; extract `lab_v7_motor_response.zip` into this same directory (it creates these code/report files without modifying existing V1–V6 code).
2. Run **offline**: `python -B .\motor_response_v7.py --self-test`. Stop if any assertion or import fails.
3. Run **offline retrospective**: `python -B .\replay_v7_gates.py --root .\captures`. Check actual capture data exists. No drone commands are sent.
4. **Before ANY real flight**, explicitly confirm with the lab instructor that an unobstructed, physically safe recovery area is available, not just the bare table. Verify spotter, people clear, battery ≥40%, camera/stream, and adequate overhead clearance for automatic takeoff. If not verified, do NOT fly.
5. **Only when all safety prerequisites are true**, the intended command is:
   `python -B .\motor_response_v7.py --fly --axis right --allow-takeoff-overshoot --verified-recovery-area`
   It still asks for the exact operator safety consent. If safety gates fail, it logs the reason and lands without sending pulses. Do NOT weaken or bypass them to force a result.
6. When propellers are stopped and landing is physically confirmed, V7 prints `DATASET: captures\motor_v7_right_...` and summary. For later analysis, run `python -B .\motor_response_v7.py --analyze .\captures\motor_v7_right_YYYYMMDD_HHMMSS` (replace the folder with actual one). `audit_all_motor_data.py --root .\captures` includes old + new experiments.

## How to judge tomorrow's experiment (predefined)

- **Safety pass:** no uncommanded flight; operator confirms a physical safe operating area, independent of camera markers; final actual touchdown confirmed. Any unverified safety point = **no flight**.
- **Data collection pass:** at least one logged nonzero `PULSE_COMMAND`, at least 3 marker detections in a post-pulse window, an explicit neutral event, and landing acknowledged/physically verified. If no pulse commands, record a fail-closed **NO MOTOR DATA** result (not a success).
- **Candidate directional motor response:** compare continuous M23 trajectories in `NEUTRAL_BEFORE`, `POSITIVE_PULSE`, and `AFTER_POSITIVE`, and, if safely obtained, `NEGATIVE_PULSE` and `AFTER_NEGATIVE`; report observed Δx/Δy, sample counts and gaps. These deltas alone **do not prove** a motor-caused physical translation.
- **Validated motor-axis mapping:** do not claim this unless independent repeated controlled trials show a repeatable response sign beyond no-RC drift and detection noise. V7 cannot guarantee that in a single flight.
- **No-go:** failed height/marker/camera freshness gates, uncertain table-edge clearance, low battery, no spotter, no safe recovery area, repeated motor-neutral failures, or a drone still airborne after failed landing acknowledgment.

## Boundaries

The repository's exact recorded motors are command messages, not feedback from motors/IMU. The uploaded evidence does not include independently measured 3-D physical motion. SDK-reported height and image-space px cannot be used as ground truth. Do not automatically connect the offline hover correction suggestions to motors until direction and stability have been validated.
