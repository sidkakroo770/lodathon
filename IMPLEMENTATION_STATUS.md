# Coverage implementation and validation status

## Accuracy correction verified in Gazebo — 2026-09-27

Final deployment remains Raspberry Pi 5 with real sensors; Gazebo is the current
target. No hardware readiness is claimed. The following changes supersede the
zero-height-datum and unconditional image-admission assumptions below:

- Stock-Iris profiles use a ground/HOME datum of -0.192 m, derived from landing
  geometry and the field surface height. The generic default remains zero and
  the commanded HOME-relative altitude remains 10 m.
- Projected observations require two-sided 50 ms pose brackets, angular rate
  at most 0.10 rad/s and combined tilt at most 5 degrees. This reduces timing and
  projection sensitivity, not unknown heading bias.
  The injected 2-degree heading bias and 4 cm position oscillation are unchanged.
- Camera health and map admission are separate. Fresh usable images permit at
  most 1 source second of motion through already observed, checked static ground
  without a new projection. No traversal is credited without a fresh accepted
  observation. Invalid/missing camera pixels still expire at 0.20 source seconds.
- Worker dispatch is 20 Hz in source time, with a 50 Hz wall-time producer cap,
  bounded image/pose caches and latest-only queues. The additional attitude test
  is constant-size; no dense projection or neural inference was added.
- 47 tests pass. Current-code biased-pose repeats `accuracy_final_3/noise`,
  `accuracy_final_4/noise` and `accuracy_final_5/noise` complete with zero independent
  gaps/incursions/fence violations, peak errors 0.22080 m, 0.22974 m and 0.21774 m,
  and both tracking checks passing. All 15 selected current-code campaign cases
  pass, including seven fault cases and the full-sized field.
- The stricter gate exposed frontier churn in residual observation repair:
  `accuracy_regression_2/row_end_0` finished traversal but blocked with 234 independently
  unseen cells. Repair now retains its checked viewpoint, settles and waits for an
  admitted observation before choosing another. `accuracy_repair_1/row_end_0`
  passes: zero unseen cells/physical violations, peak projection error 0.08747 m.
  The varied-start row-end repeats and alternate camera profile also pass; exact
  evidence is listed in `VALIDATION_RESULTS.md`.
- `accuracy_full_3/full_multiple`: the current-code 40 × 30 m mission completes in
  2107.35 source seconds with zero independent coverage gaps/red incursions/fence
  violations, peak corner error 0.10598 m, and both tracking checks passing. Final
  hold moves at most 0.0129 m during the following 3.183 recorded source seconds.
- Current full-field desktop worker processing: median 4.27 ms, p95 10.42 ms,
  maximum 124.68 ms, excluding queue/acquisition overhead. This is not a Pi benchmark.
  Python compilation and environment-script syntax checks pass. All owned test
  processes have stopped. Original-corridor integration and hardware calibration,
  acquisition timing, throughput/thermal and outdoor tests remain separate work.

Retained intermediate failures: `accuracy_noise_1/noise` and
`accuracy_noise_3/noise` blocked because projection rejection also caused repeated
camera-failure holds. `accuracy_check_2/row_end_0` likewise blocked under the earlier
coupled gate. Increasing scheduling frequency alone did not fix that coupling.
These failures are not erased or counted as accepted missions. Accuracy limits
(0.25 m) and physical/planning margins (0.40 m + 0.30 m) were not relaxed.
The pre-tilt-gate `accuracy_full_1` trial was deliberately interrupted when the final
tilt gate was added; it is not validation evidence. `accuracy_check_2/noise` passed
at 0.24917 m before that addition, but is not the final robustness claim.
The environment restarted during `accuracy_full_2`, `accuracy_final_1` and
`accuracy_final_2`; these incomplete campaigns are retained, not counted as passes.
Their interrupted stages were repeated successfully against the current code.

## Previous commissioning pass — 2026-09-26

This section supersedes the older implementation details below. The retained
2026-09-25 record is historical evidence, not the current feature list.

Implemented since that record:

- MAVLink TIMESYNC round-trip clock alignment instead of receipt-time pairing;
  source-time interpolation and freshness bounds are retained. Projection error
  is checked against both the planning reserve and the original **0.25 m target**.
- 0.1 m grid with cached coverage-cell indexing; the original tight row-end layout
  can complete without changing its geometry or reducing physical safety margins.
- Immediate provisional red blocking, two-observation confirmation and clearing
  of unconfirmed evidence after three valid clear reobservations.
- Configurable asymmetric focal lengths/principal point, camera mount/distortion
  and sensor topics. Gazebo fixture generation rejects unsupported non-ideal
  camera profiles rather than silently rendering the wrong model.
- EKF-status freshness/flag checks, origin/yaw/position/clock reset aborts, rejection
  of unusable black/overexposed images, and bounded worker/telemetry supervision.
- Fine-grid escape look-ahead with observed-ground/fence/distance checks, keeping
  recovery speed from being limited by individual 0.1 m waypoint stops.
- Clearance-aware route costs and checked shortcuts: prefer tracking room over
  narrow passages without deleting coverage obligations or reducing margins.
- Measured stop-before-turn for lateral momentum/reversals, with 0.3 source seconds
  continuously below 0.1 m/s before resuming a sharp connector. Incursion escape
  remains higher priority. Dedicated partial-zone and full-field repeats passed.
- Owned Gazebo/SITL campaign and matrix runners; explicit model/port isolation,
  source/config manifests, independent truth, fault schedules and motion evaluation.
- Diagnostics for commanded legs and independently measured lateral error at
  credited route points. These are different metrics; a short leg is not a tested
  long straight segment merely because its endpoint was reached.

Completed evidence at the start of the current matrix:

| Run under `artifacts/` | Result |
|---|---|
| `roundtrip_multiple_1` | COMPLETE; zero independent coverage gaps/incursions/fence violations; peak projection error 0.1564 m |
| `roundtrip_row_end_2` | Original tight layout COMPLETE; peak projection error 0.1611 m; no coverage/safety violations |
| `fault_recoverable_3` | COMPLETE; all six stop-command, measured stopping-distance and settling checks passed |
| `fault_incursion_1` | Late discovery backdated; autonomous warning at estimated 5 s; actual physical residence 7.516 s; COMPLETE |
| `full_multiple_3` | Full 40 × 30 m COMPLETE in 1959.06 source seconds; zero independent coverage gaps/incursions/fence violations; peak projection error 0.1685 m |
| `full_multiple_4` | Final controller, including turn braking: full 40 × 30 m COMPLETE in 1976.42 source seconds; zero independent gaps/physical violations; peak projection error 0.1812 m; all tracking checks passed |
| `matrix_nominal_1` | All three multiple-zone repeats passed with yaw/airborne-entry variation |
| `matrix_faults_2` reset/deadline cases | Yaw, position and clock resets aborted; deliberately overlong red residence warned, latched deadline failure and aborted after exit |
| `matrix_faults_3` | Pause/incursion, pause-only and six recoverable sensor/worker faults all passed |

All 18 nominal scene/profile/start cases now pass the stricter oracle checks;
see `VALIDATION_RESULTS.md` for the selected evidence and aggregate results.
The biased-pose accuracy target still fails in a final-controller repeat
(final repeats: 0.2489 m pass, 0.2558 m fail, 0.2599 m fail); full coverage and
physical safety passed in all three. Their per-case
`evaluation.json`, terminal result and matrix `summary.json` preserve the evidence;
`VALIDATION_RESULTS.md` identifies the accepted repeats and the explicitly failed
biased-pose accuracy criterion. Nominal validation is not universal acceptance.

Failures retained, not relabelled as passes:

- `roundtrip_row_end_1`: reordered TIMESYNC response was mistaken for a reset;
  fixed by rejecting old responses while retaining telemetry reset detection.
- `fault_recoverable_1`: worker-stall settling speed 0.110 m/s at the short
  evaluation time; the longer six-second outage now measures actual settling
  and retains an independent stopping-distance bound.
- `fault_recoverable_2`: switching repeatedly between GUIDED position and velocity
  control caused oscillations/fence-buffer exit. Removed that trial. Fault holds
  remain zero velocity; only terminal hold uses a latched position target.
- `full_multiple_1`: test-model resolution selected the wrong physics port. Exact
  generated model paths and isolated resource paths now prevent this aliasing.
- `full_multiple_2`: narrow-route overshoot crossed the operational inset and
  aborted; independent physical fence was not crossed. Clearance preference was
  added; the full-field repeat must establish whether it resolves the failure.
  `full_multiple_3` subsequently passed the independent checks without shrinking
  margins. This is one full-sized successful flight, not a universal dynamics bound.
- `matrix_faults_1`: reset aborts passed, but deadline takeoff and pause tests did
  not. A short paused simulator now causes a supervised hold with the existing
  bounded outage limit, rather than an immediate heartbeat-age abort.
- `matrix_faults_2/incursion_pause`: a clock exchange spanning the pause poisoned
  the fit window, delaying recovery past the residence deadline. Long host-delay
  exchanges and individual out-of-contract clock samples are now rejected without
  retaining them for the whole window. The independent repeat in `matrix_faults_3`
  completed with 7.506 s physical residence; time remained frozen during pause.
- `matrix_nominal_camera_1/wide_1`: a rejected clock exchange produced an infinite
  diagnostic error; strict JSON logging aborted. Unknown clock metrics now serialize
  as `null`, with a regression test. This changes diagnostics, not validity bounds;
  the repeat `matrix_tracking_repeat_1/wide_1` passed all nominal criteria.
- `matrix_nominal_visibility_1/partial_0`: coverage, physical safety and credited
  lane tracking passed, but the separate long-connector metric reached 0.492 m
  (target 0.30 m). The older matrix summary did not gate on that diagnostic;
  **do not treat its `passed: true` as satisfying the full tracking criterion**.
  Current matrix checks include that metric when long legs are present. The new
  measured turn stop was tested in `matrix_tracking_repeat_1`.
  Its first repeated `partial_0` flight passed: maximum connector deviation
  **0.2208 m**, down from 0.4916 m, with zero coverage/safety violations.
- `matrix_nominal_camera_1/noise`: COMPLETE, zero independent gaps/incursions/fence
  violations, but the injected 2° yaw bias plus 4 cm position perturbation produced
  **0.2537 m** peak projection error. It is within the 0.30 m planning reserve but
  fails the original 0.25 m accuracy target. The threshold has not been increased.

Current focused regression suite: **39 passing tests**, including the pause-spanning
clock exchange, fixture isolation, camera calibration, mapping, recovery, routing
and independent evaluator. Python compilation and launcher shell syntax checks pass.

The nominal, tracking-repeat, fault and full-field batches are complete. The
repeated wider-FOV logging-failure case and all three partial-zone turn-braking
repeats passed. All owned simulator/aircraft test processes are stopped. The
pose-bias accuracy limit above is still open, not an outstanding unrun test.

No change to original mission files, autopilot gains or arming checks. The real
corridor/world registration, Pi camera acquisition/timing and outdoor/aircraft
commissioning remain separate from this **independent Gazebo segment**.

## Historical record — 2026-09-25

Date: 2026-09-25. Workspace: `/home/sid/sae_mission2_coverage`.

The original mission directory has not been edited. The copied launchers point to
this workspace. The old sweep is retained under `legacy/`; the top-level entry
point now launches the new non-ROS implementation. See `README_COVERAGE.md` for use.

## Implemented

- Independent two-camera Gazebo fixture based on the existing Iris/corridor physics
  model, using the proven 1 ms physics step and normal ArduPilot arming checks.
- Separate bounded GUIDED/arm/NAV_TAKEOFF test helper; airborne-only coverage entry.
- FOV/overlap-derived lanes, conservative footprint/endpoints, explicit field config.
- Full-attitude image-to-flat-ground projection and all-red-contour segmentation.
- Source-timestamp telemetry interpolation and bounded image/pose association;
  bounded-rate SITL/Gazebo epoch tracking, wall-time health watchdogs.
- Persistent observed/red map, clearance inflation, checked A* detours/frontiers,
  measured route progress, residual ground observation and blocked outcomes.
- Immediate incursion recovery, historical entry estimate, autonomous 5 s warning,
  10 s deadline latch; stale images do not erase residence or earn coverage.
- Separate worker and I/O supervisor, camera/map GUI, JSONL decisions and final map.
- Measured stationary completion and latched position hold; no return/landing task.
- Synthetic forward-rendered image tests and an independent read-only Gazebo truth
  recorder that is never imported by the mission.

## Evidence recorded so far

- Unit/regression suite: **28 passing tests** (`python3 -m pytest -q tests`), including
  camera-to-completion replays, blocked layouts and an independent geometry oracle.
- Current 12 × 10 m synthetic cases: clear, central, multiple and `row_end_open`
  reach COMPLETE, with zero modeled incursion/fence samples and zero pending/unseen
  cells. The original tight `row_end` now reports BLOCKED with 62 unseen cells after
  increasing safety margins. Its geometry has not been changed or silently credited.
- Full 40 × 30 m synthetic multiple-zone case: COMPLETE after approximately 1537
  source seconds, 10,248 steps, zero modeled incursions/fence samples and no pending
  or unseen cells. This is ideal kinematics, not a full-sized Gazebo flight.
- Live Gazebo takeoff: accepted using normal arming checks, verified near 10 m.
- Live restarted smoke run: COMPLETE (`artifacts/gazebo_smoke_recovery.result.json`),
  zero pending/unseen cells; this started at the earlier stopped pose, not a clean
  initial fixture. Its independent truth trace covered only part of the flight.
- That partial truth trace recorded 12,933 airborne samples, zero incursion/fence
  samples, minimum red clearance approximately 0.494 m and maximum source sample
  gap 18 ms. **It does not prove the whole flight.**
- Clean-start central-zone test with GUI: COMPLETE in approximately 254 source
  seconds. Independent truth encloses the mission: 14,971 samples, zero incursion/
  fence samples, minimum clearance 0.493 m using the earlier 0.30 m body radius.
  This trace predates orientation recording; it cannot independently validate camera
  coverage/projection. See `artifacts/gazebo_central_clean.evaluation.json`.
- Clean-start multiple-zone test with GUI and 0.40 m body radius: COMPLETE in
  approximately 120 source seconds. Independent truth: zero incursion/fence samples,
  minimum clearance 0.415 m, zero uncovered permissible 0.2 m grid cells. Peak corner
  projection error 0.226 m exceeded the then-configured 0.20 m uncertainty allowance.
  That allowance check failed; see `artifacts/gazebo_multiple_clean.evaluation.json`.
- Repeated multiple-zone test with **current 0.40 m body radius + 0.30 m uncertainty**:
  COMPLETE in 117.984 source seconds. Independent truth encloses the whole mission:
  6,941 pose samples, maximum gap 18 ms, zero incursion/fence samples, minimum physical
  red clearance **0.555 m**. All 1,105 accepted observation timestamps have matching
  truth; **zero uncovered permissible cells** at 0.2 m. Peak projected-corner error
  **0.291 m**, 95th percentile **0.162 m**. This fits the new allowance, but **does not
  meet the original 0.25 m maximum-error design target**, and leaves little reserve.
  See `artifacts/gazebo_multiple_margin.evaluation.json` and associated manifest,
  map, terminal result and truth trace. These are sampled, single-run results.
- Final Python compilation and shell syntax checks passed. Owned Gazebo/SITL,
  MAVProxy and evaluator processes were stopped after tests; nothing is left flying.

## Issues found during commissioning and corrected

1. A 2 ms fixture physics step caused ArduPilot IMU-rate prearm rejection. Restored
   1 ms; did not disable/bypass checks.
2. Heartbeat filtering used an unset MAVLink target component. Bind source IDs from
   the accepted ArduPilot heartbeat.
3. Latest-value attitude/altitude joins were not genuine timestamp interpolation.
   Buffer independent source streams and interpolate before creating poses.
4. A latest image could remain unbracketed and starve processing. Retain a short,
   bounded image history and select the newest valid bracketed observation.
5. Fixed clock epoch fit drifted beyond the freshness bound over a flight. Track
   small rate drift with a bounded correction; abrupt changes still invalidate data.
6. A recovery endpoint's arrival tolerance could lie inside the clearance boundary.
   Require actual free-cell entry before consuming the final recovery waypoint.
7. Footprint erosion clipped at the map boundary could permanently hide field edges.
   Rasterize with padding before erosion and crop afterward.
8. Arrival tolerance could cut an A* corner. Check the next connector from the actual
   position before advancing the path.
9. The latest mapped pose could lie milliseconds ahead of the Gazebo clock within
   the epoch-fit uncertainty, creating intermittent health holds. Interpolate within
   available pose history at the current clock; do not extrapolate or widen freshness.
10. The initial 0.30 m body radius did not enclose the Iris rotor discs. The model's
    rotor centres plus 0.10 m propellers fit within 0.36 m; use a conservative 0.40 m.
11. Measured projection error exceeded the initial 0.20 m allowance. Increased the
    provisional planning reserve to 0.30 m and reran the flight. This does not prove
    a general error bound or resolve the original projection-accuracy target.

## Remaining before claiming this segment proven in Gazebo

The full design proof matrix is **not complete**. At minimum:

- Repeat clean flights with independent truth across supported layouts/start poses;
  current evidence is one central run and two multiple-zone runs with changed margins.
- Full 40 × 30 m Gazebo runs, multiple/edge/row-end/partial/revealed zones and a second
  camera profile. Synthetic successes do not substitute for these flight tests.
- Resolve the 0.291 m peak projection error and validate the uncertainty allocation
  under perturbations. Original 0.25 m target remains unmet. Independent projection
  and image-coverage evaluation now exists for the small multiple-zone runs; extend
  it across the matrix and add cross-track/braking checks. Do not merely inflate the
  margin again to label an accuracy target passed.
- The tight row-end corner remains unobservable under the current conservative grid/
  clearance policy. Investigate finer boundary representation or other checked viewing
  paths if that layout must be supported; never erase the gap or shrink safety margins
  solely to turn BLOCKED into COMPLETE.
- Injected physical incursions and delayed recognition, including 5 s warning/10 s
  deadline boundary tests in Gazebo; unit tests currently establish timer semantics.
- Sensor loss/delay, clock reset, estimator/yaw errors, CPU load and pause/resume
  fault-injection campaigns. Validate actual aircraft response, not just decision codes.
- Real corridor-exit handoff/field registration while preserving the known origin.

## Deliberate first-version limitations and design gaps

- Red evidence is latched conservatively from a single valid observation. Independent
  multi-frame confirmation/contradiction handling is not implemented yet. A spurious
  detection can block/exclude route points; this is not outdoor-ready perception.
- Received timestamp alignment bounds observed residuals but does not establish true
  exposure-time synchronization. Hardware requires a different calibrated adapter.
- The map includes a scalar camera offset and configurable FOV/distortion, but real
  asymmetric focal lengths/principal-point/cropping and mounting calibration require
  extending the camera profile before physical-camera deployment.
- Recovery needs observed ground and fresh localization. A completely unknown/too-large
  zone or missing historical entry evidence has no guaranteed compliant escape.
- Autopilot mode/arming/link/pose checks exist; comprehensive estimator-health and
  aircraft-specific localization-loss contingencies are not yet commissioned.
- Existing uncertainty/speed/braking margins are provisional, not aircraft certification.
- Pi 5 throughput, memory/camera copies, exposure, outdoor HSV robustness, thermal
  behavior and real dynamics remain untested. No neural model is used or needed here.
- GUI shows the downward camera and map. The front camera remains published/available
  but is not fused into this segment.

The implementation therefore supplies a testable first solution, not a claim of
complete design acceptance or real-flight readiness.
