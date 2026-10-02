# Independent Gazebo coverage validation

Workspace: `/home/sid/sae_mission2_coverage`. Original mission files are preserved.
Baseline date: 2026-09-26. Accuracy correction: 2026-09-27.
Raspberry Pi hardware remains the final target; this is Gazebo/SITL evidence only.

## Accuracy correction — 2026-09-27

The reproduced biased-pose failure is resolved in three final-code repeats against
the unchanged 0.25 m projected-corner target. The 2-degree heading bias and 4 cm
position perturbation remain enabled. Aircraft radius 0.40 m and planning reserve
0.30 m are unchanged. These are bounded tests, not a guarantee for arbitrary
localization error, optics, terrain or timing.

| Current-code run under `artifacts/` | Peak corner error | Accepted observations | Duration (source s) | Result |
|---|---:|---:|---:|---|
| `accuracy_final_3/noise` | 0.22080 m | 1411 | 205.96 | COMPLETE; all applicable checks pass |
| `accuracy_final_4/noise` | 0.22974 m | 1464 | 215.64 | COMPLETE; all applicable checks pass |
| `accuracy_final_5/noise` | 0.21774 m | 1582 | 204.80 | COMPLETE; all applicable checks pass |

All three have zero independently unseen permissible cells, zero physical red
incursions/fence violations, complete truth intervals, and maximum truth gaps of
35 ms. Both credited-route and long-connector tracking criteria pass. Their
controller source hashes match the current code, including the viewpoint-retention
follow-up below. Only accepted observation timestamps
are evaluated; discarded high-motion images do not earn coverage evidence.

Three earlier accuracy-gate repeats also passed before the localized repair change:
`accuracy_noise_5/noise` (0.22025 m), `accuracy_regression_1/noise` (0.23131 m), and
`accuracy_noise_6/noise` (0.22056 m). Their results remain separate revision evidence.

Corrections address three different causes:

1. The stock Iris HOME datum is above the ground. The profile value -0.192 m is
   derived from the 0.195 m landing-gear height and +0.003 m field surface, not
   fitted to oracle poses. The generic default and 10 m altitude command are unchanged.
2. Timestamp interpolation does not eliminate all estimator/timing error during
   attitude changes. Admission requires two-sided 50 ms brackets, angular rate
   <=0.10 rad/s, and combined vehicle tilt <=5 degrees. The tilt limit also rejects
   attitude peaks with low instantaneous rate. Unknown heading bias is not removed.
3. Rejected mapping observations must not masquerade as dead-camera events.
   Usable raw-camera freshness remains <=0.20 source seconds; up to 1 source second
   without a newly admitted projection permits checked motion in already observed
   static free space, with no new traversal credit. Dispatch is 20 Hz in source
   time with a bounded wall-time CPU cap, not a fixed wall-time rate at accelerated RTF.

No simulator truth enters control. Computation remains HSV/contours, sparse ray
projection, a bounded grid and constant-size attitude tests. No AI model or dense
reconstruction was added. Pi capture synchronization, actual throughput and thermal
performance still need hardware measurements.

All seven current-code fault cases pass their expected outcomes:

| Campaign under `artifacts/` | Cases | Result |
|---|---|---|
| `accuracy_final_faults_1` | yaw, position, clock discontinuities | Explicit aborts; no physical fence/red violation |
| `accuracy_final_faults_2` | deadline, incursion_pause | Deadline failure latched and aborted after exit; recoverable incursion exited and completed |
| `accuracy_final_faults_3` | pause, recoverable | Watchdog hold during pause; all six outage stopping/settling checks pass; missions complete |

The recoverable incursion physically lasted 7.511 source seconds. Late detection
was backdated; autonomous warning occurred 5.00555 seconds after estimated entry.
The three-wall-second pause advanced source time by zero and commands stopped
after the watchdog. The deliberate deadline violation warned at 5.000 s, latched
failure at 10.00005 s, exited and aborted. Its `fault_evaluation.passed=false` is
intentional: the aircraft was held inside past the deadline. The matrix passes
the required failure detection/response, not a claim of successful avoidance.

Recoverable faults are camera drop, delayed camera, black images, telemetry drop,
invalid EKF and worker delay. Maximum measured excursion during the worker-delay
test was 0.393 m against its 0.436 m stopping bound; all six checks passed and the
mission completed. Source manifests match the current code for every listed case.
The current-code full-field regression also passes:

| `accuracy_full_3/full_multiple` measurement | Result |
|---|---:|
| Field / terminal state | 40 × 30 m / COMPLETE |
| Duration | 2107.35 source seconds |
| Independent truth samples / largest gap | 66,784 / 0.036 s |
| Physical red incursions / fence violations | 0 / 0 |
| Independently unseen permissible cells | 0 |
| Matched accepted observations | 21,255 / 21,255 |
| Peak / p95 projected-corner error | 0.10598 / 0.06801 m |
| Minimum physical red clearance beyond aircraft footprint | 0.43654 m |
| Maximum credited-route / long-connector lateral error | 0.22490 / 0.02718 m |

All applicable oracle criteria pass, with source hashes matching the current code.
The following 3.183 source seconds of independent truth record only 0.0129 m maximum
horizontal displacement in terminal hold. Worker processing on this desktop was
median 4.27 ms, p95 10.42 ms, p99 14.32 ms and maximum 124.68 ms. This excludes
acquisition/queue overhead and is **not a Pi benchmark**.

In total, all 15 selected current-code campaign cases pass (seven nominal/stress,
seven fault, one full-field). The earlier 18-case nominal matrix remains baseline
evidence, not a claim that every old case was repeated with the new gate. Current
software checks: 47 tests pass; Python compilation and environment-script syntax
pass. All owned Gazebo/SITL/MAVProxy/mission test processes have stopped.

The stricter gate also exposed a residual-observation failure in
`accuracy_regression_2/row_end_0`: all route points were resolved, but changing the
frontier destination every 0.5 seconds prevented reaching a useful stable view;
234 independent permissible cells remained unseen. It correctly reported BLOCKED.
The follow-up retains each checked repair viewpoint, settles below 0.1 m/s for
0.5 source seconds, then requires an admitted observation before choosing another.
`accuracy_repair_1/row_end_0` passes all checks with zero independent gaps/physical
violations and 0.08747 m peak projection error. Unsafe routes are still invalidated
immediately. No remaining cells are relabelled or ignored.

The varied-start repeats `accuracy_final_3/row_end_1` and
`accuracy_final_4/row_end_2` also pass (peak projection errors 0.08419 m and 0.09289 m).
`accuracy_final_5/wide_0` validates the alternate camera profile and passes at
0.08696 m. Each completes with zero independent gaps/incursions/fence violations.

The environment restarted during `accuracy_full_2`, `accuracy_final_1` and
`accuracy_final_2`. The remaining aircraft runs lost telemetry/terminated and have
no completed campaign summaries; these are incomplete evidence, not passes.

Intermediate evidence is retained: `accuracy_noise_1/noise`, `accuracy_noise_3/noise`
and `accuracy_check_2/row_end_0` blocked under the earlier camera-health/mapping
coupling. `accuracy_check_2/noise` completed at 0.24917 m before the tilt gate; it
is not the final robustness claim. `accuracy_full_1` was deliberately interrupted
to restart with the final gate and is not a validation pass. Previous failures in
the historical sections below remain failures; thresholds were not waived.

## Historical baseline — 2026-09-26

The remaining sections preserve the previous commissioning pass, including its
then-open accuracy failure. Current-code results are listed above; older failures
have not been retrospectively relabelled as passes.

## What the evidence establishes

Tests use real Gazebo rendering/physics and ArduPilot SITL, not only synthetic
kinematics. The controller receives camera images and MAVLink telemetry; the
independent recorder receives simulator poses and fixture geometry. Simulator
truth is never an input to navigation.

The oracle checks the whole recorded mission interval, actual aircraft footprint
against red regions/geofence, ground visibility at 0.1 m grid resolution, corner
projection error and measured lateral tracking. It requires truth sample gaps
no greater than 50 ms. These are sampled results for specified fixtures, not a
continuous-time safety proof, outdoor certification or Pi performance claim.

## Full-sized field

Final controller: `artifacts/full_multiple_4/evaluation.json`:

| Measurement | Result |
|---|---:|
| Field | 40 × 30 m |
| Terminal state | COMPLETE |
| Source-time duration | 1976.42 s |
| Independent truth samples | 62,417 |
| Largest truth sampling gap | 0.036 s |
| Physical red incursions / fence violations | 0 / 0 |
| Independently unseen permissible cells | 0 |
| Accepted observations with matching truth | 14,409 / 14,409 |
| Maximum projected-corner error | 0.1812 m |
| 95th-percentile projected-corner error | 0.1170 m |
| Minimum physical red clearance beyond aircraft footprint | 0.4551 m |
| Maximum lateral error at credited route points | 0.2172 m |
| Maximum long-connector deviation | 0.1074 m |

This final-control repeat includes measured turn braking and passes all applicable
oracle checks. The following 3.263 source seconds of independent truth show only
0.0174 m maximum horizontal displacement during terminal hold. The earlier
`full_multiple_3` also completed with no independent gaps/physical violations and
0.1685 m peak projection error. Earlier full-sized failures remain recorded.

## Repeat matrix

Small fixtures are 12 × 10 m. Each nominal scene/profile is tested with initial
yaws 0, +0.2 and −0.2 rad and airborne N/E entries (0,0), (0.2,0.2), (−0.2,0.2) m.
The latter are achieved after takeoff without redefining HOME/local origin.

| Cases | Evidence folder | Status |
|---|---|---|
| Multiple zones, three starts | `matrix_nominal_1` | 3 passed |
| Tight row-end obstruction, three starts | `matrix_nominal_boundaries_1` | 3 passed |
| Field-edge zone, three starts | `matrix_nominal_boundaries_1` | 3 passed |
| Partially visible/revealed zones, three starts | `matrix_tracking_repeat_1` | 3 turn-braking repeats passed |
| Clear field, three starts | `matrix_nominal_visibility_1` | 3 passed |
| 35° HFOV, three starts | `matrix_nominal_camera_1/{wide_0,wide_2}`, `matrix_tracking_repeat_1/wide_1` | 3 passed; earlier logging failure retained |
| 2° yaw bias plus 4 cm position perturbation | `matrix_nominal_camera_1/noise` | Coverage/safety passed; accuracy target failed |

Nominal acceptance requires COMPLETE, no coverage gaps or physical violations,
maximum projection error ≤0.25 m, and lateral tracking ≤0.30 m (long-connector
metric where such legs exist, plus credited-route tracking). Missing long legs
are reported as unavailable, not fabricated as a passing measurement.

All **18 nominal cases** meet those checks. The selected evidence above explicitly
identifies repeats that supersede failed earlier attempts; it does not relabel or
delete those failures. Runtime source hashes accompany every flight.
Across those 18 runs: 22,127 matched observations and 96,868 truth samples;
maximum projection error 0.1866 m, maximum credited-route lateral error 0.2301 m,
maximum long-connector deviation 0.2777 m, and minimum physical red clearance
0.3856 m. No required ground gaps or physical red/fence violations were recorded.

The earlier `partial_0` summary in `matrix_nominal_visibility_1` predates the
long-connector acceptance gate: its 0.4916 m connector deviation is a failure
despite its older `passed: true`. The turn-braking repeat reduced that maximum
to 0.2208 m with no coverage/safety violations.

### Explicit unresolved accuracy limit

The biased-pose stress flight completed with zero independent gaps/incursions/
fence violations, but its peak projection error was **0.2537 m**. This fits the
0.30 m planning reserve and **does not meet the original 0.25 m accuracy target**.
Neither the threshold nor the injected error has been reduced to manufacture a
pass. An unreported heading bias is not observable from arbitrary unknown red
patches alone; the geometric error allocation and real calibration/localization
quality must be established before claiming that accuracy under biased poses.
All three further trials with turn braking are recorded as
`matrix_noise_turn_{1,2,3}`. They completed with zero independent coverage gaps
and no physical safety violations. Their peak errors were **0.2489, 0.2558 and
0.2599 m**: one passed the accuracy target and two failed. This confirms the
limitation was not resolved by turn braking; no best-run-only selection is made.
Nominal Gazebo operation is validated by the matrix above, but the full accuracy
acceptance envelope under biased localization is **not proven**. The code cannot
remove an unknown heading bias merely by changing the camera projection formula.

## Fault campaigns

| Test | Evidence | Observed outcome |
|---|---|---|
| Yaw, position and source-clock resets | `matrix_faults_2/{yaw,position,clock}` | ABORTED without physical fence violations |
| Deliberately prevented timely red exit | `matrix_faults_2/deadline` | Autonomous warning at 5.060 s; failure latched at 10.060 s; ABORTED after exit |
| Late red discovery plus 3 s simulator pause | `matrix_faults_3/incursion_pause` | Physical residence 7.506 source s; autonomous warning; exit and COMPLETE |
| Pause without incursion | `matrix_faults_3/pause` | Commands stopped during pause, source time did not advance, resumed and COMPLETE |
| Camera loss/delay/black frame, telemetry loss, EKF-invalid flag, stalled worker | `matrix_faults_3/recoverable` | All six command-stop, measured stopping-distance and settling checks passed; COMPLETE |

The deliberately overlong incursion is a successful **failure-detection test**,
not a compliant flight. Its standalone incursion report correctly has `passed:
false`; the negative-test matrix checks the expected deadline failure and abort.
Loss of real localization still cannot guarantee a physical hover merely because
the companion computer requests zero velocity.

## Software checks and remaining deployment boundary

- Baseline: 39 focused unit/regression tests passed; current code has 47 passing
  tests. Python compilation and shell syntax passed in that baseline pass.
- GUI remains the normal launch default; automated campaigns use `--no-gui`.
- Previous full-field desktop worker processing: median 5.69 ms, 95th percentile
  11.20 ms, maximum 166.37 ms. Freshness supervision rejects stale decisions;
  these timings are **not a Pi 5 benchmark**.
- Source/config manifests, run specifications, raw decisions, independent truth,
  terminal results, maps and failed attempts are retained under `artifacts/`.
- Still separate: registration/handoff in the original corridor world, actual
  camera acquisition timestamps/calibration, outdoor HSV reliability, Pi thermal/
  throughput testing, real stopping/wind/localization uncertainty and flight safety
  commissioning. No return, payload, QR or landing mission has been added.
- All owned test processes were stopped after that baseline campaign.

See `README_COVERAGE.md` for commands and `IMPLEMENTATION_STATUS.md` for the
chronological debugging record. Do not launch this commissioning adapter on a
real aircraft.
