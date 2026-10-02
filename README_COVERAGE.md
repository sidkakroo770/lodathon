# Isolated camera-guided coverage experiment

Implementation workspace: `/home/sid/sae_mission2_coverage`. The original
`/home/sid/sae_mission2` is unchanged. This folder started as a complete copy;
copied corridor/approach assets and historical logs are not new coverage results.
Copied runnable launch paths now point here. Shared ArduPilot/Gazebo installations
remain external dependencies.

**Gazebo commissioning software, not approved for real aircraft.** See
`IMPLEMENTATION_STATUS.md` for measured results and outstanding validation.
`VALIDATION_RESULTS.md` records the corrected biased-pose accuracy results,
full-field/fault regressions and retained failures.

The final deliverable remains Raspberry Pi 5 hardware deployment; Gazebo is the
current validation target. No simulator truth, neural inference, or dense image
reconstruction is used by the controller.
Gazebo/SITL and the independent oracle belong on the development workstation, not
the Pi. Real camera acquisition and clock alignment still need a hardware adapter
and measured validation; the geometry, mapping and mission logic are separate.

## What runs

The new `boustrophedon_sweep.py` is an entry point to `coverage_mission.runtime`.
The previous image-column avoidance script is archived in `legacy/` for reference.
Do not run it alongside the new controller.

- `config.py`, `config/*.json`: explicit field, camera, motion and validity settings.
- `geometry.py`: HSV multi-region segmentation; calibrated optical rays intersect
  a flat plane using full interpolated attitude, HOME-relative altitude and N/E position.
- `planning.py`: persistent observed/red grid, aircraft-clearance inflation,
  FOV/overlap-based lane obligations, measured traversal and checked A* connectors.
- `engine.py`: coverage/recovery state machine, braking checks, residence timer,
  measured stop-before-turn, residual observation passes, blocked/fault outcomes
  and measured final settling.
- `runtime.py`: Gazebo image transport, MAVLink telemetry/commands, timestamp joins,
  bounded worker process, freshness supervision, OpenCV camera/map GUI and logs.
- `fixture.py`, `takeoff_test.py`: separate isolated simulator fixture and test takeoff.
- `replay.py`, `truth_monitor.py`, `tests/`: synthetic tests and independent Gazebo oracle.

There is no ROS or neural model. Only contour vertices and image corners are
projected, not a dense point cloud. Nominal map: 400 × 300 cells at 0.1 m.

## Contracts and scope

Planning coordinates are local NED **north/east**, metres; angles are radians.
The isolated world uses Gazebo X=east, Y=north, and vehicle spawn yaw=pi/2.
The initial HOME/entry is N=E=0. The field is an explicit test fixture, not surveyed
competition coordinates. `coverage.json` is 40 × 30 m; `smoke.json` is 12 × 10 m.
The full-world/corridor handoff has NOT yet been wired to this experiment.

The Iris physical radius is conservatively 0.40 m (rotor centre radius about
0.256 m plus a 0.10 m propeller). An additional provisional 0.30 m uncertainty
reserve is used; a commissioning run measured 0.226 m peak projection error,
which exceeded the earlier 0.20 m reserve. Further perturbation tests must establish
whether this new reserve is sufficient across the intended operating envelope.

The coverage component never arms or takes off. It accepts an armed, GUIDED,
settled airborne vehicle near 10 m with valid pose and camera timing. Operational
integration must preserve the same local origin or explicitly transform the field.
It ends after route obligations and required observed ground are resolved, followed
by a two-source-second low-velocity check and a latched position hold. No return,
QR, payload, landing mission, LiDAR avoidance or mission-time optimization is added.

Both cameras are rendered at 15 Hz. Downward 640 × 480 / approximately 28.2° HFOV
is the provisional 10 mm profile; front is 640 × 480 / approximately 60° and remains
available at `/coverage/front/image`. The GUI shows downward imagery and the map;
front-camera fusion is deliberately absent. Change the camera profile and regenerate
the fixture together. The 10 mm lens is not embedded in the planning algorithm.
Real lens intrinsics/distortion/exposure timing still require hardware calibration.

The flat plane is registered relative to HOME. Generic `ground_above_home` and
`camera.down_offset` defaults remain zero. The three stock-Iris Gazebo profiles
now set `ground_above_home=-0.192`: the landing feet are 0.195 m below the model
origin (leg centre -0.110 m, length 0.170 m), and the field surface is at +0.003 m.
This is a model-dimension datum calibration, not a correction fitted from flight
truth. It changes projection height, not the 10 m HOME-relative altitude command.
Hardware must use its measured HOME-to-ground datum; do not copy this Iris value.

Projection admission requires bracketed attitude 50 ms on either side of the
exposure, angular rate at most 0.10 rad/s and combined vehicle tilt at most 5°.
The tilt gate also rejects the poorly conditioned peak of an attitude oscillation,
where instantaneous angular rate can be small. This limits timing sensitivity; it
does not estimate or cancel an unknown attitude bias. Fresh usable camera images
are tracked separately from accepted map observations. During at most one source
second without an accepted projection, motion may continue only through the
already observed, clearance-checked static map; no new coverage is credited.
Camera loss/invalid images still expire after 0.20 source seconds. Worker jobs use
a 20 Hz source-time schedule with a 50 Hz wall-time CPU cap; bounded latest-only
queues/cache avoid backlog. Pi throughput and thermal behavior remain unmeasured.

Residual observation routes retain their selected viewpoint instead of chasing
a newly scored frontier every half-second. At arrival, the aircraft must settle
below 0.1 m/s for `viewpoint_settle` (0.5 source seconds) and receive an admitted
projection after that interval before selecting another viewpoint. New hazards
still invalidate unsafe routes immediately; unseen cells are never waived.

Unknown ground is not executable free space. Red evidence blocks motion immediately;
two distinct observations confirm it. Three clear reobservations can clear an
unconfirmed detection; confirmed red remains latched after leaving the image.
Persistent false detections can therefore still block a mission.
Safety-inflated route points are marked excluded, not traversed; permissible ground
in those buffer bands still has to be observed. The current grid ledger is an
engineering discretization, not proof of infinitely fine coverage.

Recovery starts immediately on a mapped incursion/clearance violation. Residence
is backdated through recent pose history where possible, warns autonomously at 5 s,
and latches failure at 10 s without a confirmed exit. Camera loss freezes new mapping;
fresh localization can continue checked recovery through the already known map.
Missing localization does not authorize a blind escape. No algorithm can recover an
unknown physical entry time when the relevant ground was never detected/history is
absent. Deliberate late-discovery/physical-incursion tests are recorded in
`VALIDATION_RESULTS.md`; they do not prove recovery from every unseen zone.

Source time governs geometry/progress/residence. Host monotonic time governs link
and process health. SITL/Gazebo epoch alignment uses MAVLink TIMESYNC request/response
midpoints, rejecting source-time round trips above 40 ms and fit residuals above
25 ms. This removes the earlier telemetry-reception bias but is NOT a hardware
exposure-time guarantee. Images need bracketed poses, source gaps ≤150 ms
and a nominal maximum age of 200 ms. No extrapolation onto a latest image is used.

## Run the isolated smoke test

Do not run another SITL/mission on the same ports. Gazebo partitions do not isolate
ArduPilot's JSON port 9002 or TCP 5760. No script kills other processes or bypasses
arming checks. Use one movement-command owner. Run from this directory.

Generate the fixture **before** starting Gazebo:

```bash
source scripts/env.sh
python3 -m coverage_mission.fixture --config config/smoke.json --scenario central
```

Separate terminals, in order:

```bash
bash scripts/gazebo.sh
# Headless alternative: bash scripts/gazebo.sh -s --headless-rendering
```

```bash
bash scripts/sitl.sh --no-rebuild
```

```bash
bash scripts/mavproxy.sh
```

Start the independent evaluator before takeoff (it cannot command the vehicle):

```bash
source scripts/env.sh
python3 -m coverage_mission.truth_monitor --seconds 600 --output artifacts/truth_smoke.jsonl
```

Explicit standalone test takeoff, then the mission:

```bash
source scripts/env.sh
python3 -m coverage_mission.takeoff_test --altitude 10
bash scripts/mission.sh --config config/smoke.json --fly --log artifacts/smoke.jsonl
```

Omit `--fly` for inspection only; add `--no-gui` for automated/headless tests.
The default mission config is the full field, so pass `--config config/smoke.json`
when testing the small fixture. Keep config/fixture dimensions identical.
The GUI's `q` aborts. A terminal result requests hold, not disarm or land.
Stop only your test processes after inspecting the result. Restart simulator/SITL
for an independent repeat; restarting the controller alone discards its map/ledger.

Environment scripts use the existing multicast-capable interface without sudo.
Set `COVERAGE_GZ_IP` explicitly if a different interface or the project's configured
multicast loopback is required. The isolated partition is `sae_coverage`.

## Tests and artifacts

```bash
source scripts/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
OPENBLAS_NUM_THREADS=1 python3 -m coverage_mission.replay --scenario central --output artifacts/replay_central
OPENBLAS_NUM_THREADS=1 python3 -m coverage_mission.replay --full --scenario multiple --output artifacts/replay_full_multiple
```

Replay scenarios: `clear`, `central`, `multiple`, `row_end`, `row_end_open`.
The original tight `row_end` layout now completes on the 0.1 m grid without
reducing the 0.40 m body radius or 0.30 m uncertainty reserve. The retained older
0.2 m result was BLOCKED. `row_end_open` is a separate wider-passage test.

Replay renders synthetic
images through a forward camera model and ideal kinematics: it is **not** a Gazebo
or vehicle-dynamics result. Independent truth is never an input to mission planning.

Runtime writes decisions/poses/validity metrics as JSONL, terminal `.result.json`,
and final grid/route ledger `.map.npz`. Truth monitor writes actual simulator poses
and a separate `.report.json`, scoring above 9 m. Its start/end timestamps must
enclose the complete mission; partial traces cannot prove an entire flight.
Fixture snapshots and runtime source/config manifests are saved beside new traces.
Evaluate a completed flight offline (truth pose orientation is needed for geometry):

```bash
python3 -m coverage_mission.evaluate_run --mission artifacts/smoke.jsonl --truth artifacts/truth_smoke.jsonl --output artifacts/smoke.evaluation.json
```

The oracle uses Gazebo camera rays and actual model pose/height. It checks sampled
physical clearance, projected-corner error and coverage at the declared grid
resolution independently of the navigation map. This remains sampled evidence,
not proof between samples or of smaller-than-grid features.

### Owned automated campaigns

These commands start an isolated simulator, normal-arming SITL takeoff, the
headless coverage controller and independent truth recorder. They stop only
their own process groups. Output directories must be new; failed evidence is
never overwritten. GUI operation remains available through the manual launch.

```bash
source scripts/env.sh
python3 -m coverage_mission.campaign --scenario multiple --output artifacts/my_new_run --speedup 2
python3 -m coverage_mission.matrix --group nominal --output artifacts/my_nominal_matrix
python3 -m coverage_mission.matrix --group faults --output artifacts/my_fault_matrix
python3 -m coverage_mission.matrix --group full --output artifacts/my_full_matrix
```

The nominal matrix varies layouts, yaw, airborne entry position (without changing
HOME/origin), FOV and bounded pose error. The fault matrix tests reset aborts,
red-zone deadline failure and pause/resume. The recoverable-fault schedule can
be run separately:

```bash
python3 -m coverage_mission.campaign --scenario clear --faults config/fault_recoverable.json --output artifacts/my_recovery_run --speedup 2
python3 -m coverage_mission.evaluate_faults artifacts/my_recovery_run
```

Fault injection is opt-in and restricted by a harness environment gate; never
enable it on a real aircraft. `--instance 1` isolates campaign ports/partition
when deliberately running a second stack; it does not guarantee enough compute.
Ordinary manual scripts use instance 0 ports and must not overlap an instance 0
campaign. The fixture rejects unsupported non-ideal camera calibrations rather
than silently rendering a different model. Real K/distortion/mount settings are
supported by projection but need a corresponding hardware acquisition adapter.

## Before real hardware

Still required: real-camera adapter and acquisition timestamps, calibrated K and
mounting, clock characterization, estimator-health/failsafe integration, measured
braking/wind/position uncertainty, outdoor color robustness, Pi 5 throughput/thermal
tests, and a validated localization-loss contingency. Do not interpret zero-velocity
commands with invalid localization as a guaranteed hover. Lens finalization can wait;
these hardware commissioning gates cannot be skipped before flight.
