# Changelog

## [Unreleased]

### Changed

- `just install` builds `motor_demo` and `motor_cad` with a `deploy` profile (release plus incremental) and puts both on PATH once through `tpm install` when tpm is there (else `cargo install --locked`) instead of `cargo install` (`motor_cad` was not installed by the documented flow before); the tests run in `just deploy` instead, which also refuses on local changes and checks both `--version`s against main's commit.
- `motor_demo`: the `foc` level samples through wire_weaver's observe cells (API-50, `observe_cells = "root_foc"`):
  each step writes the 21 values into a ring of atomics and commits the row, `poll_properties` reads it; `control`
  stays on its sampler, so the demo runs both paths. Same wire and rates: `observe --foc samples:20ms` sends
  811.6 kB/s in 781 updates/s, as before. Needs wire_weaver with observe cells (its branch
  `s/18oct2026-observe-cells`, landing together).

## 2026-10-18: motor_sim 0.2.0, motor_demo 0.1.2, motor_bench 0.1.1

### Added

- Layered engine (MS-7): `Plant<M: Machine>` runs the plant as a stack of layers (drive, power stage,
  electromagnetics, mechanics, thermal and NVH slots, sensors) exchanging plain-data values once per base substep.
  A `Machine` is a magnetic two-port in its `Frame` (`Dq` or `AlphaBeta`) with flux as state, current from its map
  and torque from the co-energy: `PmsmParams`, `LinearDq` (full inductance matrix, mutual `Ldq`),
  `LinearAlphaBeta`, and `MapMachine` on uniform tables `Grid2`/`Grid3` over borrowed slices (feature `maps`,
  default on).
- `Carrier` (centre or edge aligned) with `set_carrier`: duties become compare values and every substep gets the
  legs' exact volt-seconds, so steps need not be one PWM period.
- `SimClock` (`Plant::clock`: base ticks and simulated time) and `Plant::snapshot`/`restore` (`PlantState`, a copy
  of the whole state) for pause, step, rewind and replay; a restored run repeats bit for bit.
- `ThermalParams`/`NvhParams` with their levels (`Off` for now), run at integer rate ratios in a fixed order.
- `motor_bench`: MS-7 engine rows (full inductance matrix, stationary frame, carrier, 2D and 3D maps) and the code
  size without `maps`; baseline re-measured on a quiet gpd-omarchy after the engine change (+8-14% per L1 step).

### Changed

- **Breaking:** `Motor` is `Plant<PmsmParams>` and `MotorParams` is `PlantParams<PmsmParams>`; its field `pmsm` is
  now `machine`, and it has `thermal` and `nvh` (presets set them off). `with_machine` gives the same parameters
  with another machine. Every MS-1 capability and preset stays, with the same results: the exponential step
  reproduces MS-1's closed form to 1e-6 of the peak current and is formed as an increment, so f32 rounding no
  longer piles up over many short steps. `Motor::magnetic_energy_j` comes from the machine's field energy.
- `motor_demo` follows the `motor_sim` 0.2 API (`params.machine` instead of `params.pmsm`); no behaviour change.

## 2026-10-18: motor_cad 0.1.0, motor_twin 0.1.0

New crates; per-crate versions (the workspace has no single one yet, P2630#18oct-1556).

### Added

- `motor_cad` (DEMO-13, CLI-3): STEP assemblies into classified motor parts (names, instances, colours, layers,
  units; roles with confidence and reasons; `<model>.motor.toml` overrides), a Markdown report, the scene as `.glb`,
  parametric 12N14P outrunner and NEMA17 stepper written to STEP; the `motor_cad` CLI (`import`, `init-overrides`,
  `gen`). Geometry finds stacked laminations drawn plate by plate (vendor NEMA34), and a name over many bodies of
  one product counts less than each body's geometry. The naming, colour and axis convention for our own models and
  how to export STEP from SolidWorks and FreeCAD: `motor_cad/README.md`.
- `motor_twin` (DEMO-2, first slice): Bevy view of a motor from `motor_cad`: rotor spinning at a set rpm, role
  toggles, exploded view, cutaway, `--screenshot` headless PNG. Outside the default members: `just lint-twin`,
  `just twin FILE`; `just test` and `just lint` cover the default members only.

## [0.1.1] - 2026-10-18

### Added

- `motor_demo` observes every measured `Foc` and `Control` value in `Samples` mode (API-41/API-39,
  `+observe+batched`, see motor_demo_api/CHANGELOG.md): `device.rs` samples `FocSamples`/`ControlSamples` once
  per tier through `ObserveState::samplers()`, `stats.rs`'s `levels!` table and `present()` carry
  `Option<BatchedOwned<T>>` slots (one `Present` per sample of a batch, oldest first), a new test
  (`samples_mode_gets_every_sample`) exercises it end to end. Measured (`Options::default`, `CodecPolicy::SMALLEST`,
  n = 32): `foc` 3.7 MB/s unbatched (periodic, every sample) down to 1.0 MB/s batched (3.7x), `control` 99 kB/s
  down to 26 kB/s (3.8x) — see wire_weaver's `docs/api/batching.md` for the full numbers and how they compare to
  the design's prediction (`tpm prj/P2401-wireweaver/design/observe_batching_measure.md`).
- `motor_bench` (MS-8): `just bench` measures ns per `Motor::step` per configuration (preset × load × dt ×
  sensors, thread CPU time) and the `thumbv7em-none-eabihf` `.text` size of `motor_sim` per cargo feature,
  against a committed `baseline.tsv` that flags a >20% regression; `-- --update` re-measures and rewrites it.

### Changed

- `motor_demo` on wire_weaver's shared observe clock (API-49): `stats.rs` pairs `BatchedOwned::Samples` with the
  update's `sample_times` (`Observed::sample_times`; a count mismatch is left out, as the client does), the level
  functions and `Stats::record` take them, and the message bytes count them in the `PropertyUpdate` head.
  Measured the same way: `foc` 983 kB/s → 812 kB/s (-17 %, 4.6x below unbatched), `control` 25.8 kB/s → 19.0 kB/s
  (-26 %, 5.2x), and `control`'s properties now seal together (48 updates/s instead of 238).

### Fixed

- `motor_demo` builds again against wire_weaver's API-39 batched observe: `device::LIMITS` carries
  `samples_max_age_us` / `samples_min_every_us`, `args::mode` / `mode_str` parse and print `ObserveMode::Samples`
  as `samples:20ms` (every sample) or `samples:1ms/20ms` (decimated).
- `samples_mode_gets_every_sample` no longer assumes device (simulated) time tracks wall-clock time over a fixed
  sleep, which went flaky under a loaded PC (fewer simulated PWM periods ran in the same real time); it now waits
  on `i_q`'s own samples to span a fixed amount of *device* time and checks `received + missed` against that span
  exactly (every sample received once, in order). `stats::Present` carries a `+batched` sample's own device time,
  `stats::PropStats` gets `first_t`/`last_t`/`out_of_order` from it.

## [0.1.1] - 2026-10-18

### Changed

- `motor_demo_api` 0.3.0: motor_demo_api/CHANGELOG.md, section Unreleased

## [0.1.1] - 2026-10-18

### Changed

- The repo is a Cargo workspace: the plant moved to `motor_sim/`, its package and API are unchanged.

### Added

- `motor_demo` and `motor_demo_api` (DEMO-1), moved here with their history from wire_weaver's `examples/` (EX-11
  there): a VESC-like FOC controller on `motor_sim` observed over wire_weaver, and the `motor_demo` CLI. They build
  against a sibling `../wire_weaver` checkout; `just deps-pull` updates it and `just deploy` calls it.

## [0.1.0] - 2026-10-15

### Added

- dq PMSM plant (MS-1): `Motor::step` solves the electrical equations exactly per step, with an average inverter
  (`Drive::Duty`, `AlphaBeta`, `PhaseVolts`, `Off`), rotor mechanics and loads (`Load::Constant`, `Viscous`,
  `Fan`, `Inertia`, `Profile`, `Speed`, `Locked`).
- `Truth` (true state, DC link Ah/Wh, copper, friction and load energy) apart from `Sensors` (current and DC link
  ADCs with offsets, noise and quantisation, encoder, halls).
- Seeded `Faults` on the sensor and inverter paths (stuck or glitching channels, offsets, gain, missed encoder
  counts, stuck legs, missed pulses); the same seed gives the same run on every target.
- `Motor::visit` lists every variable with name, unit and rate tier.
- Presets: VESC default hobby outrunner, small industrial SPM, IPM.
