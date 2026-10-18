# Changelog

## [Unreleased]

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
