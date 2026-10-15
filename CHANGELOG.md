# Changelog

## [Unreleased]

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
