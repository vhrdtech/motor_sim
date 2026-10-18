# Features

Single source of truth for what motor_sim does and what is planned (see [AGENTS.md](AGENTS.md)).

Status: ✅ done, 🚧 partial, 🐛 buggy, 📋 planned, 💡 idea.

## Plant (`MS`)

- ✅ **MS-1 dq PMSM L1**: dq model solved exactly per step (ω frozen at the predicted mid-step speed, rotating
  αβ voltage included; RK4 compared in `tests/discretisation.rs`), average inverter (duty, αβ, phase volts, off;
  SVPWM saturation), mechanics (inertia, viscous and Coulomb friction), loads (constant, viscous, fan, second
  inertia via spring, time profile, dyno speed, locked), sensors (current and Vdc ADC with offsets, noise and
  quantisation, encoder, halls), seeded faults on sensor and inverter paths, DC link Ah/Wh and loss accumulators,
  variable visitor with unit and rate tier, presets (VESC default outrunner, industrial SPM, IPM)
  (P2401#15oct-1827).
- 💡 **MS-2 abc and BLDC L2**: phase-frame model with trapezoidal back-EMF and per-phase R/L, for BLDC motors and
  six-step commutation.
- 💡 **MS-3 thermal**: winding and magnet temperature (lumped RC), R and λ following temperature, a temperature
  sensor.
- 💡 **MS-4 DC link sag**: battery with internal resistance and a DC link capacitor instead of the ideal source.
- 💡 **MS-5 switching inverter L3**: gate-level legs, dead time, sampling instant within the PWM period, phase
  faults (open phase, shorted switch).
- 💡 **MS-6 diode conduction**: with the inverter off, the back-EMF drives current through the freewheeling diodes
  once it exceeds the DC link (today the currents drop to zero).

## Demo (`DEMO`)

- ✅ **DEMO-1 Motor controller demo**: `motor_demo` (+ `motor_demo_api`), a VESC-like FOC controller on `motor_sim`
  with 52 `+observe` properties in three trait levels by rate tier (25 kHz, 1 kHz, 50 Hz), their values `ww_si`
  quantities, on simulated time faster or slower than real time, in-process and over UDP; the `motor_demo` CLI
  observes chosen sets at chosen rates and shows achieved rates, bytes against the budget, missed sends and the cost
  as one `StreamData` per value. Moved from wire_weaver (its EX-11) with history (P2401, 18 Oct 2026); docs page:
  wire_weaver `docs/examples/motor_demo.md`.
- 💡 **DEMO-2 Simulator GUI**: a desktop app with a 3D view of the motor and load, live plots and controls, on top
  of `motor_sim` and the demo's wire_weaver device; its heavy dependencies are why the demo lives in this repo.
