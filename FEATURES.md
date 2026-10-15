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
