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
- 💡 **MS-2 BLDC trapezoidal**: BLDC motors on the MS-7 engine: trapezoidal back-EMF as a harmonic spectrum in the
  flux map, per-phase R/L asymmetry in the phase frame, six-step commutation from halls with the floating phase's
  terminal voltage visible (for zero-crossing sensorless controllers).
- 💡 **MS-3 thermal network**: a generic lumped thermal network (N nodes, implicit, ~1 kHz in the multirate schedule)
  for winding, stator, magnet, housing, bearings and the power stage, from a geometry-based template or reduced from
  3D FEM (FEM-16); R, λ, Br and friction follow temperature (MS-11); NTC and thermocouple sensors.
- 💡 **MS-4 DC link**: battery (OCV vs SOC, internal resistance, RC pairs, warming), cable, DC link capacitor with
  ESR, brake chopper and overvoltage in regen, instead of the ideal source.
- 💡 **MS-5 switching inverter**: legs as exact switching segments between known edges (up to ~7 per PWM period),
  each solved exactly, so the CPU path is switching-exact without 1 µs steps; dead time with current-sign-dependent
  voltage and zero-crossing split, sampling instant within the period (with MS-14), phase faults (open phase,
  shorted switch).
- 💡 **MS-6 diode conduction**: with the inverter off or in dead time, the back-EMF drives current through the
  freewheeling diodes once it exceeds the DC link, charging it (today the currents drop to zero).
- 📋 **MS-7 layered flux-state engine**: the plant as layers (drive, power stage, electromagnetics, mechanics,
  thermal, NVH, sensors) with per-layer levels and cargo features; a magnetic n-port generic over its frame (dq0 or
  phase) with flux as state, current from an inverse map and an exponential step around the local incremental
  inductance (closed form like today, linear maps reproduce MS-1); inverter volt-seconds over any interval so the
  step is independent of the PWM period; multirate skeleton; `Motor` stays the façade with every MS-1 capability.
- 📋 **MS-8 cost bench**: per-feature cost per step (ns on the host, code size for thumbv7em), a tracked table that
  every new feature adds a row to and that flags regressions.
- 💡 **MS-9 saturation maps**: ψd, ψq (id, iq) with cross-saturation and incremental inductances from tables (uniform
  grids), saliency that moves with load (for HF-injection sensorless tests).
- 💡 **MS-10 spatial harmonics**: flux and torque maps over rotor angle (dense 3D or 2D + Fourier terms): back-EMF
  harmonics, cogging, torque ripple, radial force waves for NVH.
- 💡 **MS-11 temperature dependence**: copper R(T), magnet λ(T) and coercivity (T), from the thermal network.
- 💡 **MS-12 losses**: AC copper (skin, proximity, PWM ripple), iron loss as a loss current from Steinmetz/Bertotti
  maps (acts on torque and dynamics), magnet eddy loss, mechanical losses; a loss breakdown in `Truth`.
- 💡 **MS-13 demagnetisation**: irreversible demagnetisation as a slow state from the magnet working point (minimum
  B vs id, iq, T from FEM), leaving a permanently weaker motor.
- 💡 **MS-14 MCU peripherals**: what a controller writes and reads, register-like: PWM timer (period, compares,
  centre/edge aligned, dead time), ADC trigger instants and sample-and-hold, shunt topologies (3 and 2 low-side,
  single shunt, inline), encoder/hall timer capture, latencies; `Drive` stays as the convenience.
- 💡 **MS-15 power stage losses**: Rds(on)(T), diode drop, Eon/Eoff tables, gate driver delay, bootstrap droop,
  power stage temperature through MS-3.
- 💡 **MS-16 windings and plant faults**: star, delta (circulating zero-sequence current), open winding; plant faults
  as a third seeded category: open phase, inter-turn short, unbalanced phases, demagnetised pole.
- 💡 **MS-17 hybrid stepper**: two-phase, ~50 pole pairs, detent torque, saturation, H-bridge drive with chopper
  current regulation and microstepping, mid-band resonance, lost steps.
- 💡 **MS-18 can-stack stepper**: PM claw-pole stepper with large detent torque and strong harmonics.
- 💡 **MS-19 brushed DC**: commutator segments, brush contact drop, commutation ripple (for ripple counting),
  armature reaction, H-bridge drive.
- 💡 **MS-20 induction motor**: rotor flux states, saturated magnetising flux, deep-bar ladder, rotor R(T), rotor
  slot harmonics in the current.
- 💡 **MS-21 SynRM**: synchronous reluctance on the flux-map engine with strong cross-saturation; preset and tests.
- 💡 **MS-22 switched reluctance**: phase-frame ψ(i, θ) tables per phase, asymmetric half-bridge converter.
- 💡 **MS-23 other machines**: coreless/slotless, axial flux, voice coil and linear motors.
- 💡 **MS-24 drivetrain**: torsional multi-body chain: inertias, shafts, couplings, gears with ratio, mesh stiffness,
  direction-dependent efficiency and backlash, belts with slip, clutches.
- 💡 **MS-25 friction models**: Stribeck, LuGre (presliding), bearing friction by load, speed and temperature, seals,
  air-gap windage.
- 💡 **MS-26 load library**: propeller CT/CQ tables with inflow (P2508), fan and pump curves, robot joint with
  gravity and link inertia (P2472), wheel and vehicle, spring, caliper, powder and eddy-current brakes, a second
  plant as dynamometer (motor-as-dyno).
- 💡 **MS-27 rotor dynamics**: lateral (Jeffcott) rotor on bearing stiffness, static and dynamic unbalance, static and
  dynamic eccentricity with unbalanced magnetic pull, bearing defect impulses, balancing.
- 💡 **MS-28 vibration**: modal synthesis of stator and housing driven by radial force waves, torque ripple and
  bearing forces; accelerometer outputs.
- 💡 **MS-29 sound**: electromagnetic noise, PWM whine, bearing, gear mesh and aerodynamic sources rendered to
  pressure at microphone positions at 48 kHz.
- 💡 **MS-30 sensor detail**: resolver with excitation and demodulation, magnetic encoders (nonlinearity, latency),
  encoder eccentricity, hall placement errors, current sensor bandwidth.
- 💡 **MS-31 high-frequency effects**: winding capacitance to frame, common-mode current, shaft voltage and bearing
  currents, reflected-wave overvoltage on long cables.

## Model data (`MDL`)

- 💡 **MDL-1 model format**: a motor model as `motor.toml` (parameters with unit, provenance, uncertainty, reference
  temperature, validity range) plus tables, compiled into one versioned blob that the `no_std` core reads zero-copy.
- 💡 **MDL-2 real presets**: documented presets with sources: Prius 2004 IPM benchmark, a drone motor, a robot
  actuator motor, NEMA17 hybrid stepper, can-stack stepper, brushed DC, small induction motor.
- 💡 **MDL-3 analytic map generators**: flux maps, harmonics and cogging from a few parameters (saturation knee,
  cross-coupling, EMF spectrum, slot/pole combination), so every feature runs before FEM.
- 💡 **MDL-4 tolerance instances**: Monte Carlo motor instances from the model's uncertainties (magnet strength per
  pole, placement, air gap, resistance).
- 💡 **MDL-5 model upload**: send a model blob to a HIL plant over wire_weaver.

## Calibration (`CAL`)

- 💡 **CAL-1 log format**: one format for virtual and real rig logs (stream groups plus a sidecar naming experiment,
  rig, sensors, motor and temperatures).
- 💡 **CAL-2 virtual rig**: a plant, a load motor, rig sensor models (torque sensor, encoder, probes, thermocouples,
  accelerometers, microphones) and experiment scripts writing rig-like logs.
- 💡 **CAL-3 L1 identification**: R, Ld, Lq, λ, inertia, friction, encoder offset from virtual-rig experiments, with
  uncertainty; a CI round trip truth → logs → fit → compare.
- 💡 **CAL-4 flux map identification**: dyno constant-speed current grid for ψ and torque maps, back-EMF harmonics,
  cogging, torque ripple.
- 💡 **CAL-5 thermal identification**: heat steps with resistance thermometry → thermal network fit.
- 💡 **CAL-6 losses and efficiency map**: loss separation and measured efficiency maps.
- 💡 **CAL-7 self-commissioning**: `no_std` on-drive identification like InstaSPIN (Rs, Ls d/q, flux, inertia,
  friction, inverter voltage error), usable in firmware and in `motor_demo`.
- 💡 **CAL-8 validation and model cards**: held-out experiments, waveform, harmonic, efficiency, thermal and spectrum
  metrics, a model card per fitted motor.
- 💡 **CAL-9 family-specific identification**: steppers (detent, holding torque, resonance sweep), brushed DC,
  induction (no-load, locked rotor).
- 💡 **CAL-12 vibration and sound identification**: tap test for modes, run-up order tracking with accelerometers and
  microphones.
- 💡 **CAL-13 real rigs**: adapters for P2439 motor-stand and P2508 prop-stand running the same experiment scripts.

## Offline field tools (`FEM`)

- 💡 **FEM-10 geometry and windings**: parametric motor geometry (SPM, IPM, slots, magnets) and windings, winding
  function analysis (MMF harmonics, winding factors, cogging orders); the twin draws the same geometry.
- 💡 **FEM-11 2D magnetostatics**: nonlinear A_z solver in Rust (pure-Rust mesh and sparse solver, moving band,
  Maxwell stress and virtual-work torque).
- 💡 **FEM-12 table generation**: forward and inverse flux maps, torque, air-gap force waves and flux density
  waveforms over current and angle, using symmetry and periodicity.
- 💡 **FEM-13 loss post-processing**: iron (Bertotti per element) and magnet eddy losses into loss maps.
- 💡 **FEM-14 skew and end effects**: multi-slice 2D for skew, analytic end-winding inductance.
- 💡 **FEM-15 cross-checks**: the same geometry in FEMM (xfemm or wine) and pyleecan, and the Prius 2004 published
  data.
- 💡 **FEM-16 thermal FEM**: 3D heat conduction and model-order reduction into the MS-3 network; a geometry-based
  network template first.
- 💡 **FEM-17 structural modes and radiation**: analytic ring/cylinder modes, FEM modes later, radiation per mode, for
  MS-28 and MS-29.
- 💡 **FEM-18 3D magnetics**: axial-flux and claw-pole machines and end effects through Elmer as an external tool.

## HIL, FPGA and batch (`HIL`)

- 💡 **HIL-7 lockstep HIL**: controller firmware on its MCU triggered by the plant over wire_weaver on simulated
  time, deterministic.
- 💡 **HIL-8 MCU plant**: `motor_sim` on a fast MCU board: PWM capture in, DAC or emulated SPI ADC, encoder and hall
  out, 100 kHz for L1/L2.
- 💡 **HIL-9 fixed-point golden model**: bit-accurate Rust model of the FPGA arithmetic, checked against the f32 core.
- 💡 **HIL-10 FPGA plant**: electrical layer and gate capture on an FPGA at 50-200 ns steps from the same model blob.
- 💡 **HIL-11 batch runs**: parameter sweeps and Monte Carlo on all cores (rayon), GPU through the same `no_std` core
  later.
- 💡 **HIL-12 live rig twin**: a fitted model running beside the real motor on the same inputs, residuals live and in
  rig CI.

## Demo (`DEMO`)

- ✅ **DEMO-1 Motor controller demo**: `motor_demo` (+ `motor_demo_api`), a VESC-like FOC controller on `motor_sim`
  with 52 `+observe` properties in three trait levels by rate tier (25 kHz, 1 kHz, 50 Hz), their values `ww_si`
  quantities, on simulated time faster or slower than real time, in-process and over UDP; the `motor_demo` CLI
  observes chosen sets at chosen rates and shows achieved rates, bytes against the budget, missed sends and the cost
  as one `StreamData` per value. Moved from wire_weaver (its EX-11) with history (P2401, 18 Oct 2026); docs page:
  wire_weaver `docs/examples/motor_demo.md`.
- 💡 **DEMO-2 Bevy twin**: `motor_twin`, a 3D digital twin in Bevy on top of `motor_sim`, in process or on a
  wire_weaver device (the demo, a HIL plant, a real rig): the motor from FEM-10 geometry, rotor in slow motion or
  strobe, dq phasors, heat colours, the load, egui plots and controls, time controls; its heavy dependencies are why
  the demo lives in this repo.
- 💡 **DEMO-8 field view**: flux density on the cross-section from FEM field tables, live with the operating point.
- 💡 **DEMO-9 twin sound**: MS-29 microphone signals played as spatial audio.
- 💡 **DEMO-10 rig scene**: the P2439 stand with sensors, cameras and lights; a real rig's live data beside the twin.
- 💡 **DEMO-11 WASM**: core and twin in the browser, nothing to install.
- 💡 **DEMO-12 replay and faults**: deterministic replay from seed and input log, fault injection from the UI.

## CLI (`CLI`)

- 💡 **CLI-2 motor CLI**: `motor` with model build/check/show, fem, ident, rig, validate and bench subcommands
  (completions, colour, progress bars, `--version`).

## Education (`EDU`)

- 💡 **EDU-6 Python wrapper**: `motor_py` (PyO3, maturin, uv) over the core, models, identification and the
  virtual rig, for notebooks.
- 💡 **EDU-7 lab book**: an mdBook with step-by-step labs on the core, the twin and the notebooks.
- 💡 **EDU-8 reproducible labs**: seeded runs with expected answers and a checker.
- 💡 **EDU-9 preset docs**: every preset documented with its real parameters and sources.
- 💡 **EDU-10 student rig**: a cheap student motor stand (a P2439 variant).
