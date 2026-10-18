# HANDOFF: ms-engine (MS-7 layered engine)

Session ms-engine, branch s/18oct2026-ms-engine, task P2630#18oct-1240. Design: tpm hobby/P2630-motor-sim/design/architecture.md.

## Plan

Module layout (layers of the design, files under `motor_sim/src/`):

- `lib.rs`: `Motor` façade (API unchanged), `Truth`, re-exports. `Motor { params: MotorParams, core: Core }`; every step calls the generic core with `&params.pmsm` as the machine.
- `plant.rs`: `Plant<M: Machine>` for what L1 can't express (non-diagonal L, tables, phase frame): same methods as `Motor` (step, truth, sensors, visit, load, faults, energies) plus `machine()`, `machine_mut()`, `flux()`. `PlantParams<M> { machine, mech, inverter, sensors, max_substep_s }`.
- `core.rs`: `Core` = the state of all layers (truth, flux state, mechanics, load, faults, sensors, rngs, carrier, slow clocks) and `Core::step<M: Machine>(&mut self, machine, params..., drive, dt)` running the layers in fixed order per substep: power stage → electromagnetics → mechanics → slow layers (multirate) → sensors once per step.
- Layer 1 drive: `drive.rs` (`Drive`, moved from inverter.rs).
- Layer 2 power stage: `power/` (`inverter.rs` today's duty stage; `carrier.rs` PWM carrier, volt-seconds over any interval).
- Layer 3 electromagnetics: `em/` (`frame.rs` `Frame` trait + `Dq`, `AlphaBeta`; `machine.rs` `Machine` trait; `etd.rs` the 2×2 exponential step; `linear.rs` `PmsmParams` as a `Machine`, `LinearDq` (constant full 2×2 L), `LinearAlphaBeta`; `table.rs` `MapMachine<G, F>` behind feature `maps`).
- `grid.rs`: `Axis`, `Grid2<'a>`, `Grid3<'a>` (uniform axes, borrowed `&'a [f32]`, bi/trilinear value + gradient, periodic θ axis), feature `maps`.
- Layer 4 mechanics: `mechanics.rs` unchanged.
- Layer 5 thermal, layer 6 NVH: `thermal.rs`, `nvh.rs`, level enums `#[non_exhaustive]` with `Off` only, slots in the multirate schedule; features `thermal`, `nvh` (default on) for MS-3/MS-28 to fill.
- `multirate.rs`: `SlowClock { every: u32 }` integer ratio of base substeps, accumulates its own h, fixed order.
- Layer 7: `sensors.rs`, `faults.rs`, `visit.rs` unchanged.

Engine math (dq frame rotating at ω_f = ω, αβ frame ω_f = 0), state x = ψ − ψm(θ) (flux minus open-circuit flux: no cancellation against λ in f32):

- dx/dt = v − R·i − ω_f·J·(x + ψm) − ω·ψm′, i = I(x, θ), J = [[0,−1],[1,0]].
- Linearise at the substep start: i ≈ i0 + Γ·y + g·ω·t (Γ = ∂I/∂x, g = ∂I/∂θ), ψm ≈ ψm + ψm′·ωt: dy/dt = A·y + Re(W·e^{−jω_f t}) + c + d·t, A = −RΓ − ω_fJ. Exponential Rosenbrock-Euler: exact for linear maps, 2nd order with exact Jacobian.
- Closed form: e^{Ah} as today's step_exact, sinusoid via (A + jω_f)⁻¹, ramp via A⁻¹. Then i_new = I(x_new, θ_new) from the map.
- Torque/(1.5·p) = [rotating] (ψ×i) + i·ψm′ − ∂W̃/∂θ|x, W̃ field energy with i = ∂W̃/∂x. Energy balance by construction for consistent maps.
- `machine_mut`/`params_mut` keep the currents continuous (flux re-derived from the currents at the next step), like MS-1.

Inverter: `Carrier { period_s, align: Center|Edge }`; `Motor::set_carrier(Some(..))` makes duties compare values held for the step, applied through the carrier from the motor's time: the step's average duty is the leg's exact high time over [t, t+h]. `None` (default) is MS-1.

## Status

- [ ] step 1 plan (this file)
- [ ] step 2 restructure + engine + Plant, MS-1 tests unchanged
- [ ] step 3 grids + MapMachine
- [ ] step 4 carrier + multirate
- [ ] step 5 new tests, bench, docs

## Gotchas

- `just test` fails today in motor_demo: wire_weaver main has API-39 `ObserveMode::Samples`; the fix is on branch s/18oct2026-batching-docs (another session). Test with `cargo nextest run -p motor_sim` until it lands, then rebase.
- A/B timing harness outside the repo: $CLAUDE_JOB_DIR/tmp/bench (base = detached worktree of 424efa3). gpd-omarchy baseline ~155-170 ns/step at 10 µs.
- MS-8 bench (session ms-bench) not on main yet; add rows after rebasing.
