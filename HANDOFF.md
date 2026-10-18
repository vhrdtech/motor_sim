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

- [x] step 1 plan (this file)
- [x] step 2-4 restructure, engine, Plant, grids + MapMachine, carrier, multirate (32db5e2); `core.rs` became `stack.rs` (a module named `core` shadows `::core`)
- [x] step 5 tests in tests/engine.rs (5f9d49b + docs commit): MS-1 closed form to <1e-6 of peak, LinearDq Ldq≠0 1-50 µs vs RK4 < 4e-6, saturated map second order (7.7e-4 at 50 µs, 2.9e-5 at 10 µs), energy balance with 2D maps 1e-4 / 3D maps 1e-3 incl. a saturating pulse, carrier 10 µs vs per-period steps 2.6e-4 of peak, params_mut keeps currents
- [ ] waiting: MS-8 bench (session ms-bench) to land on main, then `git rebase main`, add the engine rows (L1 Motor, Plant<LinearDq>, Plant<MapMachine Grid2/Grid3>, carrier at 10 µs), check L1 within 20 % of base
- [ ] waiting: demo fix (batching-docs) to land so `just test` builds motor_demo; then just test + just lint
- [ ] FEATURES MS-7: set done if the plan branch's entry is on main after rebase, else say "flip MS-7 after the plan lands" in the ready note
- [ ] `git rm HANDOFF.md`, `tpm work ready`

## Decisions

- Flux state is x = ψ − ψm(θ) (relative to the magnet flux): no f32 cancellation of the small winding flux against λ.
- The step is an increment (E − I via expm1 and half angles, rotation − I): rounding scales with the change per step; at 1 µs the error fell from 1e-4 to 3e-6 of peak, and the old IPM discretisation test error went from 2.4e-3 A to 2.5e-4 A.
- Machine Jacobian from the tables' in-cell gradient (no Jacobian tables): with the exact Jacobian of the interpolant the step is exponential Rosenbrock-Euler, second order.
- Frames are two-axis (star without neutral). Zero sequence / open winding / SRM phases need an n×n step (no 2×2 closed form): left for MS-16/MS-21.
- No `thermal`/`nvh` cargo features yet: their layers are empty slots (levels `Off`), and cfg'd pub fields would break feature unification; the features come with content (MS-3, MS-28). `maps` is the one feature now.
- Motor keeps its own `magnetic_energy_j` formula (from Ld, Lq and the truth currents) to stay bit-compatible; Plant's uses the map's field energy.
- Perf on gpd-omarchy, best of 15 pinned: MS-1 208-215 ns/step, new 224 ns/step (+5-8 %).

## Gotchas

- `just test` fails today in motor_demo: wire_weaver main has API-39 `ObserveMode::Samples`; the fix is on branch s/18oct2026-batching-docs (another session, ready to land). Test with `cargo nextest run -p motor_sim` until it lands, then rebase.
- A/B timing harness outside the repo: $CLAUDE_JOB_DIR/tmp/bench (base = detached worktree of 424efa3 at $CLAUDE_JOB_DIR/tmp/base, its version patched to 0.0.99 to avoid a lockfile collision; `git worktree remove --force` it at the end). The PC is shared: pin with taskset and take the best of many runs.
