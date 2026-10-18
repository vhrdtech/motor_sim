# AGENTS.md

Guidance for AI code assistants working in this repository.

## What this is

`motor_sim` is a pure Rust, `no_std`, `libm`-only simulator of electric motors for testing motor controllers in
software: the plant (a PMSM in dq first), an average inverter, mechanics and loads, with sensors (ADC quantisation,
noise, offsets, encoder/hall) and seeded faults kept apart from the true physics. It is deterministic: the same
parameters and seed give the same run on every target. Design background: tpm `hobby/P2630-motor-sim/design/architecture.md`
(architecture and roadmap), earlier `prj/P2401-wireweaver/design/multi_observe.md` ("Motor model crate").

The repo is a Cargo workspace:

- `motor_sim/`: the plant, `no_std`. Must keep building for `thumbv7em-none-eabihf`.
- `motor_demo_api/`, `motor_demo/`: the motor controller demo (DEMO-1), a VESC-like FOC controller driving the plant,
  served as a [wire_weaver](https://github.com/vhrdtech/wire_weaver) device whose ~50 variables a host observes, and
  the `motor_demo` CLI. Moved here from wire_weaver's `examples/`; its docs page stays in wire_weaver
  (`docs/examples/motor_demo.md`, `docs/api/observing.md`), update it there when the demo changes what it shows.
- Later (FEATURES roadmap): `motor_model`, `motor_ident`, `motor_rig`, `motor_fem`, the `motor` CLI, `motor_py` and the
  Bevy twin `motor_twin` (DEMO-2). Heavy std/GUI dependencies go into such crates, never into `motor_sim`.
- `motor_bench/`: `just bench` (MS-8), std only, outside `motor_sim`'s own dependencies. Measures ns per
  `Motor::step` per configuration and the `thumbv7em` code size of `motor_sim` per cargo feature, against
  `motor_bench/baseline.tsv` (committed, machine-specific), flagging a >20% regression; `-- --update` re-measures
  and rewrites it after a deliberate cost change.

The demo depends on wire_weaver crates by path, from a checkout next to this one (`../wire_weaver`, declared once
in the root `[workspace.dependencies]`). `just deps-pull` brings it to its main; when a wire_weaver change breaks
the demo, fix the demo here. Observe tests count per second of simulated device time, never wall-clock rates (the
simulation falls behind real time on a busy PC); `.config/nextest.toml` keeps the 3 s default test limit with one
override for them.

`FEATURES.md` is the feature tracker (ids `MS-n` for the real-time plant, `MDL-n` model data, `CAL-n` calibration,
`FEM-n` offline field tools, `HIL-n` HIL/FPGA/batch, `CTL-n` control toolkit, `DEMO-n` the demo and apps, `CLI-n`,
`EDU-n` education; never renumbered or reused; name them with a short slug when talking to people). Update the
matching item in the same change.

## Commands

```sh
just test       # cargo nextest run --workspace + doctests
just lint       # fmt --check, clippy -D warnings, no_std build of motor_sim for thumbv7em-none-eabihf
just bench      # motor_bench: ns per Motor::step and thumbv7em code size, against baseline.tsv (MS-8)
just install    # tests, then installs the motor_demo CLI
just deps-pull  # bring ../wire_weaver up to its main
just deploy     # deps-pull, bring this checkout up to main, install
cargo run --release -p motor_demo -- observe --compare
```

## Rules

- `motor_sim`: `no_std`, no allocation in the core, `libm` for math, no other dependencies without a reason in the commit.
- f32 for the state, f64 for accumulators that grow (multi-turn angle, Ah/Wh, time).
- Units in names or docs of every quantity (SI: V, A, rad/s, N·m, s); the variable visitor reports name and unit.
- Shared dependencies and package metadata live in the root workspace (`dep.workspace = true`).
- Every user-visible change gets an entry under `## [Unreleased]` in `CHANGELOG.md`; commits are Conventional
  Commits with a scope.
