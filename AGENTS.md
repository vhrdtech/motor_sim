# AGENTS.md

Guidance for AI code assistants working in this repository.

## What this is

`motor_sim` is a pure Rust, `no_std`, `libm`-only simulator of electric motors for testing motor controllers in
software: the plant (a PMSM in dq first), an average inverter, mechanics and loads, with sensors (ADC quantisation,
noise, offsets, encoder/hall) and seeded faults kept apart from the true physics. It is deterministic: the same
parameters and seed give the same run on every target. First user: the wire_weaver EX-11 motor demo (a VESC-like
FOC controller exposed as a wire_weaver device). Design background: tpm `prj/P2401-wireweaver/design/multi_observe.md`
("Motor model crate").

`FEATURES.md` is the feature tracker (ids `MS-n`, never renumbered or reused; name them with a short slug when
talking to people). Update the matching item in the same change.

## Commands

```sh
just test     # cargo nextest run + doctests
just lint     # fmt --check, clippy -D warnings, no_std build for thumbv7em-none-eabihf
just install  # library only: builds and tests, nothing to install
just deploy   # bring this checkout up to main
```

## Rules

- `no_std`, no allocation in the core, `libm` for math, no other dependencies without a reason in the commit.
- f32 for the state, f64 for accumulators that grow (multi-turn angle, Ah/Wh, time).
- Units in names or docs of every quantity (SI: V, A, rad/s, N·m, s); the variable visitor reports name and unit.
- Every user-visible change gets an entry under `## [Unreleased]` in `CHANGELOG.md`; commits are Conventional
  Commits with a scope.
