# HANDOFF: ms-bench (MS-8)

## Done

`motor_bench` (new workspace crate, std only, not a `motor_sim` dependency): `just bench` runs it.

- 24 timing configurations: 3 presets (vesc, spm, ipm) × load (none, fan) × dt (10/40 µs) × sensors (on/off),
  min of 7 repeats of 1,000,000 `Motor::step` calls each, timed via this thread's own CPU time
  (`CLOCK_THREAD_CPUTIME_ID`), not wall-clock.
- 1 code-size configuration today (`codesize.default`): `.text` bytes of `motor_sim`'s compiled rlib for
  `thumbv7em-none-eabihf` (`size -t`, not a linked binary, but consistent run to run). `SIZE_CASES` in
  `motor_bench/src/main.rs` is where MS-7's layers add a row per cargo feature as they land.
- `motor_bench/baseline.tsv` is committed. Plain run: prints the table, grows the file with any new
  configuration, exits non-zero if an existing one got more than 20% slower (`REGRESSION_FACTOR`).
  `-- --update` re-measures and rewrites every row (for a deliberate, reviewed cost change, or to re-baseline
  on a different machine).
- `README.md`, `AGENTS.md` (workspace crate table + `just bench` under Commands), `CHANGELOG.md` updated.
  FEATURES.md MS-8 is deliberately **not** touched here — see "FEATURES.md" below.

## Decisions

- **"sensors off" isn't a skipped code path.** `Motor::step` always calls `sample_sensors()`; there is no
  toggle in `motor_sim` today to skip it, and adding one felt like scope creep onto MS-7 (the engine-layering
  refactor running in parallel, which is exactly where that kind of gate belongs). "Off" here means the
  cheapest a controller could configure `SensorParams` (`current_noise_a` / `vdc_noise_v` = 0, `encoder_cpr` =
  0), not a different code path. Measured difference is within the noise floor, which is itself the honest
  finding: today sensor sampling cost doesn't depend on the noise/encoder config. Re-visit this row's meaning
  once MS-7 or a later feature actually makes sensors gateable.
- **Code size is measured on the unlinked rlib**, not a linked, dead-code-eliminated binary. Building a real
  `#![no_main]` probe binary would need a linker script, `cortex-m-rt`, a panic handler — real embedded
  scaffolding this repo doesn't have yet, for a "small, lands first" bench crate. `size -t` on the rlib is a
  consistent, zero-extra-dependency proxy for relative regression tracking; it will overstate true flash
  footprint (no linker GC). If a later feature needs real linked-size numbers, that's worth its own task, not
  a quiet scope-add here.
- **Timing is CPU time, not wall-clock**, specifically because gpd-omarchy runs several sibling sessions
  concurrently (confirmed via the RAM-usage hook mid-session: other sessions' `cargo test` runs were peaking
  at 5-14 GB RAM, with system load average 20-35 on 24 cores throughout this work). Wall-clock `Instant`
  produced 2-3x swings between consecutive runs with zero code change — useless for a 20%-regression gate.
  CPU time plus a 7-repeat minimum fixed scheduler-preemption noise, but **not** a machine-wide CPU frequency
  drop under sustained load, which still moves every row together, cross-run. See the crate doc comment in
  `motor_bench/src/main.rs` for what this means for trusting a flagged regression.
- **The committed baseline** is the row-wise minimum across four independent full-suite captures taken during
  this session (machine load fluctuating the whole time) — the same "minimum beats noise" logic as the
  in-process repeats, just applied across runs. It's in the right range for a sanity check (~135-220 ns,
  matching the plan session's ~190-215 ns estimate from omarchy-m1, a different machine) but don't treat it as
  a clean, low-noise reference. **Re-run `just bench -- --update` once the machine is quiet** before relying
  on it for a tight regression call; until then, treat any single flagged regression with suspicion and look
  at whether it reproduces.
- Chose a flat `name\tvalue` TSV over TOML/JSON for `baseline.tsv`: no new dependency, and the file is just a
  map, nothing nested.

## Open for whoever picks this up next (not blocking)

- `just lint` (`cargo clippy --workspace --all-targets`) currently fails on `motor_demo`, unrelated to this
  work: the sibling `../wire_weaver` checkout has moved ahead (`ObserveMode::Samples`, `ObserveLimits` grew
  fields) of what `motor_demo`/`motor_demo_api` in this repo expect. Confirmed pre-existing: `git status` here
  shows no edits to `motor_demo`, and `cargo clippy -p motor_sim -p motor_demo_api -p motor_bench` (everything
  this task touches) is clean. Said to parent via `tpm work say`; not fixed here since it's a `motor_demo` ↔
  wire_weaver API migration, not a bench-harness task.
- `motor_sim` tests (19) and doctest pass; `motor_sim` still builds for `thumbv7em-none-eabihf`.

## FEATURES.md

Per the parent session's message: the MS-8 entry (and the new roadmap area ids) comes from the plan branch
`s/18oct2026-motor-sim-plan` (commit 36e5911), not written here. Before `tpm work ready`, rebase onto main: if
MS-8 is there, flip it to ✅ with one line of what landed; if not, leave `FEATURES.md` alone and say so in the
ready note.
