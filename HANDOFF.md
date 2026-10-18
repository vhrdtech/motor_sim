# Handoff: twin-cad (DEMO-2 first slice, P2630#18oct-1242f)

## Done
- `motor_cad` crate: STEP read (monstertruck 0.4.1 + own P21 walk for colours, layers, units, escapes), parametric outrunner/stepper/anonymised writer, classifier (tags > keywords > colour > geometry), overrides TOML, Markdown report, scene + GLB write/read, CLI (`import`, `init-overrides`, `gen`, completions).
- Tests: 11 pass in 0.4 s (dev profile optimises monstertruck crates, root Cargo.toml).
- Real assets: open-licence ones in ~/data/motor_sim-cad (SOURCES.md), Roman's in ~/sync/solid/3D Components (read in place). BE4108 (MIT) is a test fixture.

## Next
- motor_twin crate (Bevy + bevy_egui, outside default-members), headless screenshots.
- Convention docs page (motor_cad/docs or README), design note in tpm, FEATURES ids via `tpm work claim DEMO`, CHANGELOG.

## Decisions / gotchas
- truck master fails 4/11 real files (OOM on NEMA17, panic on NEMA11); monstertruck reads all 11. Upstream truck is ahead only on writing assemblies with real solids.
- monstertruck's StepDesign can't hold solids (fixed entity index 16): step_write drops StepModels' preamble (#1..#15) and writes product structure + colours itself.
- Baking placements into the B-rep (builder::transformed) makes rational B-splines that monstertruck's parser hangs on: anonymised variant keeps instances.
- Debug-build meshing is slow: profile.dev.package overrides in the root Cargo.toml.
