## Unreleased

## 0.4.0 - 2026-10-18

### ⚠️ Breaking

* Every `Foc` property and `Control`'s 13 measured values (not its setpoints) are `+observe+batched` (API-41/API-39): their slot in `FocUpdate`/`ControlUpdate` is `Option<Batched<T>>` (host: `Option<BatchedOwned<T>>`) instead of `Option<T>`. A host observing in the existing modes (`OnChange`, `Periodic`, `OnChangeOrBeat`) still gets one value per update (`Batched::One`), now behind one extra tag; the new `ObserveMode::Samples` gets every sample instead, in batches (`Batched::Series`). `FocSamples` / `ControlSamples` (generated) are what the device's `sample_<level>(t)` call takes.

## 0.3.0 - 2026-10-17

### ⚠️ Breaking

* `Load::Viscous { b }` and `Load::Fan { k }` are `ww_si` quantities now (STD-38): `b: Quantity<Per<Nm, RadPerS>>` (N·m/(rad/s)) and `k: Quantity<Per<Nm, Times<RadPerS, RadPerS>>>` (N·m/(rad/s)²) instead of `f32`. The wire changes (a prefix and a number instead of 4 bytes), hosts and devices are rebuilt together. Build one with `Quantity::from_f32(1.2e-4)`, read it with `to_f32()`.
