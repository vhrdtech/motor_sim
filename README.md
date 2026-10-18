# motor_sim

Deterministic, `no_std` electric motor simulator in pure Rust, for testing motor controllers in software.

- PMSM in dq, solved exactly per PWM period, with an average inverter, rotor mechanics and loads (constant,
  viscous, fan, second inertia through a spring, time profile, dyno speed, locked rotor).
- The true state (`Truth`) is kept apart from what a controller measures (`Sensors`: current and DC link ADCs with
  offsets, noise and quantisation, encoder, halls). Faults break only the sensor and inverter paths.
- Same parameters and seed, same run on every target. `libm` is the only dependency.
- `Motor::visit` lists every variable with name, unit and a rate tier (PWM, control, telemetry).
- Presets: the VESC firmware's default hobby outrunner, a small industrial SPM servo, an IPM.

```rust
use motor_sim::{Drive, Load, Motor, presets};

let mut motor = Motor::new(presets::vesc_hobby_outrunner(), 42);
motor.set_load(Load::Fan { k_nms2_per_rad2: 2e-7 });
let dt = 1.0 / 25_000.0;
for _ in 0..25_000 {
    let _s = motor.sensors();          // i_abc_a, vdc_v, encoder_count, hall
    let drive = Drive::Duty([0.5; 3]); // your controller here
    motor.step(&drive, dt);
}
```

## Workspace

| Crate | What |
|---|---|
| `motor_sim` | the `no_std` plant above, builds for `thumbv7em-none-eabihf` |
| `motor_demo_api` | a [wire_weaver](https://github.com/vhrdtech/wire_weaver) API: 52 `+observe` properties of a motor controller in three rate tiers |
| `motor_demo` | a VESC-like FOC controller driving `motor_sim`, served as a wire_weaver device (in process or on UDP), and the `motor_demo` CLI that observes it |

`motor_demo` builds against a wire_weaver checkout next to this one (`../wire_weaver`); `just deps-pull` brings it up to its main.

```sh
git clone https://github.com/vhrdtech/wire_weaver
git clone https://github.com/vhrdtech/motor_sim
cd motor_sim
cargo run --release -p motor_demo -- observe --compare        # a device in this process, 3 tiers, 1 s reports
cargo run --release -p motor_demo -- device --listen 127.0.0.1:9000
cargo run --release -p motor_demo -- observe --udp 127.0.0.1:9000 --prop i_q=every:1ms,speed_electrical,fault
cargo run --release -p motor_demo -- list                     # every property with the unit it is shown in
```

What the demo shows and its measured numbers: [Motor controller demo](https://ww.vhrd.tech/examples/motor_demo/) in the wire_weaver docs.

Status and plans: [FEATURES.md](FEATURES.md). Development: [AGENTS.md](AGENTS.md).
