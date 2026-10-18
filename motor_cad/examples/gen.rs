//! Writes the parametric motors to STEP: `cargo run --release -p motor_cad --example gen -- DIR`
fn main() {
    let dir = std::path::PathBuf::from(std::env::args().nth(1).unwrap_or(".".into()));
    let o = motor_cad::generate::outrunner(&Default::default());
    let s = motor_cad::generate::stepper(&Default::default());
    for (name, doc) in [("outrunner_5065.step", &o), ("stepper_nema17.step", &s)] {
        let t = std::time::Instant::now();
        std::fs::write(dir.join(name), motor_cad::step_write::write_step(doc)).unwrap();
        println!("{name}: {:.2} s", t.elapsed().as_secs_f64());
    }
    std::fs::write(
        dir.join("outrunner_bare.step"),
        motor_cad::step_write::write_step(&motor_cad::generate::anonymised(&o, "Motor")),
    )
    .unwrap();
}
