//! Prints the parts a STEP file imports to: `cargo run --release -p motor_cad --example dump -- FILE`
fn main() {
    let path = std::env::args().nth(1).expect("STEP file");
    let t = std::time::Instant::now();
    let m = motor_cad::step_read::read_step_file(path.as_ref()).unwrap();
    println!(
        "schema {} | system {} | unit {} m | {} parts | {:.2} s",
        m.schema,
        m.system,
        m.unit_m,
        m.parts.len(),
        t.elapsed().as_secs_f64()
    );
    for n in &m.notes {
        println!("note: {n}");
    }
    for p in &m.parts {
        let bb = p.mesh.aabb();
        println!(
            "{:<40} inst={:<24} path={:<30} body {}/{} col={:?} layer={:?} tris={} size_mm={:?}",
            p.name,
            p.instance,
            p.path.join("/"),
            p.body + 1,
            p.bodies,
            p.colour.map(|c| c.map(|x| (x * 100.0).round() / 100.0)),
            p.layer,
            p.mesh.triangle_count(),
            bb.size().map(|x| (x * 1e4).round() / 10.0)
        );
    }
}
