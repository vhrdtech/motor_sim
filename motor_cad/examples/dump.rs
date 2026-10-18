//! Classifies a STEP file and prints the report: `cargo run --release -p motor_cad --example dump -- FILE`
fn main() {
    let path = std::path::PathBuf::from(std::env::args().nth(1).expect("STEP file"));
    let t = std::time::Instant::now();
    let m = motor_cad::step_read::read_step_file(&path).unwrap();
    let t_read = t.elapsed().as_secs_f64();
    let o = motor_cad::overrides::Overrides::load_beside(&path).unwrap();
    let c = motor_cad::classify::classify(&m, &o);
    let title = path.file_name().unwrap().to_string_lossy();
    print!("{}", motor_cad::report::markdown(&title, &m, &c));
    if std::env::var_os("MOTOR_CAD_DEBUG").is_some() {
        for (i, (p, f)) in m.parts.iter().zip(&c.features).enumerate() {
            println!(
                "{i:3} {:<34} cov {:.2} order {:3} amp {:.3} genus {:?} fill {:.2} r {:.1}..{:.1} rc {:.1} z {:.1}..{:.1} ext {:.1}/{:.1}/{:.1} along {:.2}",
                p.name.chars().take(34).collect::<String>(),
                f.coverage,
                f.order,
                f.order_amp,
                f.genus,
                f.fill,
                f.r_min * 1e3,
                f.r_max * 1e3,
                f.r_c * 1e3,
                f.z_min * 1e3,
                f.z_max * 1e3,
                f.extents[0] * 1e3,
                f.extents[1] * 1e3,
                f.extents[2] * 1e3,
                f.along_axis
            );
        }
    }
    println!(
        "\nread {t_read:.2} s, classify {:.2} s",
        t.elapsed().as_secs_f64() - t_read
    );
}
