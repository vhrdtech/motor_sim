//! The classification report: each part (identical instances folded into one line), its role, confidence and why.

use crate::classify::{Classification, Source, Topology};
use crate::step_read::CadModel;
use std::fmt::Write;

/// A Markdown report.
pub fn markdown(title: &str, model: &CadModel, c: &Classification) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "# {title}\n");
    let _ = writeln!(
        s,
        "- source: {} ({}), unit {} m",
        model.system, model.schema, model.unit_m
    );
    let a = &c.axis;
    let _ = writeln!(
        s,
        "- axis: origin ({:.1}, {:.1}, {:.1}) mm, direction ({:.3}, {:.3}, {:.3}); {}",
        a.origin[0] * 1e3,
        a.origin[1] * 1e3,
        a.origin[2] * 1e3,
        a.dir[0],
        a.dir[1],
        a.dir[2],
        a.how
    );
    let topo = match c.topology {
        Topology::Outrunner => "outrunner",
        Topology::Inrunner => "inrunner",
        Topology::Unknown => "unknown",
    };
    let _ = writeln!(
        s,
        "- {topo}, envelope radius {:.1} mm, slots {}, magnets {}",
        c.envelope_r * 1e3,
        c.slots.map(|x| x.to_string()).unwrap_or("?".into()),
        c.poles.map(|x| x.to_string()).unwrap_or("?".into())
    );
    let named = c
        .verdicts
        .iter()
        .filter(|v| {
            v.evidence
                .iter()
                .any(|e| matches!(e.source, Source::Tag | Source::Override))
        })
        .count();
    let _ = writeln!(
        s,
        "- {} parts, {named} with a tag or override",
        model.parts.len()
    );
    for n in model.notes.iter().chain(&c.notes) {
        let _ = writeln!(s, "- note: {n}");
    }
    let _ = writeln!(
        s,
        "\nn: how many placed solids share the line (instances, bodies of one part).\n"
    );
    let _ = writeln!(
        s,
        "| part | n | role | conf. | rot. | why |\n|---|---|---|---|---|---|"
    );
    // fold instances: same product, body, verdict
    let mut rows: Vec<(String, usize, usize)> = Vec::new();
    for (i, p) in model.parts.iter().enumerate() {
        let v = &c.verdicts[i];
        let label = if p.bodies > 1 {
            format!("{} ({} bodies)", p.name, p.bodies)
        } else {
            p.name.clone()
        };
        let key = format!(
            "{label}\u{0}{}\u{0}{}\u{0}{:?}",
            v.role, v.rotating, v.polarity
        );
        match rows.iter_mut().find(|r| r.0 == key) {
            Some(r) => r.1 += 1,
            None => rows.push((key, 1, i)),
        }
    }
    for (key, count, i) in rows {
        let v = &c.verdicts[i];
        let label = key.split('\u{0}').next().unwrap_or("");
        let why: Vec<String> = v
            .evidence
            .iter()
            .take(3)
            .map(|e| format!("{:?}: {}", e.source, e.why).to_lowercase())
            .collect();
        let mut why = why.join("; ");
        if let Some(conf) = &v.conflict {
            why = format!("**{conf}**; {why}");
        }
        let role = match v.polarity {
            Some(p) => format!("{} {p:?}", v.role),
            None => v.role.to_string(),
        };
        let _ = writeln!(
            s,
            "| {} | {count} | {role} | {:.2} | {} | {} |",
            label.replace('|', "/"),
            v.confidence,
            if v.rotating { "yes" } else { "" },
            why.replace('|', "/")
        );
    }
    s
}
