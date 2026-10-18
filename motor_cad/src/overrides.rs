//! Manual corrections kept beside the model: `<model>.motor.toml` (for `motor.step`: `motor.motor.toml`).
//!
//! ```toml
//! # optional: the motor axis in model units, when geometry finds the wrong one
//! [axis]
//! origin = [0.0, 0.0, 0.0]
//! direction = [0.0, 0.0, 1.0]
//!
//! # first match wins; `match` is a product name, an instance name or a path `Assembly/Sub/Part`, `*` matches anything
//! [[part]]
//! match = "Body001"
//! role = "stator"
//!
//! [[part]]
//! match = "*/Rotor/*screw*"
//! role = "fastener"
//! rotating = true
//! ```

use crate::role::{Polarity, Role};
use crate::step_read::Part;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Overrides {
    pub axis: Option<AxisOverride>,
    #[serde(default, rename = "part")]
    pub parts: Vec<PartOverride>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AxisOverride {
    /// model units
    pub origin: [f64; 3],
    pub direction: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartOverride {
    #[serde(rename = "match")]
    pub pattern: String,
    pub role: Option<Role>,
    pub rotating: Option<bool>,
    pub polarity: Option<Polarity>,
}

#[derive(Debug, thiserror::Error)]
pub enum OverrideError {
    #[error("reading {0}: {1}")]
    Io(String, std::io::Error),
    #[error("{0}: {1}")]
    Toml(String, toml::de::Error),
}

/// `*` matches any run of characters, case-insensitive.
pub fn glob(pattern: &str, text: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (
        pattern.to_lowercase().chars().collect(),
        text.to_lowercase().chars().collect(),
    );
    let (mut pi, mut ti, mut star, mut mark) = (0, 0, None, 0);
    while ti < t.len() {
        if pi < p.len() && p[pi] != '*' && p[pi] == t[ti] {
            pi += 1;
            ti += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some(pi);
            mark = ti;
            pi += 1;
        } else if let Some(s) = star {
            pi = s + 1;
            mark += 1;
            ti = mark;
        } else {
            return false;
        }
    }
    p[pi..].iter().all(|c| *c == '*')
}

impl Overrides {
    /// The override file beside a model, if there is one.
    pub fn path_for(model: &Path) -> PathBuf {
        model.with_extension("motor.toml")
    }

    pub fn load_beside(model: &Path) -> Result<Overrides, OverrideError> {
        let path = Self::path_for(model);
        if !path.exists() {
            return Ok(Overrides::default());
        }
        let text = std::fs::read_to_string(&path)
            .map_err(|e| OverrideError::Io(path.display().to_string(), e))?;
        toml::from_str(&text).map_err(|e| OverrideError::Toml(path.display().to_string(), e))
    }

    /// The first override matching a part by product name, instance name or path.
    pub fn find(&self, p: &Part) -> Option<&PartOverride> {
        let path = p
            .path
            .iter()
            .chain(std::iter::once(&p.name))
            .cloned()
            .collect::<Vec<_>>()
            .join("/");
        self.parts.iter().find(|o| {
            glob(&o.pattern, &p.name) || glob(&o.pattern, &p.instance) || glob(&o.pattern, &path)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn globs() {
        assert!(glob("Body*", "Body001"));
        assert!(glob("*/rotor/*screw*", "Motor/Rotor/M3 screw"));
        assert!(!glob("Body", "Body001"));
        assert!(glob("*", ""));
    }

    #[test]
    fn parses() {
        let o: Overrides = toml::from_str(
            "[axis]\norigin = [0.0, 0.0, 0.0]\ndirection = [0.0, 1.0, 0.0]\n[[part]]\nmatch = \"Body001\"\nrole = \"stator\"\npolarity = \"N\"\n",
        )
        .unwrap();
        assert_eq!(o.parts[0].role, Some(Role::Stator));
        assert_eq!(o.parts[0].polarity, Some(Polarity::N));
    }
}
