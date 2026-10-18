//! Part roles and what names and colours say about them: the naming convention (`[role]` tags) and keywords in a
//! few languages, for files that don't follow it.

use serde::{Deserialize, Serialize};

/// What a part is in the motor.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    Shaft,
    /// rotor core, bell, hub, rotor cups
    Rotor,
    Magnet,
    /// stator core and teeth (laminations)
    Stator,
    Winding,
    Bearing,
    /// housing, end caps, flanges, mounts, stator hub
    Housing,
    Fastener,
    Pcb,
    /// cables, leads, connectors
    Wiring,
    /// encoders, hall sensors, sensor magnets
    Sensor,
    Other,
}

impl Role {
    pub const ALL: [Role; 12] = [
        Role::Shaft,
        Role::Rotor,
        Role::Magnet,
        Role::Stator,
        Role::Winding,
        Role::Bearing,
        Role::Housing,
        Role::Fastener,
        Role::Pcb,
        Role::Wiring,
        Role::Sensor,
        Role::Other,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Role::Shaft => "shaft",
            Role::Rotor => "rotor",
            Role::Magnet => "magnet",
            Role::Stator => "stator",
            Role::Winding => "winding",
            Role::Bearing => "bearing",
            Role::Housing => "housing",
            Role::Fastener => "fastener",
            Role::Pcb => "pcb",
            Role::Wiring => "wiring",
            Role::Sensor => "sensor",
            Role::Other => "other",
        }
    }

    /// The role a tag word names (`[magnet]`, `[coil]`, ...), English only: tags are the convention.
    pub fn from_tag(s: &str) -> Option<Role> {
        Some(match s.trim().to_lowercase().as_str() {
            "shaft" | "axle" => Role::Shaft,
            "rotor" | "bell" | "rotor_core" | "hub" => Role::Rotor,
            "magnet" | "magnets" => Role::Magnet,
            "stator" | "stator_core" | "lamination" | "laminations" => Role::Stator,
            "winding" | "windings" | "coil" | "coils" => Role::Winding,
            "bearing" | "bearings" => Role::Bearing,
            "housing" | "case" | "cap" | "flange" | "mount" => Role::Housing,
            "fastener" | "screw" | "bolt" | "nut" | "washer" | "clip" => Role::Fastener,
            "pcb" | "board" => Role::Pcb,
            "wiring" | "cable" | "wire" | "lead" | "connector" => Role::Wiring,
            "sensor" | "encoder" | "hall" => Role::Sensor,
            "other" => Role::Other,
            _ => return None,
        })
    }

    /// Parts of these roles turn with the rotor unless told otherwise.
    pub fn rotates(self) -> bool {
        matches!(self, Role::Shaft | Role::Rotor | Role::Magnet)
    }

    /// When one name holds keywords of several roles, the more specific one wins ("rotor magnet" is a magnet).
    fn specificity(self) -> u8 {
        match self {
            Role::Magnet
            | Role::Winding
            | Role::Bearing
            | Role::Fastener
            | Role::Pcb
            | Role::Sensor => 3,
            Role::Wiring | Role::Shaft => 2,
            Role::Stator | Role::Rotor => 1,
            Role::Housing | Role::Other => 0,
        }
    }
}

impl std::fmt::Display for Role {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// Magnet polarity facing the air gap.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Polarity {
    N,
    S,
}

/// What a `[...]` tag says: a role and modifiers (`rotating`, `static`, `N`, `S`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tag {
    pub role: Option<Role>,
    pub rotating: Option<bool>,
    pub polarity: Option<Polarity>,
}

/// The first `[...]` tag in a name that names a role or a modifier.
pub fn parse_tag(name: &str) -> Option<Tag> {
    let mut rest = name;
    while let Some(open) = rest.find('[') {
        let after = &rest[open + 1..];
        let Some(close) = after.find(']') else { break };
        let mut tag = Tag::default();
        for word in after[..close]
            .split([',', ' ', ';'])
            .filter(|w| !w.is_empty())
        {
            match word.to_lowercase().as_str() {
                "rotating" | "rot" => tag.rotating = Some(true),
                "static" | "fixed" => tag.rotating = Some(false),
                "n" => tag.polarity = Some(Polarity::N),
                "s" => tag.polarity = Some(Polarity::S),
                w => {
                    if let Some(r) = Role::from_tag(w) {
                        tag.role.get_or_insert(r);
                    }
                }
            }
        }
        if tag != Tag::default() {
            return Some(tag);
        }
        rest = &after[close + 1..];
    }
    None
}

/// Keywords per role: English, German, French, Spanish, Russian, Chinese, and common designations. Matched as
/// word prefixes of the lowercased name (so "magnets", "Wicklungen" match), Chinese as substrings.
const KEYWORDS: &[(Role, &[&str])] = &[
    (
        Role::Magnet,
        &[
            "magnet",
            "aimant",
            "imán",
            "iman",
            "магнит",
            "磁钢",
            "磁铁",
            "n52",
            "n48",
            "n45",
            "ndfeb",
        ],
    ),
    (
        Role::Winding,
        &[
            "winding",
            "coil",
            "wicklung",
            "spule",
            "bobinage",
            "bobina",
            "devanado",
            "обмотк",
            "катушк",
            "线圈",
            "绕组",
            "copper",
        ],
    ),
    (
        Role::Bearing,
        &[
            "bearing",
            "lager",
            "roulement",
            "rodamiento",
            "подшипник",
            "轴承",
            "ball bearing",
        ],
    ),
    (
        Role::Fastener,
        &[
            "screw",
            "bolt",
            "nut",
            "washer",
            "circlip",
            "e-clip",
            "eclip",
            "clip",
            "pin",
            "dowel",
            "schraube",
            "mutter",
            "scheibe",
            "vis",
            "écrou",
            "tornillo",
            "tuerca",
            "винт",
            "болт",
            "гайк",
            "шайб",
            "螺钉",
            "螺丝",
            "螺母",
            "垫片",
            "iso 4762",
            "iso 7045",
            "iso 7380",
            "iso 10642",
            "din 912",
            "din 7985",
            "din 933",
            "din 125",
            "din 471",
            "din 6799",
            "socket head",
            "set screw",
            "grub",
        ],
    ),
    (
        Role::Pcb,
        &[
            "pcb",
            "board",
            "platine",
            "leiterplatte",
            "carte",
            "placa",
            "плата",
            "电路板",
        ],
    ),
    (
        Role::Sensor,
        &[
            "encoder",
            "hall",
            "sensor",
            "as5047",
            "as5048",
            "as5600",
            "ma730",
            "resolver",
            "geber",
            "датчик",
            "энкодер",
            "编码器",
            "传感器",
        ],
    ),
    (
        Role::Wiring,
        &[
            "cable",
            "wire",
            "lead",
            "connector",
            "conector",
            "plug",
            "jst",
            "molex",
            "xt30",
            "xt60",
            "mr30",
            "kabel",
            "litze",
            "stecker",
            "câble",
            "cable",
            "провод",
            "кабель",
            "разъ",
            "电缆",
            "导线",
            "连接器",
        ],
    ),
    (
        Role::Shaft,
        &[
            "shaft", "axle", "spindle", "welle", "achse", "arbre", "eje", "вал", "ось", "轴",
        ],
    ),
    (
        Role::Stator,
        &[
            "stator",
            "lamination",
            "core",
            "blechpaket",
            "blech",
            "tôle",
            "статор",
            "定子",
            "铁芯",
        ],
    ),
    (
        Role::Rotor,
        &[
            "rotor",
            "bell",
            "can",
            "glocke",
            "cloche",
            "campana",
            "ротор",
            "колокол",
            "转子",
            "flywheel",
        ],
    ),
    (
        Role::Housing,
        &[
            "housing",
            "case",
            "casing",
            "cover",
            "cap",
            "flange",
            "mount",
            "base",
            "frame",
            "end bell",
            "endcap",
            "gehäuse",
            "deckel",
            "flansch",
            "boîtier",
            "carcasa",
            "tapa",
            "корпус",
            "крышк",
            "фланец",
            "основани",
            "外壳",
            "端盖",
            "法兰",
            "壳",
        ],
    ),
];

fn word_prefix_match(name: &str, kw: &str) -> bool {
    if !kw.is_ascii() && kw.chars().next().is_some_and(|c| c as u32 >= 0x2E80) {
        return name.contains(kw); // CJK: no word boundaries
    }
    let mut start = 0;
    while let Some(i) = name[start..].find(kw) {
        let at = start + i;
        let before_ok = name[..at]
            .chars()
            .next_back()
            .is_none_or(|c| !c.is_alphanumeric());
        if before_ok {
            return true;
        }
        start = at + kw.len().max(1);
        while !name.is_char_boundary(start) {
            start += 1;
        }
    }
    false
}

/// Bearing designations: 608ZZ, 6808-2RS, MR115-2RS1, 625-Z, L2112KK, 61804.
fn is_bearing_designation(word: &str) -> bool {
    let w = word.to_lowercase();
    let w = ["smr", "mr", "r", "l"]
        .iter()
        .find_map(|p| w.strip_prefix(p))
        .unwrap_or(&w);
    let digits = w.chars().take_while(|c| c.is_ascii_digit()).count();
    if !(3..=5).contains(&digits) {
        return false;
    }
    let suffix = w[digits..].trim_start_matches('-');
    const SUFFIXES: [&str; 8] = ["zz", "2rs", "rs", "z", "2z", "kk", "llu", "ddu"];
    SUFFIXES.iter().any(|s| suffix.starts_with(s))
        || (suffix.is_empty() && digits >= 4 && w.starts_with('6'))
}

/// Metric fastener designations: M3x8, M2.5x25, M4.
fn is_fastener_designation(word: &str) -> bool {
    let w = word.to_lowercase();
    let Some(rest) = w.strip_prefix('m') else {
        return false;
    };
    let num: String = rest
        .chars()
        .take_while(|c| c.is_ascii_digit() || *c == '.')
        .collect();
    !num.is_empty() && num.parse::<f64>().is_ok_and(|d| (1.0..=24.0).contains(&d)) && {
        let after = &rest[num.len()..];
        after.is_empty() || after.starts_with('x') || after.starts_with('×')
    }
}

/// The roles a name's keywords point at, the most specific first, with the keyword.
pub fn keyword_roles(name: &str) -> Vec<(Role, String)> {
    let lower = name.to_lowercase();
    let mut out: Vec<(Role, String)> = Vec::new();
    for (role, kws) in KEYWORDS {
        if let Some(kw) = kws.iter().find(|kw| word_prefix_match(&lower, kw)) {
            out.push((*role, (*kw).to_string()));
        }
    }
    for word in
        lower.split(|c: char| c.is_whitespace() || c == '_' || c == '(' || c == ')' || c == ',')
    {
        if is_bearing_designation(word) && !out.iter().any(|(r, _)| *r == Role::Bearing) {
            out.push((Role::Bearing, word.to_string()));
        }
        if is_fastener_designation(word) && !out.iter().any(|(r, _)| *r == Role::Fastener) {
            out.push((Role::Fastener, word.to_string()));
        }
    }
    out.sort_by_key(|(r, _)| std::cmp::Reverse(r.specificity()));
    out
}

/// What a colour suggests, weakly: copper for windings, PCB green, the convention's magnet red and blue.
pub fn colour_hint(c: [f32; 3]) -> Option<(Role, Option<Polarity>, &'static str)> {
    let [r, g, b] = c.map(|x| x as f64);
    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let sat = if max > 0.0 { (max - min) / max } else { 0.0 };
    if sat < 0.35 || max < 0.15 {
        return None;
    }
    let hue = {
        let h = if max == r {
            ((g - b) / (max - min)).rem_euclid(6.0)
        } else if max == g {
            (b - r) / (max - min) + 2.0
        } else {
            (r - g) / (max - min) + 4.0
        };
        h * 60.0
    };
    if (15.0..=40.0).contains(&hue) && (0.35..=0.95).contains(&max) && sat > 0.45 {
        return Some((Role::Winding, None, "copper colour"));
    }
    if (90.0..=170.0).contains(&hue) && max < 0.7 {
        return Some((Role::Pcb, None, "PCB green"));
    }
    if !(15.0..=345.0).contains(&hue) && sat > 0.6 {
        return Some((Role::Magnet, Some(Polarity::N), "magnet N red"));
    }
    if (200.0..=250.0).contains(&hue) && sat > 0.6 {
        return Some((Role::Magnet, Some(Polarity::S), "magnet S blue"));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tags() {
        assert_eq!(parse_tag("Bell [rotor]").unwrap().role, Some(Role::Rotor));
        let t = parse_tag("Screw M3x6 [fastener, rotating]").unwrap();
        assert_eq!((t.role, t.rotating), (Some(Role::Fastener), Some(true)));
        assert_eq!(
            parse_tag("Magnet [magnet N]").unwrap().polarity,
            Some(Polarity::N)
        );
        assert_eq!(
            parse_tag("Part [rev B] [coil]").unwrap().role,
            Some(Role::Winding)
        );
        assert!(parse_tag("Bracket (rev 2)").is_none());
    }

    #[test]
    fn keywords() {
        assert_eq!(keyword_roles("Rotor magnet ring")[0].0, Role::Magnet);
        assert_eq!(keyword_roles("MR115-2RS1")[0].0, Role::Bearing);
        assert_eq!(keyword_roles("6808-2RS BEARING")[0].0, Role::Bearing);
        assert_eq!(keyword_roles("M2.5x25-Screw006")[0].0, Role::Fastener);
        assert_eq!(keyword_roles("Wicklung Phase U")[0].0, Role::Winding);
        assert_eq!(keyword_roles("Обмотка")[0].0, Role::Winding);
        assert_eq!(keyword_roles("定子铁芯")[0].0, Role::Stator);
        // FreeCAD's "Body" is a generic container, not a housing
        assert!(keyword_roles("Body001").is_empty());
        // "pin" must not match inside "spindle", "can" not inside "scan"
        assert_eq!(keyword_roles("spindle")[0].0, Role::Shaft);
        assert!(keyword_roles("scanner").is_empty());
    }

    #[test]
    fn colours() {
        assert_eq!(colour_hint([0.72, 0.45, 0.20]).unwrap().0, Role::Winding);
        assert_eq!(colour_hint([0.08, 0.42, 0.20]).unwrap().0, Role::Pcb);
        assert_eq!(
            colour_hint([0.80, 0.16, 0.16]).unwrap().1,
            Some(Polarity::N)
        );
        assert_eq!(
            colour_hint([0.16, 0.28, 0.80]).unwrap().1,
            Some(Polarity::S)
        );
        assert!(colour_hint([0.42, 0.44, 0.48]).is_none());
    }
}
