//! What observing costs, counted on the host: per property how often it arrived, per level updates and message
//! bytes against the granted budget, missed sends, and what the same values would have taken as one `StreamData`
//! event per property.
//!
//! Bytes are whole serialized `ww_client_server::Event`s, the messages the link carries, rebuilt from what arrived:
//! a `PropertyUpdate` with the level's path, timestamp, missed count and the update's data. Framing (USB packets,
//! UDP datagrams) comes on top and is shared by every message in a frame, so it is left out of both sides.

use std::collections::BTreeMap;
use std::time::Instant;

use wire_weaver::shrink_wrap::tail_bytes::TailBytes;
use wire_weaver::shrink_wrap::{
    RefVec, SerializeShrinkWrap, SerializeShrinkWrapOwned, UNib32, UVlq32,
};
use wire_weaver_client::observe::Observed;
use ww_client_server::observe::BatchedOwned;
use ww_client_server::{Event, EventKind};
use ww_si::Second;
use ww_si::ww_numeric::NumericValue;

use crate::units::Human;

use crate::client::api_client::motor_demo_api_motor_demo::{
    motor_demo_api_control::ControlUpdate, motor_demo_api_foc::FocUpdate,
    motor_demo_api_telemetry::TelemetryUpdate,
};

/// The observable levels, by resource index at the root.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    Foc = 0,
    Control = 1,
    Telemetry = 2,
}

impl Level {
    pub const ALL: [Level; 3] = [Level::Foc, Level::Control, Level::Telemetry];

    pub fn name(self) -> &'static str {
        match self {
            Level::Foc => "foc",
            Level::Control => "control",
            Level::Telemetry => "telemetry",
        }
    }

    /// The rate the controller computes the level's values at, Hz of simulated time.
    pub fn tier_hz(self) -> u32 {
        match self {
            Level::Foc => 25_000,
            Level::Control => 1_000,
            Level::Telemetry => 50,
        }
    }

    pub fn props(self) -> impl Iterator<Item = &'static Prop> {
        PROPS.iter().filter(move |p| p.level == self)
    }
}

/// An observable property.
#[derive(Clone, Copy, Debug)]
pub struct Prop {
    pub level: Level,
    /// Resource index in its level.
    pub index: u32,
    pub name: &'static str,
    /// What the host shows the value in: VESC's erpm, Ah and Wh where it has them, the base unit of the quantity
    /// otherwise.
    pub unit: &'static str,
}

impl Prop {
    /// `level.name`
    pub fn path(&self) -> String {
        format!("{}.{}", self.level.name(), self.name)
    }
}

/// A property present in an update.
#[derive(Clone, Debug)]
pub struct Present {
    pub index: u32,
    pub value: String,
    /// Bytes of the value alone.
    pub len: usize,
    /// Device time of this one sample, for a `+batched` property's own `Series` item (API-39: every sample has
    /// its own `t`, see `docs/api/batching.md`); `None` for a plain property or a `Batched::One` value, which
    /// have no time of their own apart from the update's head `t`.
    pub t: Option<u64>,
}

fn one_present<T: SerializeShrinkWrap + Human>(
    index: u32,
    unit: &str,
    v: &T,
    t: Option<u64>,
    out: &mut Vec<Present>,
) {
    let mut buf = [0u8; 32];
    let len = v.to_ww_bytes(&mut buf).map(|b| b.len()).unwrap_or(0);
    out.push(Present {
        index,
        value: v.human(unit),
        len,
        t,
    });
}

fn present<T: SerializeShrinkWrap + Human>(
    index: u32,
    unit: &str,
    value: &Option<T>,
    out: &mut Vec<Present>,
) {
    if let Some(v) = value {
        one_present(index, unit, v, None, out);
    }
}

/// A `+batched` property's slot (API-39): one [Present] for [BatchedOwned::One], one per sample of a
/// [BatchedOwned::Samples], oldest first, each at its time from the update's `sample_times` (shared by the level,
/// API-49), so [Stats::record]'s per-property count and cost follow every sample, and `last` ends up the newest one.
/// Samples whose count doesn't match the times are malformed and left out, as the client does.
fn present_batched<T: SerializeShrinkWrap + Human>(
    index: u32,
    unit: &str,
    value: &Option<BatchedOwned<T>>,
    sample_times: Option<&[Second]>,
    out: &mut Vec<Present>,
) {
    match value {
        None => {}
        Some(BatchedOwned::One(v)) => one_present(index, unit, v, None, out),
        Some(BatchedOwned::Samples(series)) => match sample_times {
            Some(times) if times.len() == series.len() => {
                for (value, t) in series.iter().zip(times) {
                    one_present(index, unit, value, t_us(Some(t)), out);
                }
            }
            _ => {}
        },
    }
}

/// What an update carried: the properties present and the update's data as the device serialized it.
pub struct Carried {
    pub present: Vec<Present>,
    pub data: Vec<u8>,
}

/// The property table, and per level a function listing what an update carried. `plain` fields are `Option<T>`,
/// `batched` ones `Option<BatchedOwned<T>>` (API-39): every sample counted, oldest first. The data is the update
/// re-serialized with its own (owned, client-side) type, so it has the bytes that crossed the wire.
macro_rules! levels {
    ($($level:ident $Update:ident $fn:ident {
        $(plain: $($pindex:literal $pname:ident $punit:literal;)*)?
        $(batched: $($bindex:literal $bname:ident $bunit:literal;)*)?
    })*) => {
        /// Every observable property, in level and index order.
        pub const PROPS: &[Prop] = &[
            $(
                $($(Prop { level: Level::$level, index: $pindex, name: stringify!($pname), unit: $punit },)*)?
                $($(Prop { level: Level::$level, index: $bindex, name: stringify!($bname), unit: $bunit },)*)?
            )*
        ];

        $(
            /// What an update of the level carried, `sample_times` its samples' times.
            #[allow(unused_variables)]
            pub fn $fn(u: &$Update, sample_times: Option<&[Second]>) -> Carried {
                let mut present_ = vec![];
                $($(present($pindex, $punit, &u.$pname, &mut present_);)*)?
                $($(present_batched($bindex, $bunit, &u.$bname, sample_times, &mut present_);)*)?
                let data = u.to_ww_bytes_owned().unwrap_or_default();
                Carried { present: present_, data }
            }
        )*
    };
}

levels! {
    Foc FocUpdate foc_present {
        batched:
        0 i_a "A"; 1 i_b "A"; 2 i_c "A"; 3 i_alpha "A"; 4 i_beta "A"; 5 i_d "A"; 6 i_q "A";
        7 v_d "V"; 8 v_q "V"; 9 v_alpha "V"; 10 v_beta "V";
        11 duty_a ""; 12 duty_b ""; 13 duty_c "";
        14 phase "rad"; 15 phase_observer "rad"; 16 phase_encoder "rad";
        17 observer_x1 "Wb"; 18 observer_x2 "Wb"; 19 v_bus "V"; 20 phase_true "rad";
    }
    Control ControlUpdate control_present {
        plain:
        0 mode ""; 1 sensor_mode ""; 2 duty_target ""; 3 current_target "A"; 4 speed_target "erpm";
        batched:
        5 speed_electrical "erpm"; 6 speed "rpm"; 7 id_set "A"; 8 iq_set "A"; 9 id_filtered "A";
        10 iq_filtered "A"; 11 i_motor "A"; 12 i_input "A"; 13 duty_now ""; 14 torque "N·m"; 15 position "°";
        16 observer_error "°"; 17 speed_true "rpm";
    }
    Telemetry TelemetryUpdate telemetry_present {
        plain:
        0 time "s"; 1 charge_drawn "Ah"; 2 charge_returned "Ah"; 3 energy_drawn "Wh"; 4 energy_returned "Wh";
        5 tachometer ""; 6 tachometer_abs ""; 7 v_in "V"; 8 power_in "W"; 9 avg_motor_current "A";
        10 avg_input_current "A"; 11 fault ""; 12 real_time_factor "";
    }
}

/// Find a property by `level.name` or by its name alone when that is unique.
pub fn find(name: &str) -> Option<&'static Prop> {
    if let Some((level, name)) = name.split_once('.') {
        return PROPS
            .iter()
            .find(|p| p.level.name() == level && p.name == name);
    }
    let mut found = PROPS.iter().filter(|p| p.name == name);
    let first = found.next()?;
    found.next().is_none().then_some(first)
}

/// A device time as the device sent it: the host unwraps the device's microseconds, on the wire they are a
/// wrapping `u32`.
fn as_sent(t: &Second) -> Second {
    match (t.prefix, t_us(Some(t))) {
        (ww_si::Prefix::Micro, Some(us)) => {
            Second::from_parts(ww_si::Prefix::Micro, NumericValue::U32(us as u32))
        }
        _ => *t,
    }
}

/// Bytes of the `PropertyUpdate` event that carried `data`, with the update's time `t` and its samples' times
/// `sample_times` (API-49) as the device sent them.
pub fn property_update_len(
    level: Level,
    t: Option<&Second>,
    sample_times: Option<&[Second]>,
    missed: u32,
    data: &[u8],
) -> usize {
    let path = [UNib32(level as u32)];
    let sample_times: Option<Vec<Second>> =
        sample_times.map(|times| times.iter().map(as_sent).collect());
    let event = Event {
        seq: UVlq32(0),
        result: Ok(EventKind::PropertyUpdate {
            path: RefVec::Slice { slice: &path },
            t: t.map(as_sent),
            missed: (missed > 0).then_some(UNib32(missed)),
            sample_times: sample_times
                .as_deref()
                .map(wire_weaver::shrink_wrap::series::Series::new),
            data: TailBytes(data),
        }),
    };
    let times_len = sample_times.as_ref().map_or(0, |t| t.len() * 16);
    let mut buf = vec![0u8; data.len() + times_len + 64];
    event.to_ww_bytes(&mut buf).map(|b| b.len()).unwrap_or(0)
}

/// Bytes of a `StreamData` event carrying one property's value on its own path, as `stream_data_ser()` sends it.
pub fn stream_data_len(level: Level, index: u32, value_len: usize) -> usize {
    let path = [UNib32(level as u32), UNib32(index)];
    let data = vec![0u8; value_len];
    let event = Event {
        seq: UVlq32(0),
        result: Ok(EventKind::StreamData {
            path: RefVec::Slice { slice: &path },
            data: TailBytes(&data),
        }),
    };
    let mut buf = vec![0u8; value_len + 64];
    event.to_ww_bytes(&mut buf).map(|b| b.len()).unwrap_or(0)
}

/// Device time of an update, µs.
pub fn t_us(t: Option<&Second>) -> Option<u64> {
    match t?.value {
        NumericValue::U64(t) => Some(t),
        NumericValue::U32(t) => Some(t as u64),
        _ => None,
    }
}

/// Counts of one property.
#[derive(Clone, Debug, Default)]
pub struct PropStats {
    pub count: u64,
    pub last: String,
    /// Bytes as one `StreamData` per value.
    pub stream_data_bytes: u64,
    /// Device time of this property's first and last received sample, from samples that carry their own time
    /// (a `+batched` property's `Series` items, API-39); `None` for a property whose values don't.
    pub first_t: Option<u64>,
    pub last_t: Option<u64>,
    /// Samples received whose own time didn't strictly increase from the previous one of this property: every
    /// sample should arrive once, in order (a device/client bug, not something `missed` explains).
    pub out_of_order: u64,
}

/// Counts of one level.
#[derive(Clone, Debug, Default)]
pub struct LevelStats {
    pub updates: u64,
    pub bytes: u64,
    pub missed: u64,
    /// Device time of the first and the last update, µs.
    pub first_t: Option<u64>,
    pub last_t: Option<u64>,
    pub props: BTreeMap<u32, PropStats>,
}

impl LevelStats {
    pub fn stream_data_bytes(&self) -> u64 {
        self.props.values().map(|p| p.stream_data_bytes).sum()
    }

    /// Values received, all properties together.
    pub fn values(&self) -> u64 {
        self.props.values().map(|p| p.count).sum()
    }

    /// Simulated seconds between the first and the last update.
    pub fn sim_span_s(&self) -> f64 {
        match (self.first_t, self.last_t) {
            (Some(a), Some(b)) => (b - a) as f64 / 1e6,
            _ => 0.0,
        }
    }
}

/// Everything counted since [Stats::new] or the last [Stats::take].
#[derive(Clone, Debug)]
pub struct Stats {
    pub since: Instant,
    pub levels: BTreeMap<Level, LevelStats>,
}

impl Default for Stats {
    fn default() -> Self {
        Self::new()
    }
}

impl Stats {
    pub fn new() -> Self {
        Stats {
            since: Instant::now(),
            levels: BTreeMap::new(),
        }
    }

    /// Count one update of `level`: `data` is the update's data, `sample_times` its samples' times, `present` what
    /// it carried.
    pub fn record(
        &mut self,
        level: Level,
        t: Option<&Second>,
        sample_times: Option<&[Second]>,
        missed: u32,
        data: &[u8],
        present: &[Present],
    ) {
        let l = self.levels.entry(level).or_default();
        l.updates += 1;
        l.bytes += property_update_len(level, t, sample_times, missed, data) as u64;
        l.missed += missed as u64;
        if let Some(t) = t_us(t) {
            l.first_t.get_or_insert(t);
            l.last_t = Some(t);
        }
        for p in present {
            let s = l.props.entry(p.index).or_default();
            s.count += 1;
            s.last.clone_from(&p.value);
            s.stream_data_bytes += stream_data_len(level, p.index, p.len) as u64;
            if let Some(t) = p.t {
                if s.last_t.is_some_and(|last| t <= last) {
                    s.out_of_order += 1;
                }
                s.first_t.get_or_insert(t);
                s.last_t = Some(t);
            }
        }
    }

    /// Count an update as it arrived from an observer.
    pub fn record_observed<U>(
        &mut self,
        level: Level,
        o: &Observed<U>,
        carried: impl FnOnce(&U, Option<&[Second]>) -> Carried,
    ) {
        let times = o.sample_times.as_deref();
        let c = carried(&o.value, times);
        self.record(level, o.t.as_ref(), times, o.missed, &c.data, &c.present);
    }

    /// The counts so far, starting over.
    pub fn take(&mut self) -> Stats {
        std::mem::take(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wire_weaver::shrink_wrap::DeserializeShrinkWrap;

    #[test]
    fn table_is_complete() {
        assert_eq!(PROPS.len(), 52);
        for level in Level::ALL {
            for (k, p) in level.props().enumerate() {
                assert_eq!(p.index, k as u32, "{}", p.path());
            }
        }
        assert_eq!(find("i_q").unwrap().path(), "foc.i_q");
        assert_eq!(find("control.speed_electrical").unwrap().index, 5);
        assert!(find("nope").is_none());
    }

    #[test]
    fn one_value_costs_more_alone() {
        // one current of the foc level, timestamp in µs
        let t = Second::from_parts(ww_si::Prefix::Micro, NumericValue::U64(123_456));
        let mut u = crate::device::api_impl::FocUpdate::from_ww_bytes(&[]).unwrap();
        u.i_q = Some(ww_client_server::observe::Batched::One(
            ww_si::Ampere::milli(1500i32),
        ));
        let mut buf = [0u8; 64];
        let data = u.to_ww_bytes(&mut buf).unwrap().to_vec();
        let update = property_update_len(Level::Foc, Some(&t), None, 0, &data);
        let alone = stream_data_len(Level::Foc, 6, 5);
        assert!(update > 5 && alone > 5, "{update} {alone}");
    }
}
