//! What a device report means, from the names its own family uses.
//!
//! The sensor protocols are a table, not code: a hundred and twenty seven
//! descriptions say where the bits are and what each field is called, and the
//! names come from rtl_433, which everybody who works on these devices reads.
//! So the one place that knows `temperature_c` is a temperature in Celsius is
//! here, and a description gains a reading on the map and in the chart by
//! spelling its field the way the family spells it.
//!
//! A name this does not know carries nothing. The bits are still in the
//! frame, under the layout the description gives, which is where a value
//! nobody has named yet belongs.

use common::Unit;
use common::packet::{Event, EventKind, Fact, Quantity, Reading};

use crate::Report;

/// What a device report is, as a protocol layer.
///
/// The model is part of the identifier rather than of the space: a sensor's
/// id is a handful of bits chosen when its batteries went in, so two makes
/// sharing one is ordinary and merging them would report a single station
/// reading two temperatures. The space stays the family, because that is
/// what a house filtering for its own sensors asks for.
pub fn proto_of(r: &Report) -> common::packet::Proto {
    use common::packet::{Entity, Id, Link, Party, Proto};
    let mut p = Proto::new("ism", r.model);
    if let Some(id) = &r.device {
        p = p
            .by(Entity::new("ism", Id::Text(format!("{}/{id}", r.model))))
            .between(Link::beacon(Party::unit(id.clone())));
    }
    for f in of_report(r) {
        p = p.saying(f);
    }
    p
}

/// What a report says, as statements.
pub fn of_report(r: &Report) -> Vec<Fact> {
    let mut out = Vec::new();
    for (name, value) in &r.fields {
        if let Some(f) = of_field(name, value) {
            out.push(f);
        }
    }
    out
}

/// What one field says, or nothing where the name is not one of the family's.
pub fn of_field(name: &str, value: &common::Value) -> Option<Fact> {
    if let Some((q, u)) = quantity(name) {
        return value.as_f64().map(|v| Fact::Sensed(Reading::new(q, scale(name, v), u)));
    }
    let on = match value {
        common::Value::Bool(b) => *b,
        v => v.as_f64().is_some_and(|n| n != 0.0),
    };
    // A battery that is well is not news, and a battery that is not is the
    // one thing anybody wants out of these: the flag reads the same way round
    // in every description, so the statement is made here rather than by a
    // view that would have to know which way.
    if name == "battery_ok" {
        return Some(Fact::Event(Event { kind: EventKind::LowBattery, on: !on }));
    }
    let kind = event(name)?;
    Some(Fact::Event(Event { kind, on }))
}

/// The quantity and unit a field name states, where the family has one.
fn quantity(name: &str) -> Option<(Quantity, Unit)> {
    // The numbered forms are one device reporting several probes, which is
    // the same quantity read more than once.
    let squashed = squash(name);
    Some(match squashed.as_str() {
        "temperature_c" | "setpoint_c" => (Quantity::Temperature, Unit::Celsius),
        "temperature_f" => (Quantity::Temperature, Unit::Fahrenheit),
        "humidity_pct" | "hum" => (Quantity::Humidity, Unit::Percent),
        "pressure_hpa" => (Quantity::Pressure, Unit::HectoPascal),
        "pressure_kpa" => (Quantity::Pressure, Unit::KiloPascal),
        "pressure_psi" => (Quantity::Pressure, Unit::Psi),
        "rain_total_mm" | "rain_mm" => (Quantity::Rainfall, Unit::Millimetre),
        "rain_in" => (Quantity::Rainfall, Unit::Millimetre),
        "wind_avg_ms" | "wind_speed_ms" => (Quantity::WindSpeed, Unit::MetresPerSecond),
        "wind_avg_km_h" => (Quantity::WindSpeed, Unit::KmPerHour),
        "wind_avg_mi_h" => (Quantity::WindSpeed, Unit::KmPerHour),
        "wind_gust_ms" | "wind_max_ms" => (Quantity::WindGust, Unit::MetresPerSecond),
        "wind_direction_deg" => (Quantity::WindDirection, Unit::Degree),
        "moisture_pct" => (Quantity::Moisture, Unit::Percent),
        "depth_cm" | "depth_mm" => (Quantity::Depth, Unit::Millimetre),
        "uv_index" => (Quantity::Ultraviolet, Unit::Ppm),
        "light_lux" | "lux" => (Quantity::Illuminance, Unit::Ppm),
        "battery_mv" => (Quantity::Battery, Unit::Millivolt),
        "battery_v" | "supercap_v" | "starting_v" => (Quantity::Battery, Unit::Volt),
        "voltage_v" => (Quantity::Voltage, Unit::Volt),
        "current_a" | "current_used_a" | "current_pv_a" => (Quantity::Current, Unit::Ampere),
        "power_w" => (Quantity::Power, Unit::Watt),
        "energy_kwh" => (Quantity::Energy, Unit::KilowattHour),
        "consumption" | "volume_m3" => (Quantity::Consumption, Unit::Ppm),
        "strike_count" | "pulse_count" => (Quantity::Count, Unit::Ppm),
        "strike_distance" => (Quantity::Range, Unit::Metre),
        _ => return None,
    })
}

/// A numbered field under the name its family gives the quantity:
/// `temperature_2_c` is a temperature like any other.
fn squash(name: &str) -> String {
    name.split('_').filter(|p| p.parse::<u32>().is_err()).collect::<Vec<_>>().join("_")
}

/// Inches of rain and miles an hour, in the unit the reading is stated in.
fn scale(name: &str, v: f64) -> f64 {
    match name {
        "rain_in" => v * 25.4,
        "wind_avg_mi_h" => v * 1.609_344,
        "depth_cm" => v * 10.0,
        _ => v,
    }
}

/// What happened at the device, where the name says.
fn event(name: &str) -> Option<EventKind> {
    Some(match name {
        "button" | "btn" | "button_code" | "multi_press" => EventKind::Button(0),
        "alarm" | "timer_alarm" | "temperature_alarm" => EventKind::Alarm,
        "tamper" | "physical_tamper" | "encoder_tamper" => EventKind::Tamper,
        "motion" | "moving" => EventKind::Motion,
        "water" | "leaking" => EventKind::Water,
        "test" => EventKind::Test,
        "pairing" | "learn" => EventKind::Pairing,
        "startup" => EventKind::Startup,
        "heartbeat" | "supervised" => EventKind::Heartbeat,
        "opened" | "contact_open" | "reed_open" => EventKind::Contact,
        _ => return None,
    })
}

/// What a radiosonde's frame says about its flight.
///
/// Every sonde sends the same six things in its own layout, so the statements
/// are made once here: where it is, how it is moving, how high, and what its
/// battery is doing. A reading the sonde did not send is left out rather than
/// carried as not-a-number.
pub fn of_flight(
    lat: f64,
    lon: f64,
    altitude_m: f64,
    climb_ms: f64,
    speed_kt: f64,
    course_deg: f64,
    battery_v: Option<f32>,
) -> Vec<Fact> {
    use common::packet::{Fix, Motion};
    let mut out = vec![
        Fact::Position(Fix { lat, lon, precision_bits: None }),
        Fact::Motion(Motion {
            speed_kt: Some(speed_kt),
            course_deg: Some(course_deg),
            climb_ms: Some(climb_ms),
            heading_deg: None,
        }),
        Fact::sensed(Quantity::Altitude, altitude_m, Unit::Metre),
    ];
    if let Some(v) = battery_v.filter(|v| v.is_finite()) {
        out.push(Fact::sensed(Quantity::Battery, f64::from(v), Unit::Volt));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Value;

    #[test]
    fn a_name_the_family_uses_states_a_reading() {
        let f = of_field("temperature_c", &Value::Float(16.2)).expect("a temperature");
        assert_eq!(f, Fact::sensed(Quantity::Temperature, 16.2, Unit::Celsius));
        let f = of_field("temperature_2_c", &Value::Float(4.0)).expect("the second probe");
        assert_eq!(f, Fact::sensed(Quantity::Temperature, 4.0, Unit::Celsius));
    }

    #[test]
    fn a_reading_in_the_wrong_unit_is_converted_where_it_is_read() {
        // Inches and miles an hour are what two dozen descriptions carry, and
        // a chart plotting both against millimetres had a shower reading as a
        // deluge.
        let f = of_field("rain_in", &Value::Float(1.0)).expect("rainfall");
        assert_eq!(f, Fact::sensed(Quantity::Rainfall, 25.4, Unit::Millimetre));
    }

    #[test]
    fn a_well_battery_is_the_low_battery_alarm_the_other_way_up() {
        assert_eq!(
            of_field("battery_ok", &Value::Bool(true)),
            Some(Fact::Event(Event { kind: EventKind::LowBattery, on: false }))
        );
        assert_eq!(
            of_field("battery_ok", &Value::Bool(false)),
            Some(Fact::Event(Event { kind: EventKind::LowBattery, on: true }))
        );
    }

    #[test]
    fn a_name_nobody_has_claimed_carries_nothing() {
        // The bits are still in the frame under the description's layout,
        // which is where a value nobody has named belongs.
        assert_eq!(of_field("maybetemp", &Value::Int(7)), None);
        assert_eq!(of_field("flags", &Value::Int(3)), None);
    }
}
