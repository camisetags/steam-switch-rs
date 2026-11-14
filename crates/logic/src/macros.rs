//! Timed controller playback and the JSON format used by the macro editor.
use serde::{Deserialize, Deserializer, Serialize, Serializer};

pub const MAX_STEPS: usize = 2_000;
pub const MAX_JSON_BYTES: usize = 96 * 1024;
const MIN_HOLD_MS: u16 = 10;
const MAX_HOLD_MS: u16 = 60_000;
const NEUTRAL: u16 = 0x800;

// Button names are part of the V1 macro file format. Each tuple identifies
// the matching byte and mask in a Switch 2 Pro input report.
const BUTTONS: [(&str, usize, u8); 20] = [
    ("A", 2, 0x02),
    ("B", 2, 0x01),
    ("X", 2, 0x08),
    ("Y", 2, 0x04),
    ("L", 3, 0x10),
    ("R", 2, 0x10),
    ("ZL", 3, 0x20),
    ("ZR", 2, 0x20),
    ("Plus", 2, 0x40),
    ("Minus", 3, 0x40),
    ("Home", 4, 0x01),
    ("Capture", 4, 0x02),
    ("LStick", 3, 0x80),
    ("RStick", 2, 0x80),
    ("Up", 3, 0x08),
    ("Down", 3, 0x01),
    ("Left", 3, 0x04),
    ("Right", 3, 0x02),
    ("GL", 4, 0x08),
    ("GR", 4, 0x04),
];

fn center() -> u16 {
    NEUTRAL
}
fn is_center(value: &u16) -> bool {
    *value == NEUTRAL
}
fn version() -> u8 {
    1
}
fn default_name() -> String {
    "Untitled macro".into()
}

fn deserialize_version<'de, D: Deserializer<'de>>(de: D) -> Result<u8, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Version {
        Number(u8),
        String(String),
    }
    match Version::deserialize(de)? {
        Version::Number(1) => Ok(1),
        Version::String(v) if v == "1" => Ok(1),
        _ => Err(serde::de::Error::custom("Unsupported macro version")),
    }
}

fn duration<'de, D: Deserializer<'de>>(de: D) -> Result<u16, D::Error> {
    Ok(f64::deserialize(de)?
        .round()
        .clamp(MIN_HOLD_MS.into(), MAX_HOLD_MS.into()) as u16)
}

fn axis<'de, D: Deserializer<'de>>(de: D) -> Result<u16, D::Error> {
    Ok(f64::deserialize(de)?.round().clamp(0.0, 4095.0) as u16)
}

fn deserialize_buttons<'de, D: Deserializer<'de>>(de: D) -> Result<u32, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Buttons {
        Mask(u32),
        Names(Vec<String>),
    }
    match Buttons::deserialize(de)? {
        Buttons::Mask(mask) => Ok(mask & ((1 << BUTTONS.len()) - 1)),
        Buttons::Names(names) => names.iter().try_fold(0, |mask, name| {
            let name = match name.as_str() {
                "+" => "Plus",
                "-" => "Minus",
                other => other,
            };
            let index = BUTTONS
                .iter()
                .position(|button| button.0.eq_ignore_ascii_case(name))
                .ok_or_else(|| serde::de::Error::custom(format!("Unknown button: {name}")))?;
            Ok(mask | (1 << index))
        }),
    }
}

fn serialize_buttons<S: Serializer>(mask: &u32, s: S) -> Result<S::Ok, S::Error> {
    s.collect_seq(
        BUTTONS
            .iter()
            .enumerate()
            .filter_map(|(index, button)| (mask & (1 << index) != 0).then_some(button.0)),
    )
}

#[derive(Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
pub struct Controls {
    #[serde(
        rename = "b",
        default,
        deserialize_with = "deserialize_buttons",
        serialize_with = "serialize_buttons"
    )]
    buttons: u32,
    #[serde(
        default = "center",
        deserialize_with = "axis",
        skip_serializing_if = "is_center"
    )]
    lx: u16,
    #[serde(
        default = "center",
        deserialize_with = "axis",
        skip_serializing_if = "is_center"
    )]
    ly: u16,
    #[serde(
        default = "center",
        deserialize_with = "axis",
        skip_serializing_if = "is_center"
    )]
    rx: u16,
    #[serde(
        default = "center",
        deserialize_with = "axis",
        skip_serializing_if = "is_center"
    )]
    ry: u16,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            buttons: 0,
            lx: NEUTRAL,
            ly: NEUTRAL,
            rx: NEUTRAL,
            ry: NEUTRAL,
        }
    }
}

impl Controls {
    fn read(report: &[u8; 63]) -> Self {
        let mut buttons = 0;
        for (i, &(_, byte, mask)) in BUTTONS.iter().enumerate() {
            if report[byte] & mask != 0 {
                buttons |= 1 << i;
            }
        }
        let unpack = |at: usize| {
            let [a, b, c] = [
                report[at] as u16,
                report[at + 1] as u16,
                report[at + 2] as u16,
            ];
            (a | (b & 15) << 8, (b >> 4) | c << 4)
        };
        let (lx, ly) = unpack(5);
        let (rx, ry) = unpack(8);
        Self {
            buttons,
            lx,
            ly,
            rx,
            ry,
        }
    }

    fn write(self, mut report: [u8; 63]) -> [u8; 63] {
        for (i, &(_, byte, mask)) in BUTTONS.iter().enumerate() {
            report[byte] &= !mask;
            if self.buttons & (1 << i) != 0 {
                report[byte] |= mask;
            }
        }
        let pack = |x: u16, y: u16| [x as u8, ((x >> 8) | (y << 4)) as u8, (y >> 4) as u8];
        report[5..8].copy_from_slice(&pack(self.lx, self.ly));
        report[8..11].copy_from_slice(&pack(self.rx, self.ry));
        // A macro records buttons and stick positions only. Live motion would
        // otherwise steer the camera while replaying a saved sequence.
        report[14..].fill(0);
        report
    }
}

#[derive(Clone, Deserialize, Serialize)]
pub struct MacroStep {
    #[serde(rename = "d", deserialize_with = "duration")]
    hold_ms: u16,
    #[serde(flatten)]
    controls: Controls,
}

#[derive(Deserialize, Serialize)]
pub struct MacroDocument {
    #[serde(
        rename = "v",
        default = "version",
        deserialize_with = "deserialize_version"
    )]
    format_version: u8,
    #[serde(default)]
    pub id: String,
    #[serde(default = "default_name")]
    pub name: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub game: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub author: String,
    #[serde(default)]
    pub created: u64,
    pub steps: Vec<MacroStep>,
}

impl MacroDocument {
    pub fn parse(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() > MAX_JSON_BYTES {
            return Err("Macro JSON is too large".into());
        }
        let mut doc: Self = serde_json::from_slice(bytes).map_err(|err| err.to_string())?;
        if doc.steps.is_empty() || doc.steps.len() > MAX_STEPS {
            return Err(format!(
                "A macro must contain between 1 and {MAX_STEPS} steps"
            ));
        }
        if doc.name.trim().is_empty() {
            doc.name = default_name();
        }
        if doc.id.is_empty() {
            doc.id = slug(&doc.name);
        }
        Ok(doc)
    }

    pub fn duration_ms(&self) -> u64 {
        self.steps.iter().map(|step| u64::from(step.hold_ms)).sum()
    }
}

pub fn valid_id(value: &str) -> bool {
    // SPIFFS counts the leading slash and extension in its object-name length.
    !value.is_empty()
        && value.len() <= 57
        && value.bytes().all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'_' | b'-')
        })
}

fn slug(name: &str) -> String {
    let id: String = name
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|word| !word.is_empty())
        .map(str::to_ascii_lowercase)
        .collect::<Vec<_>>()
        .join("-")
        .chars()
        .take(48)
        .collect();
    let id = id.trim_end_matches('-');
    if id.is_empty() {
        "macro".into()
    } else {
        id.into()
    }
}

struct Playback {
    steps: Vec<MacroStep>,
    started_us: u64,
    duration_ms: u64,
    index: usize,
    end_ms: u64,
}

struct Recording {
    started_us: u64,
    since_us: u64,
    last: Controls,
    steps: Vec<MacroStep>,
}

impl Recording {
    fn close_step(&mut self, now_us: u64) {
        let held_ms = (now_us.saturating_sub(self.since_us) / 1000).max(MIN_HOLD_MS.into());
        let pieces = held_ms
            .div_ceil(MAX_HOLD_MS.into())
            .min((MAX_STEPS - self.steps.len()) as u64);
        for index in 0..pieces {
            let remaining = held_ms.saturating_sub(index * u64::from(MAX_HOLD_MS));
            self.steps.push(MacroStep {
                hold_ms: remaining.clamp(MIN_HOLD_MS.into(), MAX_HOLD_MS.into()) as u16,
                controls: self.last,
            });
        }
        self.since_us = now_us;
    }
}

#[derive(Default)]
pub struct MacroEngine {
    playback: Option<Playback>,
    recording: Option<Recording>,
    release: bool,
}

#[derive(Serialize)]
pub struct MacroStatus {
    pub playing: bool,
    pub recording: bool,
    pub pos: u64,
    pub dur: u64,
    pub rec_ms: u64,
}

impl MacroEngine {
    pub fn play(&mut self, doc: MacroDocument, now_us: u64) {
        if let Some(first) = doc.steps.first() {
            self.recording = None;
            self.release = false;
            self.playback = Some(Playback {
                duration_ms: doc.duration_ms(),
                end_ms: first.hold_ms.into(),
                steps: doc.steps,
                started_us: now_us,
                index: 0,
            });
        }
    }

    pub fn stop(&mut self) {
        self.release |= self.playback.take().is_some();
    }

    pub fn output(&mut self, live: &[u8; 63], now_us: u64) -> [u8; 63] {
        if let Some(play) = self.playback.as_mut() {
            let elapsed = now_us.saturating_sub(play.started_us) / 1000;
            if elapsed < play.duration_ms {
                while elapsed >= play.end_ms {
                    play.index += 1;
                    play.end_ms += u64::from(play.steps[play.index].hold_ms);
                }
                return play.steps[play.index].controls.write(*live);
            }
            self.stop();
        }
        if std::mem::take(&mut self.release) {
            Controls::default().write(*live)
        } else {
            *live
        }
    }

    pub fn record(&mut self, now_us: u64, live: &[u8; 63]) {
        self.stop();
        self.recording = Some(Recording {
            started_us: now_us,
            since_us: now_us,
            last: Controls::read(live),
            steps: Vec::new(),
        });
    }

    pub fn capture(&mut self, report: &[u8; 63], now_us: u64) {
        let Some(recording) = self.recording.as_mut() else {
            return;
        };
        let controls = Controls::read(report);
        if controls == recording.last || recording.steps.len() >= MAX_STEPS - 1 {
            return;
        }
        if now_us.saturating_sub(recording.since_us) >= 20_000 {
            recording.close_step(now_us);
        }
        recording.last = controls;
    }

    pub fn finish_recording(&mut self, now_us: u64) -> Option<MacroDocument> {
        let mut recording = self.recording.take()?;
        recording.close_step(now_us);
        Some(MacroDocument {
            format_version: 1,
            id: String::new(),
            name: "Recorded macro".into(),
            game: String::new(),
            author: String::new(),
            created: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |time| time.as_secs()),
            steps: recording.steps,
        })
    }

    pub fn status(&self, now_us: u64) -> MacroStatus {
        let (pos, dur) = self.playback.as_ref().map_or((0, 0), |play| {
            (
                now_us.saturating_sub(play.started_us) / 1000,
                play.duration_ms,
            )
        });
        MacroStatus {
            playing: self.playback.is_some(),
            recording: self.recording.is_some(),
            pos,
            dur,
            rec_ms: self.recording.as_ref().map_or(0, |recording| {
                now_us.saturating_sub(recording.started_us) / 1000
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{MacroDocument, MacroEngine};

    #[test]
    fn reads_existing_json_and_releases_controls_at_the_end() {
        let raw = br#"{"v":"1","name":"Quick step","steps":[{"d":40,"b":["a","+"],"lx":4095},{"d":20,"b":2,"ry":0}]}"#;
        let document = MacroDocument::parse(raw).unwrap();
        assert_eq!(document.id, "quick-step");
        let mut engine = MacroEngine::default();
        engine.play(document, 1_000);
        let mut live = [0; 63];
        live[1] = 36;
        live[2] = 8;
        live[4] = 128;
        live[14] = 30;
        assert_eq!(&engine.output(&live, 1_000)[1..6], &[36, 0x42, 0, 128, 255]);
        assert_eq!(engine.output(&live, 1_000)[14], 0);
        assert_eq!(engine.output(&live, 41_000)[2] & 0x0f, 1);
        assert_eq!(engine.output(&live, 61_000)[2], 0);
        assert_eq!(engine.output(&live, 62_000), live);
    }

    #[test]
    fn records_runs_and_closes_the_last_step() {
        let mut engine = MacroEngine::default();
        let neutral = [0; 63];
        let mut pressed = neutral;
        pressed[2] = 2;
        engine.record(10_000, &neutral);
        engine.capture(&pressed, 40_000);
        engine.capture(&pressed, 55_000);
        let recording = engine.finish_recording(65_000).unwrap();
        assert_eq!(recording.steps.len(), 2);
        assert_eq!(recording.duration_ms(), 55);
        let serialized = serde_json::to_value(recording).unwrap();
        assert_eq!(serialized["steps"][1]["b"], serde_json::json!(["A"]));
    }
}
