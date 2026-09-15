//! Every tool an agent has, as one list.
//!
//! A tool is a name, the sentence an agent reads before choosing it, the
//! schema of what it takes, and the [`Action`] it becomes. Nothing here
//! touches a transport: the MCP server publishes this list, the chat in the
//! Agent view sends the same list to a model, and a third front end would do
//! the same. Two lists would drift, and the one that drifted would be the one
//! nobody was looking at.

use super::Action;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};

pub struct Tool {
    pub name: &'static str,
    /// What an agent reads before choosing this tool.
    pub about: &'static str,
    /// JSON Schema of the arguments, always an object.
    pub schema: Value,
    parse: Box<dyn Fn(Value) -> Result<Action, String> + Send + Sync>,
}

impl Tool {
    /// The action this call is, or why the arguments are not it.
    pub fn action(&self, args: Value) -> Result<Action, String> {
        (self.parse)(args)
    }
}

/// A tool that takes nothing.
fn plain(name: &'static str, about: &'static str, make: fn() -> Action) -> Tool {
    Tool {
        name,
        about,
        schema: json!({ "type": "object", "properties": {} }),
        parse: Box::new(move |_| Ok(make())),
    }
}

/// A tool that takes one argument object.
fn takes<T>(name: &'static str, about: &'static str, make: fn(T) -> Action) -> Tool
where
    T: DeserializeOwned + schemars::JsonSchema + 'static,
{
    let schema = schemars::SchemaGenerator::default().into_root_schema_for::<T>();
    let schema = serde_json::to_value(schema).unwrap_or_else(|_| json!({ "type": "object" }));
    Tool {
        name,
        about,
        schema,
        parse: Box::new(move |v| {
            // A model that has nothing to pass often passes nothing at all,
            // and every field of these types is optional where that is
            // allowed, so an absent object is an empty one rather than an
            // error nobody can act on.
            let v = if v.is_null() { json!({}) } else { v };
            serde_json::from_value::<T>(v).map(make).map_err(|e| e.to_string())
        }),
    }
}

/// The tools, built once.
pub fn all() -> &'static [Tool] {
    static ALL: std::sync::OnceLock<Vec<Tool>> = std::sync::OnceLock::new();
    ALL.get_or_init(build)
}

pub fn find(name: &str) -> Option<&'static Tool> {
    all().iter().find(|t| t.name == name)
}

/// What an agent is told the receiver is, before it is told what it can do.
pub const BRIEF: &str = "WaveShark, a wideband software radio receiver. The tools drive the \
     receiver a person is looking at: what you tune, open or switch on appears in its window, \
     and what you read is what it is showing.\n\n\
     Call a tool when you need a reading you have not got. A greeting, a question about \
     what you already read, or anything a sentence answers wants no tool at all: say the \
     sentence. When you do need to look, start with `status`, and take one reading at a \
     time rather than sweeping every list before answering.\n\n\
     The receiver must be running before anything is heard: \
     `start_receiver` opens the radio. `tune` moves the whole span, and `add_channel` opens \
     something inside it: a demodulator, a protocol decoder, or `auto`, which finds what \
     transmits in the channel and decodes it. `set_decode` runs a front end across the whole \
     span instead, which is how a band is swept.\n\n\
     Levels are dBFS and are only meaningful against the noise floor `spectrum` reports. \
     Frequencies are MHz in, hertz out.\n\n\
     The receiver transmits where the radio can. `key` puts one channel on air and `unkey` \
     takes it off, half duplex, and `set_transmit` chooses what the modulator is fed. \
     `list_transmit_modes` says what can go out at all, and `say` is how to talk to \
     somebody: it keys once the channel is clear and lets go when the words run out, which \
     `key` does not. Check that the operator is licensed for the frequency and that the channel's mode is one the \
     other end can read before keying.\n\n\
     The graph can be drawn by hand. `patch` lists the stages and wires with the ids an edit \
     names them by, `list_stage_kinds` is what can be added, and `add_stage`, `connect`, \
     `disconnect` and `remove_stage` change it. What you change is kept as a difference from \
     the graph the receiver draws for itself, so it survives a retune. Ids from `patch` are \
     not the node ids `chain` reports; `chain` carries the patch id beside each node as \
     `stage`.";

fn build() -> Vec<Tool> {
    vec![
        // What the receiver is doing.
        plain(
            "status",
            "What the receiver is: which radio, where it is pointed, how wide, what it is \
             decoding, what it has heard, and every gain, switch and setting the radio itself \
             offers. Start here.",
            || Action::Status,
        ),
        plain(
            "list_devices",
            "Radios attached, on the network, or open as a recorded capture.",
            || Action::Devices,
        ),
        takes(
            "spectrum",
            "The spectrum as the waterfall is drawing it: the span reduced to a few bins by \
             peak, the noise floor, and the strongest signals by frequency.",
            Action::Spectrum,
        ),
        plain(
            "list_channels",
            "The channels open on the strip: where each is, what it does, and what it hears.",
            || Action::Channels,
        ),
        takes(
            "packets",
            "Packets decoded anywhere in the span, newest first, with the level and the signal \
             to noise each was heard at.",
            Action::Packets,
        ),
        takes(
            "calls",
            "Voice traffic: who called whom, on what, for how long, and whether it was \
             enciphered.",
            Action::Calls,
        ),
        takes(
            "transcript",
            "What the local speech model read off the audio bus.",
            Action::Transcript,
        ),
        takes(
            "messages",
            "Text sent over the air: pager messages, TETRA SDS, APRS, mesh traffic.",
            Action::Messages,
        ),
        takes(
            "links",
            "Who is talking to whom, folded from every decode that named both ends.",
            Action::Links,
        ),
        plain(
            "control_links",
            "Model control links in earshot: the handset, its frame rate, and where its sticks \
             are.",
            || Action::ControlLinks,
        ),
        takes(
            "tracks",
            "Aircraft and vessels the tracker is holding, with position, altitude and course.",
            Action::Tracks,
        ),
        takes(
            "satellites",
            "Satellite passes over the receiver's own position, soonest first.",
            Action::Satellites,
        ),
        plain(
            "chain",
            "The flow graph the receiver is running: every node, its ports, the wires between \
             them, what each is costing, and the parameters `set_node_param` can move.",
            || Action::Chain,
        ),
        plain(
            "patch",
            "The graph as something to edit: every stage with the id an edit names it by, which \
             are the receiver's own until you add one, every wire, and what the operator has \
             changed. `chain` is the same graph as it is running; this is what `add_stage`, \
             `connect` and `remove_stage` work on.",
            || Action::Patch,
        ),
        plain(
            "list_stage_kinds",
            "Every kind of stage that can be added to the graph, with what each is for.",
            || Action::StageKinds,
        ),
        plain(
            "scanners",
            "The scanner table: which front end the receiver places on which frequency.",
            || Action::Scanners,
        ),
        plain("memory", "The memory bank: channels somebody saved, in groups.", || Action::Memory),
        plain(
            "list_protocols",
            "Every protocol this build can decode, with the id `add_channel` takes as a mode and \
             where each is usually found.",
            || Action::Protocols,
        ),
        plain("screenshot", "A PNG of the interface as it is now.", || Action::Screenshot),
        // What it is set to.
        plain("start_receiver", "Start the radio: open the device and run the graph.", || {
            Action::Start
        }),
        plain(
            "stop_receiver",
            "Stop the radio and release the device, without closing the window.",
            || Action::Stop,
        ),
        takes(
            "select_device",
            "Use a different radio, by any part of its label. Restarts the graph on it.",
            Action::SelectDevice,
        ),
        takes(
            "tune",
            "Point the receiver at a frequency, in MHz. This moves the whole span.",
            Action::Tune,
        ),
        takes(
            "set_span",
            "Work in a span this wide, in kHz. The nearest the radio can manage is used.",
            Action::Span,
        ),
        takes(
            "set_gain",
            "Set one gain stage on the radio, or hand it back to the hardware's own control.",
            Action::Gain,
        ),
        takes(
            "set_toggle",
            "Flip one of the radio's own switches: bias tee, digital AGC, and so on.",
            Action::Toggle,
        ),
        takes(
            "set_choice",
            "Pick one of the radio's list settings, such as which antenna port the cable is in.",
            Action::Choice,
        ),
        takes(
            "set_ppm",
            "Correct the reference oscillator of the radio in use, in parts per million.",
            Action::Ppm,
        ),
        takes(
            "set_location",
            "Tell the receiver where it is, in degrees. One ADS-B frame then fixes an aircraft, \
             and satellite passes are computed from here.",
            Action::Location,
        ),
        // Channels and what is heard.
        takes(
            "add_channel",
            "Open a channel inside the span: a demodulator to listen to, a protocol decoder, or \
             the auto front end to find whatever transmits in it. Returns its id.",
            Action::AddChannel,
        ),
        takes(
            "set_channel",
            "Change a channel: its frequency, mode, width, level, squelch or label.",
            Action::SetChannel,
        ),
        takes("remove_channel", "Close a channel.", Action::RemoveChannel),
        takes(
            "listen",
            "Make this the channel the chain view and the spectrum follow.",
            Action::Listen,
        ),
        takes(
            "set_volume",
            "The master level on the speaker, and whether anything is heard.",
            Action::Volume,
        ),
        // Transmitting.
        takes(
            "key",
            "Put a channel on air and leave it there until `unkey`. The radio has to be able to \
             transmit and the channel's mode has to have a modulator behind it; both are \
             reported by `list_channels`. One channel at a time.",
            Action::Key,
        ),
        plain("unkey", "Take the transmitter off air.", || Action::Unkey),
        takes(
            "set_transmit",
            "What a keyed channel puts through the modulator: a tone for a deviation or power \
             check, the microphone, or the agent's own voice. Also the tone's frequency, the \
             level into the modulator, and the file a digital mode transmits.",
            Action::Transmit,
        ),
        takes("set_tx_gain", "The radio's transmit gain, in dB.", Action::TxGain),
        takes(
            "say",
            "Say something over the air, in the agent's own voice. The channel keys itself once \
             the channel is clear and lets go when the words run out, so this is the way to \
             talk to somebody rather than `key`. One over: a long line is cut. Needs a channel \
             whose transmit source is the agent, which this sets when given one.",
            Action::Say,
        ),
        plain(
            "list_transmit_modes",
            "What this radio and this build can transmit: the mode to open a channel in, what \
             feeds the modulator, and how wide each is. A channel transmits in the mode it \
             receives, so this is the list `add_channel` takes a mode from before `key`.",
            || Action::TransmitModes,
        ),
        // What it watches, writes and shows.
        takes(
            "set_decode",
            "Decode every channel in the span, or stop. This is the most expensive thing the \
             receiver does.",
            Action::Decode,
        ),
        takes(
            "set_dc_block",
            "Remove the centre spur a direct-conversion receiver produces, or leave it in.",
            Action::DcBlock,
        ),
        takes(
            "set_view",
            "Show a view in the window, which is what a following screenshot then holds.",
            Action::View,
        ),
        takes("set_record", "Write every burst that decodes to a folder, or stop.", Action::Record),
        takes(
            "set_capture_iq",
            "Write the raw span to a file, or stop. This fills a disk quickly.",
            Action::CaptureIq,
        ),
        takes(
            "set_packet_log",
            "Write the binary packet log, one file a day, or stop.",
            Action::PacketLog,
        ),
        takes(
            "set_node_param",
            "Set one parameter on one node of the running graph, by the node id `chain` reports. \
             This reaches anything the chain view can change.",
            Action::NodeParam,
        ),
        // Drawing the graph.
        takes(
            "add_stage",
            "Add a stage to the graph, unconnected. Wire it up with `connect`. Answers once the \
             receiver has rebuilt, so a stage that will not build is reported rather than \
             assumed.",
            Action::AddStage,
        ),
        takes(
            "remove_stage",
            "Delete a stage and every wire that touched it. A stage the receiver drew for itself \
             can be deleted too; it comes back if the plan draws it again.",
            Action::RemoveStage,
        ),
        takes(
            "connect",
            "Draw a wire: feed one stage's input from the span or from another stage's output. \
             An input takes one producer, so this replaces what was there.",
            Action::Connect,
        ),
        takes("disconnect", "Take the wire off one input, leaving it unfed.", Action::Disconnect),
        plain("undo_edit", "Take back the last graph edit.", || Action::UndoEdit),
        plain("redo_edit", "Put back the edit that was last taken away.", || Action::RedoEdit),
        plain(
            "reset_graph",
            "Throw away every edit and go back to the graph the receiver draws for itself from \
             the dial, the scanner table and the strip.",
            || Action::ResetGraph,
        ),
        takes(
            "set_manual",
            "Unlock the graph in the window so a person can drag and wire it. Edits made through \
             these tools apply either way; this only changes what the chain view lets a hand do.",
            Action::Manual,
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_two_tools_share_a_name() {
        let mut names: Vec<&str> = all().iter().map(|t| t.name).collect();
        names.sort_unstable();
        let before = names.len();
        names.dedup();
        assert_eq!(names.len(), before, "a tool name is used twice");
        assert_eq!(before, 55, "the catalogue changed size");
    }

    /// Every schema is an object, because that is what both the protocol and
    /// every model's function calling require of arguments.
    #[test]
    fn every_schema_is_an_object() {
        for t in all() {
            assert_eq!(
                t.schema.get("type").and_then(|v| v.as_str()),
                Some("object"),
                "{} has no object schema",
                t.name
            );
            assert!(!t.about.is_empty(), "{} says nothing about itself", t.name);
        }
    }

    #[test]
    fn a_tool_with_no_arguments_takes_null_or_nothing() {
        let t = find("status").expect("status is in the catalogue");
        assert!(matches!(t.action(Value::Null), Ok(Action::Status)));
        assert!(matches!(t.action(json!({})), Ok(Action::Status)));
    }

    #[test]
    fn arguments_are_parsed_and_bad_ones_are_refused() {
        let t = find("tune").expect("tune is in the catalogue");
        match t.action(json!({ "mhz": 145.5 })) {
            Ok(Action::Tune(a)) => assert_eq!(a.mhz, 145.5),
            _ => panic!("tune did not parse"),
        }
        assert!(t.action(json!({ "mhz": "145.5" })).is_err(), "a string is not a frequency");
    }

    /// A tool whose fields are all optional is callable with nothing, which
    /// is what a model does when it wants the default.
    #[test]
    fn an_all_optional_tool_takes_nothing() {
        let t = find("packets").expect("packets is in the catalogue");
        assert!(t.action(Value::Null).is_ok());
    }
}
