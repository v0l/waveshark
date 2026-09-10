//! The tools an agent sees, one per [`Action`].
//!
//! Every one of them does the same thing: put the action on the desk and hand
//! back what the interface answered. The work is in `ui::agent`, where the
//! interface can be touched; what lives here is the name, the schema and the
//! sentence an agent reads before choosing.

use super::{Action, Desk, args};
use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::wrapper::{Json, Parameters};
use rmcp::model::{
    CallToolResult, ContentBlock, ErrorData, Implementation, ServerCapabilities, ServerInfo,
};
use rmcp::{ServerHandler, tool, tool_handler, tool_router};

#[derive(Clone)]
pub struct Tools {
    desk: Desk,
    tool_router: ToolRouter<Self>,
}

impl Tools {
    pub fn new(desk: Desk) -> Self {
        Self { desk, tool_router: Self::tool_router() }
    }

    async fn ask(&self, action: Action) -> Result<Json<serde_json::Value>, ErrorData> {
        match self.desk.ask(action).await {
            Ok(v) => Ok(Json(v)),
            Err(e) => Err(ErrorData::internal_error(e, None)),
        }
    }
}

#[tool_router]
impl Tools {
    #[tool(
        description = "What the receiver is: which radio, where it is pointed, how wide, what \
                       it is decoding, what it has heard, and every gain, switch and setting \
                       the radio itself offers. Start here."
    )]
    async fn status(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Status).await
    }

    #[tool(description = "Radios attached, on the network, or open as a recorded capture.")]
    async fn list_devices(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Devices).await
    }

    #[tool(
        description = "The spectrum as the waterfall is drawing it: the span reduced to a few \
                       bins by peak, the noise floor, and the strongest signals by frequency."
    )]
    async fn spectrum(
        &self,
        Parameters(a): Parameters<args::Spectrum>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Spectrum(a)).await
    }

    #[tool(description = "The channels open on the strip: where each is, what it does, and what it hears.")]
    async fn list_channels(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Channels).await
    }

    #[tool(
        description = "Packets decoded anywhere in the span, newest first, with the level and \
                       the signal to noise each was heard at."
    )]
    async fn packets(
        &self,
        Parameters(a): Parameters<args::Packets>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Packets(a)).await
    }

    #[tool(description = "Voice traffic: who called whom, on what, for how long, and whether it was enciphered.")]
    async fn calls(
        &self,
        Parameters(a): Parameters<args::Limit>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Calls(a)).await
    }

    #[tool(description = "What the local speech model read off the audio bus.")]
    async fn transcript(
        &self,
        Parameters(a): Parameters<args::Limit>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Transcript(a)).await
    }

    #[tool(description = "Text sent over the air: pager messages, TETRA SDS, APRS, mesh traffic.")]
    async fn messages(
        &self,
        Parameters(a): Parameters<args::Limit>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Messages(a)).await
    }

    #[tool(description = "Who is talking to whom, folded from every decode that named both ends.")]
    async fn links(
        &self,
        Parameters(a): Parameters<args::Limit>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Links(a)).await
    }

    #[tool(description = "Model control links in earshot: the handset, its frame rate, and where its sticks are.")]
    async fn control_links(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::ControlLinks).await
    }

    #[tool(description = "Aircraft and vessels the tracker is holding, with position, altitude and course.")]
    async fn tracks(
        &self,
        Parameters(a): Parameters<args::Limit>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Tracks(a)).await
    }

    #[tool(description = "Satellite passes over the receiver's own position, soonest first.")]
    async fn satellites(
        &self,
        Parameters(a): Parameters<args::Limit>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Satellites(a)).await
    }

    #[tool(
        description = "The flow graph the receiver is running: every node, its ports, the wires \
                       between them, what each is costing, and the parameters `set_node_param` \
                       can move."
    )]
    async fn chain(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Chain).await
    }

    #[tool(
        description = "The graph as something to edit: every stage with the id an edit names it \
                       by, which are the receiver's own until you add one, every wire, and what \
                       the operator has changed. `chain` is the same graph as it is running; this \
                       is what `add_stage`, `connect` and `remove_stage` work on."
    )]
    async fn patch(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Patch).await
    }

    #[tool(
        description = "Every kind of stage that can be added to the graph, with what each is for."
    )]
    async fn list_stage_kinds(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::StageKinds).await
    }

    #[tool(
        description = "Add a stage to the graph, unconnected. Wire it up with `connect`. Answers \
                       once the receiver has rebuilt, so a stage that will not build is reported \
                       rather than assumed."
    )]
    async fn add_stage(
        &self,
        Parameters(a): Parameters<args::StageKind>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::AddStage(a)).await
    }

    #[tool(
        description = "Delete a stage and every wire that touched it. A stage the receiver drew \
                       for itself can be deleted too; it comes back if the plan draws it again."
    )]
    async fn remove_stage(
        &self,
        Parameters(a): Parameters<args::StageId>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::RemoveStage(a)).await
    }

    #[tool(
        description = "Draw a wire: feed one stage's input from the span or from another stage's \
                       output. An input takes one producer, so this replaces what was there."
    )]
    async fn connect(
        &self,
        Parameters(a): Parameters<args::Connect>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Connect(a)).await
    }

    #[tool(description = "Take the wire off one input, leaving it unfed.")]
    async fn disconnect(
        &self,
        Parameters(a): Parameters<args::Disconnect>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Disconnect(a)).await
    }

    #[tool(description = "Take back the last graph edit.")]
    async fn undo_edit(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::UndoEdit).await
    }

    #[tool(description = "Put back the edit that was last taken away.")]
    async fn redo_edit(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::RedoEdit).await
    }

    #[tool(
        description = "Throw away every edit and go back to the graph the receiver draws for \
                       itself from the dial, the scanner table and the strip."
    )]
    async fn reset_graph(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::ResetGraph).await
    }

    #[tool(
        description = "Unlock the graph in the window so a person can drag and wire it. Edits \
                       made through these tools apply either way; this only changes what the \
                       chain view lets a hand do."
    )]
    async fn set_manual(
        &self,
        Parameters(a): Parameters<args::Switch>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Manual(a)).await
    }

    #[tool(description = "The scanner table: which front end the receiver places on which frequency.")]
    async fn scanners(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Scanners).await
    }

    #[tool(description = "The memory bank: channels somebody saved, in groups.")]
    async fn memory(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Memory).await
    }

    #[tool(
        description = "Every protocol this build can decode, with the id `add_channel` takes as \
                       a mode and where each is usually found."
    )]
    async fn list_protocols(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Protocols).await
    }

    #[tool(description = "A PNG of the interface as it is now.")]
    async fn screenshot(&self) -> Result<CallToolResult, ErrorData> {
        let v = self.desk.ask(Action::Screenshot).await.map_err(|e| ErrorData::internal_error(e, None))?;
        let png = v.get("png_base64").and_then(|p| p.as_str()).unwrap_or_default().to_string();
        Ok(CallToolResult::success(vec![ContentBlock::image(png, "image/png")]))
    }

    #[tool(description = "Start the radio: open the device and run the graph.")]
    async fn start_receiver(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Start).await
    }

    #[tool(description = "Stop the radio and release the device, without closing the window.")]
    async fn stop_receiver(&self) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Stop).await
    }

    #[tool(description = "Use a different radio, by any part of its label. Restarts the graph on it.")]
    async fn select_device(
        &self,
        Parameters(a): Parameters<args::Device>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::SelectDevice(a)).await
    }

    #[tool(description = "Point the receiver at a frequency, in MHz. This moves the whole span.")]
    async fn tune(
        &self,
        Parameters(a): Parameters<args::Tune>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Tune(a)).await
    }

    #[tool(description = "Work in a span this wide, in kHz. The nearest the radio can manage is used.")]
    async fn set_span(
        &self,
        Parameters(a): Parameters<args::Span>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Span(a)).await
    }

    #[tool(description = "Set one gain stage on the radio, or hand it back to the hardware's own control.")]
    async fn set_gain(
        &self,
        Parameters(a): Parameters<args::Gain>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Gain(a)).await
    }

    #[tool(description = "Flip one of the radio's own switches: bias tee, digital AGC, and so on.")]
    async fn set_toggle(
        &self,
        Parameters(a): Parameters<args::Toggle>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Toggle(a)).await
    }

    #[tool(description = "Pick one of the radio's list settings, such as which antenna port the cable is in.")]
    async fn set_choice(
        &self,
        Parameters(a): Parameters<args::Choice>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Choice(a)).await
    }

    #[tool(description = "Correct the reference oscillator of the radio in use, in parts per million.")]
    async fn set_ppm(
        &self,
        Parameters(a): Parameters<args::Ppm>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Ppm(a)).await
    }

    #[tool(
        description = "Tell the receiver where it is, in degrees. One ADS-B frame then fixes an \
                       aircraft, and satellite passes are computed from here."
    )]
    async fn set_location(
        &self,
        Parameters(a): Parameters<args::Location>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Location(a)).await
    }

    #[tool(
        description = "Open a channel inside the span: a demodulator to listen to, a protocol \
                       decoder, or the auto front end to find whatever transmits in it. Returns its id."
    )]
    async fn add_channel(
        &self,
        Parameters(a): Parameters<args::AddChannel>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::AddChannel(a)).await
    }

    #[tool(description = "Change a channel: its frequency, mode, width, level, squelch or label.")]
    async fn set_channel(
        &self,
        Parameters(a): Parameters<args::SetChannel>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::SetChannel(a)).await
    }

    #[tool(description = "Close a channel.")]
    async fn remove_channel(
        &self,
        Parameters(a): Parameters<args::Channel>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::RemoveChannel(a)).await
    }

    #[tool(description = "Make this the channel the chain view and the spectrum follow.")]
    async fn listen(
        &self,
        Parameters(a): Parameters<args::Channel>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Listen(a)).await
    }

    #[tool(description = "The master level, and whether the mix leaves the bus at all.")]
    async fn set_volume(
        &self,
        Parameters(a): Parameters<args::Volume>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Volume(a)).await
    }

    #[tool(
        description = "Decode every channel in the span, or stop. This is the most expensive \
                       thing the receiver does."
    )]
    async fn set_decode(
        &self,
        Parameters(a): Parameters<args::Switch>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Decode(a)).await
    }

    #[tool(description = "Remove the centre spur a direct-conversion receiver produces, or leave it in.")]
    async fn set_dc_block(
        &self,
        Parameters(a): Parameters<args::Switch>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::DcBlock(a)).await
    }

    #[tool(description = "Show a view in the window, which is what a following screenshot then holds.")]
    async fn set_view(
        &self,
        Parameters(a): Parameters<args::View>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::View(a)).await
    }

    #[tool(description = "Write every burst that decodes to a folder, or stop.")]
    async fn set_record(
        &self,
        Parameters(a): Parameters<args::Record>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::Record(a)).await
    }

    #[tool(description = "Write the raw span to a file, or stop. This fills a disk quickly.")]
    async fn set_capture_iq(
        &self,
        Parameters(a): Parameters<args::Switch>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::CaptureIq(a)).await
    }

    #[tool(description = "Write the binary packet log, one file a day, or stop.")]
    async fn set_packet_log(
        &self,
        Parameters(a): Parameters<args::PacketLog>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::PacketLog(a)).await
    }

    #[tool(
        description = "Set one parameter on one node of the running graph, by the node id `chain` \
                       reports. This reaches anything the chain view can change."
    )]
    async fn set_node_param(
        &self,
        Parameters(a): Parameters<args::NodeParam>,
    ) -> Result<Json<serde_json::Value>, ErrorData> {
        self.ask(Action::NodeParam(a)).await
    }
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for Tools {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(Implementation::new("waveshark", env!("CARGO_PKG_VERSION")))
            .with_instructions(
            "WaveShark, a wideband software radio receiver. The tools drive the receiver a \
             person is looking at: what you tune, open or switch on appears in its window, and \
             what you read is what it is showing.\n\n\
             Start with `status`. The receiver must be running before anything is heard: \
             `start_receiver` opens the radio. `tune` moves the whole span, and `add_channel` \
             opens something inside it: a demodulator, a protocol decoder, or `auto`, which \
             finds what transmits in the channel and decodes it. `set_decode` runs a front end \
             across the whole span instead, which is how a band is swept.\n\n\
             Levels are dBFS and are only meaningful against the noise floor `spectrum` \
             reports. Frequencies are MHz in, hertz out. This receiver does not transmit \
             through these tools.\n\n\
             The graph can be drawn by hand. `patch` lists the stages and wires with the ids an \
             edit names them by, `list_stage_kinds` is what can be added, and `add_stage`, \
             `connect`, `disconnect` and `remove_stage` change it. What you change is kept as a \
             difference from the graph the receiver draws for itself, so it survives a retune. \
             Ids from `patch` are not the node ids `chain` reports; `chain` carries the patch id \
             beside each node as `stage`.",
        )
    }
}
