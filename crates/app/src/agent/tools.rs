//! The MCP face of the catalogue.
//!
//! Every tool does the same thing: put the action on the desk and hand back
//! what the interface answered. The work is in `ui::agent`, where the
//! interface can be touched; the names, the schemas and the sentences are in
//! [`super::catalog`], which the chat reads too.

use super::{Action, Desk, catalog};
use rmcp::model::{
    CallToolRequestParams, CallToolResponse, CallToolResult, ContentBlock, ErrorData,
    Implementation, ListToolsResult, PaginatedRequestParams, ServerCapabilities, ServerInfo,
};
use rmcp::service::RequestContext;
use rmcp::{RoleServer, ServerHandler};

#[derive(Clone)]
pub struct Tools {
    desk: Desk,
}

impl Tools {
    pub fn new(desk: Desk) -> Self {
        Self { desk }
    }
}

/// The arguments' schema, as the protocol wants it: a JSON object, by pointer.
fn schema_of(
    tool: &'static catalog::Tool,
) -> std::sync::Arc<serde_json::Map<String, serde_json::Value>> {
    let map = tool.schema.as_object().cloned().unwrap_or_default();
    std::sync::Arc::new(map)
}

impl ServerHandler for Tools {
    /// The tools, with no output schemas.
    ///
    /// Every tool here answers with whatever JSON the receiver has to hand: a
    /// list of packets, a spectrum, a whole patch. A schema that describes
    /// nothing is worse than none at all, since a schema with no `type` is
    /// not an object schema and strict clients refuse the whole list over it.
    async fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, ErrorData> {
        let tools = catalog::all()
            .iter()
            .map(|t| rmcp::model::Tool::new(t.name, t.about, schema_of(t)))
            .collect::<Vec<_>>();
        Ok(ListToolsResult::with_all_items(tools))
    }

    async fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> Result<CallToolResponse, ErrorData> {
        let tool = catalog::find(&request.name)
            .ok_or_else(|| ErrorData::invalid_params(format!("no tool {}", request.name), None))?;
        let args =
            request.arguments.map(serde_json::Value::Object).unwrap_or(serde_json::Value::Null);
        let action = tool.action(args).map_err(|e| ErrorData::invalid_params(e, None))?;
        let shot = matches!(action, Action::Screenshot);
        let value = self.desk.ask(action).await.map_err(|e| ErrorData::internal_error(e, None))?;
        if shot {
            let png =
                value.get("png_base64").and_then(|p| p.as_str()).unwrap_or_default().to_string();
            return Ok(CallToolResult::success(vec![ContentBlock::image(png, "image/png")]).into());
        }
        Ok(CallToolResult::structured(value).into())
    }

    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(Implementation::new("waveshark", env!("CARGO_PKG_VERSION")))
            .with_instructions(catalog::BRIEF)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What is served is the catalogue, whole, with the schemas intact. A
    /// tool published without an object schema is one a strict client
    /// refuses the entire list over.
    #[test]
    fn the_server_publishes_the_catalogue() {
        let served: Vec<rmcp::model::Tool> = catalog::all()
            .iter()
            .map(|t| rmcp::model::Tool::new(t.name, t.about, schema_of(t)))
            .collect();
        assert_eq!(served.len(), catalog::all().len());
        for (tool, from) in served.iter().zip(catalog::all()) {
            assert_eq!(tool.name, from.name);
            assert_eq!(tool.input_schema.get("type").and_then(|v| v.as_str()), Some("object"));
            assert!(tool.output_schema.is_none(), "{} publishes an output schema", tool.name);
        }
    }
}
