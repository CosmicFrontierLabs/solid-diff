//! Parasolid XT binary transmit decoding. See `docs/FORMAT.md` §3.
//!
//! Port of `vendor/ps-parser/psparser/parser.py` (MIT). A transmit is a header
//! followed by node records; each node type carries its field layout the first
//! time it appears, either as "use the base schema", a delta against the base
//! schema, or a full embedded schema.

pub mod reader;
pub mod schema;

use std::collections::HashMap;

use crate::value::{Node, Value};
use reader::Reader;
use schema::{base_schema, base_schema_name, parse_embedded_field, FieldDef, TypeDef};

#[derive(Debug)]
pub struct XtError(pub String);

impl std::fmt::Display for XtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for XtError {}

/// A parsed transmit: the file header fields plus every decoded node.
pub struct Document {
    pub modeler_version: String,
    pub schema_name: String,
    pub schema_min_type: i16,
    pub schema_max_type: i16,
    pub nodes: Vec<Node>,
}

/// Decode every node record in a transmit blob.
pub fn parse_transmit(blob: &[u8]) -> Result<Vec<Node>, XtError> {
    Ok(read_document(blob)?.nodes)
}

/// Parse the `PS` header (optionally preceded by the ASCII `**PARASOLID`
/// banner found in standalone `.x_b` files).
fn parse_file_header(r: &mut Reader) -> Result<(String, String, i16, i16), XtError> {
    if r.data.starts_with(b"**") {
        let window = &r.data[..r.data.len().min(1024)];
        let marker = b"**END_OF_HEADER**";
        let mpos = (0..window.len().saturating_sub(marker.len()))
            .find(|&i| &window[i..i + marker.len()] == marker)
            .ok_or_else(|| XtError("invalid file: missing **END_OF_HEADER** marker".into()))?;
        let nl = window[mpos + marker.len()..]
            .iter()
            .position(|b| *b == b'\n')
            .ok_or_else(|| XtError("invalid file: missing newline after header marker".into()))?;
        r.pos = mpos + marker.len() + nl + 1;
    }

    if r.remaining() < 2 || &r.data[r.pos..r.pos + 2] != b"PS" {
        return Err(XtError(
            "invalid file: only PS binary format is supported".into(),
        ));
    }
    r.pos += 2;

    let modeler_version = r.str_i32_len()?;
    let schema_name = r.str_i32_len()?;
    let schema_max_type = r.i16_raw()?;
    let schema_min_type = r.i16_raw()?;
    r.u8()?; // Two unknown bytes.
    r.u8()?;

    Ok((
        modeler_version,
        schema_name,
        schema_min_type,
        schema_max_type,
    ))
}

/// Resolve a node type's field layout from embedded schema data: delta
/// instructions against the base layout, or a full embedded schema for a type
/// the base schema doesn't know.
fn resolve_node_schema(
    r: &mut Reader,
    node_type: i16,
    field_count: u8,
    types: &mut HashMap<i16, TypeDef>,
) -> Result<Vec<FieldDef>, XtError> {
    if let Some(base) = types.get(&node_type) {
        let old_fields = base.fields.clone();
        let mut merged: Vec<FieldDef> = Vec::new();
        let mut old_index = 0usize;

        loop {
            match r.u8()? {
                b'Z' => break,
                b'C' => {
                    let f = old_fields.get(old_index).ok_or_else(|| {
                        XtError(format!(
                            "delta-schema 'C' past end of base fields for node type {node_type}"
                        ))
                    })?;
                    merged.push(f.clone());
                    old_index += 1;
                }
                b'D' => old_index += 1,
                b'I' => {
                    let mut inserted = parse_embedded_field(r)?;
                    // HACK (from ps-parser): observed inserted n_elements are
                    // stored one too high when > 2.
                    if inserted.n_elements > 2 {
                        inserted.n_elements -= 1;
                    }
                    merged.push(inserted.to_field_def()?);
                }
                b'A' => {
                    let appended = parse_embedded_field(r)?;
                    merged.push(appended.to_field_def()?);
                }
                other => {
                    return Err(XtError(format!(
                        "unknown delta-schema instruction: {:?}",
                        other as char
                    )))
                }
            }
        }
        return Ok(merged);
    }

    let node_name = r.str_u8_len()?;
    let description = r.str_u8_len()?;
    let mut embedded = Vec::with_capacity(field_count as usize);
    for _ in 0..field_count {
        embedded.push(parse_embedded_field(r)?);
    }
    let fields: Vec<FieldDef> = embedded
        .iter()
        .map(|f| f.to_field_def())
        .collect::<Result<_, _>>()?;
    let variable = embedded.last().map(|f| f.xmt_code).unwrap_or(false);

    types.insert(
        node_type,
        TypeDef {
            node_type,
            node_name,
            description,
            variable,
            fields: fields.clone(),
        },
    );
    Ok(fields)
}

/// Parse a whole transmit: header, node records, terminator.
pub fn read_document(blob: &[u8]) -> Result<Document, XtError> {
    let mut r = Reader::new(blob);
    let (modeler_version, schema_name, schema_min_type, schema_max_type) =
        parse_file_header(&mut r)?;

    if !schema_name.ends_with(base_schema_name()) {
        return Err(XtError(format!(
            "file schema name '{schema_name}' does not match expected base schema '{}'",
            base_schema_name()
        )));
    }

    // Embedded full schemas are per-document, so work on a copy of the base.
    let mut types: HashMap<i16, TypeDef> = base_schema().clone();
    let mut layouts: HashMap<i16, Vec<FieldDef>> = HashMap::new();
    let mut nodes: Vec<Node> = Vec::new();

    loop {
        let node_type = r.i16_raw()?;

        if node_type == 1 {
            r.i16_raw()?; // partition value
            if !r.eof() {
                return Err(XtError(format!(
                    "expected end of file after termination node ({} bytes left)",
                    r.remaining()
                )));
            }
            return Ok(Document {
                modeler_version,
                schema_name,
                schema_min_type,
                schema_max_type,
                nodes,
            });
        }

        if node_type > schema_max_type {
            return Err(XtError(format!(
                "invalid node type {node_type}; max allowed is {schema_max_type}"
            )));
        }

        if let std::collections::hash_map::Entry::Vacant(slot) = layouts.entry(node_type) {
            let field_count = r.u8()?;
            let fields = if field_count == 255 {
                types
                    .get(&node_type)
                    .ok_or_else(|| {
                        XtError(format!(
                            "node type #{node_type} missing from base schema and no embedded \
                             schema provided"
                        ))
                    })?
                    .fields
                    .clone()
            } else {
                resolve_node_schema(&mut r, node_type, field_count, &mut types)?
            };
            slot.insert(fields);
        }

        let type_def = types
            .get(&node_type)
            .ok_or_else(|| XtError(format!("node type #{node_type} has no type definition")))?;
        let node_name = type_def.node_name.clone();
        let variable = type_def.variable;
        let node_fields = &layouts[&node_type];

        let mut count = None;
        let mut repeat_count = 0i32;
        if variable {
            repeat_count = r.i32()?;
            count = Some(repeat_count);
        }

        let id = r.i16_raw()?;

        let last = node_fields.len().saturating_sub(1);
        let mut fields: Vec<(String, Value)> = Vec::with_capacity(node_fields.len());
        for (idx, fdef) in node_fields.iter().enumerate() {
            let value = if variable && idx == last {
                fdef.read(&mut r, repeat_count)?
            } else {
                fdef.read(&mut r, 1)?
            };
            fields.push((fdef.name.clone(), value));
        }

        nodes.push(Node {
            node_type,
            name: node_name,
            id,
            count,
            fields,
        });
    }
}
