//! Parasolid XT schema: the bundled base schema plus embedded (delta and full)
//! schema definitions. Port of `vendor/ps-parser/psparser/schema.py`.
//!
//! The base schema is the MIT-licensed `sch_13006.s_t` text file shipped with
//! ps-parser; it is compiled into the binary and parsed once, lazily.

use std::collections::HashMap;
use std::sync::OnceLock;

use super::reader::Reader;
use super::XtError;
use crate::value::Value;

/// The bundled base schema text (ps-parser, MIT).
pub const BASE_SCHEMA_TEXT: &str = include_str!("../../assets/sch_13006.s_t");

/// Field type codes, as they appear in schema text and embedded field defs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldType {
    U8,
    Char,
    Logical,
    I16,
    Utf16,
    I32,
    Pointer,
    F64,
    Interval,
    Vector,
    Box,
    /// `h`: a vector, distinguished only by name in the schema.
    H,
}

impl FieldType {
    pub fn from_code(code: &str) -> Option<FieldType> {
        Some(match code {
            "u" => FieldType::U8,
            "c" => FieldType::Char,
            "l" => FieldType::Logical,
            "n" => FieldType::I16,
            "w" => FieldType::Utf16,
            "d" => FieldType::I32,
            "p" => FieldType::Pointer,
            "f" => FieldType::F64,
            "i" => FieldType::Interval,
            "v" => FieldType::Vector,
            "b" => FieldType::Box,
            "h" => FieldType::H,
            _ => return None,
        })
    }
}

/// One transmitted schema field with decode metadata.
#[derive(Debug, Clone)]
pub struct FieldDef {
    pub name: String,
    pub field_type: FieldType,
    pub node_class: i32,
    pub n_elements: i32,
}

impl FieldDef {
    fn read_one(&self, r: &mut Reader) -> Result<Value, XtError> {
        Ok(match self.field_type {
            FieldType::U8 => Value::U8(r.u8()?),
            FieldType::Char => Value::Char(r.char(1)?),
            FieldType::Logical => Value::Bool(r.bool8()?),
            FieldType::I16 => match r.i16()? {
                Some(v) => Value::I16(v),
                None => Value::Null,
            },
            FieldType::Utf16 => Value::Utf16(r.utf16_be(1)?),
            FieldType::I32 => Value::I32(r.i32()?),
            FieldType::Pointer => Value::Ptr(r.pointer()?),
            FieldType::F64 => match r.f64()? {
                Some(v) => Value::F64(v),
                None => Value::Null,
            },
            FieldType::Interval => Value::Interval(r.interval()?),
            FieldType::Vector | FieldType::H => Value::Vec3(r.vector()?),
            FieldType::Box => Value::Box3(r.box3()?),
        })
    }

    /// Decode this field. For variable-length nodes the final field passes the
    /// node's repeat count as `count_override` (mirrors `FieldDef.read`).
    pub fn read(&self, r: &mut Reader, count_override: i32) -> Result<Value, XtError> {
        let count = if count_override > 1 {
            count_override
        } else {
            self.n_elements
        };

        if count > 1 {
            let n = count as usize;
            return Ok(match self.field_type {
                FieldType::Char => Value::Char(r.char(n)?),
                FieldType::Utf16 => Value::Utf16(r.utf16_be(n)?),
                _ => {
                    // Cap the pre-allocation: a corrupt count must not turn
                    // into a huge allocation (every element costs >= 1 byte).
                    let mut items = Vec::with_capacity(n.min(r.remaining()));
                    for _ in 0..n {
                        items.push(self.read_one(r)?);
                    }
                    Value::Array(items)
                }
            });
        }

        self.read_one(r)
    }
}

/// One node type description.
#[derive(Debug, Clone)]
pub struct TypeDef {
    pub node_type: i16,
    pub node_name: String,
    pub description: String,
    pub variable: bool,
    pub fields: Vec<FieldDef>,
}

/// Transient representation of an embedded schema field.
#[derive(Debug, Clone)]
pub struct EmbeddedField {
    pub name: String,
    pub ptr_class: i16,
    pub n_elements: i16,
    pub field_type: String,
    pub xmt_code: bool,
}

impl EmbeddedField {
    pub fn to_field_def(&self) -> Result<FieldDef, XtError> {
        let ft = FieldType::from_code(&self.field_type)
            .ok_or_else(|| XtError(format!("'{}' is not a valid field type", self.field_type)))?;
        Ok(FieldDef {
            name: self.name.clone(),
            field_type: ft,
            node_class: self.ptr_class as i32,
            n_elements: self.n_elements as i32,
        })
    }
}

/// Parse one field definition from embedded schema data.
pub fn parse_embedded_field(r: &mut Reader) -> Result<EmbeddedField, XtError> {
    let name = r.str_u8_len()?;
    let ptr_class = r.i16_raw()?;
    let n_elements = r.i16_raw()?;
    let field_type = if ptr_class == 0 {
        r.str_u8_len()?
    } else {
        "p".to_string()
    };
    let xmt_code = if n_elements == 2 { r.bool8()? } else { false };
    Ok(EmbeddedField {
        name,
        ptr_class,
        n_elements,
        field_type,
        xmt_code,
    })
}

// ---------------------------------------------------------------------------
// Base schema text parsing (hand-rolled equivalents of schema.py's two regexes)
// ---------------------------------------------------------------------------

struct Cursor<'a> {
    s: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn lit(&mut self, needle: &[u8]) -> bool {
        if self.s.len() >= self.pos + needle.len()
            && &self.s[self.pos..self.pos + needle.len()] == needle
        {
            self.pos += needle.len();
            true
        } else {
            false
        }
    }

    /// `\d+`
    fn digits(&mut self) -> Option<&'a str> {
        let start = self.pos;
        while self.pos < self.s.len() && self.s[self.pos].is_ascii_digit() {
            self.pos += 1;
        }
        if self.pos == start {
            None
        } else {
            std::str::from_utf8(&self.s[start..self.pos]).ok()
        }
    }

    fn take_while(&mut self, pred: impl Fn(u8) -> bool) -> &'a [u8] {
        let start = self.pos;
        while self.pos < self.s.len() && pred(self.s[self.pos]) {
            self.pos += 1;
        }
        &self.s[start..self.pos]
    }
}

fn is_word_byte(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// `(?P<fieldname>[a-z]\w*); (?P<type>\w); (?P<transmit>\d+) (?P<nodeclass>\d+) (?P<n_elements>\d+) `
struct RawField<'a> {
    name: &'a str,
    type_code: &'a str,
    transmit: i32,
    node_class: i32,
    n_elements: i32,
}

fn match_field(s: &[u8], at: usize) -> Option<(RawField<'_>, usize)> {
    let mut c = Cursor { s, pos: at };
    if c.pos >= s.len() || !s[c.pos].is_ascii_lowercase() {
        return None;
    }
    let name = c.take_while(is_word_byte);
    if !c.lit(b"; ") {
        return None;
    }
    let tstart = c.pos;
    if c.pos >= s.len() || !is_word_byte(s[c.pos]) {
        return None;
    }
    c.pos += 1;
    let type_code = &s[tstart..c.pos];
    if !c.lit(b"; ") {
        return None;
    }
    let transmit = c.digits()?;
    if !c.lit(b" ") {
        return None;
    }
    let node_class = c.digits()?;
    if !c.lit(b" ") {
        return None;
    }
    let n_elements = c.digits()?;
    if !c.lit(b" ") {
        return None;
    }
    Some((
        RawField {
            name: std::str::from_utf8(name).ok()?,
            type_code: std::str::from_utf8(type_code).ok()?,
            transmit: transmit.parse().ok()?,
            node_class: node_class.parse().ok()?,
            n_elements: n_elements.parse().ok()?,
        },
        c.pos,
    ))
}

/// `^(\d+) ([A-Z_]+); ([^;]+); (\d+) (\d+) (\d+) \n((?:\D.+\n)*)`, anchored at
/// a line start.
struct RawType<'a> {
    node_type: i64,
    node_name: &'a str,
    description: &'a str,
    transmit: i32,
    n_fields: usize,
    variable: bool,
    fields: &'a [u8],
}

fn match_type(s: &[u8], at: usize) -> Option<(RawType<'_>, usize)> {
    let mut c = Cursor { s, pos: at };
    let node_type = c.digits()?;
    if !c.lit(b" ") {
        return None;
    }
    let name = c.take_while(|b| b.is_ascii_uppercase() || b == b'_');
    if name.is_empty() || !c.lit(b"; ") {
        return None;
    }
    // `[^;]+` is greedy but cannot cross a ';', so it ends at the next one.
    let desc = c.take_while(|b| b != b';');
    if desc.is_empty() || !c.lit(b"; ") {
        return None;
    }
    let transmit = c.digits()?;
    if !c.lit(b" ") {
        return None;
    }
    let n_fields = c.digits()?;
    if !c.lit(b" ") {
        return None;
    }
    let variable = c.digits()?;
    if !c.lit(b" \n") {
        return None;
    }

    // `(?:\D.+\n)*`: lines that start with a non-digit and have >= 2 chars.
    let fields_start = c.pos;
    loop {
        let line_start = c.pos;
        if line_start >= s.len() || s[line_start].is_ascii_digit() {
            break;
        }
        let Some(nl) = s[line_start..].iter().position(|b| *b == b'\n') else {
            break;
        };
        if nl < 2 {
            break;
        }
        c.pos = line_start + nl + 1;
    }
    let fields = &s[fields_start..c.pos];

    Some((
        RawType {
            node_type: node_type.parse().ok()?,
            node_name: std::str::from_utf8(name).ok()?,
            description: std::str::from_utf8(desc).ok()?,
            transmit: transmit.parse().ok()?,
            n_fields: n_fields.parse().ok()?,
            variable: variable.parse::<i32>().ok()? != 0,
            fields,
        },
        c.pos,
    ))
}

/// Parse base schema text into transmitted type definitions.
///
/// Mirrors `parse_base_schema`: non-transmitted types are dropped, so are
/// non-transmitted *fields*, and a type whose transmitted fields include an
/// unknown type code is dropped entirely (the error surfaces inside the loop and
/// catches it per type — e.g. TAG_VALUES, which has a `t` field).
pub fn parse_base_schema(text: &str) -> HashMap<i16, TypeDef> {
    let s = text.as_bytes();
    let mut out = HashMap::new();

    let mut pos = 0usize;
    while pos < s.len() {
        let line_end = s[pos..]
            .iter()
            .position(|b| *b == b'\n')
            .map(|i| pos + i + 1)
            .unwrap_or(s.len());
        let Some((raw, end)) = match_type(s, pos) else {
            pos = line_end;
            continue;
        };
        pos = end.max(line_end);

        if raw.transmit == 0 {
            continue;
        }
        let Ok(node_type) = i16::try_from(raw.node_type) else {
            continue;
        };

        let mut fields = Vec::new();
        let mut parsed_count = 0usize;
        let mut bad_code = false;
        let mut fpos = 0usize;
        while fpos < raw.fields.len() {
            let Some((f, fend)) = match_field(raw.fields, fpos) else {
                fpos += 1;
                continue;
            };
            fpos = fend;
            parsed_count += 1;
            if f.transmit == 0 {
                continue;
            }
            match FieldType::from_code(f.type_code) {
                Some(ft) => fields.push(FieldDef {
                    name: f.name.to_string(),
                    field_type: ft,
                    node_class: f.node_class,
                    n_elements: f.n_elements,
                }),
                None => {
                    // The whole type is thrown out in this case.
                    bad_code = true;
                    break;
                }
            }
        }
        if bad_code {
            continue;
        }
        debug_assert_eq!(
            parsed_count, raw.n_fields,
            "field count mismatch for node type {node_type}"
        );

        out.insert(
            node_type,
            TypeDef {
                node_type,
                node_name: raw.node_name.to_string(),
                description: raw.description.to_string(),
                variable: raw.variable,
                fields,
            },
        );
    }

    out
}

/// The bundled base schema, parsed once.
pub fn base_schema() -> &'static HashMap<i16, TypeDef> {
    static SCHEMA: OnceLock<HashMap<i16, TypeDef>> = OnceLock::new();
    SCHEMA.get_or_init(|| parse_base_schema(BASE_SCHEMA_TEXT))
}

/// Schema name from the text header (`KEY=SCH_13006`), used to check that a
/// transmit's schema matches the bundled one.
pub fn base_schema_name() -> &'static str {
    static NAME: OnceLock<String> = OnceLock::new();
    NAME.get_or_init(|| {
        let head = &BASE_SCHEMA_TEXT[..BASE_SCHEMA_TEXT.len().min(1024)];
        match head.find("KEY=SCH_") {
            Some(i) => {
                let rest = &head[i + "KEY=SCH_".len()..];
                let end = rest
                    .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                    .unwrap_or(rest.len());
                rest[..end].to_string()
            }
            None => "13006".to_string(),
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_schema_parses_to_expected_shape() {
        let s = base_schema();
        // psparser's parse_base_schema keeps exactly these many types.
        assert_eq!(s.len(), 104);
        assert_eq!(base_schema_name(), "13006");
        // TAG_VALUES (88) has a 't'-coded field, so the type is dropped.
        assert!(!s.contains_key(&88));

        let cyl = s.get(&52).unwrap();
        assert_eq!(cyl.node_name, "CONE");
        let plane = s.values().find(|t| t.node_name == "PLANE").unwrap();
        assert!(!plane.variable);
        let names: Vec<&str> = plane.fields.iter().map(|f| f.name.as_str()).collect();
        assert!(names.contains(&"pvec"));
        assert!(names.contains(&"normal"));

        let ws = s.get(&2).unwrap();
        assert_eq!(ws.node_name, "WORKSPACE");
        assert!(ws.variable);
        assert_eq!(ws.fields.len(), 1);
        assert_eq!(ws.fields[0].field_type, FieldType::Char);
    }
}
