//! Decoded Parasolid XT field values and node records.

/// Node ids are the i16 pointer targets used throughout an XT transmit.
pub type NodeId = i16;

/// One decoded field value. Mirrors the XT primitive type codes; see
/// `docs/FORMAT.md` §3.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Null,
    U8(u8),
    Char(String),
    Bool(bool),
    I16(i16),
    I32(i32),
    F64(f64),
    Utf16(String),
    /// Pointer field: `None` when the XT null pointer (value 1) was stored.
    Ptr(Option<NodeId>),
    Vec3([f64; 3]),
    Interval([f64; 2]),
    Box3([[f64; 2]; 3]),
    Array(Vec<Value>),
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::F64(v) => Some(*v),
            Value::I16(v) => Some(*v as f64),
            Value::I32(v) => Some(*v as f64),
            Value::U8(v) => Some(*v as f64),
            _ => None,
        }
    }

    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Value::I16(v) => Some(*v as i64),
            Value::I32(v) => Some(*v as i64),
            Value::U8(v) => Some(*v as i64),
            Value::F64(v) => Some(*v as i64),
            Value::Bool(b) => Some(*b as i64),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            Value::U8(v) => Some(*v != 0),
            Value::I16(v) => Some(*v != 0),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Char(s) | Value::Utf16(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Pointer target, if this is a non-null pointer.
    pub fn as_ptr(&self) -> Option<NodeId> {
        match self {
            Value::Ptr(p) => *p,
            _ => None,
        }
    }

    pub fn as_vec3(&self) -> Option<[f64; 3]> {
        match self {
            Value::Vec3(v) => Some(*v),
            Value::Array(items) if items.len() == 3 => {
                let mut out = [0.0; 3];
                for (i, it) in items.iter().enumerate() {
                    out[i] = it.as_f64()?;
                }
                Some(out)
            }
            _ => None,
        }
    }

    /// Flatten to a list of f64 (handles `Array` of scalars or of vectors).
    pub fn as_f64_vec(&self) -> Option<Vec<f64>> {
        match self {
            Value::Array(items) => {
                let mut out = Vec::with_capacity(items.len() * 3);
                for it in items {
                    match it {
                        Value::Vec3(v) => out.extend_from_slice(v),
                        Value::Interval(v) => out.extend_from_slice(v),
                        other => out.push(other.as_f64()?),
                    }
                }
                Some(out)
            }
            Value::Vec3(v) => Some(v.to_vec()),
            Value::F64(v) => Some(vec![*v]),
            _ => None,
        }
    }

    pub fn as_i64_vec(&self) -> Option<Vec<i64>> {
        match self {
            Value::Array(items) => items.iter().map(|v| v.as_i64()).collect(),
            other => other.as_i64().map(|v| vec![v]),
        }
    }

    /// Pointer list: a pointer field that may hold one or several targets.
    pub fn as_ptr_vec(&self) -> Vec<NodeId> {
        match self {
            Value::Ptr(Some(p)) => vec![*p],
            Value::Ptr(None) => vec![],
            Value::Array(items) => items.iter().filter_map(|v| v.as_ptr()).collect(),
            _ => vec![],
        }
    }

    pub fn is_null(&self) -> bool {
        matches!(self, Value::Null | Value::Ptr(None))
    }
}

/// One decoded node record.
#[derive(Debug, Clone)]
pub struct Node {
    pub node_type: i16,
    pub name: String,
    pub id: NodeId,
    /// Repeat count for variable-length node types.
    pub count: Option<i32>,
    /// Fields in schema order. Nodes have few fields, so a Vec with linear
    /// lookup beats a map here.
    pub fields: Vec<(String, Value)>,
}

impl Node {
    pub fn field(&self, name: &str) -> Option<&Value> {
        self.fields
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v)
            .filter(|v| !matches!(v, Value::Null))
    }

    pub fn f64(&self, name: &str) -> Option<f64> {
        self.field(name)?.as_f64()
    }

    pub fn i64(&self, name: &str) -> Option<i64> {
        self.field(name)?.as_i64()
    }

    pub fn bool(&self, name: &str) -> bool {
        self.field(name).and_then(|v| v.as_bool()).unwrap_or(false)
    }

    pub fn str(&self, name: &str) -> Option<&str> {
        self.field(name)?.as_str()
    }

    pub fn vec3(&self, name: &str) -> Option<[f64; 3]> {
        self.field(name)?.as_vec3()
    }

    pub fn ptr(&self, name: &str) -> Option<NodeId> {
        self.field(name)?.as_ptr()
    }

    pub fn ptrs(&self, name: &str) -> Vec<NodeId> {
        self.field(name).map(|v| v.as_ptr_vec()).unwrap_or_default()
    }

    pub fn f64_vec(&self, name: &str) -> Option<Vec<f64>> {
        self.field(name)?.as_f64_vec()
    }

    pub fn i64_vec(&self, name: &str) -> Option<Vec<i64>> {
        self.field(name)?.as_i64_vec()
    }

    /// XT `sense` field: `true` for '+', `false` for '-'. Absent means '+'.
    pub fn sense_positive(&self) -> bool {
        self.str("sense").map(|s| !s.starts_with('-')).unwrap_or(true)
    }
}
