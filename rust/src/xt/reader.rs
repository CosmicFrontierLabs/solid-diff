//! Big-endian primitive readers for Parasolid XT fields.
//! Port of `vendor/ps-parser/psparser/reader.py`. See `docs/FORMAT.md` §3.

use super::XtError;

/// i16 null sentinel: `80 04` (-32764).
pub const I16_NULL: i16 = -32764;
/// f64 null sentinel: `C2 BC 92 8F 99 6E 00 00` (-3.14158e13).
pub const F64_NULL: [u8; 8] = [0xC2, 0xBC, 0x92, 0x8F, 0x99, 0x6E, 0x00, 0x00];
/// Pointer null: the literal id 1.
pub const PTR_NULL: i16 = 1;

pub struct Reader<'a> {
    pub data: &'a [u8],
    pub pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Reader { data, pos: 0 }
    }

    pub fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }

    pub fn eof(&self) -> bool {
        self.pos >= self.data.len()
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], XtError> {
        let end = self
            .pos
            .checked_add(n)
            .ok_or_else(|| XtError("overflow".into()))?;
        if end > self.data.len() {
            return Err(XtError(format!(
                "unexpected end of transmit: want {n} bytes at offset {}, {} left",
                self.pos,
                self.remaining()
            )));
        }
        let out = &self.data[self.pos..end];
        self.pos = end;
        Ok(out)
    }

    pub fn u8(&mut self) -> Result<u8, XtError> {
        Ok(self.take(1)?[0])
    }

    /// Raw i16, sentinel not interpreted (used for node ids and pointers).
    pub fn i16_raw(&mut self) -> Result<i16, XtError> {
        let b = self.take(2)?;
        Ok(i16::from_be_bytes([b[0], b[1]]))
    }

    /// i16 field value; `None` for the null sentinel.
    pub fn i16(&mut self) -> Result<Option<i16>, XtError> {
        let v = self.i16_raw()?;
        Ok(if v == I16_NULL { None } else { Some(v) })
    }

    pub fn i32(&mut self) -> Result<i32, XtError> {
        let b = self.take(4)?;
        Ok(i32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// f64 field value; `None` for the null sentinel.
    pub fn f64(&mut self) -> Result<Option<f64>, XtError> {
        let b = self.take(8)?;
        if b == F64_NULL {
            return Ok(None);
        }
        let mut a = [0u8; 8];
        a.copy_from_slice(b);
        Ok(Some(f64::from_be_bytes(a)))
    }

    /// f64 with the null sentinel folded to NaN, for use inside composites
    /// (vector/interval/box) which have no per-component null representation.
    fn f64_nan(&mut self) -> Result<f64, XtError> {
        Ok(self.f64()?.unwrap_or(f64::NAN))
    }

    /// ASCII string of `n` bytes; strict, non-ASCII is an error.
    pub fn char(&mut self, n: usize) -> Result<String, XtError> {
        let b = self.take(n)?;
        if b.iter().any(|c| *c >= 0x80) {
            return Err(XtError("non-ASCII byte in char field".into()));
        }
        Ok(b.iter().map(|c| *c as char).collect())
    }

    pub fn bool8(&mut self) -> Result<bool, XtError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            v => Err(XtError(format!("invalid boolean value: {v}"))),
        }
    }

    /// Pointer token: `None` for the XT null pointer (id 1).
    pub fn pointer(&mut self) -> Result<Option<i16>, XtError> {
        let v = self.i16_raw()?;
        Ok(if v == PTR_NULL { None } else { Some(v) })
    }

    /// `n` UTF-16 big-endian code units.
    pub fn utf16_be(&mut self, n: usize) -> Result<String, XtError> {
        let b = self.take(n * 2)?;
        let units: Vec<u16> = b
            .chunks_exact(2)
            .map(|c| u16::from_be_bytes([c[0], c[1]]))
            .collect();
        String::from_utf16(&units).map_err(|e| XtError(format!("bad utf-16: {e}")))
    }

    pub fn interval(&mut self) -> Result<[f64; 2], XtError> {
        Ok([self.f64_nan()?, self.f64_nan()?])
    }

    pub fn vector(&mut self) -> Result<[f64; 3], XtError> {
        Ok([self.f64_nan()?, self.f64_nan()?, self.f64_nan()?])
    }

    pub fn box3(&mut self) -> Result<[[f64; 2]; 3], XtError> {
        Ok([self.interval()?, self.interval()?, self.interval()?])
    }

    pub fn str_u8_len(&mut self) -> Result<String, XtError> {
        let n = self.u8()? as usize;
        self.char(n)
    }

    pub fn str_i32_len(&mut self) -> Result<String, XtError> {
        let n = self.i32()?;
        if n < 0 {
            return Err(XtError(format!("negative string length {n}")));
        }
        self.char(n as usize)
    }
}
