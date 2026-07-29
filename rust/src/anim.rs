//! Animated turntable GIFs.
//!
//! Two sweeps, played back to back: first a full turn about the vertical
//! axis, then a full tumble about the horizontal one. Each sweep ends on the
//! pose it started from, and the last frame of the second sweep is the first
//! frame of the first, so the whole thing loops with no visible seam. The
//! duplicate closing frame of each sweep is dropped for exactly that reason --
//! leaving it in makes the animation hitch once per revolution.
//!
//! Frames are quantised independently. A shared palette would be tidier, but
//! the shading ramp shifts as the part turns and a fixed palette bands badly
//! on the large flat faces these renders are mostly made of.

use std::path::Path;

use crate::iso::Image;

/// How a turntable is swept.
pub struct SpinOptions {
    /// Frames in the azimuth (vertical-axis) sweep.
    pub az_frames: usize,
    /// Frames in the elevation (tumble) sweep.
    pub el_frames: usize,
    /// Hundredths of a second per frame, which is the unit GIF stores.
    pub delay_cs: u16,
}

impl Default for SpinOptions {
    fn default() -> Self {
        SpinOptions {
            az_frames: 48,
            el_frames: 48,
            delay_cs: 5,
        }
    }
}

/// The camera angles of a full turntable, in order.
///
/// `(az, el)` pairs starting from the given pose. Returns
/// `az_frames + el_frames` of them; the closing duplicate of each sweep is
/// omitted so playback loops seamlessly.
pub fn spin_angles(az0: f64, el0: f64, opts: &SpinOptions) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(opts.az_frames + opts.el_frames);
    for i in 0..opts.az_frames {
        let t = i as f64 / opts.az_frames as f64;
        out.push((az0 + 360.0 * t, el0));
    }
    for i in 0..opts.el_frames {
        let t = i as f64 / opts.el_frames as f64;
        out.push((az0, el0 + 360.0 * t));
    }
    out
}

/// Write frames as a looping GIF.
///
/// Every frame must be the same size; the first one sets the canvas.
pub fn write_gif(path: &Path, frames: &[Image], delay_cs: u16) -> std::io::Result<()> {
    let Some(first) = frames.first() else {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "no frames to write",
        ));
    };
    let (w, h) = (first.w as u16, first.h as u16);
    let file = std::fs::File::create(path)?;
    let mut out = std::io::BufWriter::new(file);
    let mut enc =
        gif::Encoder::new(&mut out, w, h, &[]).map_err(|e| std::io::Error::other(e.to_string()))?;
    enc.set_repeat(gif::Repeat::Infinite)
        .map_err(|e| std::io::Error::other(e.to_string()))?;
    for f in frames {
        // GIF has no alpha channel, only one fully transparent palette entry,
        // so anything partly transparent has to be composited first. These
        // renders sit on the viewer's dark background, so that is what they
        // are flattened onto.
        let mut rgba = f.px.clone();
        for px in rgba.chunks_exact_mut(4) {
            let a = px[3] as u32;
            if a == 255 {
                continue;
            }
            for c in 0..3 {
                let bg = [26u32, 27, 38][c];
                px[c] = ((px[c] as u32 * a + bg * (255 - a)) / 255) as u8;
            }
            px[3] = 255;
        }
        let mut frame = gif::Frame::from_rgba_speed(w, h, &mut rgba, 10);
        frame.delay = delay_cs;
        enc.write_frame(&frame)
            .map_err(|e| std::io::Error::other(e.to_string()))?;
    }
    Ok(())
}
