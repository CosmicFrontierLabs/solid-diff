//! Painter's-algorithm SVG renderer. STUB — owned by the renderer task.

use std::collections::HashMap;

use crate::mesh::Mesh;
use crate::value::NodeId;

#[derive(Debug, Clone)]
pub struct RenderOptions {
    pub alpha: f64,
    pub elev: f64,
    pub azim: f64,
    /// Perspective field of view in degrees; `None` for orthographic.
    pub fov: Option<f64>,
    pub size: f64,
    pub title: Option<String>,
    pub order: Order,
    pub edges: bool,
    /// Per-face colour override, keyed by FACE node id (the diff hook).
    pub color_map: HashMap<NodeId, [f64; 3]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    Auto,
    Bsp,
    Depth,
}

impl Default for RenderOptions {
    fn default() -> Self {
        RenderOptions {
            alpha: 0.55,
            elev: 28.0,
            azim: -55.0,
            fov: None,
            size: 520.0,
            title: None,
            order: Order::Auto,
            edges: true,
            color_map: HashMap::new(),
        }
    }
}

/// Render one mesh to an SVG `<g>` fragment.
pub fn render_mesh_svg(_mesh: &Mesh, _opts: &RenderOptions) -> String {
    String::new()
}

/// Lay fragments out in a grid and wrap them in an `<svg>` document.
pub fn svg_document(_fragments: &[String], _cols: usize, _cell: f64) -> String {
    String::new()
}
