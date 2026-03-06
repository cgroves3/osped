// src/transforms/mod.rs
//
// A coordinate frame tree inspired by the tf paper (Foote, 2013), implemented
// using sguaba's `RigidBodyTransform` for the underlying spatial math.
//
// # Design rationale (tf paper → osped)
//
// The tf paper identifies these core requirements:
//   1. Transforms are edges in a tree (acyclic directed graph) with frames as nodes.
//   2. Each edge is time-stamped; a history buffer enables interpolation (SLERP).
//   3. The net transform between any two frames is the product of edges along the
//      spanning path through their common ancestor.
//   4. The tree must support asynchronous, out-of-order updates.
//   5. Frames can be dynamically added, removed, or re-parented.
//
// sguaba provides compile-time type-safe coordinate systems via its generic
// `RigidBodyTransform<From, To>` and `Coordinate<In>` types. However, a robot's
// frame tree is *dynamic* — frames are identified by runtime strings like
// "base_link", "camera_optical", "odom". We cannot generate a unique Rust type
// per frame at compile time.
//
// Our bridge: we define a single sguaba coordinate system type per "role" that
// the user cares about (e.g. `system!(struct BaseLink using XYZ)`), and the
// `TransformTree` operates on *runtime string names* internally using raw
// isometries (nalgebra `Isometry3<f64>`). When the user wants sguaba type
// safety for a *specific* lookup, they call `lookup_typed::<From, To>()` which
// returns a `sguaba::math::RigidBodyTransform<From, To>`.
//
// This gives the best of both worlds:
//   - Runtime flexibility for the tree (hundreds of frames, dynamic re-parenting).
//   - Compile-time safety at the point of *use* — you cannot accidentally apply
//     a `base_link → camera` transform to data that is in the `odom` frame.

use std::collections::HashMap;

use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use thiserror::Error;

use crate::messages::TransformStamped;

/// Errors from the transform tree.
#[derive(Debug, Error)]
pub enum TfError {
    #[error("frame '{0}' not found in the tree")]
    FrameNotFound(String),

    #[error("no path between '{from}' and '{to}' — they are in disconnected subtrees")]
    Disconnected { from: String, to: String },

    #[error("no transform data available for edge '{parent}' → '{child}' at time {time_ns}")]
    NoDataAtTime { parent: String, child: String, time_ns: u64 },

    #[error("buffer for edge '{parent}' → '{child}' is empty")]
    EmptyBuffer { parent: String, child: String },
}

/// A single time-stamped transform value stored in an edge's history buffer.
#[derive(Debug, Clone, Copy)]
struct StampedIsometry {
    timestamp_ns: u64,
    isometry: Isometry3<f64>,
}

/// The history buffer for one edge (parent → child) in the tree.
/// Kept sorted by timestamp for binary-search interpolation.
struct EdgeBuffer {
    parent: String,
    child: String,
    /// Chronologically sorted. Oldest first.
    history: Vec<StampedIsometry>,
    /// Maximum number of entries to retain.
    max_history: usize,
}

impl EdgeBuffer {
    fn new(parent: &str, child: &str, max_history: usize) -> Self {
        Self {
            parent: parent.into(),
            child: child.into(),
            history: Vec::with_capacity(max_history.min(256)),
            max_history,
        }
    }

    /// Insert a new stamped transform. Maintains sorted order (optimised for
    /// the common case where data arrives in chronological order → append).
    fn insert(&mut self, stamp: StampedIsometry) {
        // Fast path: append if newest
        if self.history.last().map_or(true, |last| stamp.timestamp_ns >= last.timestamp_ns) {
            self.history.push(stamp);
        } else {
            // Out-of-order: binary search for correct position
            let pos = self.history
                .binary_search_by_key(&stamp.timestamp_ns, |s| s.timestamp_ns)
                .unwrap_or_else(|i| i);
            self.history.insert(pos, stamp);
        }
        // Evict oldest if over capacity
        while self.history.len() > self.max_history {
            self.history.remove(0);
        }
    }

    /// Look up the transform at a specific time via linear interpolation.
    /// `time_ns == 0` is treated as "latest available" (matching tf convention).
    fn lookup(&self, time_ns: u64) -> Result<Isometry3<f64>, TfError> {
        if self.history.is_empty() {
            return Err(TfError::EmptyBuffer {
                parent: self.parent.clone(),
                child: self.child.clone(),
            });
        }

        // time_ns == 0 → return latest
        if time_ns == 0 {
            return Ok(self.history.last().unwrap().isometry);
        }

        // Exact match or interpolation
        match self.history.binary_search_by_key(&time_ns, |s| s.timestamp_ns) {
            Ok(idx) => Ok(self.history[idx].isometry),
            Err(idx) => {
                if idx == 0 {
                    // Before earliest sample — return earliest (no extrapolation)
                    Ok(self.history[0].isometry)
                } else if idx >= self.history.len() {
                    // After latest sample — return latest (no extrapolation)
                    Ok(self.history.last().unwrap().isometry)
                } else {
                    // Interpolate between [idx-1] and [idx]
                    let a = &self.history[idx - 1];
                    let b = &self.history[idx];
                    let t = if b.timestamp_ns == a.timestamp_ns {
                        0.0
                    } else {
                        (time_ns - a.timestamp_ns) as f64
                            / (b.timestamp_ns - a.timestamp_ns) as f64
                    };
                    Ok(a.isometry.lerp_slerp(&b.isometry, t))
                }
            }
        }
    }

    fn latest_timestamp(&self) -> Option<u64> {
        self.history.last().map(|s| s.timestamp_ns)
    }
}

/// Internal record for a frame (node in the tree).
struct FrameRecord {
    /// `None` for root frames.
    parent: Option<String>,
}

/// The transform tree: a runtime-dynamic tree of coordinate frames with
/// time-stamped transform histories on each edge.
///
/// This is the osped equivalent of tf's `Listener` — it receives transform
/// updates and responds to queries for the net transform between any two frames.
///
/// # Example
///
/// ```ignore
/// use osped::transforms::TransformTree;
///
/// let mut tree = TransformTree::new(5.0); // 5 second buffer
/// tree.set_transform("odom", "base_link", 1000, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
/// tree.set_transform("base_link", "laser", 1000, [0.1, 0.0, 0.2], [0.0, 0.0, 0.0, 1.0]);
///
/// // Net transform from odom to laser at time 1000:
/// let iso = tree.lookup_transform("odom", "laser", 1000).unwrap();
/// ```
pub struct TransformTree {
    /// Frame name → record
    frames: HashMap<String, FrameRecord>,
    /// Edge key "parent\0child" → buffer
    edges: HashMap<String, EdgeBuffer>,
    /// Max history entries per edge (derived from buffer_duration × assumed rate).
    max_history: usize,
    /// Buffer duration in seconds (for documentation / debugging).
    buffer_duration_secs: f64,
}

impl TransformTree {
    /// Create a new transform tree.
    ///
    /// `buffer_duration_secs` controls how much history each edge retains.
    /// Internally this is converted to a maximum entry count assuming a 100 Hz
    /// update rate (adjustable with `set_max_history`).
    pub fn new(buffer_duration_secs: f64) -> Self {
        let max_history = (buffer_duration_secs * 100.0).max(10.0) as usize;
        Self {
            frames: HashMap::new(),
            edges: HashMap::new(),
            max_history,
            buffer_duration_secs,
        }
    }

    /// Override the maximum number of history entries per edge.
    pub fn set_max_history(&mut self, n: usize) {
        self.max_history = n;
    }

    /// Insert or update a transform edge.
    ///
    /// If the child frame already exists with a different parent, it is
    /// re-parented (matching tf's dynamic re-parenting behaviour).
    pub fn set_transform(
        &mut self,
        parent: &str,
        child: &str,
        timestamp_ns: u64,
        translation: [f64; 3],
        rotation_xyzw: [f64; 4],
    ) {
        // Ensure both frames exist
        self.frames.entry(parent.into()).or_insert(FrameRecord { parent: None });
        let child_record = self.frames.entry(child.into()).or_insert(FrameRecord {
            parent: Some(parent.into()),
        });

        // Dynamic re-parenting
        if child_record.parent.as_deref() != Some(parent) {
            tracing::info!(
                child = %child,
                old_parent = ?child_record.parent,
                new_parent = %parent,
                "re-parenting frame"
            );
            child_record.parent = Some(parent.into());
        }

        // Build isometry
        let t = Translation3::new(translation[0], translation[1], translation[2]);
        let [qx, qy, qz, qw] = rotation_xyzw;
        let q = UnitQuaternion::from_quaternion(
            nalgebra::Quaternion::new(qw, qx, qy, qz),
        );
        let iso = Isometry3::from_parts(t, q);

        // Insert into edge buffer
        let edge_key = edge_key(parent, child);
        let buf = self.edges.entry(edge_key).or_insert_with(|| {
            EdgeBuffer::new(parent, child, self.max_history)
        });
        buf.insert(StampedIsometry { timestamp_ns, isometry: iso });

        tracing::trace!(parent = %parent, child = %child, t = timestamp_ns, "transform updated");
    }

    /// Convenience: insert a transform from our wire-format `TransformStamped` message.
    pub fn set_transform_stamped(&mut self, msg: &TransformStamped) {
        let parent = msg.header.frame_id.as_str();
        let child = msg.child_frame_id.as_str();
        let t = msg.translation;
        let r = msg.rotation;
        self.set_transform(parent, child, msg.header.timestamp_ns,
            [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]);
    }

    /// Look up the net transform from `source` to `target` at `time_ns`.
    ///
    /// This computes the spanning path through the common ancestor (exactly as
    /// described in Section IV-D of the tf paper). When traversing up from source
    /// to the ancestor, the *inverse* of each edge is used; when traversing down
    /// from the ancestor to the target, the edge value is used directly.
    ///
    /// `time_ns == 0` returns the latest available transform.
    pub fn lookup_transform(
        &self,
        source: &str,
        target: &str,
        time_ns: u64,
    ) -> Result<Isometry3<f64>, TfError> {
        if source == target {
            return Ok(Isometry3::identity());
        }

        // Find paths to root for both frames
        let source_chain = self.chain_to_root(source)?;
        let target_chain = self.chain_to_root(target)?;

        // Find the lowest common ancestor (LCA)
        let source_set: HashMap<&str, usize> = source_chain.iter()
            .enumerate()
            .map(|(i, s)| (s.as_str(), i))
            .collect();

        let mut lca: Option<&str> = None;
        let mut lca_target_idx = 0;
        for (ti, frame) in target_chain.iter().enumerate() {
            if source_set.contains_key(frame.as_str()) {
                lca = Some(frame.as_str());
                lca_target_idx = ti;
                break;
            }
        }

        let lca = lca.ok_or_else(|| TfError::Disconnected {
            from: source.into(),
            to: target.into(),
        })?;
        let lca_source_idx = source_set[lca];

        // Compose: source → LCA (using inverse edges going up)
        let mut result = Isometry3::identity();
        for i in 0..lca_source_idx {
            let child = &source_chain[i];
            let parent = &source_chain[i + 1];
            let edge = self.get_edge(parent, child, time_ns)?;
            // Going up: we have parent→child, need child→parent = inverse
            result = edge.inverse() * result;
        }

        // Compose: LCA → target (using direct edges going down)
        // target_chain is [target, ..., LCA, ...root], we need LCA down to target
        // which is target_chain[lca_target_idx] down to target_chain[0]
        for i in (0..lca_target_idx).rev() {
            let child = &target_chain[i];
            let parent = &target_chain[i + 1];
            let edge = self.get_edge(parent, child, time_ns)?;
            result = edge * result;
        }

        Ok(result)
    }

    /// Look up a transform and wrap it in a sguaba `RigidBodyTransform<From, To>`.
    ///
    /// This is the type-safe bridge: you specify the sguaba coordinate system
    /// types at the call site, and get back a transform that can only be applied
    /// to `Coordinate<From>` to produce `Coordinate<To>`.
    ///
    /// # Safety
    ///
    /// The caller asserts that `source_frame` truly corresponds to the sguaba
    /// coordinate system `From`, and `target_frame` corresponds to `To`.
    /// This mirrors sguaba's own `unsafe` convention for frame assertions.
    pub unsafe fn lookup_typed<From, To>(
        &self,
        source_frame: &str,
        target_frame: &str,
        time_ns: u64,
    ) -> Result<sguaba::math::RigidBodyTransform<From, To>, TfError>
    where
        From: sguaba::CoordinateSystem,
        To: sguaba::CoordinateSystem,
    {
        let iso = self.lookup_transform(source_frame, target_frame, time_ns)?;
        use uom::si::f64::Length;
        use uom::si::length::meter;
        #[allow(deprecated)]
        let translation = sguaba::Vector::<From>::from_cartesian(
            Length::new::<meter>(iso.translation.x),
            Length::new::<meter>(iso.translation.y),
            Length::new::<meter>(iso.translation.z),
        );
        let rotation = unsafe { sguaba::math::Rotation::<From, To>::from_quaternion(
            iso.rotation.quaternion().w,
            iso.rotation.quaternion().i,
            iso.rotation.quaternion().j,
            iso.rotation.quaternion().k,
        ) };
        Ok(unsafe { sguaba::math::RigidBodyTransform::new(translation, rotation) })
    }

    /// List all known frame names.
    pub fn frame_names(&self) -> Vec<&str> {
        self.frames.keys().map(|s| s.as_str()).collect()
    }

    /// Get the parent of a frame, if any.
    pub fn parent_of(&self, frame: &str) -> Option<&str> {
        self.frames.get(frame)?.parent.as_deref()
    }

    /// Get the latest timestamp for an edge.
    pub fn latest_timestamp(&self, parent: &str, child: &str) -> Option<u64> {
        self.edges.get(&edge_key(parent, child))?.latest_timestamp()
    }

    /// Return the number of history entries for an edge.
    pub fn edge_history_len(&self, parent: &str, child: &str) -> usize {
        self.edges.get(&edge_key(parent, child))
            .map_or(0, |b| b.history.len())
    }

    pub fn buffer_duration_secs(&self) -> f64 {
        self.buffer_duration_secs
    }

    // ── Internal helpers ─────────────────────────────────────────────────

    /// Walk from `frame` up to the root, returning the chain [frame, parent, grandparent, ..., root].
    fn chain_to_root(&self, frame: &str) -> Result<Vec<String>, TfError> {
        let mut chain = Vec::new();
        let mut current = frame.to_string();

        // Guard against cycles (shouldn't happen in a tree, but defensive)
        let max_depth = self.frames.len() + 1;
        for _ in 0..max_depth {
            chain.push(current.clone());
            match self.frames.get(&current) {
                Some(record) => match &record.parent {
                    Some(p) => current = p.clone(),
                    None => return Ok(chain), // reached a root
                },
                None => return Err(TfError::FrameNotFound(current)),
            }
        }
        // If we get here, there's a cycle — treat as disconnected
        Err(TfError::FrameNotFound(frame.into()))
    }

    /// Get the interpolated isometry for one edge at a given time.
    fn get_edge(&self, parent: &str, child: &str, time_ns: u64) -> Result<Isometry3<f64>, TfError> {
        let key = edge_key(parent, child);
        match self.edges.get(&key) {
            Some(buf) => buf.lookup(time_ns),
            None => Err(TfError::NoDataAtTime {
                parent: parent.into(),
                child: child.into(),
                time_ns,
            }),
        }
    }
}

/// Deterministic key for an edge. Uses null byte separator (cannot appear in frame names).
fn edge_key(parent: &str, child: &str) -> String {
    format!("{parent}\0{child}")
}

// ── Unit tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool { (a - b).abs() < eps }

    // ── Basic tree operations ────────────────────────────────────────────

    #[test]
    fn identity_when_source_equals_target() {
        let tree = TransformTree::new(5.0);
        let iso = tree.lookup_transform("any", "any", 0).unwrap();
        assert!(approx_eq(iso.translation.x, 0.0, 1e-12));
    }

    #[test]
    fn single_edge_lookup() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("world", "base", 100, [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);

        // world → base
        let iso = tree.lookup_transform("world", "base", 100).unwrap();
        assert!(approx_eq(iso.translation.x, 1.0, 1e-9));
        assert!(approx_eq(iso.translation.y, 2.0, 1e-9));
        assert!(approx_eq(iso.translation.z, 3.0, 1e-9));
    }

    #[test]
    fn inverse_lookup() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("world", "base", 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        // base → world should be the inverse: translation = [-1, 0, 0]
        let iso = tree.lookup_transform("base", "world", 100).unwrap();
        assert!(approx_eq(iso.translation.x, -1.0, 1e-9));
    }

    #[test]
    fn chained_transforms() {
        let mut tree = TransformTree::new(5.0);
        // world → base: translate +1 on X
        tree.set_transform("world", "base", 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        // base → sensor: translate +0.5 on X
        tree.set_transform("base", "sensor", 100, [0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        // world → sensor = 1.0 + 0.5 = 1.5 on X
        let iso = tree.lookup_transform("world", "sensor", 100).unwrap();
        assert!(approx_eq(iso.translation.x, 1.5, 1e-9));
    }

    #[test]
    fn sibling_frames_through_common_ancestor() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("base", "left_wheel", 100, [0.0, 0.5, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("base", "right_wheel", 100, [0.0, -0.5, 0.0], [0.0, 0.0, 0.0, 1.0]);

        // left_wheel → right_wheel: should be [0, -1, 0]
        let iso = tree.lookup_transform("left_wheel", "right_wheel", 100).unwrap();
        assert!(approx_eq(iso.translation.y, -1.0, 1e-9));
    }

    #[test]
    fn rotation_90deg_z() {
        let mut tree = TransformTree::new(5.0);
        // 90° rotation about Z axis: quat = (0, 0, sin(π/4), cos(π/4))
        let s = FRAC_PI_2.sin() / 2.0_f64.sqrt(); // sin(45°) ≈ 0.7071
        let c = FRAC_PI_2.cos() / 2.0_f64.sqrt();
        // Actually for 90° about Z: qz = sin(45°), qw = cos(45°)
        let qz = (FRAC_PI_2 / 2.0).sin();
        let qw = (FRAC_PI_2 / 2.0).cos();
        tree.set_transform("world", "rotated", 100, [0.0, 0.0, 0.0], [0.0, 0.0, qz, qw]);

        let iso = tree.lookup_transform("world", "rotated", 100).unwrap();
        // The rotation part should map X-axis to Y-axis
        let rotated_x = iso.rotation * nalgebra::Vector3::new(1.0, 0.0, 0.0);
        assert!(approx_eq(rotated_x.x, 0.0, 1e-9));
        assert!(approx_eq(rotated_x.y, 1.0, 1e-9));
    }

    // ── Time interpolation ───────────────────────────────────────────────

    #[test]
    fn interpolation_midpoint() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("w", "b", 1000, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("w", "b", 2000, [10.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        // At t=1500, should be halfway: x=5.0
        let iso = tree.lookup_transform("w", "b", 1500).unwrap();
        assert!(approx_eq(iso.translation.x, 5.0, 1e-9));
    }

    #[test]
    fn time_zero_returns_latest() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("w", "b", 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("w", "b", 200, [2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        let iso = tree.lookup_transform("w", "b", 0).unwrap();
        assert!(approx_eq(iso.translation.x, 2.0, 1e-9));
    }

    // ── Error cases ──────────────────────────────────────────────────────

    #[test]
    fn unknown_frame_errors() {
        let tree = TransformTree::new(5.0);
        assert!(tree.lookup_transform("a", "b", 0).is_err());
    }

    #[test]
    fn disconnected_subtrees_error() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("root1", "a", 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("root2", "b", 100, [2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        let result = tree.lookup_transform("a", "b", 100);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), TfError::Disconnected { .. }));
    }

    // ── Dynamic re-parenting ─────────────────────────────────────────────

    #[test]
    fn reparenting_changes_tree_structure() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("world", "obj", 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(tree.parent_of("obj"), Some("world"));

        // Re-parent obj to gripper
        tree.set_transform("gripper", "obj", 200, [0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(tree.parent_of("obj"), Some("gripper"));
    }

    // ── Frame enumeration ────────────────────────────────────────────────

    #[test]
    fn frame_names_lists_all() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("a", "b", 100, [0.0; 3], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("b", "c", 100, [0.0; 3], [0.0, 0.0, 0.0, 1.0]);
        let mut names = tree.frame_names();
        names.sort();
        assert_eq!(names, vec!["a", "b", "c"]);
    }

    // ── Out-of-order insertion ───────────────────────────────────────────

    #[test]
    fn out_of_order_insertion_maintains_sorted_history() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("w", "b", 300, [3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("w", "b", 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("w", "b", 200, [2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

        // Interpolation at t=150 should give x=1.5
        let iso = tree.lookup_transform("w", "b", 150).unwrap();
        assert!(approx_eq(iso.translation.x, 1.5, 1e-9));
    }

    // ── Edge metadata ────────────────────────────────────────────────────

    #[test]
    fn latest_timestamp_and_history_len() {
        let mut tree = TransformTree::new(5.0);
        tree.set_transform("a", "b", 100, [0.0; 3], [0.0, 0.0, 0.0, 1.0]);
        tree.set_transform("a", "b", 200, [0.0; 3], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(tree.latest_timestamp("a", "b"), Some(200));
        assert_eq!(tree.edge_history_len("a", "b"), 2);
    }

    // ── set_transform_stamped ────────────────────────────────────────────

    #[test]
    fn set_transform_stamped_integration() {
        use crate::messages::*;
        let mut tree = TransformTree::new(5.0);

        let msg = TransformStamped {
            header: Header {
                timestamp_ns: 1000,
                seq: 0,
                frame_id: FixedString::from_str("odom"),
            },
            child_frame_id: FixedString::from_str("base_link"),
            translation: Vector3 { x: 5.0, y: 0.0, z: 0.0 },
            rotation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        };
        tree.set_transform_stamped(&msg);

        let iso = tree.lookup_transform("odom", "base_link", 1000).unwrap();
        assert!(approx_eq(iso.translation.x, 5.0, 1e-9));
    }

    // ── Deep chain ───────────────────────────────────────────────────────

    #[test]
    fn deep_chain_five_levels() {
        let mut tree = TransformTree::new(5.0);
        // map → odom → base → shoulder → elbow → gripper, each +1 on X
        let frames = ["map", "odom", "base", "shoulder", "elbow", "gripper"];
        for w in frames.windows(2) {
            tree.set_transform(w[0], w[1], 100, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        }

        let iso = tree.lookup_transform("map", "gripper", 100).unwrap();
        assert!(approx_eq(iso.translation.x, 5.0, 1e-9));

        // gripper → map should be -5
        let inv = tree.lookup_transform("gripper", "map", 100).unwrap();
        assert!(approx_eq(inv.translation.x, -5.0, 1e-9));
    }
}
