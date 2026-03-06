// src/messages/mod.rs
//
// All message types must be `#[repr(C)]` and implement `Copy` for iceoryx2
// zero-copy shared-memory transport. No heap allocations allowed in these types.

use iceoryx2::prelude::ZeroCopySend;

/// Fixed-size string for use in shared memory messages.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
pub struct FixedString<const N: usize> {
    buf: [u8; N],
    len: u16,
}

impl<const N: usize> FixedString<N> {
    pub const fn empty() -> Self {
        Self { buf: [0u8; N], len: 0 }
    }

    pub fn from_str(s: &str) -> Self {
        let bytes = s.as_bytes();
        let copy_len = bytes.len().min(N);
        let mut buf = [0u8; N];
        buf[..copy_len].copy_from_slice(&bytes[..copy_len]);
        Self { buf, len: copy_len as u16 }
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len as usize]).unwrap_or("<invalid>")
    }
}

impl<const N: usize> Default for FixedString<N> {
    fn default() -> Self { Self::empty() }
}

/// Common header — every message carries its coordinate frame and timestamp.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct Header {
    pub timestamp_ns: u64,
    pub seq: u64,
    pub frame_id: FixedString<64>,
}

// ── Sensor messages ──────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct Imu {
    pub header: Header,
    pub linear_acceleration: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub magnetic_field: [f64; 3],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, ZeroCopySend)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub num_ranges: u32,
    pub ranges: [f32; Self::MAX_POINTS],
}

impl LaserScan {
    pub const MAX_POINTS: usize = 1024;
}

impl Default for LaserScan {
    fn default() -> Self {
        Self {
            header: Header::default(), angle_min: 0.0, angle_max: 0.0,
            angle_increment: 0.0, range_min: 0.0, range_max: 0.0,
            num_ranges: 0, ranges: [0.0; Self::MAX_POINTS],
        }
    }
}

// ── Geometry / transform messages ────────────────────────────────────────────

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
}

/// Rigid-body transform between two coordinate frames (analogous to tf's `TransformStamped`).
/// This is the IPC wire format; see `transforms::TransformTree` for the runtime tree.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: FixedString<64>,
    pub translation: Vector3,
    pub rotation: Quaternion,
}

// ── Actuation messages ───────────────────────────────────────────────────────

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct Twist {
    pub header: Header,
    pub linear: Vector3,
    pub angular: Vector3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct JointCommand {
    pub header: Header,
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, ZeroCopySend)]
pub struct JointState {
    pub header: Header,
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub temperature_c: f32,
    pub error_flags: u32,
}

// ── Unit tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_string_round_trip() {
        let s = FixedString::<64>::from_str("base_link");
        assert_eq!(s.as_str(), "base_link");
    }

    #[test]
    fn fixed_string_truncates_on_overflow() {
        let s = FixedString::<8>::from_str("0123456789");
        assert_eq!(s.as_str(), "01234567");
    }

    #[test]
    fn fixed_string_empty_and_default() {
        assert_eq!(FixedString::<32>::empty().as_str(), "");
        assert_eq!(FixedString::<32>::default().as_str(), "");
    }

    #[test]
    fn quaternion_default_is_identity() {
        let q = Quaternion::default();
        assert_eq!(q.w, 1.0);
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        assert!((norm - 1.0).abs() < 1e-12);
    }

    #[test]
    fn laser_scan_max_points() {
        assert_eq!(LaserScan::MAX_POINTS, 1024);
        let scan = LaserScan::default();
        assert_eq!(scan.num_ranges, 0);
    }

    #[test]
    fn vector3_default() {
        let v = Vector3::default();
        assert_eq!((v.x, v.y, v.z), (0.0, 0.0, 0.0));
    }

    #[test]
    fn message_sizes_stable() {
        assert_eq!(std::mem::size_of::<Vector3>(), 24);
        assert_eq!(std::mem::size_of::<Quaternion>(), 32);
    }

    #[test]
    fn all_types_are_copy_send() {
        fn assert_cs<T: Copy + Send>() {}
        assert_cs::<Header>();
        assert_cs::<Imu>();
        assert_cs::<LaserScan>();
        assert_cs::<TransformStamped>();
        assert_cs::<Twist>();
        assert_cs::<JointCommand>();
        assert_cs::<JointState>();
    }
}
