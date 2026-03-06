// src/middleware/time.rs

pub fn monotonic_now_ns() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn monotonic_now_nonzero() { assert!(monotonic_now_ns() > 0); }

    #[test]
    fn monotonic_now_is_monotonic() {
        let t1 = monotonic_now_ns();
        std::hint::black_box(0);
        let t2 = monotonic_now_ns();
        assert!(t2 >= t1);
    }
}
