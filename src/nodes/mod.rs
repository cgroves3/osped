// src/nodes/mod.rs

use std::time::{Duration, Instant};

pub fn run_periodic(rate_hz: f64, mut body: impl FnMut(u64) -> bool) {
    let period = Duration::from_secs_f64(1.0 / rate_hz);
    let mut tick: u64 = 0;
    tracing::info!(rate_hz, "starting periodic loop");
    loop {
        let t0 = Instant::now();
        if !body(tick) {
            tracing::info!(tick, "periodic loop stopped");
            break;
        }
        let elapsed = t0.elapsed();
        if elapsed < period {
            std::thread::sleep(period - elapsed);
        } else {
            tracing::warn!(tick, elapsed_us = elapsed.as_micros() as u64, "loop overrun");
        }
        tick += 1;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lifecycle { Unconfigured, Inactive, Active, Finalized }

pub trait ManagedNode {
    fn on_configure(&mut self) -> Result<(), Box<dyn std::error::Error>>;
    fn on_activate(&mut self) -> Result<(), Box<dyn std::error::Error>>;
    fn on_tick(&mut self, tick: u64) -> bool;
    fn on_shutdown(&mut self);
}

pub fn drive_lifecycle(node: &mut dyn ManagedNode, rate_hz: f64) {
    tracing::info!("lifecycle: configuring");
    if let Err(e) = node.on_configure() {
        tracing::error!(error = %e, "configure failed");
        return;
    }
    tracing::info!("lifecycle: activating");
    if let Err(e) = node.on_activate() {
        tracing::error!(error = %e, "activate failed");
        node.on_shutdown();
        return;
    }
    tracing::info!("lifecycle: running");
    run_periodic(rate_hz, |tick| node.on_tick(tick));
    tracing::info!("lifecycle: shutting down");
    node.on_shutdown();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn periodic_stops_on_false() {
        let mut count = 0u64;
        run_periodic(10000.0, |tick| { count = tick + 1; tick < 9 });
        assert_eq!(count, 10);
    }

    #[test]
    fn periodic_tick_counter() {
        let mut ticks = Vec::new();
        run_periodic(10000.0, |tick| { ticks.push(tick); tick < 4 });
        assert_eq!(ticks, vec![0, 1, 2, 3, 4]);
    }

    struct MockNode { events: Vec<&'static str>, max_ticks: u64, fail_configure: bool, fail_activate: bool }
    impl MockNode {
        fn new(max_ticks: u64) -> Self { Self { events: vec![], max_ticks, fail_configure: false, fail_activate: false } }
    }
    impl ManagedNode for MockNode {
        fn on_configure(&mut self) -> Result<(), Box<dyn std::error::Error>> {
            self.events.push("configure");
            if self.fail_configure { Err("fail".into()) } else { Ok(()) }
        }
        fn on_activate(&mut self) -> Result<(), Box<dyn std::error::Error>> {
            self.events.push("activate");
            if self.fail_activate { Err("fail".into()) } else { Ok(()) }
        }
        fn on_tick(&mut self, tick: u64) -> bool { self.events.push("tick"); tick < self.max_ticks }
        fn on_shutdown(&mut self) { self.events.push("shutdown"); }
    }

    #[test]
    fn lifecycle_happy_path() {
        let mut n = MockNode::new(2);
        drive_lifecycle(&mut n, 10000.0);
        assert_eq!(n.events[0], "configure");
        assert_eq!(n.events[1], "activate");
        assert_eq!(n.events.iter().filter(|&&e| e == "tick").count(), 3);
        assert_eq!(*n.events.last().unwrap(), "shutdown");
    }

    #[test]
    fn lifecycle_configure_failure() {
        let mut n = MockNode::new(0);
        n.fail_configure = true;
        drive_lifecycle(&mut n, 1000.0);
        assert_eq!(n.events, vec!["configure"]);
    }

    #[test]
    fn lifecycle_activate_failure_still_shuts_down() {
        let mut n = MockNode::new(0);
        n.fail_activate = true;
        drive_lifecycle(&mut n, 1000.0);
        assert_eq!(n.events, vec!["configure", "activate", "shutdown"]);
    }
}
