// tests/integration.rs
//
// End-to-end integration tests that exercise iceoryx2 shared-memory pub/sub.

use std::time::Duration;
use osped::messages::*;
use osped::middleware::{monotonic_now_ns, NodeHandle};

fn unique_topic(base: &str) -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static CTR: AtomicU64 = AtomicU64::new(0);
    format!("{base}_{}", CTR.fetch_add(1, Ordering::Relaxed))
}

#[test]
fn imu_round_trip() {
    let topic = unique_topic("test/imu");
    let node = NodeHandle::new("test_imu").expect("node");
    let mut publ = node.advertise::<Imu>(&topic).expect("advertise");
    let sub = node.subscribe::<Imu>(&topic).expect("subscribe");
    let now = monotonic_now_ns();
    publ.publish(|msg| {
        msg.header = Header { timestamp_ns: now, seq: 0, frame_id: FixedString::from_str("imu_link") };
        msg.linear_acceleration = [0.0, 0.0, 9.81];
    }).unwrap();
    let rx = sub.recv_timeout(Duration::from_secs(1)).unwrap().unwrap();
    assert_eq!(rx.header.frame_id.as_str(), "imu_link");
    assert!((rx.linear_acceleration[2] - 9.81).abs() < f64::EPSILON);
}

#[test]
fn twist_round_trip() {
    let topic = unique_topic("test/twist");
    let node = NodeHandle::new("test_twist").expect("node");
    let mut publ = node.advertise::<Twist>(&topic).expect("advertise");
    let sub = node.subscribe::<Twist>(&topic).expect("subscribe");
    publ.publish(|msg| { msg.linear = Vector3 { x: 1.0, y: 0.0, z: 0.0 }; }).unwrap();
    let rx = sub.recv_timeout(Duration::from_secs(1)).unwrap().unwrap();
    assert_eq!(rx.linear.x, 1.0);
}

#[test]
fn multi_message_drain() {
    let topic = unique_topic("test/drain");
    let node = NodeHandle::new("test_drain").expect("node");
    let mut publ = node.advertise::<JointCommand>(&topic).expect("advertise");
    let sub = node.subscribe::<JointCommand>(&topic).expect("subscribe");
    for i in 0..5u64 {
        publ.publish(|msg| { msg.position = i as f64 * 0.1; }).unwrap();
    }
    std::thread::sleep(Duration::from_millis(10));
    let all = sub.drain().unwrap();
    assert_eq!(all.len(), 5);
}

#[test]
fn try_recv_empty() {
    let topic = unique_topic("test/empty");
    let node = NodeHandle::new("test_empty").expect("node");
    let sub = node.subscribe::<Imu>(&topic).expect("subscribe");
    assert!(sub.try_recv().unwrap().is_none());
}

#[test]
fn publisher_seq_increments() {
    let topic = unique_topic("test/seq");
    let node = NodeHandle::new("test_seq").expect("node");
    let mut publ = node.advertise::<Imu>(&topic).expect("advertise");
    assert_eq!(publ.publish(|_| {}).unwrap(), 0);
    assert_eq!(publ.publish(|_| {}).unwrap(), 1);
    assert_eq!(publ.publish(|_| {}).unwrap(), 2);
}

#[test]
fn laser_scan_round_trip() {
    let topic = unique_topic("test/scan");
    let node = NodeHandle::new("test_scan").expect("node");
    let mut publ = node.advertise::<LaserScan>(&topic).expect("advertise");
    let sub = node.subscribe::<LaserScan>(&topic).expect("subscribe");
    publ.publish(|msg| {
        msg.num_ranges = 360;
        for i in 0..360 { msg.ranges[i] = 5.0 + 0.01 * i as f32; }
    }).unwrap();
    let rx = sub.recv_timeout(Duration::from_secs(1)).unwrap().unwrap();
    assert_eq!(rx.num_ranges, 360);
    assert!((rx.ranges[0] - 5.0).abs() < f32::EPSILON);
}
