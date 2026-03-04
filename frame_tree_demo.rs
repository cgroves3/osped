// examples/frame_tree_demo.rs
//
// Demonstrates the TransformTree coordinate frame tracker:
//   - Building a robot's frame tree (map → odom → base_link → camera → laser)
//   - Querying net transforms between arbitrary frames
//   - Time interpolation
//   - Dynamic re-parenting (picking up an object)
//   - Type-safe lookup via sguaba's RigidBodyTransform

use osped::transforms::TransformTree;

fn main() {
    osped::init_tracing();

    let mut tree = TransformTree::new(10.0); // 10-second history buffer

    // ── Build the robot's frame tree ─────────────────────────────────────
    // map → odom → base_link → camera_link → laser_link
    //                       └→ gripper_link

    // The robot is at (2, 1, 0) in the map, odom has drifted slightly
    tree.set_transform("map", "odom", 1000, [0.05, -0.02, 0.0], [0.0, 0.0, 0.0, 1.0]);
    tree.set_transform("odom", "base_link", 1000, [2.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

    // Camera is 0.3m forward and 0.5m up from base
    tree.set_transform("base_link", "camera_link", 1000, [0.3, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]);

    // Laser is on top of the camera
    tree.set_transform("camera_link", "laser_link", 1000, [0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0]);

    // Gripper is 0.5m forward from base
    tree.set_transform("base_link", "gripper_link", 1000, [0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

    // ── Query transforms ─────────────────────────────────────────────────

    let map_to_laser = tree.lookup_transform("map", "laser_link", 1000).unwrap();
    println!("map → laser_link: translation = ({:.3}, {:.3}, {:.3})",
        map_to_laser.translation.x,
        map_to_laser.translation.y,
        map_to_laser.translation.z,
    );

    // Sibling query: camera → gripper (goes up through base_link)
    let cam_to_grip = tree.lookup_transform("camera_link", "gripper_link", 1000).unwrap();
    println!("camera → gripper: translation = ({:.3}, {:.3}, {:.3})",
        cam_to_grip.translation.x,
        cam_to_grip.translation.y,
        cam_to_grip.translation.z,
    );

    // ── Time interpolation ───────────────────────────────────────────────

    // Robot moves from x=2 to x=4 between t=1000 and t=2000
    tree.set_transform("odom", "base_link", 2000, [4.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]);

    let mid = tree.lookup_transform("odom", "base_link", 1500).unwrap();
    println!("odom → base_link at t=1500 (interpolated): x = {:.3}", mid.translation.x);
    // Should be 3.0 (midpoint)

    // ── Dynamic re-parenting (pick up object) ────────────────────────────

    // An object "cup" is initially in the world
    tree.set_transform("map", "cup", 1000, [3.0, 2.0, 0.8], [0.0, 0.0, 0.0, 1.0]);
    println!("cup parent before grasp: {:?}", tree.parent_of("cup"));

    // Robot grasps the cup → re-parent to gripper
    tree.set_transform("gripper_link", "cup", 2000, [0.0, 0.0, 0.05], [0.0, 0.0, 0.0, 1.0]);
    println!("cup parent after grasp: {:?}", tree.parent_of("cup"));

    // Now querying map → cup will go through the robot's arm
    let map_to_cup = tree.lookup_transform("map", "cup", 2000).unwrap();
    println!("map → cup (via gripper): translation = ({:.3}, {:.3}, {:.3})",
        map_to_cup.translation.x,
        map_to_cup.translation.y,
        map_to_cup.translation.z,
    );

    // ── All frames ───────────────────────────────────────────────────────
    let mut frames = tree.frame_names();
    frames.sort();
    println!("all frames: {frames:?}");
}
