# osped — Zero-Copy Robotics IPC + Coordinate Frame Tracking

A production-ready, single-machine robotics framework built on **iceoryx2** (shared-memory zero-copy IPC), **sguaba** (type-safe rigid body transforms), and **tracing** (structured logging).

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Shared Memory (iceoryx2)               │
└────────┬──────────────┬──────────────┬──────────────────┘
         │              │              │
   ┌─────┴─────┐  ┌────┴────┐   ┌─────┴──────┐
   │  sensor    │  │  motor  │   │    tf       │
   │  publisher │  │  ctrl   │   │  monitor    │
   └───────────┘  └─────────┘   └────────────┘
                      │
              ┌───────┴────────┐
              │ TransformTree  │ ← coordinate frame tracking
              │ (sguaba + tf)  │
              └────────────────┘
```

## Transform Tree Design

The `TransformTree` module bridges two libraries that solve complementary problems:

**The tf paper** (Foote, 2013) describes a runtime tree of named coordinate frames with time-stamped edges, SLERP interpolation, spanning-path traversal, and dynamic re-parenting. It solves the *system-level* problem of tracking dozens of frames that update asynchronously from different sources.

**sguaba** (Helsing) provides compile-time type-safe rigid body transforms where coordinate systems are encoded in Rust generics (`RigidBodyTransform<From, To>`). It solves the *correctness* problem of never accidentally applying a transform to data in the wrong frame.

These libraries operate at different levels. sguaba handles individual transforms with perfect type safety but has no concept of a tree, time history, or dynamic frame names. tf has the tree and time semantics but uses untyped strings for everything.

### How osped bridges them

The `TransformTree` uses nalgebra's `Isometry3<f64>` (the same representation sguaba uses internally) for its runtime tree. Frame names are strings, edges carry time-stamped isometry histories, and spanning-path traversal computes net transforms between any two frames — exactly as the tf paper describes.

At the boundary where the user *consumes* the result, `lookup_typed::<From, To>()` wraps the raw isometry in a `sguaba::math::RigidBodyTransform<From, To>`, giving compile-time frame safety for downstream code. This call is `unsafe` following sguaba's convention — the caller asserts that the string frame name truly corresponds to the generic type parameter.

### Key tf concepts implemented

| tf Concept | osped Implementation |
|---|---|
| **Tree of frames** | `HashMap<String, FrameRecord>` with parent pointers |
| **Time-stamped edges** | `EdgeBuffer` with chronologically sorted `Vec<StampedIsometry>` |
| **SLERP interpolation** | `Isometry3::lerp_slerp()` between bracketing samples |
| **Spanning path / LCA** | `chain_to_root()` for both frames, find lowest common ancestor |
| **Inverse traversal** | Going *up* the tree uses `edge.inverse()`; going *down* uses the direct value |
| **Dynamic re-parenting** | New `set_transform()` with a different parent automatically re-parents |
| **Stamp (frame + time)** | Every message carries `Header { timestamp_ns, seq, frame_id }` |
| **time=0 → latest** | `lookup_transform(src, tgt, 0)` returns latest available |
| **Out-of-order data** | Binary-search insertion maintains sorted order |

### What sguaba adds

| sguaba Feature | How osped uses it |
|---|---|
| **`system!` macro** | Users define typed frames: `system!(struct BaseLink using XYZ)` |
| **`RigidBodyTransform<From, To>`** | Returned by `lookup_typed()` — cannot be applied to wrong frame |
| **`Coordinate<In>`** | Points in a specific frame — type mismatch is a compile error |
| **`Rotation<From, To>`** | Orientation component of the typed transform |
| **`and_then()` chaining** | Users can compose the typed result with other sguaba transforms |

## Project structure

```
osped/
├── Cargo.toml
├── src/
│   ├── lib.rs                  # Crate root, tracing init
│   ├── messages/mod.rs         # #[repr(C)] zero-copy message types
│   ├── middleware/             # iceoryx2 pub/sub wrappers
│   ├── nodes/mod.rs            # Periodic loops, lifecycle management
│   └── transforms/mod.rs      # TransformTree (tf + sguaba bridge)
├── examples/
│   ├── sensor_publisher.rs
│   ├── motor_controller.rs
│   ├── transform_monitor.rs
│   └── frame_tree_demo.rs     # Shows the full transform tree API
└── tests/
    └── integration.rs          # iceoryx2 round-trip tests
```

## Building and testing

```bash
cargo build
cargo test --lib           # Unit tests (messages, middleware, transforms, nodes)
cargo test --test integration  # iceoryx2 pub/sub round-trips
cargo test                 # All tests
```

## Running examples

```bash
cargo run --example frame_tree_demo    # Transform tree demo (no iceoryx2 needed)
cargo run --example sensor_publisher   # IMU publisher at 200 Hz
cargo run --example motor_controller   # Lifecycle node
```
