// src/lib.rs
//!
//! `osped` — a production-ready, single-machine robotics IPC and coordinate frame
//! tracking framework built on **iceoryx2** (zero-copy shared memory), **sguaba**
//! (type-safe rigid body transforms), and **tracing** (structured logging).

pub mod messages;
pub mod middleware;
pub mod nodes;
pub mod transforms;

/// Initialise the `tracing` subscriber.
///
/// - **`RUST_LOG`** controls verbosity (default: `info`).
/// - **`LOG_FORMAT=json`** switches to machine-readable JSON output.
pub fn init_tracing() {
    use tracing_subscriber::{fmt, EnvFilter};

    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new("info"));

    let use_json = std::env::var("LOG_FORMAT")
        .map(|v| v.eq_ignore_ascii_case("json"))
        .unwrap_or(false);

    if use_json {
        fmt()
            .json()
            .with_env_filter(filter)
            .with_target(true)
            .with_thread_ids(true)
            .with_file(true)
            .with_line_number(true)
            .init();
    } else {
        fmt()
            .with_env_filter(filter)
            .with_target(true)
            .with_thread_ids(true)
            .init();
    }

    tracing::info!("tracing initialised");
}
