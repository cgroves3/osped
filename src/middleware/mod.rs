// src/middleware/mod.rs

mod publisher;
mod subscriber;
mod time;

pub use publisher::Publisher;
pub use subscriber::Subscriber;
pub use time::monotonic_now_ns;

use iceoryx2::prelude::*;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum IpcError {
    #[error("failed to open/create iceoryx2 service '{name}': {detail}")]
    ServiceOpen { name: String, detail: String },
    #[error("failed to create publisher on '{name}': {detail}")]
    PublisherCreate { name: String, detail: String },
    #[error("failed to create subscriber on '{name}': {detail}")]
    SubscriberCreate { name: String, detail: String },
    #[error("publish failed on '{name}': {detail}")]
    PublishFailed { name: String, detail: String },
    #[error("loan failed on '{name}': {detail}")]
    LoanFailed { name: String, detail: String },
    #[error("receive failed on '{name}': {detail}")]
    ReceiveFailed { name: String, detail: String },
}

pub struct NodeHandle {
    pub(crate) node: Node<ipc::Service>,
    node_name: String,
}

impl NodeHandle {
    pub fn new(name: &str) -> Result<Self, IpcError> {
        let node_name = NodeName::new(name).map_err(|e| IpcError::ServiceOpen {
            name: name.into(), detail: format!("{e:?}"),
        })?;
        let node = NodeBuilder::new().name(&node_name).create::<ipc::Service>()
            .map_err(|e| IpcError::ServiceOpen {
                name: name.into(), detail: format!("{e:?}"),
            })?;
        tracing::info!(node = %name, "iceoryx2 node created");
        Ok(Self { node, node_name: name.into() })
    }

    pub fn advertise<T: std::fmt::Debug + Default + Copy + Sized + Send + ZeroCopySend + 'static>(
        &self, topic: &str,
    ) -> Result<Publisher<T>, IpcError> {
        Publisher::new(&self.node, &self.node_name, topic)
    }

    pub fn subscribe<T: std::fmt::Debug + Default + Copy + Sized + Send + ZeroCopySend + 'static>(
        &self, topic: &str,
    ) -> Result<Subscriber<T>, IpcError> {
        Subscriber::new(&self.node, &self.node_name, topic)
    }

    pub fn name(&self) -> &str { &self.node_name }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ipc_error_display() {
        let err = IpcError::ServiceOpen { name: "test".into(), detail: "oops".into() };
        assert!(format!("{err}").contains("test"));
    }

    #[test]
    fn ipc_error_is_std_error() {
        fn assert_std_error<T: std::error::Error>() {}
        assert_std_error::<IpcError>();
    }
}
