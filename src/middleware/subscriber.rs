// src/middleware/subscriber.rs

use std::fmt::Debug;
use iceoryx2::prelude::*;
use super::IpcError;

pub struct Subscriber<T: Debug + Default + Copy + Sized + Send + ZeroCopySend + 'static> {
    inner: iceoryx2::port::subscriber::Subscriber<ipc::Service, T, ()>,
    topic: String,
}

impl<T: Debug + Default + Copy + Sized + Send + ZeroCopySend + 'static> Subscriber<T> {
    pub(crate) fn new(
        node: &Node<ipc::Service>, node_name: &str, topic: &str,
    ) -> Result<Self, IpcError> {
        let service_name = ServiceName::new(topic).map_err(|e| IpcError::ServiceOpen {
            name: topic.into(), detail: format!("{e:?}"),
        })?;
        let service = node.service_builder(&service_name).publish_subscribe::<T>()
            .open_or_create().map_err(|e| IpcError::ServiceOpen {
                name: topic.into(), detail: format!("{e:?}"),
            })?;
        let subscriber = service.subscriber_builder().create()
            .map_err(|e| IpcError::SubscriberCreate {
                name: topic.into(), detail: format!("{e:?}"),
            })?;
        tracing::debug!(node = %node_name, topic = %topic, msg_type = std::any::type_name::<T>(), "subscriber created");
        Ok(Self { inner: subscriber, topic: topic.into() })
    }

    pub fn try_recv(&self) -> Result<Option<T>, IpcError> {
        match self.inner.receive() {
            Ok(Some(sample)) => {
                let value = *sample.payload();
                tracing::trace!(topic = %self.topic, "received sample");
                Ok(Some(value))
            }
            Ok(None) => Ok(None),
            Err(e) => Err(IpcError::ReceiveFailed {
                name: self.topic.clone(), detail: format!("{e:?}"),
            }),
        }
    }

    pub fn drain(&self) -> Result<Vec<T>, IpcError> {
        let mut out = Vec::new();
        loop {
            match self.try_recv()? {
                Some(v) => out.push(v),
                None => break,
            }
        }
        Ok(out)
    }

    pub fn recv_timeout(&self, timeout: std::time::Duration) -> Result<Option<T>, IpcError> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            if let Some(v) = self.try_recv()? { return Ok(Some(v)); }
            if std::time::Instant::now() >= deadline { return Ok(None); }
            std::thread::sleep(std::time::Duration::from_micros(100));
        }
    }

    pub fn topic(&self) -> &str { &self.topic }
}
