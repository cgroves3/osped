// src/middleware/publisher.rs

use std::fmt::Debug;
use iceoryx2::prelude::*;
use super::{IpcError, monotonic_now_ns};

pub struct Publisher<T: Debug + Default + Copy + Sized + Send + 'static> {
    inner: iceoryx2::port::publisher::Publisher<ipc::Service, T, ()>,
    topic: String,
    seq: u64,
}

impl<T: Debug + Default + Copy + Sized + Send + 'static> Publisher<T> {
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
        let publisher = service.publisher_builder().create()
            .map_err(|e| IpcError::PublisherCreate {
                name: topic.into(), detail: format!("{e:?}"),
            })?;
        tracing::debug!(node = %node_name, topic = %topic, msg_type = std::any::type_name::<T>(), "publisher created");
        Ok(Self { inner: publisher, topic: topic.into(), seq: 0 })
    }

    pub fn publish(&mut self, fill: impl FnOnce(&mut T)) -> Result<u64, IpcError> {
        let sample = self.inner.loan_uninit().map_err(|e| IpcError::LoanFailed {
            name: self.topic.clone(), detail: format!("{e:?}"),
        })?;
        let mut sample = sample.write_payload(T::default());
        let seq = self.seq;
        fill(sample.payload_mut());
        self.seq += 1;
        sample.send().map_err(|e| IpcError::PublishFailed {
            name: self.topic.clone(), detail: format!("{e:?}"),
        })?;
        tracing::trace!(topic = %self.topic, seq, "published");
        Ok(seq)
    }

    pub fn now_ns(&self) -> u64 { monotonic_now_ns() }
    pub fn topic(&self) -> &str { &self.topic }
}
