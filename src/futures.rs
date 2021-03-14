//! DMA futures

use crate::{interrupt::Transfer, Channel, Element, ErrorStatus};

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

/// A memcpy operation
///
/// The future yields when the copy is complete.
pub struct Memcpy<'a> {
    transfer: Transfer<'a>,
    channel: &'a Channel,
}

/// Perform a DMA-powered `memcpy` between the `source` and `destination` buffers
///
/// Copies the minimum number of elements between the two buffers. `memcpy` modifies
/// the channel's state in an implementation-specific manner. You will need to reconfigure
/// the channel for the next transfer.
pub fn memcpy<'a, E: Element>(
    source: &'a [E],
    destination: &'a [E],
    channel: &'a mut Channel,
) -> Memcpy<'a> {
    channel.set_interrupt_on_completion(true);
    channel.set_disable_on_completion(true);

    channel.set_channel_configuration(super::ChannelConfiguration::Off);
    super::set_source_linear_buffer(channel, source);
    super::set_destination_linear_buffer(channel, destination);
    channel.set_minor_loop_bytes(source.len().min(destination.len()) as u32);
    channel.set_transfer_iterations(1);

    Memcpy {
        // Safety: transfer is properly prepared
        transfer: unsafe { Transfer::new(channel) },
        channel,
    }
}

impl<'a> Future for Memcpy<'a> {
    type Output = Result<(), ErrorStatus>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: data not moved
        let transfer = unsafe { self.as_mut().map_unchecked_mut(|this| &mut this.transfer) };
        let poll = transfer.poll(cx);
        if poll.is_pending() && !self.channel.is_active() {
            // Safety: memory properly prepared
            unsafe { self.channel.start() };
        }
        poll
    }
}

// Drop handled by Transfer impl
