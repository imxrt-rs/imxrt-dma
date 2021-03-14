//! DMA-powered memcpy

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
/// Copies the minimum number of elements between the two buffers. You're responsible
/// for enabling any interrupts, and calling [`on_interrupt`](crate::interrupt::on_interrupt)
/// if the interrupt fires.
pub fn memcpy<'a, E: Element>(
    source: &'a [E],
    destination: &'a mut [E],
    channel: &'a mut Channel,
) -> Memcpy<'a> {
    super::set_source_linear_buffer(channel, source);
    super::set_destination_linear_buffer(channel, destination);

    // Turn off any DMAMUX configuration.
    //
    // Alternatively, we could use an always-on transfer, which might not need an
    // explicit "start()" activation. This means we could express the transfer
    // as a series of major loops, each transferring sizeof(E) bytes in the minor
    // loop. TBD...
    channel.set_channel_configuration(super::ChannelConfiguration::Off);
    // Transfer all elements in a single major loop
    channel.set_minor_loop_bytes(
        core::mem::size_of::<E>().saturating_mul(source.len().min(destination.len())) as u32,
    );
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
