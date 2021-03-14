//! DMA-powered memcpy

use crate::{interrupt::Transfer, Channel, Element, Error};

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

/// A memcpy operation
///
/// `Memcpy` yields when it's moved the minimum amount of elements between two linear
/// buffers. Use the [`memcpy`](crate::memcpy::memcpy) function to define the transfer.
pub struct Memcpy<'a> {
    transfer: Transfer<'a>,
    channel: &'a Channel,
}

/// Perform a DMA-powered `memcpy` between the `source` and `destination` buffers
///
/// Copies the minimum number of elements between the two buffers. You're responsible
/// for enabling any interrupts, and calling [`on_interrupt`](crate::interrupt::on_interrupt)
/// if the interrupt fires. Otherwise, you may poll the transfer until completion.
///
/// # Example
///
/// Transfer 5 `u32`s between a source and destination buffer. The transfer completes when
/// the DMA channel 7 interrupt fires.
///
/// ```no_run
/// use imxrt_dma::{Channel, memcpy, on_interrupt};
///
/// // #[cortex_m_rt::interrupt]
/// fn DMA7() {
///     // Safety: DMA channel 7 valid
///     unsafe { on_interrupt(7) };
/// }
///
/// let mut channel_7: Channel = // DMA channel 7
///     # unsafe { Channel::new(7) };
/// channel_7.set_interrupt_on_completion(true);
/// // TODO unmask DMA7 interrupt!
///
/// let source = [4u32, 5, 6, 7, 8];
/// let mut destination = [0; 5];
///
/// let transfer = memcpy::memcpy(&source, &mut destination, &mut channel_7);
/// # mod executor { pub fn wfi(_: impl core::future::Future) {} }
/// executor::wfi(transfer);
/// ```
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
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: data not moved
        let transfer = unsafe { self.as_mut().map_unchecked_mut(|this| &mut this.transfer) };
        let poll = transfer.poll(cx);
        if poll.is_pending() && !self.channel.is_active() {
            self.channel.start();
        }
        poll
    }
}

// Drop handled by Transfer impl
