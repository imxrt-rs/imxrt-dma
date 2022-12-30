//! DMA interrupt support

use crate::{channel::Channel, Error};
use core::{
    cell::RefCell,
    future::Future,
    marker::PhantomPinned,
    pin::Pin,
    sync::atomic,
    task::{Context, Poll, Waker},
};

use cortex_m::interrupt::{self, Mutex};

impl<const CHANNELS: usize> super::Dma<CHANNELS> {
    /// Handle a DMA interrupt
    ///
    /// Checks the interrupt status for the channel identified by `channel`.
    /// If the channel completed its transfer, `on_interrupt` wakes the channel's
    /// waker.
    ///
    /// Consider calling `on_interrupt` in a DMA channel's interrupt handler:
    ///
    /// ```
    /// use imxrt_dma::Dma;
    /// static DMA: Dma<32> = // Handle to DMA driver.
    /// # unsafe { Dma::new(core::ptr::null(), core::ptr::null()) };
    ///
    /// // #[cortex_m_rt::interrupt]
    /// fn DMA7_DMA23() {
    ///     // Safety: only checking channels 7 and 23, which
    ///     // are both valid on an i.MX RT 1060 chip.
    ///     unsafe {
    ///         DMA.on_interrupt(7);
    ///         DMA.on_interrupt(23);
    ///     }
    /// }
    /// ```
    ///
    /// # Safety
    ///
    /// This should only be used when the associated DMA channel is exclusively referenced
    /// by a DMA transfer future. Caller must ensure that `on_interrupt` is called in
    /// the correct interrupt handler.
    ///
    /// # Panics
    ///
    /// Panics if `channel` is greater than or equal to the maximum number of channels.
    #[inline(always)]
    pub unsafe fn on_interrupt(&'static self, channel: usize) {
        let channel = self.channel(channel);
        if channel.is_interrupt() {
            channel.clear_interrupt();
        }

        if channel.is_complete() | channel.is_error() {
            interrupt::free(|cs| {
                let waker = self.wakers[channel.channel()].borrow(cs);
                let mut waker = waker.borrow_mut();
                if let Some(waker) = waker.take() {
                    waker.wake();
                }
            });
        }
    }
}

pub(crate) type SharedWaker = Mutex<RefCell<Option<Waker>>>;
#[allow(clippy::declare_interior_mutable_const)] // Very convenient, and usage for static init deemed OK in clippy docs
pub(crate) const NO_WAKER: SharedWaker = Mutex::new(RefCell::new(None));

/// The core DMA transfer future
///
/// `Transfer` is a future that drives the DMA transfer. `Transfer` will
/// initiate a DMA transfer when it is first polled. You may then poll it
/// to understand when the transfer completes.
///
/// To cancel a transfer, drop the `Transfer`.
///
/// If you've enabled DMA interrupts, consider using [`on_interrupt`](crate::Dma::on_interrupt)
/// to wake an executor when the DMA transfer completes, The interrupt interface assumes that you've
///
/// - configured your channel to generate interrupts
/// - registered a DMA ISR with your embedded runtime
/// - unmasked the DMA interrupt in the NVIC
///
/// `Transfer` calls the unsafe [`enable`](crate::channel::Channel::enable) method to enable a
/// DMA transfer. To properly use `Transfer`, make sure that you've configured your DMA
/// channel for a valid transfer.
///
/// `Transfer` is the core DMA future used in `imxrt_dma`. For safe DMA transfers, consider
/// using
///
/// - [`Memcpy`](crate::memcpy::Memcpy) for buffer-to-buffer DMA transfers
/// - [`Read`](crate::peripheral::Read) for peripheral-to-memory DMA transfers
/// - [`Write`](crate::peripheral::Write) for memory-to-peripheral DMA transfers
///
/// `Transfer` is designed to the DMA `Channel` public interface. If you need to implement
/// your own transfer future, you may do so.
///
/// ```no_run
/// use imxrt_dma::{channel::Channel, Transfer};
///
/// # static DMA: imxrt_dma::Dma<32> = unsafe { imxrt_dma::Dma::new(core::ptr::null(), core::ptr::null()) };
/// # async fn f() -> imxrt_dma::Result<()> {
/// let my_channel: Channel = // Acquire your channel...
///     # unsafe { DMA.channel(0) };
/// // Properly prepare your transfer...
/// // Safety: transfer properly prepared
/// unsafe { Transfer::new(&my_channel) }.await?;
/// # Ok(()) }
/// ```
pub struct Transfer<'a> {
    channel: &'a Channel,
    _pinned: PhantomPinned,
}

impl<'a> Transfer<'a> {
    /// Create a new `Transfer` that performs the DMA transfer described by `channel`
    ///
    /// # Safety
    ///
    /// Assumes that the transfer is correctly defined in the DMA channel memory.
    /// The transfer enables after the first call to `poll()`.
    pub unsafe fn new(channel: &'a Channel) -> Self {
        Transfer {
            channel,
            _pinned: PhantomPinned,
        }
    }
}

impl Future for Transfer<'_> {
    type Output = Result<(), Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        interrupt::free(|cs| {
            let waker = self.channel.waker.borrow(cs);
            let mut waker = waker.borrow_mut();
            *waker = Some(cx.waker().clone());
        });

        loop {
            // This driver is only expecting to catch synchronous errors
            // (those that manifest once we enable the transfer). If there
            // is a misconfiguration that only the hardware detects, we expect
            // to see it as soon as we loop back around after the enable.
            if self.channel.is_error() {
                let es = self.channel.error_status();
                self.channel.clear_error();
                return Poll::Ready(Err(es));
            } else if self.channel.is_complete() {
                self.channel.clear_complete();
                return Poll::Ready(Ok(()));
            } else if self.channel.is_enabled() {
                return Poll::Pending;
            } else {
                atomic::fence(atomic::Ordering::SeqCst);
                unsafe { self.channel.enable() };
            }
        }
    }
}

impl Drop for Transfer<'_> {
    fn drop(&mut self) {
        self.channel.disable();
        self.channel.clear_complete();
        self.channel.clear_error();
        interrupt::free(|cs| {
            let waker = self.channel.waker.borrow(cs);
            let mut waker = waker.borrow_mut();
            *waker = None;
        });
    }
}
