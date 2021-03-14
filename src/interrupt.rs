//! DMA interrupt support

use crate::{Channel, Error};
use core::{
    cell::RefCell,
    future::Future,
    marker::PhantomPinned,
    pin::Pin,
    sync::atomic,
    task::{Context, Poll, Waker},
};

use cortex_m::interrupt::{self, Mutex};

/// Handle a DMA interrupt
///
/// Checks the interrupt status for the channel identified by `channel`.
/// If the channel completed its transfer, or it's in an error state,
/// `on_interrupt` wakes the channel's waker.
///
/// Consider calling `on_interrupt` in a DMA channel's interrupt handler:
///
/// ```
/// use imxrt_dma::on_interrupt;
///
/// // #[cortex_m_rt::interrupt]
/// fn DMA7_DMA23() {
///     // Safety: only checking channels 7 and 23, which
///     // are both valid on an i.MX RT 1060 chip.
///     unsafe {
///         on_interrupt(7);
///         on_interrupt(23);
///     }
/// }
/// ```
///
/// # Safety
///
/// Caller must ensure that `on_interrupt` is called in the correct interrupt
/// handler. Caller must ensure that `channel` is valid for the given system,
/// and for the interrupt handler.
#[inline(always)]
pub unsafe fn on_interrupt(channel: usize) {
    let channel = Channel::new(channel);
    if channel.is_interrupt() {
        channel.clear_interrupt();
    }

    if channel.is_complete() | channel.is_error() {
        interrupt::free(|cs| {
            let waker = WAKERS[channel.channel()].borrow(cs);
            let mut waker = waker.borrow_mut();
            if let Some(waker) = waker.take() {
                waker.wake();
            }
        });
    }
}

/// Handle a DMA error on one or more channels
///
/// `on_error` will find all DMA channels below `max_channel` that have
/// an error. `on_error` then wakes the waker that channel's waker, if
/// it exists.
///
/// `max_channel` is the total number of channels, starting from channel 0,
/// that you want to check for errors. This should be 32, or 16 on a system
/// with half the normal DMA channels. However, you may specify fewer channels
/// if you only use a handful of channels.
///
/// Consider calling `on_error` in the DMA error interrupt:
///
/// ```
/// use imxrt_dma::on_error;
///
/// // #[cortex_m_rt::interrupt]
/// fn DMA_ERROR() {
///     // Safety: on an i.MX RT 1062 chip, there are
///     // 32 DMA channels. This usage is valid.
///     // This usage is NOT valid for an i.MX RT 1011
///     // chip, which only has 16 DMA channels.
///     unsafe { on_error(32) };
/// }
/// ```
///
/// # Safety
///
/// `max_channel` must be valid for your system. Specifying a value greater than
/// 16 on a system that only has 16 channels results in undefined behavior.
///
/// # Panics
///
/// Panics if `max_channel` is greater than 32.
#[inline(always)]
pub unsafe fn on_error(max_channel: usize) {
    interrupt::free(|cs| {
        (0..max_channel)
            .map(|chan| Channel::new(chan))
            .filter(|chan| chan.is_error())
            .flat_map(|channel| {
                let waker = WAKERS[channel.channel()].borrow(cs);
                let mut waker = waker.borrow_mut();
                waker.take()
            })
            .for_each(|waker| {
                waker.wake();
            })
    });
}

type SharedWaker = Mutex<RefCell<Option<Waker>>>;
const NO_WAKER: SharedWaker = Mutex::new(RefCell::new(None));
static WAKERS: [SharedWaker; 32] = [NO_WAKER; 32];

/// The root DMA transfer future
///
/// `Transfer` is woken by a call to `on_interrupt` once the transfer is complete.
pub struct Transfer<'a> {
    channel: &'a Channel,
    _pinned: PhantomPinned,
}

impl<'a> Transfer<'a> {
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

impl<'a> Future for Transfer<'a> {
    type Output = Result<(), Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        interrupt::free(|cs| {
            let waker = WAKERS[self.channel.channel()].borrow(cs);
            let mut waker = waker.borrow_mut();
            *waker = Some(cx.waker().clone());
        });

        loop {
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

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.channel.disable();
        self.channel.clear_complete();
        self.channel.clear_error();
        interrupt::free(|cs| {
            let waker = WAKERS[self.channel.channel()].borrow(cs);
            let mut waker = waker.borrow_mut();
            *waker = None;
        });
    }
}
