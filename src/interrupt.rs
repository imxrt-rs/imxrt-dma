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

    if channel.is_complete() {
        interrupt::free(|cs| {
            let waker = WAKERS[channel.channel()].borrow(cs);
            let mut waker = waker.borrow_mut();
            if let Some(waker) = waker.take() {
                waker.wake();
            }
        });
    }
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
