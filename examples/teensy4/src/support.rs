//! Support library (qualified as `support`) for all examples.

#![no_std]

pub mod logging;

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};
use imxrt_dma::Channel;
use teensy4_bsp::hal::ral;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("{}", info);
    teensy4_panic::sos()
}

/// Allocate all DMA channels
pub fn channels(_: ral::dma0::Instance, _: ral::dmamux::Instance) -> [Option<Channel>; 32] {
    const NO_CHANNEL: Option<Channel> = None;
    let mut channels: [Option<Channel>; 32] = [NO_CHANNEL; 32];

    for (idx, channel) in channels.iter_mut().enumerate() {
        // Safety: own the DMA instances, so we're OK to fabricate the channels.
        // It would be unsafe for the user to subsequently access the DMA instances.
        let mut chan = unsafe { Channel::new(idx) };
        chan.reset();
        *channel = Some(chan);
    }
    channels
}

/// Block until the future resolves...
pub fn block<F: Future>(mut fut: F) -> F::Output {
    static VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(core::ptr::null(), &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );

    let raw_waker = RawWaker::new(core::ptr::null(), &VTABLE);
    // Safety: RawWaker is inert.
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut context = Context::from_waker(&waker);
    // Safety: not unpinning the memory
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };

    loop {
        match fut.as_mut().poll(&mut context) {
            Poll::Pending => {}
            Poll::Ready(result) => return result,
        }
    }
}

/// Blocks on a `WFI` until a waker is invoked
///
/// `wfi` will ignore interrupts that are fired but are not
/// registered with a waker.
pub fn wfi<F: Future>(mut fut: F) -> F::Output {
    use core::sync::atomic::{AtomicBool, Ordering};
    static WOKEN: AtomicBool = AtomicBool::new(false);

    static VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(core::ptr::null(), &VTABLE),
        |_| WOKEN.store(true, Ordering::Relaxed),
        |_| WOKEN.store(true, Ordering::Relaxed),
        |_| {},
    );

    let raw_waker = RawWaker::new(core::ptr::null(), &VTABLE);
    // Safety: RawWaker uses a static atomic. Always valid, and race-free.
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut context = Context::from_waker(&waker);
    // Safety: not unpinning the memory
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };

    loop {
        match fut.as_mut().poll(&mut context) {
            Poll::Pending => loop {
                cortex_m::asm::wfi();
                if WOKEN.swap(false, Ordering::Relaxed) {
                    break;
                }
            },
            Poll::Ready(result) => return result,
        }
    }
}
