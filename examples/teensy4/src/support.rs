//! Support library (qualified as `support`) for all examples.

#![no_std]

pub mod logging;

use bsp::hal::ral;
use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};
use imxrt_dma::channel::Channel;
use teensy4_bsp as bsp;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("{}", info);
    teensy4_panic::sos()
}

// Safety: imxrt-ral has correct addresses for DMA and DMAMUX peripherals.
pub static DMA: imxrt_dma::Dma<32> =
    unsafe { imxrt_dma::Dma::new(ral::dma0::DMA0.cast(), ral::dmamux::DMAMUX.cast()) };

/// Allocate all DMA channels
pub fn channels(_: ral::dma0::Instance, _: ral::dmamux::Instance) -> [Option<Channel>; 32] {
    const NO_CHANNEL: Option<Channel> = None;
    let mut channels: [Option<Channel>; 32] = [NO_CHANNEL; 32];

    for (idx, channel) in channels.iter_mut().enumerate() {
        // Safety: own the DMA instances, so we're OK to fabricate the channels.
        // It would be unsafe for the user to subsequently access the DMA instances.
        let mut chan = unsafe { DMA.channel(idx) };
        chan.reset();
        *channel = Some(chan);
    }
    channels
}

/// Return the Teensy 4.0 pins collection
pub fn pins(_: ral::iomuxc::Instance) -> bsp::t40::Pins {
    // Safety: we own the IOMUXC instance
    unsafe { bsp::t40::Pins::new() }
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

/// Poll a future with a dummy waker.
///
/// Use `poll_no_wake` when you want to drive a future to completion, but you
/// don't care about the future waking an executor. It may be used to initiate
/// a DMA transfer that will later be awaited with [`block`].
///
/// Do not use `poll_no_wake` if you want an executor to be woken when the DMA
/// transfer completes.
pub fn poll_no_wake<F>(future: Pin<&mut F>) -> Poll<F::Output>
where
    F: Future,
{
    const VTABLE: RawWakerVTable = RawWakerVTable::new(|_| RAW_WAKER, |_| {}, |_| {}, |_| {});

    const RAW_WAKER: RawWaker = RawWaker::new(core::ptr::null(), &VTABLE);
    // Safety: raw waker meets documented requirements.
    let waker = unsafe { Waker::from_raw(RAW_WAKER) };
    let mut context = Context::from_waker(&waker);
    future.poll(&mut context)
}

/// Block until the future returns a result.
///
/// `block` invokes [`poll_no_wake`] in a loop until the future
/// returns a result. Consider using `block` after starting a transfer
/// with `poll_no_wake`, and after doing other work.
pub fn block<F>(mut future: Pin<&mut F>) -> F::Output
where
    F: Future,
{
    loop {
        match poll_no_wake(future.as_mut()) {
            Poll::Ready(result) => return result,
            Poll::Pending => {}
        }
    }
}
