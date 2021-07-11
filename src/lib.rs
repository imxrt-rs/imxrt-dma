//! Direct Memory Access (DMA) driver for i.MX RT processors
//!
//! `imxrt-dma` provides
//!
//! - an unsafe API for defining and scheduling transfers with DMA `Channel`s
//!   and `Transfer`s
//! - safe DMA futures for memcpy, peripheral-to-memory, and memory-to-peripheral
//!   transfers
//!
//! This DMA driver may be re-exported from a HAL. If it is, you should consider
//! using the safer APIs provided by your HAL.
//!
//! # Portability
//!
//! This DMA driver works across all considered i.MX RT variants (1010 and 1060
//! family). You must make sure that the DMA channel you're creating is valid for
//! your i.MX RT processor. This only matters on i.MX RT 1010 processors, which
//! only support 16 DMA channels. Creating an invalid channel for your 1010 processor
//! will result in a channel that references reserved memory.
//!
//! ### License
//!
//! Licensed under either of
//!
//! - [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
//! - [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))
//!
//! at your option.
//!
//! Unless you explicitly state otherwise, any contribution intentionally submitted
//! for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
//! dual licensed as above, without any additional terms or conditions.

#![no_std]

mod channel;
mod element;
mod error;
mod interrupt;
pub mod memcpy;
pub mod peripheral;
mod ral;

pub use channel::{Channel, ChannelConfiguration};
pub use element::Element;
pub use error::Error;
pub use interrupt::{on_error, on_interrupt, Transfer};
pub use ral::tcd::BandwidthControl;

/// A DMA result
pub type Result<T> = core::result::Result<T, Error>;

/// Set a hardware peripheral as the source for a DMA transfer
///
/// `hardware_source` is expected to be a pointer to a peripheral register that
/// can provide DMA data. This function configures the DMA channel always read from
/// this register.
pub fn set_source_hardware<E: Element>(chan: &mut Channel, hardware_source: *const E) {
    chan.set_source_address(hardware_source);
    chan.set_source_offset(0);
    chan.set_source_attributes::<E>(0);
    chan.set_source_last_address_adjustment(0);
}

/// Set a hardware peripheral as the destination for a DMA transfer
///
/// `hardware_destination` is expected to point at a peripheral register that can
/// receive DMA data. This function configures the DMA channel to always write to
/// this register.
pub fn set_destination_hardware<E: Element>(chan: &mut Channel, hardware_destination: *const E) {
    chan.set_destination_address(hardware_destination);
    chan.set_destination_offset(0);
    chan.set_destination_attributes::<E>(0);
    chan.set_destination_last_address_adjustment(0);
}

/// Set a linear buffer as the source for a DMA transfer
///
/// When the transfer completes, the DMA channel will point at the
/// start of the buffer.
pub fn set_source_linear_buffer<E: Element>(chan: &mut Channel, source: &[E]) {
    chan.set_source_address(source.as_ptr());
    chan.set_source_offset(core::mem::size_of::<E>() as i16);
    chan.set_source_attributes::<E>(0);
    chan.set_source_last_address_adjustment(
        ((source.len() * core::mem::size_of::<E>()) as i32).wrapping_neg(),
    );
}

/// Set a linear buffer as the destination for a DMA transfer
///
/// When the transfer completes, the DMA channel will point at the
/// start of the buffer.
pub fn set_destination_linear_buffer<E: Element>(chan: &mut Channel, destination: &mut [E]) {
    chan.set_destination_address(destination.as_ptr());
    chan.set_destination_offset(core::mem::size_of::<E>() as i16);
    chan.set_destination_attributes::<E>(0);
    chan.set_destination_last_address_adjustment(
        ((destination.len() * core::mem::size_of::<E>()) as i32).wrapping_neg(),
    );
}

/// Assert properties about the circular buffer
fn circular_buffer_asserts<E>(buffer: &[E]) {
    let len = buffer.len();
    assert!(
        len.is_power_of_two(),
        "DMA circular buffer size is not power of two"
    );
    let start = buffer.as_ptr();
    let size = len * core::mem::size_of::<E>();
    assert!(
        (start as usize) % size == 0,
        "DMA circular buffer is not properly aligned"
    );
}

/// Compute the circular buffer modulo value
fn circular_buffer_modulo<E>(buffer: &[E]) -> u32 {
    31 - (buffer.len() * core::mem::size_of::<E>()).leading_zeros()
}

/// Set a circular buffer as the source for a DMA transfer
///
/// When the transfer completes, the DMA channel remain at the
/// next element in the circular buffer.
///
/// # Panics
///
/// Panics if
///
/// - the capacity is not a power of two
/// - the alignment is not a multiple of the buffer's size in bytes
pub fn set_source_circular_buffer<E: Element>(chan: &mut Channel, source: &[E]) {
    circular_buffer_asserts(source);
    let modulo = circular_buffer_modulo(source);

    chan.set_source_address(source.as_ptr());
    chan.set_source_offset(core::mem::size_of::<E>() as i16);
    chan.set_source_attributes::<E>(modulo as u8);
    chan.set_source_last_address_adjustment(0);
}

/// Set a circular buffer as the destination for a DMA transfer
///
/// When the transfer completes, the DMA channel remain at the
/// next element in the circular buffer.
///
/// # Panics
///
/// Panics if
///
/// - the capacity is not a power of two
/// - the alignment is not a multiple of the buffer's size in bytes
pub fn set_destination_circular_buffer<E: Element>(chan: &mut Channel, destination: &mut [E]) {
    circular_buffer_asserts(destination);
    let modulo = circular_buffer_modulo(destination);

    chan.set_destination_address(destination.as_ptr());
    chan.set_destination_offset(core::mem::size_of::<E>() as i16);
    chan.set_destination_attributes::<E>(modulo as u8);
    chan.set_destination_last_address_adjustment(0);
}

use core::{future::Future, pin::Pin, task::Poll};

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
    use core::task::{Context, RawWaker, RawWakerVTable, Waker};
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
