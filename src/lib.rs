//! Direct Memory Access (DMA) driver for i.MX RT processors
//!
//! `imxrt-dma` is a lower-level DMA driver for all i.MX RT processors.
//! It provides an `unsafe` interface for allocating DMA channels, and for
//! scheduling DMA transactions. `imxrt-dma` also provides some traits and
//! abstractions that help to coordinate DMA transfers.
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
//! # Example
//!
//! Use DMA channel 7 to perform a DMA-powered memory copy.
//!
//! ```no_run
//! use imxrt_dma::{Channel, Transfer, ChannelConfiguration};
//!
//! let mut channel = unsafe { Channel::new(7) };
//! channel.reset();
//!
//! let source: [u32; 32] = [5; 32];
//! let destination: [u32; 32] = [0; 32];
//!
//! let tx = unsafe { Transfer::buffer_linear(source.as_ptr(), source.len()) };
//! let rx = unsafe { Transfer::buffer_linear(destination.as_ptr(), destination.len()) };
//!
//! channel.set_channel_configuration(ChannelConfiguration::AlwaysOn);
//! channel.set_disable_on_completion(true);
//!
//! unsafe {
//!     channel.set_source_transfer(&tx);
//!     channel.set_destination_transfer(&rx);
//! }
//!
//! channel.set_minor_loop_bytes(core::mem::size_of::<u32>() as u32);
//! channel.set_transfer_iterations(source.len() as u16);
//!
//! unsafe {
//!     channel.enable();
//!     channel.start();
//! }
//!
//! if channel.is_error() {
//!     panic!("Transaction failed!");
//! }
//!
//! while !channel.is_complete() {}
//!
//! assert_eq!(destination, [5;32]);
//! ```
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
mod interrupt;
pub mod memcpy;
pub mod peripheral;
mod ral;

pub use channel::{Channel, ChannelConfiguration};
pub use element::Element;
pub use interrupt::{on_interrupt, Transfer};
pub use ral::tcd::BandwidthControl;

use core::fmt::{self, Debug, Display};

/// A wrapper around a DMA error status value
///
/// The wrapper contains a copy of the DMA controller's
/// error status register at the point of an error. The
/// wrapper implements both `Debug` and `Display`. The
/// type may be printed to understand why there was a
/// DMA error.
#[derive(Clone, Copy)]
pub struct ErrorStatus {
    /// The raw error status
    es: u32,
}

impl ErrorStatus {
    const fn new(es: u32) -> Self {
        ErrorStatus { es }
    }
    /// Returns the raw error status value
    pub const fn raw(self) -> u32 {
        self.es
    }
}

impl Debug for ErrorStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DMA_ES({:#010X})", self.es)
    }
}

impl Display for ErrorStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f,
             "DMA_ES: VLD {vld} ECX {ecx} GPE {gpe} CPE {cpe} ERRCHN {errchn} SAE {sae} SOE {soe} DAE {dae} DOE {doe} NCE {nce} SGE {sge} SBE {sbe} DBE {dbe}",
             vld = (self.es >> 31) & 0x1,
             ecx = (self.es >> 16) & 0x1,
             gpe = (self.es >> 15) & 0x1,
             cpe = (self.es >> 14) & 0x1,
             errchn = (self.es >> 8) & 0x1F,
             sae = (self.es >> 7) & 0x1,
             soe = (self.es >> 6) & 0x1,
             dae = (self.es >> 5) & 0x1,
             doe = (self.es >> 4) & 0x1,
             nce = (self.es >> 3) & 0x1,
             sge = (self.es >> 2) & 0x1,
             sbe = (self.es >> 1) & 0x1,
             dbe = self.es & 0x1
         )
    }
}

/// Set a hardware peripheral as the source for a DMA transfer
pub fn set_source_hardware<E: Element>(chan: &mut Channel, hardware_source: *const E) {
    chan.set_source_address(hardware_source);
    chan.set_source_offset(0);
    chan.set_source_attributes::<E>(0);
    chan.set_source_last_address_adjustment(0);
}

/// Set a hardware peripheral as the destination for a DMA transfer
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
