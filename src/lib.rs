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

pub mod channel;
mod element;
mod error;
mod interrupt;
pub mod memcpy;
pub mod peripheral;
mod ral;

pub use element::Element;
pub use error::Error;
pub use interrupt::Transfer;
pub use ral::tcd::BandwidthControl;

/// A DMA result
pub type Result<T> = core::result::Result<T, Error>;

/// A DMA peripheral.
///
/// This DMA driver manages the DMA controller and the multiplexer.
/// It's configured with pointers to both peripherals.
///
/// `Dma` allocates [`Channel`](channel::Channel)s. `Channel` provides
/// the interface for scheduling transfers.
pub struct Dma<const CHANNELS: usize> {
    controller: ral::Static<ral::dma::RegisterBlock>,
    multiplexer: ral::Static<ral::dmamux::RegisterBlock>,
    wakers: [SharedWaker; CHANNELS],
}

// Safety: OK to allocate a DMA driver in a static context.
unsafe impl<const CHANNELS: usize> Sync for Dma<CHANNELS> {}

impl<const CHANNELS: usize> Dma<CHANNELS> {
    /// Create the DMA driver.
    ///
    /// Note that this can evaluate at compile time. Consider using this to expose
    /// a `DMA` constant through your higher-level API that you can use to allocate
    /// DMA channels.
    ///
    /// `max_channels` specifies the total number of channels supported by the DMA
    /// controller. It's referenced when allocating channels.
    ///
    /// # Safety
    ///
    /// Caller must make sure that `controller` is a pointer to the start of the DMA
    /// controller register block. Caller must also make sure that `multiplexer` is
    /// a pointer to the start of the DMA multiplexer. Both pointers must be valid
    /// for your MCU.
    ///
    /// An incorrect `max_channels` value prevents proper bounds checking when allocating
    /// channels. This may result in DMA channels that point to invalid memory.
    pub const unsafe fn new(controller: *const (), multiplexer: *const ()) -> Self {
        Self {
            controller: ral::Static(controller.cast()),
            multiplexer: ral::Static(multiplexer.cast()),
            wakers: [NO_WAKER; CHANNELS],
        }
    }
}

use interrupt::{SharedWaker, NO_WAKER};
