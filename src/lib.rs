//! Direct Memory Access (DMA) driver for i.MX RT processors
//!
//! `imxrt-dma` is a lower-level DMA driver for all i.MX RT processors
//! It provides an `unsafe` interface for allocating DMA channels, and for
//! scheduling DMA transactions. `imxrt-dma` also provides some traits and
//! abscractions that provides simple transfer mechanisms.
//!
//! This DMA driver may be re-exported from a HAL. If it is, you should consider
//! using the safer APIs provided by your HAL, instead of this lower-level API.
//!
//! # Example
//!
//! Use DMA channel 7 to perform a DMA-powered memory copy.
//!
//! ```no_run
//! use imxrt_dma::{Channel, Transfer};
//!
//! let mut channel = unsafe { Channel::new(7) };
//! channel.reset();
//!
//! let source: [u32; 32] = [5; 32];
//! let destination: [u32; 32] = [0; 32];
//!
//! let tx = Transfer::buffer_linear(source.as_ptr(), source.len());
//! let rx = Transfer::buffer_linear(destination.as_ptr(), destination.len());
//!
//! channel.set_always_on();
//! channel.set_disable_on_completion(true);
//!
//! unsafe {
//!     channel.set_source_transfer(&tx);
//!     channel.set_destination_transfer(&rx);
//! }
//!
//! channel.set_minor_loop_elements::<u32>(1);
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

#![no_std]

//
// Copy of the RAL register macros, which are used throughout this implementation
//
// We define the macros in the root, so we can keep them as library implementation
// details. Otherwise, they might leak from the crate and conflict with any user's
// RAL macros.
//

macro_rules! write_reg {
    ( $periph:path, $instance:expr, $reg:ident, $( $field:ident : $value:expr ),+ ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        #[allow(unused_imports)]
        (*$instance).$reg.write(
            $({ use $periph::{$reg::$field::{mask, offset, W::*, RW::*}}; ($value << offset) & mask }) | *
        );
    }};
    ( $periph:path, $instance:expr, $reg:ident, $value:expr ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        (*$instance).$reg.write($value);
    }};
}

macro_rules! modify_reg {
    ( $periph:path, $instance:expr, $reg:ident, $( $field:ident : $value:expr ),+ ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        #[allow(unused_imports)]
        (*$instance).$reg.write(
            ((*$instance).$reg.read() & !( $({ use $periph::{$reg::$field::mask}; mask }) | * ))
            | $({ use $periph::{$reg::$field::{mask, offset, W::*, RW::*}}; ($value << offset) & mask }) | *);
    }};
    ( $periph:path, $instance:expr, $reg:ident, $fn:expr ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        (*$instance).$reg.write($fn((*$instance).$reg.read()));
    }};
}

macro_rules! read_reg {
    ( $periph:path, $instance:expr, $reg:ident, $( $field:ident ),+ ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        let val = ((*$instance).$reg.read());
        ( $({
            #[allow(unused_imports)]
            use $periph::{$reg::$field::{mask, offset, R::*, RW::*}};
            (val & mask) >> offset
        }) , *)
    }};
    ( $periph:path, $instance:expr, $reg:ident, $field:ident $($cmp:tt)* ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        #[allow(unused_imports)]
        use $periph::{$reg::$field::{mask, offset, R::*, RW::*}};
        (((*$instance).$reg.read() & mask) >> offset) $($cmp)*
    }};
    ( $periph:path, $instance:expr, $reg:ident ) => {{
        #[allow(unused_imports)]
        use $periph::{*};
        ((*$instance).$reg.read())
    }};
}

mod channel;
mod chip;
mod element;
mod peripheral;
mod ral;

pub use channel::{Channel, CircularError, Transfer};
pub use chip::CHANNEL_COUNT;
pub use element::Element;
pub use peripheral::{Destination, Source};
pub use ral::tcd::BandwidthControl;

use core::fmt::{self, Debug};

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
