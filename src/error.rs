//! DMA error status

use core::fmt::{self, Debug, Display};

/// A wrapper around a DMA error status value
///
/// The wrapper contains a copy of the DMA controller's
/// error status register at the point of an error. The
/// wrapper implements both `Debug` and `Display`. The
/// type may be printed to understand why there was a
/// DMA error.
#[derive(Clone, Copy)]
pub struct Error {
    /// The raw error status
    es: u32,
}

impl Error {
    pub(crate) const fn new(es: u32) -> Self {
        Error { es }
    }
    /// Returns the raw error status value
    pub const fn raw(self) -> u32 {
        self.es
    }
}

impl Debug for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DMA_ES({:#010X})", self.es)
    }
}

impl Display for Error {
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
