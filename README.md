# imxrt-dma

Direct Memory Access (DMA) driver for i.MX RT processors

`imxrt-dma` provides

- an unsafe API for defining and scheduling transfers with DMA `Channel`s
  and `Transfer`s
- safe DMA futures for memcpy, peripheral-to-memory, and memory-to-peripheral
  transfers

This DMA driver may be re-exported from a HAL. If it is, you should consider
using the safer APIs provided by your HAL.

## Portability

This DMA driver works across all considered i.MX RT variants (1010 and 1060
family). You must make sure that the DMA channel you're creating is valid for
your i.MX RT processor. This only matters on i.MX RT 1010 processors, which
only support 16 DMA channels. Creating an invalid channel for your 1010 processor
will result in a channel that references reserved memory.

#### License

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
- [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

License: MIT OR Apache-2.0
