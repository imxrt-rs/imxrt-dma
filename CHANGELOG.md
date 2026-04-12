# Changelog

## [Unreleased]

## [0.2.0] 2026-04-12

- Update package to Rust 2024.
- Use `critical-section` instead of `cortex-m` for the critical section implementation.

## [0.1.1] 2023-01-12

Fix an incorrect lifetime caught by `implied_bounds_entailment`.

## [0.1.0] 2022-12-30

First release. `imxrt-dma` is based on the `imxrt-hal` 0.4 DMA API. Use it to
schedule and coordinate DMA transfers on i.MX RT processors.

[Unreleased]: https://github.com/imxrt-rs/imxrt-dma/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/imxrt-rs/imxrt-dma/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/imxrt-rs/imxrt-dma/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/imxrt-rs/imxrt-dma/releases/tag/v0.1.0
