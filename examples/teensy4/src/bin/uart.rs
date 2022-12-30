//! Peripheral source and destination example
//!
//! Setup
//! =====
//!
//! - Baud rate: 115_200
//! - TX: pin 14
//! - RX: pin 15
//! - Parity: None
//!
//! Demo
//! ====
//!
//! After a 5 second pause, the main loop uses a
//! DMA transfer to receive a single byte. The loop
//! then replies with the same byte 32 times.
//!
//! The demo shows how you can
//!
//! - implement Source and Destination traits for a peripheral
//! - schedule transfers that are complete when an interrupt
//!   triggers

#![no_std]
#![no_main]

use bsp::hal::iomuxc;
use bsp::hal::ral;
use dma::peripheral::{Destination, Source};
use imxrt_dma as dma;
use ral::interrupt;
use teensy4_bsp as bsp;

/// Baud rate
const BAUD: u32 = 115_200;
/// Effective LPUART source clock (24MHz XTAL)
const SOURCE_CLOCK_HZ: u32 = 24_000_000;
/// Any divider for the source clock
const SOURCE_CLOCK_DIVIDER: u32 = 1;
/// Divider value, subtract 1 before writing OSR to register
const TIMINGS: (u32, u32) = timings(SOURCE_CLOCK_HZ / SOURCE_CLOCK_DIVIDER, BAUD);

/// The DMA UART adapter
struct Uart {
    instance: ral::lpuart::Instance,
}

impl Uart {
    /// Creates a new UART peripheral that can perform DMA transfers
    ///
    /// Caller should make sure that the TX and RX pins match the LPUART instance.
    /// That is, don't pass a LPUART2 instance with LPUART7 pins!
    fn new<TX, RX>(instance: ral::lpuart::Instance, tx_pin: &mut TX, rx_pin: &mut RX) -> Self
    where
        TX: iomuxc::uart::Pin<Direction = iomuxc::uart::TX>,
        RX: iomuxc::uart::Pin<Direction = iomuxc::uart::RX>,
    {
        iomuxc::uart::prepare(tx_pin);
        iomuxc::uart::prepare(rx_pin);

        // Disable peripheral
        ral::modify_reg!(ral::lpuart, instance, CTRL, TE: TE_0, RE: RE_0);
        // Set peripheral timings
        ral::modify_reg!(ral::lpuart, instance, BAUD, OSR: TIMINGS.0 - 1, SBR: TIMINGS.1, BOTHEDGE: 0);
        // Enable the peripheral
        ral::modify_reg!(ral::lpuart, instance, CTRL, TE: TE_1, RE: RE_1);
        Uart { instance }
    }

    /// Clear status flags
    fn clear_status(&mut self) {
        ral::modify_reg!(
            ral::lpuart,
            self.instance,
            STAT,
            IDLE: IDLE_1,
            OR: OR_1,
            NF: NF_1,
            FE: FE_1,
            PF: PF_1
        );
    }

    /// Flush the transmit and receive buffers
    fn flush(&mut self) {
        ral::modify_reg!(
            ral::lpuart,
            self.instance,
            FIFO,
            TXFLUSH: TXFLUSH_1,
            RXFLUSH: RXFLUSH_1
        );
    }
}

/// Safety: this UART peripheral can provide data for a DMA transfer
unsafe impl Source<u8> for Uart {
    /// See the reference manual for number documentation
    fn source_signal(&self) -> u32 {
        match &*self.instance as *const _ {
            ral::lpuart::LPUART1 => 3,
            ral::lpuart::LPUART2 => 67,
            ral::lpuart::LPUART3 => 5,
            ral::lpuart::LPUART4 => 69,
            ral::lpuart::LPUART5 => 7,
            ral::lpuart::LPUART6 => 71,
            ral::lpuart::LPUART7 => 9,
            ral::lpuart::LPUART8 => 73,
            _ => unreachable!("Handled all LPUARTs for the imxrt1060"),
        }
    }
    /// Point the DMA controller at the DATA register
    fn source_address(&self) -> *const u8 {
        &self.instance.DATA as *const _ as *const u8
    }
    /// Enable receiver DMA
    fn enable_source(&mut self) {
        self.clear_status();
        ral::modify_reg!(ral::lpuart, self.instance, BAUD, RDMAE: 1);
    }
    /// Disable receiver DMA
    fn disable_source(&mut self) {
        self.flush();
        while ral::read_reg!(ral::lpuart, self.instance, BAUD, RDMAE == 1) {
            ral::modify_reg!(ral::lpuart, self.instance, BAUD, RDMAE: 0);
        }
    }
}

/// Safety: this UART peripheral can receive data from a DMA transfer
unsafe impl Destination<u8> for Uart {
    fn destination_signal(&self) -> u32 {
        self.source_signal() - 1
    }
    /// Point the DMA controller at the DATA register
    fn destination_address(&self) -> *const u8 {
        &self.instance.DATA as *const _ as *const u8
    }
    /// Enable transfer DMA
    fn enable_destination(&mut self) {
        self.flush();
        ral::modify_reg!(ral::lpuart, self.instance, BAUD, TDMAE: 1);
    }
    /// Disable transfer DMA
    fn disable_destination(&mut self) {
        self.flush();
        while ral::read_reg!(ral::lpuart, self.instance, BAUD, TDMAE == 1) {
            ral::modify_reg!(ral::lpuart, self.instance, BAUD, TDMAE: 0);
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    support::logging::init().unwrap();

    let mut ccm = ral::ccm::CCM::take().unwrap();
    // Set DMA clock gates to ON
    ral::modify_reg!(ral::ccm, ccm, CCGR5, CG3: 0b11);
    // Enable UART clocks
    ral::modify_reg!(ral::ccm, ccm, CSCDR1, UART_CLK_SEL: 1 /* Oscillator */, UART_CLK_PODF: SOURCE_CLOCK_DIVIDER - 1);

    let mut pins = support::pins(ral::iomuxc::IOMUXC::take().unwrap());

    // Using LPUART2
    let uart = ral::lpuart::LPUART2::take().unwrap();
    set_clock_gate(&uart, &mut ccm, true);
    let mut uart = Uart::new(uart, &mut pins.p14, &mut pins.p15);

    let core_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut systick = bsp::SysTick::new(core_peripherals.SYST);
    systick.delay(5000);

    let mut channels = support::channels(
        ral::dma0::DMA0::take().unwrap(),
        ral::dmamux::DMAMUX::take().unwrap(),
    );

    let mut channel = channels[7].take().unwrap();
    channel.set_interrupt_on_completion(true);
    // Safety: all prepared to handle interrupts
    unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::DMA7_DMA23) };

    log::info!(
        "Dropping into main loop, OSR={}, SBR={}",
        TIMINGS.0,
        TIMINGS.1
    );

    loop {
        let mut buffer = [0u8; 1];
        let read = imxrt_dma::peripheral::read(&mut channel, &mut uart, &mut buffer);
        assert!(support::wfi(read).is_ok());

        log::info!("Received a byte: {}", buffer[0]);

        let buffer = [buffer[0]; 32];
        let write = imxrt_dma::peripheral::write(&mut channel, &buffer, &mut uart);
        assert!(support::wfi(write).is_ok());

        log::info!("Replied!");
        systick.delay(1);
    }
}

#[cortex_m_rt::interrupt]
fn DMA7_DMA23() {
    // Safety: channel 7 is a valid channel.
    unsafe { support::DMA.on_interrupt(7) };
}

/// Computes the `(OSR, SBR)` timing values at compile time
///
/// The returned OSR value will need a `- 1` before writing
/// to the register. The `OSR` value is chosen such that `BOTHEDGE`
/// isn't necessary.
const fn timings(source_clock_hz: u32, baud: u32) -> (u32, u32) {
    const fn max(left: u32, right: u32) -> u32 {
        if left > right {
            left
        } else {
            right
        }
    }
    const fn min(left: u32, right: u32) -> u32 {
        if left < right {
            left
        } else {
            right
        }
    }

    let mut err = u32::max_value();
    let mut best_osr = 0;
    let mut best_sbr = 0;

    let mut osr = 8;
    let mut sbr = 1;
    while osr <= 32 {
        while sbr < 8192 {
            let b = source_clock_hz / (sbr * osr);
            let e = max(baud, b) - min(baud, b);
            if e < err {
                err = e;
                best_osr = osr;
                best_sbr = sbr;
            }
            sbr += 1;
        }
        osr += 1;
    }
    (best_osr, best_sbr)
}

/// Enable (`true`) or disable (`false`) the LPAURT clock gate
fn set_clock_gate(uart: &ral::lpuart::Instance, ccm: &mut ral::ccm::Instance, gate: bool) {
    let gate = if gate { 0b11 } else { 0 };
    match &**uart as *const _ {
        ral::lpuart::LPUART1 => ral::modify_reg!(ral::ccm, ccm, CCGR5, CG12: gate),
        ral::lpuart::LPUART2 => ral::modify_reg!(ral::ccm, ccm, CCGR0, CG14: gate),
        ral::lpuart::LPUART3 => ral::modify_reg!(ral::ccm, ccm, CCGR0, CG6: gate),
        ral::lpuart::LPUART4 => ral::modify_reg!(ral::ccm, ccm, CCGR1, CG12: gate),
        ral::lpuart::LPUART5 => ral::modify_reg!(ral::ccm, ccm, CCGR3, CG1: gate),
        ral::lpuart::LPUART6 => ral::modify_reg!(ral::ccm, ccm, CCGR3, CG3: gate),
        ral::lpuart::LPUART7 => ral::modify_reg!(ral::ccm, ccm, CCGR5, CG13: gate),
        ral::lpuart::LPUART8 => ral::modify_reg!(ral::ccm, ccm, CCGR6, CG7: gate),
        _ => unreachable!("Handled all LPUARTs for the imxrt1060"),
    }
}
