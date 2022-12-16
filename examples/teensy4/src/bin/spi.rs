//! Full-duplex DMA transfers with SPI
//!
//! Setup
//! =====
//!
//! 1MHz SPI connected to an MPU9250
//!
//! - Pin 13 (SCK) to MPU's SCL (Note that we lose the LED here)
//! - Pin 11 (MOSI) to MPU's SDA/SDI
//! - Pin 12 (MISO) to MPU's AD0/SDO
//! - Pin 10 (PSC0) to MPU's NCS
//!
//! Demo
//! ====
//!
//! After a 5 second pause, the main loop uses two concurrent
//! DMA transfers to perform a SPI transfer, reading an MPU9250's
//! WHO_AM_I register. The example prints the value to the USB
//! log channel.

#![no_std]
#![no_main]

use bsp::hal::iomuxc;
use bsp::hal::ral;
use ral::interrupt;
use teensy4_bsp as bsp;

use imxrt_dma::{
    peripheral::{Bidirectional, Destination, Source},
    Element,
};

/// SPI clock rate (Hz)
const CLOCK_RATE_HZ: u32 = 1_000_000;
/// Effective LPSPI source clock (PLL2)
const SOURCE_CLOCK_HZ: u32 = 528_000_000;
/// Any divider for the source clock
const SOURCE_CLOCK_DIVIDER: u32 = 5;

struct Pins<SDO, SDI, SCK, PCS0>
where
    SDO: iomuxc::spi::Pin<Signal = iomuxc::spi::SDO>,
    SDI: iomuxc::spi::Pin<Signal = iomuxc::spi::SDI>,
    SCK: iomuxc::spi::Pin<Signal = iomuxc::spi::SCK>,
    PCS0: iomuxc::spi::Pin<Signal = iomuxc::spi::PCS0>,
{
    sdo: SDO,
    sdi: SDI,
    sck: SCK,
    pcs0: PCS0,
}

pub struct Spi {
    instance: ral::lpspi::Instance,
}

impl Spi {
    fn new<SDO, SDI, SCK, PCS0>(
        instance: ral::lpspi::Instance,
        pins: &mut Pins<SDO, SDI, SCK, PCS0>,
    ) -> Self
    where
        SDO: iomuxc::spi::Pin<Signal = iomuxc::spi::SDO>,
        SDI: iomuxc::spi::Pin<Signal = iomuxc::spi::SDI>,
        SCK: iomuxc::spi::Pin<Signal = iomuxc::spi::SCK>,
        PCS0: iomuxc::spi::Pin<Signal = iomuxc::spi::PCS0>,
    {
        iomuxc::spi::prepare(&mut pins.sdo);
        iomuxc::spi::prepare(&mut pins.sdi);
        iomuxc::spi::prepare(&mut pins.sck);
        iomuxc::spi::prepare(&mut pins.pcs0);

        ral::write_reg!(ral::lpspi, instance, CR, RST: RST_1);
        ral::write_reg!(ral::lpspi, instance, CR, RST: RST_0);
        set_clock_speed(
            &instance,
            SOURCE_CLOCK_HZ / SOURCE_CLOCK_DIVIDER,
            CLOCK_RATE_HZ,
        );
        ral::write_reg!(
            ral::lpspi,
            instance,
            CFGR1,
            MASTER: MASTER_1,
            SAMPLE: SAMPLE_1
        );
        ral::write_reg!(ral::lpspi, instance, CR, MEN: MEN_1);

        Spi { instance }
    }

    fn set_frame_size<W>(&mut self) {
        ral::modify_reg!(ral::lpspi, self.instance, TCR, FRAMESZ: ((core::mem::size_of::<W>() * 8 - 1) as u32));
    }
}

unsafe impl<E: Element> Source<E> for Spi {
    fn source_signal(&self) -> u32 {
        match &*self.instance as *const _ {
            ral::lpspi::LPSPI1 => 13,
            ral::lpspi::LPSPI2 => 77,
            ral::lpspi::LPSPI3 => 15,
            ral::lpspi::LPSPI4 => 79,
            _ => unreachable!("handled all LPSPI peripherals for imxrt1060"),
        }
    }
    fn source_address(&self) -> *const E {
        &self.instance.RDR as *const _ as *const E
    }
    fn enable_source(&mut self) {
        self.set_frame_size::<E>();
        ral::modify_reg!(ral::lpspi, self.instance, FCR, RXWATER: 0);
        ral::modify_reg!(ral::lpspi, self.instance, DER, RDDE: 1);
    }
    fn disable_source(&mut self) {
        while ral::read_reg!(ral::lpspi, self.instance, DER, RDDE == 1) {
            ral::modify_reg!(ral::lpspi, self.instance, DER, RDDE: 0);
        }
    }
}

unsafe impl<E: Element> Destination<E> for Spi {
    fn destination_signal(&self) -> u32 {
        <Self as Source<E>>::source_signal(self) + 1
    }
    fn destination_address(&self) -> *const E {
        &self.instance.TDR as *const _ as *const E
    }
    fn enable_destination(&mut self) {
        self.set_frame_size::<E>();
        ral::modify_reg!(ral::lpspi, self.instance, FCR, TXWATER: 0);
        ral::modify_reg!(ral::lpspi, self.instance, DER, TDDE: 1);
    }
    fn disable_destination(&mut self) {
        while ral::read_reg!(ral::lpspi, self.instance, DER, TDDE == 1) {
            ral::modify_reg!(ral::lpspi, self.instance, DER, TDDE: 0);
        }
    }
}

unsafe impl<E: Element> Bidirectional<E> for Spi {}

#[cortex_m_rt::entry]
fn main() -> ! {
    support::logging::init().unwrap();

    let mut ccm = ral::ccm::CCM::take().unwrap();
    // Set DMA clock gates to ON
    ral::modify_reg!(ral::ccm, ccm, CCGR5, CG3: 0b11);
    // Enable SPI clocks
    ral::modify_reg!(
        ral::ccm,
        ccm,
        CBCMR,
        LPSPI_CLK_SEL: LPSPI_CLK_SEL_2, /* PLL2 */
        LPSPI_PODF: SOURCE_CLOCK_DIVIDER - 1
    );

    let pins = support::pins(ral::iomuxc::IOMUXC::take().unwrap());

    // Using SPI4
    let spi = ral::lpspi::LPSPI4::take().unwrap();
    set_clock_gate(&spi, &mut ccm, true);
    let mut pins = Pins {
        sdo: pins.p11,
        sdi: pins.p12,
        sck: pins.p13,
        pcs0: pins.p10,
    };
    let mut spi = Spi::new(spi, &mut pins);

    let core_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut systick = bsp::SysTick::new(core_peripherals.SYST);
    systick.delay(5000);

    let mut channels = support::channels(
        ral::dma0::DMA0::take().unwrap(),
        ral::dmamux::DMAMUX::take().unwrap(),
    );

    let mut tx_channel = channels[7].take().unwrap();
    tx_channel.set_interrupt_on_completion(true);

    let mut rx_channel = channels[8].take().unwrap();
    rx_channel.set_interrupt_on_completion(true);

    // Safety: all prepared to handle interrupts
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::DMA7_DMA23);
        cortex_m::peripheral::NVIC::unmask(interrupt::DMA8_DMA24);
    }

    log::info!("Dropping into main loop...");
    loop {
        let mut buffer: [u16; 1] = [read(WHO_AM_I)];
        let transfer = imxrt_dma::peripheral::full_duplex(
            &mut rx_channel,
            &mut tx_channel,
            &mut spi,
            &mut buffer,
        );
        log::info!("Scheduled full-duplex transfer...");
        assert!(support::wfi(transfer).is_ok());
        log::info!("Received {:#X} for WHO_AM_I", buffer[0]);

        systick.delay(1000);
    }
}

#[cortex_m_rt::interrupt]
fn DMA7_DMA23() {
    // Safety: channel 7 is a valid channel.
    unsafe { support::DMA.on_interrupt(7) };
}

#[cortex_m_rt::interrupt]
fn DMA8_DMA24() {
    // Safety: channel 8 is a valid channel.
    unsafe { support::DMA.on_interrupt(8) };
}

fn set_clock_gate(spi: &ral::lpspi::Instance, ccm: &mut ral::ccm::Instance, gate: bool) {
    let gate = if gate { 0b11 } else { 0 };
    match &**spi as *const _ {
        ral::lpspi::LPSPI1 => ral::modify_reg!(ral::ccm, ccm, CCGR1, CG0: gate),
        ral::lpspi::LPSPI2 => ral::modify_reg!(ral::ccm, ccm, CCGR1, CG1: gate),
        ral::lpspi::LPSPI3 => ral::modify_reg!(ral::ccm, ccm, CCGR1, CG2: gate),
        ral::lpspi::LPSPI4 => ral::modify_reg!(ral::ccm, ccm, CCGR1, CG3: gate),
        _ => unreachable!("handled all LPSPI peripherals for imxrt1060"),
    }
}

/// Must be called while SPI is disabled
fn set_clock_speed(spi: &ral::lpspi::Instance, base: u32, hz: u32) {
    let mut div = base / hz;
    if base / div > hz {
        div += 1;
    }
    let div = div.saturating_sub(2).clamp(0, 255);
    ral::write_reg!(
        ral::lpspi,
        spi,
        CCR,
        SCKDIV: div,
        DBT: div / 2,
        SCKPCS: 0x1F,
        PCSSCK: 0x1F
    );
}

/// MPU9250 WHO_AM_I register address
const WHO_AM_I: u8 = 0x75;

/// Creates a read instruction for the MPU9250
const fn read(address: u8) -> u16 {
    ((address as u16) | (1 << 7)) << 8
}
