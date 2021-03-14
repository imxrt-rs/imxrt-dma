//! DMA-powered memcpy example
//!
//! Flash this example to your Teensy 4, and connect to the
//! Teensy 4's USB CDC interface. You should see info-level
//! messages every second indicating a successful transfer.

#![no_std]
#![no_main]

use bsp::hal::ral;
use teensy4_bsp as bsp;

#[cortex_m_rt::entry]
fn main() -> ! {
    support::logging::init().unwrap();

    let ccm = ral::ccm::CCM::take().unwrap();
    // Set DMA clock gates to ON
    ral::modify_reg!(ral::ccm, ccm, CCGR5, CG3: 0b11);

    let core_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut systick = bsp::SysTick::new(core_peripherals.SYST);
    systick.delay(5000);

    let mut channels = support::channels(
        ral::dma0::DMA0::take().unwrap(),
        ral::dmamux::DMAMUX::take().unwrap(),
    );

    let mut channel = channels[7].take().unwrap();

    let mut value = 1u32;
    loop {
        let source = [value; 256];
        let mut destination = [0u32; 256];

        let memcpy = imxrt_dma::memcpy(&source, &mut destination, &mut channel);
        let result = support::block(memcpy);
        assert!(result.is_ok());

        for (idx, dst) in destination.iter().enumerate() {
            assert_eq!(*dst, value, "index {}", idx);
        }

        log::info!("Transfer of {} OK!", value);
        systick.delay(1000);
        value += 1;
    }
}
