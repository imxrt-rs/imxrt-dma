//! DMA support for hardware peripherals.
//!
//! If a driver is compatible with this API, it implements some or all of
//! the traits in this module. Consult your HAL for more information.
//!
//! Each future documents when it resolves. To wake the executor, you can
//! route the DMA channel's interrupt handler to [`on_interrupt()`](crate::Dma::on_interrupt).
//! Otherwise, you can poll the future in a loop.

use super::{
    channel::{self, Channel, Configuration},
    Element, Error, Transfer,
};

use core::{
    future::Future,
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

/// A peripheral that can be the source of DMA data
///
/// By 'source,' we mean that it provides data for a DMA transfer.
/// A source would be a hardware device writing data into memory,
/// like a UART receiver.
///
/// # Safety
///
/// `Source` should only be implemented on peripherals that are
/// DMA capable. This trait should be implemented by HAL authors
/// who are exposing DMA capable peripherals.
pub unsafe trait Source<E: Element> {
    /// Peripheral source request signal
    ///
    /// See Table 4-3 of the reference manual. A source may
    /// has a qualifier like 'receive' in the name.
    fn source_signal(&self) -> u32;
    /// Returns a pointer to the register from which the DMA channel
    /// reads data
    ///
    /// This is the register that software reads to acquire data from
    /// a device. The type of the pointer describes the type of reads
    /// the DMA channel performs when transferring data.
    ///
    /// This memory is assumed to be static. Repeated `source` calls
    /// should always return the same address.
    fn source_address(&self) -> *const E;
    /// Perform any actions necessary to enable DMA transfers
    ///
    /// Callers use this method to put the peripheral in a state where
    /// it can supply the DMA channel with data.
    fn enable_source(&mut self);
    /// Perform any actions necessary to disable or cancel DMA transfers
    ///
    /// This may include undoing the actions in `enable_source`.
    fn disable_source(&mut self);
}

/// A peripheral that can be the destination for DMA data
///
/// By 'destination,' we mean that it receives data from a DMA transfer.
/// A destination would be a peripheral that could send data out of
/// processor memory, like a UART transmitter.
///
/// # Safety
///
/// `Destination` should only be implemented on peripherals that are
/// DMA capable. This trait should be implemented by HAL authors
/// who are exposing DMA capable peripherals.
pub unsafe trait Destination<E: Element> {
    /// Peripheral destination request signal
    ///
    /// See Table 4-3 of the reference manual. A destination mave
    /// has a qualifier like 'transfer' in the name.
    fn destination_signal(&self) -> u32;
    /// Returns a pointer to the register into which the DMA channel
    /// writes data
    ///
    /// This is the register that software writes to when sending data to a
    /// device. The type of the pointer describes the type of reads the
    /// DMA channel performs when transferring data.
    fn destination_address(&self) -> *const E;
    /// Perform any actions necessary to enable DMA transfers
    ///
    /// Callers use this method to put the peripheral into a state where
    /// it can accept transfers from a DMA channel.
    fn enable_destination(&mut self);
    /// Perform any actions necessary to disable or cancel DMA transfers
    ///
    /// This may include undoing the actions in `enable_destination`.
    fn disable_destination(&mut self);
}

/// A DMA transfer that receives data from hardware
///
/// The future resolves when the peripheral has provided all
/// expected data. Use [`read()`](crate::peripheral::read) to construct
/// this future.
pub struct Read<'a, S, E>
where
    S: Source<E>,
    E: Element,
{
    channel: &'a Channel,
    source: &'a mut S,
    transfer: Transfer<'a>,
    _elem: PhantomData<&'a mut E>,
}

impl<S, E> Future for Read<'_, S, E>
where
    S: Source<E>,
    E: Element,
{
    type Output = Result<(), Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: no movement from transfer future...
        unsafe { self.map_unchecked_mut(|this| &mut this.transfer) }.poll(cx)
    }
}

impl<S, E> Drop for Read<'_, S, E>
where
    S: Source<E>,
    E: Element,
{
    fn drop(&mut self) {
        self.source.disable_source();
        while self.channel.is_hardware_signaling() {}
        // Drop `transfer` to finish cancellation...
    }
}

fn prepare_read<S, E>(channel: &mut Channel, source: &mut S, buffer: &mut [E])
where
    S: Source<E>,
    E: Element,
{
    channel.disable();

    channel.set_disable_on_completion(true);
    channel.set_channel_configuration(Configuration::enable(source.source_signal()));
    // Safety: hardware source address must be valid, otherwise impl is unsound.
    // Destination buffer lifetime captured by future. The combination of minor
    // loops and transfer iterations ensure that we do not exceed the end of the
    // destination.
    unsafe {
        channel::set_source_hardware(channel, source.source_address());
        channel::set_destination_linear_buffer(channel, buffer);
        channel.set_minor_loop_bytes(core::mem::size_of::<E>() as u32);
        channel.set_transfer_iterations(buffer.len() as u16);
    }

    source.enable_source();
}

/// Use a DMA channel to receive a `buffer` of elements from the source peripheral.
///
/// Consider using a DMA interrupt handler that calls [`on_interrupt()`](crate::Dma::on_interrupt)
/// to wake the executor when the transfer completes. Otherwise, poll the future.
///
/// # Example
///
/// Receive 32 bytes from a LPUART peripheral. Wake the executor when the transfer completes.
///
/// ```no_run
/// use imxrt_dma::{peripheral, channel::Channel};
/// # static DMA: imxrt_dma::Dma<32> = unsafe { imxrt_dma::Dma::new(core::ptr::null(), core::ptr::null()) };
/// # struct X;
/// # unsafe impl peripheral::Source<u8> for X {
/// #   fn source_signal(&self) -> u32 { 0 }
/// #   fn source_address(&self) -> *const u8 { panic!() }
/// #   fn enable_source(&mut self) { panic!() }
/// #   fn disable_source(&mut self) { panic!() }
/// # }
///
/// // #[cortex_m_rt::interrupt]
/// fn DMA7() {
///     // Safety: DMA channel 7 valid and used by a future.
///     unsafe { DMA.on_interrupt(7) };
/// }
///
/// # async fn f() -> imxrt_dma::Result<()> {
/// let mut lpuart = // A LPUART peripheral
///     # X;
/// let mut channel_7: Channel = // DMA channel 7
///     # unsafe { DMA.channel(7) };
/// channel_7.set_interrupt_on_completion(true);
/// // TODO unmask interrupts in NVIC!
///
/// let mut buffer = [0u8; 32];
///
/// peripheral::read(
///     &mut channel_7,
///     &mut lpuart,
///     &mut buffer,
/// ).await?;
/// # Ok(()) }
/// ```
pub fn read<'a, S, E>(
    channel: &'a mut Channel,
    source: &'a mut S,
    buffer: &'a mut [E],
) -> Read<'a, S, E>
where
    S: Source<E>,
    E: Element,
{
    prepare_read(channel, source, buffer);
    Read {
        channel,
        // Safety: transfer is correctly defined
        transfer: unsafe { Transfer::new(channel) },
        source,
        _elem: PhantomData,
    }
}

/// A DMA transfer that sends data to hardware
///
/// The future resolves when the device has sent all provided data.
/// Use [`write()`](crate::peripheral::write) to construct this future.
pub struct Write<'a, D, E>
where
    D: Destination<E>,
    E: Element,
{
    channel: &'a Channel,
    destination: &'a mut D,
    transfer: Transfer<'a>,
    _elem: PhantomData<&'a E>,
}

impl<D, E> Future for Write<'_, D, E>
where
    D: Destination<E>,
    E: Element,
{
    type Output = Result<(), Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: no movement from transfer future...
        unsafe { self.map_unchecked_mut(|this| &mut this.transfer) }.poll(cx)
    }
}

impl<D, E> Drop for Write<'_, D, E>
where
    D: Destination<E>,
    E: Element,
{
    fn drop(&mut self) {
        self.destination.disable_destination();
        while self.channel.is_hardware_signaling() {}
        // Drop `transfer` to finish cancellation...
    }
}

fn prepare_write<D, E>(channel: &mut Channel, buffer: &[E], destination: &mut D)
where
    D: Destination<E>,
    E: Element,
{
    channel.disable();
    channel.set_disable_on_completion(true);
    channel.set_channel_configuration(Configuration::enable(destination.destination_signal()));
    // Safety: hardware address must be valid, otherwise impl is unsound.
    // Source buffer lifetime captured by future. The combination of minor
    // loops and transfer iterations ensure that we do not exceed the end of the
    // source.
    unsafe {
        channel::set_source_linear_buffer(channel, buffer);
        channel::set_destination_hardware(channel, destination.destination_address());
        channel.set_minor_loop_bytes(core::mem::size_of::<E>() as u32);
        channel.set_transfer_iterations(buffer.len() as u16);
    }

    destination.enable_destination();
}

/// Use a DMA channel to send a `buffer` of data to the destination peripheral.
///
/// Consider using a DMA interrupt handler that calls [`on_interrupt()`](crate::Dma::on_interrupt)
/// to wake the executor when the transfer completes. Otherwise, poll the future.
///
/// # Example
///
/// Send five bytes to a LPUART device. Wake the executor when the transfer completes.
///
/// ```no_run
/// use imxrt_dma::{peripheral, channel::Channel};
/// # static DMA: imxrt_dma::Dma<32> = unsafe { imxrt_dma::Dma::new(core::ptr::null(), core::ptr::null()) };
/// # struct X;
/// # unsafe impl peripheral::Destination<u8> for X {
/// #   fn destination_signal(&self) -> u32 { 0 }
/// #   fn destination_address(&self) -> *const u8 { panic!() }
/// #   fn enable_destination(&mut self) { panic!() }
/// #   fn disable_destination(&mut self) { panic!() }
/// # }
///
/// // #[cortex_m_rt::interrupt]
/// fn DMA7() {
///     // Safety: DMA channel 7 valid and used by a future.
///     unsafe { DMA.on_interrupt(7) };
/// }
///
/// # async fn f() -> imxrt_dma::Result<()> {
/// let mut lpuart = // A LPUART peripheral
///     # X;
/// let mut channel_7: Channel = // DMA channel 7
///     # unsafe { DMA.channel(7) };
///
/// channel_7.set_interrupt_on_completion(true);
/// // TODO unmask interrupts in NVIC!
///
/// let buffer = [4u8, 5, 6, 7, 8];
///
/// peripheral::write(
///     &mut channel_7,
///     &buffer,
///     &mut lpuart,
/// ).await?;
/// # Ok(()) }
/// ```
pub fn write<'a, D, E>(
    channel: &'a mut Channel,
    buffer: &'a [E],
    destination: &'a mut D,
) -> Write<'a, D, E>
where
    D: Destination<E>,
    E: Element,
{
    prepare_write(channel, buffer, destination);
    Write {
        channel,
        destination,
        // Safety: transfer is correctly defined
        transfer: unsafe { Transfer::new(channel) },
        _elem: PhantomData,
    }
}

/// Indicates that a peripheral can read and write from a single buffer
/// using two simultaneous DMA transfers
///
/// It's expected that the TX operation will drive the RX operation.
/// A SPI transfer can be modeled in this manner.
///
/// # Safety
///
/// `Bidirectional` assumes the same safety requirements as source and
/// destination. Addtionally, you ensure that the peripheral is capable
/// of this kind of transfer from a single buffer.
pub unsafe trait Bidirectional<E: Element>: Source<E> + Destination<E> {}

/// A full-duplex DMA transfer from a single buffer
///
/// `FullDuplex` only works with [`Bidirectional`]
/// peripherals. The transfer acts on a single buffer, sending and receiving data
/// element by element. It yields when all elements are sent and received.
///
/// To create this future, use [`full_duplex()`].
pub struct FullDuplex<'a, P, E>
where
    P: Bidirectional<E>,
    E: Element,
{
    rx_channel: &'a Channel,
    rx_transfer: Transfer<'a>,
    rx_done: bool,
    tx_channel: &'a Channel,
    tx_transfer: Transfer<'a>,
    tx_done: bool,
    peripheral: &'a mut P,
    _elem: PhantomData<E>,
}

/// Perform a full-duplex DMA transfer using two DMA channels
/// that read and write from a single buffer.
///
/// Consider using a DMA interrupt handler that calls [`on_interrupt()`](crate::Dma::on_interrupt)
/// to wake the executor when the transfer completes. Otherwise, poll the future.
///
/// # Example
///
/// Perform a full-duplex transfer of five `u32`s with a LPSPI peripheral. Generate an interrupt
/// after receiving the final LPSPI word.
///
/// ```no_run
/// use imxrt_dma::{peripheral, channel::Channel};
/// # static DMA: imxrt_dma::Dma<32> = unsafe { imxrt_dma::Dma::new(core::ptr::null(), core::ptr::null()) };
/// # struct X;
/// # unsafe impl peripheral::Source<u32> for X {
/// #   fn source_signal(&self) -> u32 { 0 }
/// #   fn source_address(&self) -> *const u32 { panic!() }
/// #   fn enable_source(&mut self) { panic!() }
/// #   fn disable_source(&mut self) { panic!() }
/// # }
/// # unsafe impl peripheral::Destination<u32> for X {
/// #   fn destination_signal(&self) -> u32 { 0 }
/// #   fn destination_address(&self) -> *const u32 { panic!() }
/// #   fn enable_destination(&mut self) { panic!() }
/// #   fn disable_destination(&mut self) { panic!() }
/// # }
/// # unsafe impl peripheral::Bidirectional<u32> for X {}
///
/// // #[cortex_m_rt::interrupt]
/// fn DMA7() {
///     // Safety: DMA channel 7 valid and used by a future.
///     unsafe { DMA.on_interrupt(7) };
/// }
///
/// # async fn f() -> imxrt_dma::Result<()> {
/// let mut lpspi = // A LPSPI peripheral
///     # X;
/// let mut channel_7: Channel = // DMA channel 7
///     # unsafe { DMA.channel(7) };
/// let mut channel_8: Channel = // DMA channel 8
///     # unsafe { DMA.channel(8) };
///
/// // Using channel_7 for the receive data. Once we've received
/// // the last word from the LPSPI peripheral, generate an interrupt.
/// channel_7.set_interrupt_on_completion(true);
/// // Don't trigger an interrupt after we _send_ the last LPSPI
/// // word from memory; we're still waiting for one word response.
/// channel_8.set_interrupt_on_completion(false);
/// // TODO unmask interrupts in NVIC!
///
/// let mut buffer = [4u32, 5, 6, 7, 8];
///
/// peripheral::full_duplex(
///     &mut channel_7,
///     &mut channel_8,
///     &mut lpspi,
///     &mut buffer,
/// ).await?;
/// # Ok(()) }
/// ```
pub fn full_duplex<'a, P, E>(
    rx_channel: &'a mut Channel,
    tx_channel: &'a mut Channel,
    peripheral: &'a mut P,
    buffer: &'a mut [E],
) -> FullDuplex<'a, P, E>
where
    P: Bidirectional<E>,
    E: Element,
{
    prepare_write(tx_channel, buffer, peripheral);
    prepare_read(rx_channel, peripheral, buffer);

    FullDuplex {
        rx_channel,
        rx_transfer: unsafe { Transfer::new(rx_channel) },
        rx_done: false,
        tx_channel,
        tx_transfer: unsafe { Transfer::new(tx_channel) },
        tx_done: false,
        peripheral,
        _elem: PhantomData,
    }
}

impl<P, E> Future for FullDuplex<'_, P, E>
where
    P: Bidirectional<E>,
    E: Element,
{
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if !self.rx_done {
            // Safety: pin projection OK, no movement from future...
            let poll = unsafe {
                self.as_mut()
                    .map_unchecked_mut(|this| &mut this.rx_transfer)
            }
            .poll(cx)?;
            // Safety: OK to toggle a bool...
            *unsafe { &mut self.as_mut().get_unchecked_mut().rx_done } = poll.is_ready();
        }

        if !self.tx_done {
            // Safety: pin projection OK, no movement from future...
            let poll = unsafe {
                self.as_mut()
                    .map_unchecked_mut(|this| &mut this.tx_transfer)
            }
            .poll(cx)?;
            // Safety: OK to toggle a bool...
            *unsafe { &mut self.as_mut().get_unchecked_mut().tx_done } = poll.is_ready();
        }

        if self.tx_done && self.rx_done {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

impl<P, E> Drop for FullDuplex<'_, P, E>
where
    P: Bidirectional<E>,
    E: Element,
{
    fn drop(&mut self) {
        self.peripheral.disable_destination();
        self.peripheral.disable_source();
        while self.tx_channel.is_hardware_signaling() {}
        while self.rx_channel.is_hardware_signaling() {}
        // Drop the transfers to finish cancellation...
    }
}
