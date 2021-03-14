//! DMA support for hardware peripherals

use super::{Channel, ChannelConfiguration, Element, ErrorStatus, Transfer};

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
///
/// The `enable_source` and `disable_source` methods may have
/// interior mutability. The implementer must ensure that any modifications
/// are atomic. The `Source` consumer will assume that `enable_source` and
/// `disable_source` are correct, and that they are safe to call.
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
///
/// The `enable_destination` and `disable_destination` methods may have
/// interior mutability. The implementer must ensure that any modifications
/// are atomic. The `Source` consumer will assume that `enable_destination` and
/// `disable_destination` are correct, and that they are safe to call.
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
/// See [`receive`](crate::peripheral::receive) to construct an `Rx`.
pub struct Rx<'a, S, E>
where
    S: Source<E>,
    E: Element,
{
    channel: &'a Channel,
    source: &'a mut S,
    transfer: Transfer<'a>,
    _elem: PhantomData<E>,
}

impl<S, E> Future for Rx<'_, S, E>
where
    S: Source<E>,
    E: Element,
{
    type Output = Result<(), ErrorStatus>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: no movement from transfer future...
        unsafe { self.map_unchecked_mut(|this| &mut this.transfer) }.poll(cx)
    }
}

impl<S, E> Drop for Rx<'_, S, E>
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

/// Use a DMA channel to receive a `buffer` of elements from the source peripheral
pub fn receive<'a, S, E>(
    channel: &'a mut Channel,
    source: &'a mut S,
    buffer: &'a mut [E],
) -> Rx<'a, S, E>
where
    S: Source<E>,
    E: Element,
{
    channel.set_channel_configuration(ChannelConfiguration::enable(source.source_signal()));
    super::set_source_hardware(channel, source.source_address());
    super::set_destination_linear_buffer(channel, buffer);
    channel.set_minor_loop_bytes(core::mem::size_of::<E>() as u32);
    channel.set_transfer_iterations(buffer.len() as u16);

    source.enable_source();
    Rx {
        channel,
        // Safety: transfer is correctly defined
        transfer: unsafe { Transfer::new(channel) },
        source,
        _elem: PhantomData,
    }
}

/// A DMA transfer that sends data to hardware
///
/// See [`transfer`](crate::peripheral::transfer) to create a `Tx`.
pub struct Tx<'a, D, E>
where
    D: Destination<E>,
    E: Element,
{
    channel: &'a Channel,
    destination: &'a mut D,
    transfer: Transfer<'a>,
    _elem: PhantomData<E>,
}

impl<D, E> Future for Tx<'_, D, E>
where
    D: Destination<E>,
    E: Element,
{
    type Output = Result<(), ErrorStatus>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: no movement from transfer future...
        unsafe { self.map_unchecked_mut(|this| &mut this.transfer) }.poll(cx)
    }
}

impl<D, E> Drop for Tx<'_, D, E>
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

/// Use a DMA channel to send a `buffer` of data to the destination peripheral
pub fn transfer<'a, D, E>(
    channel: &'a mut Channel,
    buffer: &'a [E],
    destination: &'a mut D,
) -> Tx<'a, D, E>
where
    D: Destination<E>,
    E: Element,
{
    channel.set_channel_configuration(ChannelConfiguration::enable(
        destination.destination_signal(),
    ));
    super::set_source_linear_buffer(channel, buffer);
    super::set_destination_hardware(channel, destination.destination_address());

    channel.set_minor_loop_bytes(core::mem::size_of::<E>() as u32);
    channel.set_transfer_iterations(buffer.len() as u16);

    destination.enable_destination();
    Tx {
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
/// `FullDuplex` only works with [`Bidirectional`](crate::peripheral::Bidirectional)
/// peripherals. The transfer acts on a single buffer, sending and receiving data
/// element by element.
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

/// Perform a full-suplex DMA transfer using two DMA channels
/// that read and write from a single buffer.
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
    // RX channel takes data from the peripheral, and moves it to the buffer..
    rx_channel.set_channel_configuration(ChannelConfiguration::enable(peripheral.source_signal()));
    super::set_source_hardware(rx_channel, peripheral.source_address());
    super::set_destination_linear_buffer(rx_channel, buffer);
    rx_channel.set_minor_loop_bytes(core::mem::size_of::<E>() as u32);
    rx_channel.set_transfer_iterations(buffer.len() as u16);

    // TX channel takes data from the buffer, and sends it to the peripheral
    tx_channel.set_channel_configuration(ChannelConfiguration::enable(
        peripheral.destination_signal(),
    ));
    super::set_source_linear_buffer(tx_channel, buffer);
    super::set_destination_hardware(tx_channel, peripheral.destination_address());
    tx_channel.set_minor_loop_bytes(core::mem::size_of::<E>() as u32);
    tx_channel.set_transfer_iterations(buffer.len() as u16);

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
    type Output = Result<(), ErrorStatus>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Schedule the receive first...
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
