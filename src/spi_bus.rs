use alloc::boxed::Box;
use core::ptr;

use esp_hal::{clock::Clocks,
              gpio::{InputPin, OutputPin},
              peripheral::Peripheral,
              peripherals::SPI2,
              spi::{FullDuplexMode, SpiMode},
              system::PeripheralClockControl};
use fugit::HertzU32;

static mut G_SPI_BUS: *mut SpiBusController<'static, SPI2, FullDuplexMode> = ptr::null_mut();

pub struct SpiBus();

impl SpiBus {
    pub(crate) fn init<SCK: OutputPin, MOSI: OutputPin, MISO: InputPin>(
        spi: impl Peripheral<P=SPI2> + 'static,
        sck: impl Peripheral<P=SCK> + 'static,
        mosi: impl Peripheral<P=MOSI> + 'static,
        miso: impl Peripheral<P=MISO> + 'static,
        frequency: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> SpiBus
    {
        if !unsafe { G_SPI_BUS.is_null() } {
            panic!("Tried to re-initialize SPI!");
        }
        let spi = Spi::new_no_cs(
            spi,
            sck,
            mosi,
            miso,
            frequency,
            SpiMode::Mode0,
            peripheral_clock_control,
            clocks,
        );

        unsafe { G_SPI_BUS = Box::into_raw(Box::new(SpiBusController::from_spi(spi))); }
        SpiBus()
    }

    pub(crate) fn add_device<CS: OutputPin + embedded_hal::digital::OutputPin>(&self, mut cs: CS) -> SpiBusDevice<'static, 'static, SPI2, CS, FullDuplexMode> {
        cs.set_high().unwrap();
        unsafe { (*G_SPI_BUS).add_device(cs) }
    }
}
