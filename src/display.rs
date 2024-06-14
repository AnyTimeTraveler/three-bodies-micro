use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Dimensions;
use embedded_graphics::Pixel;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::Rectangle;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::delay::Delay;
use esp_hal::gpio::{GpioPin, Output, PushPull};
use esp_hal::peripherals::SPI2;
use esp_hal::spi::FullDuplexMode;
use esp_hal::spi::master::Spi;
use st7735_lcd::ST7735;
use crate::color::Color;

pub(crate) struct Display<'a> {
    driver: ST7735<
        ExclusiveDevice<Spi<'a, SPI2, FullDuplexMode>, GpioPin<Output<PushPull>, 22>, Delay>,
        GpioPin<Output<PushPull>, 20>,
        GpioPin<Output<PushPull>, 21>
    >,
}

impl<'a> Display<'a> {
    pub(crate) fn draw_circle(&self, p0: f32, p1: f32, p2: f32, p3: Color) {
        todo!()
    }
}

impl<'a> Display<'a> {
    pub(crate) fn draw_text(&self, p0: &str, p1: f32, p2: f32, p3: f64, p4: Color) {
        todo!()
    }
}

impl<'a> Display<'a> {
    pub(crate) fn init(
        lcd_spi: ExclusiveDevice<Spi<SPI2, FullDuplexMode>, GpioPin<Output<PushPull>, 22>, Delay>,
        lcd_dc: GpioPin<Output<PushPull>, 20>,
        lcd_reset: GpioPin<Output<PushPull>, 21>,
        mut delay: Delay,
    ) -> Display<'a> {
        // let b : &mut Spi<'_, esp_hal::peripherals::SPI2, FullDuplexMode> = lcd_spi;
        // let mut a: Box<dyn SpiDevice<Error=esp_hal::spi::Error>> = Box::new(b);

        let mut st7735 = ST7735::new(lcd_spi, lcd_dc, lcd_reset, true, false, 320, 240);

        st7735.init(&mut delay).unwrap();

        // Display {
        //     driver: st7735
        // }
        todo!()
    }

    pub(crate) fn clear_background(&mut self, colour: Rgb565) {
        self.driver.clear(colour).unwrap();
    }

    pub(crate) fn screen_width(&self) -> f32 {
        self.driver.bounding_box().size.width as f32
    }

    pub(crate) fn screen_height(&self) -> f32 {
        self.driver.bounding_box().size.height as f32
    }
}

impl Dimensions for Display<'_> {
    fn bounding_box(&self) -> Rectangle {
        self.driver.bounding_box()
    }
}

impl DrawTarget for Display<'_> {
    type Color = Rgb565;
    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item=Pixel<Self::Color>>,
    {
        self.driver.draw_iter(pixels)
    }
}
