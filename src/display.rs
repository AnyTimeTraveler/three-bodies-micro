// use embedded_graphics::draw_target::DrawTarget;
// use embedded_graphics::geometry::Dimensions;
// use embedded_graphics::Pixel;
// use embedded_graphics::pixelcolor::Rgb565;
// use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
    text::Text,
};
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::text::{LineHeight, TextStyle};
use embedded_graphics_framebuf::FrameBuf;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::delay::Delay;
use esp_hal::gpio::{GpioPin, Output, PushPull};
use esp_hal::peripherals::SPI2;
use esp_hal::spi::FullDuplexMode;
use esp_hal::spi::master::Spi;
use st7735_lcd::ST7735;

use crate::color::Color;

const WIDTH: usize = 320;
const HEIGHT: usize = 240;

// const STYLE: PrimitiveStyleBuilder<Rgb565> = PrimitiveStyleBuilder::new();

pub(crate) struct Display<'a> {
    driver: ST7735<
        ExclusiveDevice<Spi<'a, SPI2, FullDuplexMode>, GpioPin<Output<PushPull>, 15>, Delay>,
        GpioPin<Output<PushPull>, 22>,
        GpioPin<Output<PushPull>, 23>
    >,
    // buf: [Rgb565; WIDTH * HEIGHT],
    buf: FrameBuf<Rgb565, [Rgb565; WIDTH * HEIGHT]>,
}

impl<'a> Display<'a> {
    pub(crate) fn init(
        lcd_spi: ExclusiveDevice<Spi<'a, SPI2, FullDuplexMode>, GpioPin<Output<PushPull>, 15>, Delay>,
        lcd_dc: GpioPin<Output<PushPull>, 22>,
        lcd_reset: GpioPin<Output<PushPull>, 23>,
        mut delay: Delay,
    ) -> Display<'a> {
        let data = [Rgb565::BLACK; WIDTH * HEIGHT];

        let buf = FrameBuf::new(data, WIDTH, HEIGHT);

        let mut st7735 = ST7735::new(lcd_spi, lcd_dc, lcd_reset, true, false, WIDTH as u32, HEIGHT as u32);

        st7735.init(&mut delay).unwrap();

        Display {
            driver: st7735,
            buf,
        }
    }

    pub(crate) fn clear_background(&mut self, colour: Rgb565) {
        self.buf.clear(colour).unwrap();
    }

    pub(crate) fn screen_width(&self) -> f32 {
        self.driver.bounding_box().size.width as f32
    }

    pub(crate) fn screen_height(&self) -> f32 {
        self.driver.bounding_box().size.height as f32
    }

    pub(crate) fn flush(&mut self) {
        let area = Rectangle::new(Point::new(0, 0), self.buf.size());
        self.driver.fill_contiguous(&area, self.buf.data).unwrap();
        self.buf.clear(Rgb565::BLACK).unwrap();
    }

    pub(crate) fn draw_circle(&mut self, x: f32, y: f32, mass: f32, color: Color) {
        Circle::new(Point::new(x as i32, y as i32), mass as u32)
            .into_styled(PrimitiveStyle::with_fill(color.into()))
            // .draw(self)
            .draw(&mut self.buf)
            .unwrap();
    }

    pub(crate) fn draw_text(&mut self, text: &str, x: f32, y: f32, size: f64, color: Color) {
        let style = MonoTextStyle::new(&FONT_6X10, color.into());
        let mut text_style = TextStyle::default();
        text_style.line_height = LineHeight::Pixels(size as u32);
        Text::with_text_style(text, Point::new(x as i32, y as i32), style, text_style)
            // .draw(self)
            .draw(&mut self.buf)
            .unwrap();
    }
}

impl From<Color> for Rgb565 {
    fn from(c: Color) -> Self {
        Rgb565::new(
            (Rgb565::MAX_R as f32 * c.r * c.a) as u8,
            (Rgb565::MAX_G as f32 * c.g * c.a) as u8,
            (Rgb565::MAX_B as f32 * c.b * c.a) as u8,
        )
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
