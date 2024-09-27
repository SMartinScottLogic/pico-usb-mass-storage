use core::default;

use embassy_time::Timer;

use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use epd_waveshare::{
    epd2in66b::{Display2in66b, Epd2in66b},
    prelude::*,
};

use crate::SIGNAL;

pub struct Screen<SPI, DISPLAY>
where
    SPI: SpiDevice,
    DISPLAY: WaveshareThreeColorDisplay<SPI, BUSY, DC, RST, embassy_time::Delay>,
{
    address: [u8; 4],
    label: [u8; 11],
    freespace: u32,
    delay: embassy_time::Delay,
    spi: SPI,
    e_paper: DISPLAY,
}

impl<SPI, DISPLAY> Screen<SPI, DISPLAY>
where
    SPI: SpiDevice,
    DISPLAY: WaveshareThreeColorDisplay<SPI, BUSY, DC, RST, embassy_time::Delay>,
{
    pub async fn build() -> Self {
        // TODO Initialise display (SPI)
        let delay;
        let spi;
        // Setup the EPD driver
        let mut e_paper = Epd2in66b::new(
            &mut spi,
            // chip_select_pin,
            is_busy_pin,
            data_or_command_pin,
            reset_pin,
            &mut delay,
            None,
        )
        .unwrap();
        Self {
            address: [0; 4],
            label: [0; 11],
            freespace: 0,
            delay,
            spi,
            e_paper,
        }
    }

    pub async fn run(&mut self) -> ! {
        Timer::after_secs(5).await;
        loop {
            match SIGNAL.wait().await {
                crate::DisplayState::Address(address) => {
                    self.address.copy_from_slice(&address);
                }
                crate::DisplayState::FileSystem(label, freespace) => {
                    self.label.copy_from_slice(&label);
                    self.freespace = freespace;
                }
            }
            let display = self.drawing();

            // Send the Display buffer to the ePaper RAM
            self.e_paper
                .update_color_frame(
                    &mut self.spi,
                    &mut self.delay,
                    &display.bw_buffer(),
                    &display.chromatic_buffer(),
                )
                .unwrap();

            // Render the ePaper RAM - takes time.
            self.e_paper
                .display_frame(&mut self.spi, &mut self.delay)
                .unwrap();

            // Always turn off your EPD as much as possible - ePaper wears out while powered on.
            self.e_paper.sleep(&mut self.spi, &mut self.delay).unwrap();
            // TODO update display
            Timer::after_millis(100).await;
        }
    }

    fn drawing(&self) -> Display2in66b {
        use embedded_graphics::{
            mono_font::{ascii::FONT_10X20, MonoTextStyle},
            prelude::*,
            primitives::PrimitiveStyle,
            text::{Alignment, Text},
        };

        // Create a Display buffer to draw on, specific for this ePaper
        let mut display = Display2in66b::default();

        // Landscape mode, USB plug to the right
        display.set_rotation(DisplayRotation::Rotate270);

        // Change the background from the default black to white
        let _ = display
            .bounding_box()
            .into_styled(PrimitiveStyle::with_fill(TriColor::White))
            .draw(&mut display);

        // Draw some text on the buffer
        let text = "Pico-ePaper-2.66 B/W/R";
        Text::with_alignment(
            text,
            display.bounding_box().center() + Point::new(1, 0),
            MonoTextStyle::new(&FONT_10X20, TriColor::Black),
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
        Text::with_alignment(
            text,
            display.bounding_box().center() + Point::new(0, 1),
            MonoTextStyle::new(&FONT_10X20, TriColor::Chromatic),
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        display
    }
}
