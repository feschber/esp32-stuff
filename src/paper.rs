//! Freehand drawing on the GDEQ0426T82 panel with its FT6336U touch overlay.
//!
//! Everything is in portrait, which is the orientation the panel's artwork
//! reads in and the one the touch controller reports, so screen and touch
//! coordinates are the same thing.
//!
//! A partial refresh costs a fixed ~614ms on this panel regardless of how small
//! a window it is given, so strokes can only ever appear about 1.4 times a
//! second. The loop below is built around that: points keep accumulating while
//! the panel is busy and land together on the next update, so the drawing lags
//! the finger but never loses any of it.

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};
use esp_idf_svc::hal::{delay::FreeRtos, gpio::InputPin, prelude::Peripherals};
use std::{sync::mpsc, thread, time::Duration};

use crate::{
    ft6336::{Ft6336, Touch},
    gdeq0426t82::{Bank, Epd, FrameBuffer, Refresh, SCREEN_WIDTH},
};

/// Comfortably faster than the controller's own scan period, so no report is
/// missed. Needs CONFIG_FREERTOS_HZ=1000 to mean anything below 10ms.
const POLL_INTERVAL_MS: u32 = 5;

const STROKE_WIDTH: u32 = 3;

/// Partial refreshes leave residue behind, so a full one has to happen
/// eventually. It flashes for ~1.7s, so it waits for the pen to lift rather
/// than interrupting a stroke.
const PARTIALS_BEFORE_FULL: u32 = 12;

/// How long to leave the panel's rails up after the last stroke. Raising them
/// costs ~84ms on the next refresh, so riding through the pauses in the middle
/// of drawing is worth it; leaving them up indefinitely fades the image.
const IDLE_POWER_OFF_MS: u64 = 3_000;

/// Whether strokes use the custom waveform from [`FAST_LUT_FRAMES`] instead of
/// the one in OTP. It is roughly four times faster, but a single unidirectional
/// phase has no DC balance the way OTP's oscillating waveform does, so residue
/// builds up faster than the periodic full refresh can clear it and old strokes
/// keep showing through. Flip this to experiment.
const FAST_STROKES: bool = false;

/// Frames the custom stroke waveform drives for. The panel runs 19.65ms per
/// frame, so this is roughly a 100ms refresh against OTP's 393ms. Short enough
/// that a stroke lands well short of full black; the periodic full refresh is
/// what takes it the rest of the way and clears the residue.
const FAST_LUT_FRAMES: u8 = 5;


const CLEAR_AT: Point = Point::new(SCREEN_WIDTH as i32 - 100, 20);
const CLEAR_SIZE: Size = Size::new(80, 56);

/// What the touch thread reports.
enum Ink {
    /// The finger is here; join it to wherever it was last seen.
    At(Point),
    /// The finger left the glass, so the next point starts a new stroke.
    Lifted,
    /// The clear button was pressed.
    Clear,
}

pub(crate) fn run() {
    if let Err(e) = try_run() {
        log::error!("paper: {e:?}");
    }
}

fn try_run() -> anyhow::Result<()> {
    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;

    let mut epd = Epd::new(
        peripherals.spi3,
        pins.gpio18, // SCLK
        pins.gpio23, // MOSI
        pins.gpio27, // CS
        pins.gpio14, // DC
        pins.gpio12, // RST
        pins.gpio13, // BUSY
    )?;
    let touch = Ft6336::new(peripherals.i2c0, pins.gpio32, pins.gpio33, pins.gpio36)?;

    epd.init()?;
    epd.fill(0xFF)?;
    log::info!("cleared in {}ms", epd.refresh(Refresh::Full)?);

    // The canvas goes into both of the controller's banks, so that the partial
    // refreshes below have a correct starting point to diff against.
    let mut frame = FrameBuffer::new();
    draw_chrome(&mut frame);
    frame.flush(&mut epd, Bank::Both)?;
    log::info!("canvas in {}ms", epd.refresh(Refresh::Full)?);
    if FAST_STROKES {
        epd.load_fast_lut(FAST_LUT_FRAMES)?;
    }

    // Touch polling runs on its own thread: a refresh blocks on BUSY for
    // ~614ms, and everything drawn in that window would otherwise be dropped.
    let (ink, strokes) = mpsc::channel();
    thread::Builder::new()
        .stack_size(4096)
        .spawn(move || poll_touch(touch, &ink))?;

    let mut pen = None;
    let mut partials = 0;
    loop {
        // Take whatever is already waiting; if nothing is, the burst is over,
        // so drop the panel's rails before settling in to wait. Leaving them up
        // is what makes a burst fast, but it fades the image if left standing.
        let first = match strokes.recv_timeout(Duration::from_millis(IDLE_POWER_OFF_MS)) {
            Ok(ink) => ink,
            // Drawing has actually stopped, not just paused mid-stroke.
            Err(mpsc::RecvTimeoutError::Timeout) => {
                epd.power_off()?;
                let Ok(ink) = strokes.recv() else {
                    return Ok(()); // touch thread is gone
                };
                ink
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => return Ok(()),
        };
        let mut state = apply(&mut pen, first, &mut frame);
        while let Ok(next) = strokes.try_recv() {
            state = state.max(apply(&mut pen, next, &mut frame));
        }

        match state {
            Update::Cleared => {
                frame.flush(&mut epd, Bank::Both)?;
                log::info!("cleared in {}ms", epd.refresh(Refresh::Full)?);
                if FAST_STROKES {
                    epd.load_fast_lut(FAST_LUT_FRAMES)?;
                }
                partials = 0;
            }
            // Take the flash now that the pen is up rather than mid-stroke.
            Update::Lifted if partials >= PARTIALS_BEFORE_FULL => {
                frame.flush(&mut epd, Bank::Both)?;
                log::info!("de-ghosted in {}ms", epd.refresh(Refresh::Full)?);
                if FAST_STROKES {
                    epd.load_fast_lut(FAST_LUT_FRAMES)?;
                }
                partials = 0;
            }
            _ => {
                frame.flush(&mut epd, Bank::Current)?;
                let mode = if FAST_STROKES {
                    Refresh::PartialFast
                } else {
                    Refresh::Partial
                };
                log::info!("stroke in {}ms", epd.refresh(mode)?);
                partials += 1;
            }
        }
    }
}

/// What the batch of events just applied to the canvas asks of the panel,
/// ordered so that the most demanding one in a batch wins.
#[derive(PartialEq, Eq, PartialOrd, Ord)]
enum Update {
    Drawn,
    Lifted,
    Cleared,
}

fn apply(pen: &mut Option<Point>, ink: Ink, frame: &mut FrameBuffer) -> Update {
    // `FrameBuffer`'s draw error is Infallible, so none of these can fail.
    match ink {
        Ink::At(point) => {
            match *pen {
                Some(from) => {
                    let _ = Line::new(from, point)
                        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, STROKE_WIDTH))
                        .draw(frame);
                }
                // Pen-down: a stroke that is only one point long still has to
                // leave a mark, and the same width as the line that may follow.
                None => {
                    let _ = Circle::with_center(point, STROKE_WIDTH)
                        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                        .draw(frame);
                }
            }
            *pen = Some(point);
            Update::Drawn
        }
        Ink::Lifted => {
            *pen = None;
            Update::Lifted
        }
        Ink::Clear => {
            *pen = None;
            frame.clear_white();
            draw_chrome(frame);
            Update::Cleared
        }
    }
}

/// The one piece of fixed furniture on the canvas.
fn draw_chrome(frame: &mut FrameBuffer) {
    let button = Rectangle::new(CLEAR_AT, CLEAR_SIZE);
    let _ = button
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
        .draw(frame);
    let label = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let centre = button.center();
    let _ = Text::with_alignment(
        "clear",
        Point::new(centre.x, centre.y + 7),
        label,
        Alignment::Center,
    )
    .draw(frame);
}

fn poll_touch<INT: InputPin>(mut touch: Ft6336<'static, INT>, ink: &mpsc::Sender<Ink>) {
    let mut was_down = false;
    // Set when a stroke began on the clear button, so that dragging off it does
    // not leave a trail behind.
    let mut swallow = false;

    loop {
        let touches = match touch.read() {
            Ok(touches) => touches,
            Err(e) => {
                log::warn!("ft6336: read failed: {e}");
                Vec::new()
            }
        };

        let point = touches.first().map(screen_point);
        let is_down = point.is_some();
        // Act on the moment a finger lands, not on every scan while it rests
        // there. Polling well inside the controller's scan period makes this
        // edge dependable: a tap or a gap between taps would have to be under
        // ~14ms to slip past, and a finger cannot move that fast.
        let pressed = is_down && !was_down;
        let released = !is_down && was_down;
        was_down = is_down;

        let event = match point {
            Some(point) if pressed && Rectangle::new(CLEAR_AT, CLEAR_SIZE).contains(point) => {
                swallow = true;
                Some(Ink::Clear)
            }
            Some(point) if !swallow => Some(Ink::At(point)),
            _ if released => {
                swallow = false;
                Some(Ink::Lifted)
            }
            _ => None,
        };

        if let Some(event) = event {
            if ink.send(event).is_err() {
                return; // render loop is gone
            }
        }
        FreeRtos::delay_ms(POLL_INTERVAL_MS);
    }
}

/// The touch controller already reports portrait screen coordinates, so this is
/// just a change of type.
fn screen_point(touch: &Touch) -> Point {
    Point::new(touch.x as i32, touch.y as i32)
}
