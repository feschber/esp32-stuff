//! Demo for the GDEQ0426T82 e-paper panel with its FT6336U touch overlay:
//! a counter with a button either side of it.
//!
//! Everything is drawn in portrait, which is the orientation the panel's own
//! artwork reads in and the one the touch controller reports, so screen and
//! touch coordinates are the same thing.

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};
use esp_idf_svc::hal::{delay::FreeRtos, gpio::InputPin, prelude::Peripherals};
use std::{
    sync::{
        atomic::{AtomicU8, Ordering},
        mpsc, Arc,
    },
    thread,
};

use crate::{
    ft6336::{Ft6336, Touch},
    gdeq0426t82::{Bank, Epd, FrameBuffer, Refresh, SCREEN_HEIGHT, SCREEN_WIDTH},
};

/// Comfortably faster than the controller's own scan period, so no report is
/// missed. Needs CONFIG_FREERTOS_HZ=1000 to mean anything below 10ms.
const POLL_INTERVAL_MS: u32 = 5;

// Portrait layout, 480 wide by 800 tall.
const BUTTON_SIZE: Size = Size::new(120, 120);
const MINUS_AT: Point = Point::new(60, 560);
const PLUS_AT: Point = Point::new(300, 560);
const COUNTER_AT: Point = Point::new(160, 260);
const COUNTER_SIZE: Size = Size::new(160, 160);

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

    // Clear whatever the panel was showing, then put the interface up. The
    // first image goes into both of the controller's banks, so that the partial
    // refreshes below have a correct starting point to diff against.
    epd.init()?;
    epd.fill(0xFF)?;
    log::info!("cleared in {}ms", epd.refresh(Refresh::Full)?);

    let counter: u8 = 0;
    let mut frame = FrameBuffer::new();
    draw_ui(&mut frame, counter);
    frame.flush(&mut epd, Bank::Both)?;
    log::info!("base image in {}ms", epd.refresh(Refresh::Full)?);

    // Touch polling runs on its own thread: a refresh blocks on BUSY for
    // ~600ms, and taps that land in that window would otherwise be dropped.
    let counter = Arc::new(AtomicU8::new(counter));
    let (changes, change) = mpsc::channel();
    thread::Builder::new().stack_size(4096).spawn({
        let counter = Arc::clone(&counter);
        move || poll_touch(touch, &counter, &changes)
    })?;

    let mut shown = 0;
    loop {
        // Block until something changed, then swallow everything else that has
        // queued up: the panel can only ever show the newest value, so a burst
        // of taps during a refresh collapses into one redraw.
        if change.recv().is_err() {
            break Ok(());
        }
        while change.try_recv().is_ok() {}

        let value = counter.load(Ordering::Relaxed);
        if value == shown {
            continue;
        }
        draw_ui(&mut frame, value);
        frame.flush(&mut epd, Bank::Current)?;
        log::info!("counter {value} in {}ms", epd.refresh(Refresh::Partial)?);
        shown = value;
    }
}

/// Polls the controller and folds button presses into `counter`, waking the
/// render loop through `changes` whenever it moves.
fn poll_touch<INT: InputPin>(
    mut touch: Ft6336<'static, INT>,
    counter: &AtomicU8,
    changes: &mpsc::Sender<()>,
) {
    let mut was_down = false;
    loop {
        let touches = match touch.read() {
            Ok(touches) => touches,
            Err(e) => {
                log::warn!("ft6336: read failed: {e}");
                Vec::new()
            }
        };

        // Act on the moment a finger lands, not on every scan while it rests
        // there, otherwise holding a button runs the counter away. Polling well
        // inside the controller's scan period makes this edge dependable: a tap
        // or a gap between taps would have to be under ~14ms to slip past, and
        // a finger cannot move that fast.
        let is_down = !touches.is_empty();
        let pressed = is_down && !was_down;
        was_down = is_down;

        if pressed {
            let point = screen_point(&touches[0]);
            log::info!("touch at {point:?}");
            if let Some(step) = button_at(point) {
                counter
                    .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
                        Some((value as i8 + step).rem_euclid(10) as u8)
                    })
                    .ok();
                if changes.send(()).is_err() {
                    return; // render loop is gone
                }
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

/// Returns the step for whichever button contains `point`, if any.
fn button_at(point: Point) -> Option<i8> {
    if Rectangle::new(MINUS_AT, BUTTON_SIZE).contains(point) {
        Some(-1)
    } else if Rectangle::new(PLUS_AT, BUTTON_SIZE).contains(point) {
        Some(1)
    } else {
        None
    }
}

fn draw_ui(frame: &mut FrameBuffer, counter: u8) {
    let outline = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
    let label = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    frame.clear_white();

    // `FrameBuffer`'s draw error is Infallible, so none of these can fail.
    let _ = Text::with_alignment(
        "GDEQ0426T82 + FT6336U",
        Point::new(SCREEN_WIDTH as i32 / 2, 100),
        label,
        Alignment::Center,
    )
    .draw(frame);

    for (at, text) in [(MINUS_AT, "-"), (PLUS_AT, "+")] {
        let button = Rectangle::new(at, BUTTON_SIZE);
        let _ = button.into_styled(outline).draw(frame);
        let _ = Text::with_alignment(text, center_of(button), label, Alignment::Center).draw(frame);
    }

    let counter_box = Rectangle::new(COUNTER_AT, COUNTER_SIZE);
    let _ = counter_box.into_styled(outline).draw(frame);
    let digit = [counter + b'0'];
    let digit = core::str::from_utf8(&digit).unwrap_or("?");
    let _ =
        Text::with_alignment(digit, center_of(counter_box), label, Alignment::Center).draw(frame);

    let _ = Text::with_alignment(
        "tap a button",
        Point::new(SCREEN_WIDTH as i32 / 2, SCREEN_HEIGHT as i32 - 80),
        label,
        Alignment::Center,
    )
    .draw(frame);
}

/// Centre of `rect`, nudged so that a single line of text sits on its middle.
fn center_of(rect: Rectangle) -> Point {
    let center = rect.center();
    Point::new(
        center.x,
        center.y + FONT_10X20.character_size.height as i32 / 3,
    )
}
