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
    gdeq0426t82::{Bank, Charge, Epd, FrameBuffer, InkTarget, Refresh, MAX_CHARGE, SCREEN_WIDTH},
};

/// The controller produces new data every ~9ms (~110Hz, its documented
/// ceiling), so polling at 5ms oversamples it about twofold and no report goes
/// unseen. Faster buys nothing; measured at 1ms, only 10% of reads returned
/// anything new. Needs CONFIG_FREERTOS_HZ=1000 to mean anything below 10ms.
const POLL_INTERVAL_MS: u32 = 5;

const STROKE_WIDTH: u32 = 6;

/// Partial refreshes leave residue behind, so a full one has to happen
/// eventually. It flashes for ~1.7s, so it waits for the pen to lift rather
/// than interrupting a stroke.
const PARTIALS_BEFORE_FULL: u32 = 120;

/// How long to leave the panel's rails up after the last stroke. Raising them
/// costs ~84ms on the next refresh, so riding through the pauses in the middle
/// of drawing is worth it; leaving them up indefinitely fades the image.
const IDLE_POWER_OFF_MS: u64 = 3_000;

/// Which waveform strokes are drawn with. Swap [`STROKES`] to compare them.
#[derive(PartialEq, Eq)]
#[allow(dead_code)]
enum Strokes {
    /// The waveform from OTP. Slowest at 393ms, but properly DC balanced.
    Otp,
    /// One flat phase driving every pixel at its target. Fast, but it has to
    /// bypass the previous image, so the paper is driven too and residue builds
    /// up faster than the periodic full refresh clears it.
    Flat,
    /// Fresh strokes driven hard, older ones topped up a little, paper left
    /// alone -- all in one pass. See [`Epd::load_hybrid_lut`].
    Hybrid,
}

const STROKES: Strokes = Strokes::Hybrid;

/// Frames a pixel is driven for on each pass it is presented as a transition.
/// Total convergence is `HYBRID_FRAMES * HEAL_PASSES`, so spreading a small
/// per-pass drive over more passes keeps every individual refresh cheap while
/// still reaching black.
const HYBRID_FRAMES: u8 = 1;

/// Frames an already-drawn pixel is topped up by on each later pass, via LUT1's
/// weak drive. Deliberately much smaller than [`HYBRID_FRAMES`] so the bulk of a
/// stroke's darkening happens in the pass it is drawn: splitting the drive into
/// equal thirds is what read as flicker, because each pixel visibly stepped
/// darker three times as the pen moved on.
///
/// The refresh still costs [`HYBRID_FRAMES`] either way -- phase lengths are
/// shared, and the top-up rides inside phase A, which the fresh pixels are being
/// driven for anyway. So healing is free in time.
const HYBRID_HEAL_FRAMES: u8 = 1;

/// Total frames the drawing path spends pushing one pixel towards black, and so
/// the amount that has to be given back the other way when it is erased.
const HYBRID_TOTAL_FRAMES: u8 = HYBRID_FRAMES + (HEAL_PASSES as u8 - 1) * HYBRID_HEAL_FRAMES;

/// How many consecutive refreshes a freshly drawn pixel keeps being driven for.
/// Bounded by the two bits [`Charge`] gives each pixel.
///
/// A pixel still healing needs its transition re-asserted on every pass, since a
/// Mode 2 refresh copies current into previous when it finishes -- so the region
/// written has to cover everything still charged, not just the new geometry.
const HEAL_PASSES: usize = MAX_CHARGE as usize;

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
    load_stroke_lut(&mut epd)?;

    // Touch polling runs on its own thread: a refresh blocks on BUSY for
    // ~614ms, and everything drawn in that window would otherwise be dropped.
    let (ink, strokes) = mpsc::channel();
    thread::Builder::new()
        .stack_size(4096)
        .spawn(move || poll_touch(touch, &ink))?;

    // How many drives each pixel still has coming. Replaces what used to be one
    // full drive mask per pass; the bounding boxes stay, since they are what
    // keeps the region written small, but they cost bytes rather than 48KB each.
    let mut charge = Charge::new();
    let mut covered: Vec<Option<Rectangle>> = vec![None; HEAL_PASSES];
    let mut newest = 0usize;
    log::info!("heap free after buffers: {} bytes", unsafe {
        esp_idf_svc::sys::esp_get_free_heap_size()
    });

    let mut pen = None;
    let mut partials = 0;
    loop {
        // Take whatever is already waiting; if nothing is, the burst is over,
        // so drop the panel's rails before settling in to wait. Leaving them up
        // is what makes a burst fast, but it fades the image if left standing.
        let mut state = Update::Drawn;
        for _ in 0..2 {
            let pnt = match strokes.recv_timeout(Duration::from_millis(IDLE_POWER_OFF_MS)) {
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
            // `Update` is ordered so the most demanding action in a batch
            // wins: a clear arrives as Clear followed by Lifted on release, and
            // a plain assignment here would throw the clear away.
            let (update, touched) = apply(&mut pen, pnt, &mut frame, &mut charge);
            state = state.max(update);
            if let Some(touched) = touched {
                grow(&mut covered[newest], touched);
            }
        }
        while let Ok(next) = strokes.try_recv() {
            let (update, touched) = apply(&mut pen, next, &mut frame, &mut charge);
            state = state.max(update);
            if let Some(touched) = touched {
                grow(&mut covered[newest], touched);
            }
        }

        match state {
            Update::Cleared => {
                // Give back what the drawing path pushed in. Everything still
                // inked is driven towards white by the same total, so a pixel's
                // charge nets out over a draw-and-erase cycle instead of only
                // ever growing.
                log::info!(
                    "balanced in {}ms",
                    epd.balance_to_white(frame.as_bytes(), HYBRID_TOTAL_FRAMES)?
                );
                frame.clear_white();
                draw_chrome(&mut frame);
                frame.flush(&mut epd, Bank::Both)?;
                log::info!("cleared in {}ms", epd.refresh(Refresh::Full)?);
                load_stroke_lut(&mut epd)?;
                charge.clear();
                covered.iter_mut().for_each(|area| *area = None);
                partials = 0;
            }
            // Take the flash now that the pen is up rather than mid-stroke.
            Update::Lifted if partials >= PARTIALS_BEFORE_FULL => {
                frame.flush(&mut epd, Bank::Both)?;
                log::info!("de-ghosted in {}ms", epd.refresh(Refresh::Full)?);
                load_stroke_lut(&mut epd)?;
                charge.clear();
                covered.iter_mut().for_each(|area| *area = None);
                partials = 0;
            }
            _ => {
                let mode = match STROKES {
                    Strokes::Otp => Refresh::Partial,
                    Strokes::Flat => Refresh::PartialFast,
                    Strokes::Hybrid => Refresh::PartialHybrid,
                };
                // Every pixel still inside the ring needs its transition
                // re-asserted, so the region covers all of them, not just the
                // newly drawn part.
                let mut region = None;
                for area in covered.iter().flatten() {
                    grow(&mut region, *area);
                }
                let Some(area) = region else {
                    continue; // nothing on the canvas moved
                };
                if STROKES == Strokes::Hybrid {
                    epd.flush_charged_area(frame.as_bytes(), &mut charge, area)?;
                } else {
                    frame.flush(&mut epd, Bank::Current)?;
                }
                log::info!(
                    "stroke in {}ms, wrote {}x{}",
                    epd.refresh(mode)?,
                    area.size.width,
                    area.size.height
                );
                partials += 1;
                // Retire the oldest box. Whatever it covered has now spent its
                // last drive, so it no longer has to be rewritten.
                newest = (newest + 1) % HEAL_PASSES;
                covered[newest] = None;
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

/// Grows `area` to also cover `by`.
fn grow(area: &mut Option<Rectangle>, by: Rectangle) {
    if by.is_zero_sized() {
        return;
    }
    *area = Some(
        match (
            *area,
            area.and_then(|a| a.bottom_right()),
            by.bottom_right(),
        ) {
            (Some(a), Some(ac), Some(bc)) => Rectangle::with_corners(
                Point::new(
                    a.top_left.x.min(by.top_left.x),
                    a.top_left.y.min(by.top_left.y),
                ),
                Point::new(ac.x.max(bc.x), ac.y.max(bc.y)),
            ),
            _ => by,
        },
    );
}

/// Returns what the panel now needs, and the region the canvas changed in.
fn apply(
    pen: &mut Option<Point>,
    ink: Ink,
    frame: &mut FrameBuffer,
    charge: &mut Charge,
) -> (Update, Option<Rectangle>) {
    // `FrameBuffer`'s draw error is Infallible, so none of these can fail.
    match ink {
        Ink::At(point) => {
            let mut ink = InkTarget::new(frame, charge);
            let touched = match *pen {
                Some(from) => {
                    let line = Line::new(from, point)
                        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, STROKE_WIDTH));
                    let _ = line.draw(&mut ink);
                    line.bounding_box()
                }
                // Pen-down: a stroke that is only one point long still has to
                // leave a mark, and the same width as the line that may follow.
                None => {
                    let dot = Circle::with_center(point, STROKE_WIDTH)
                        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On));
                    let _ = dot.draw(&mut ink);
                    dot.bounding_box()
                }
            };
            // If the stroke only crossed ink that was already there, nothing
            // changed and there is nothing to refresh.
            let touched = ink.changed.then_some(touched);
            *pen = Some(point);
            (Update::Drawn, touched)
        }
        Ink::Lifted => {
            *pen = None;
            (Update::Lifted, None)
        }
        Ink::Clear => {
            // The canvas is wiped by the render loop, not here: the image about
            // to be erased is needed first, to cancel its charge.
            *pen = None;
            (Update::Cleared, None)
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

/// A full refresh reloads the waveform from OTP, so whichever custom one we are
/// using has to be written again afterwards.
fn load_stroke_lut(epd: &mut Epd<'_>) -> Result<(), esp_idf_svc::sys::EspError> {
    match STROKES {
        Strokes::Otp => Ok(()),
        Strokes::Flat => epd.load_fast_lut(FAST_LUT_FRAMES),
        Strokes::Hybrid => epd.load_graded_lut(HYBRID_HEAL_FRAMES, HYBRID_FRAMES),
    }
}

/// The touch controller already reports portrait screen coordinates, so this is
/// just a change of type.
fn screen_point(touch: &Touch) -> Point {
    Point::new(touch.x as i32, touch.y as i32)
}
