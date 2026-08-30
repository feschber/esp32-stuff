//! A QR code as an `embedded-graphics` [`Drawable`], so the same code can go to
//! any monochrome target: the OLED, or the e-paper's [`crate::gdeq0426t82::FrameBuffer`].
//!
//! Drawing does not flush anything -- the caller decides when the target is
//! pushed to its display.

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use qrcode::{EcLevel, QrCode};

/// Blank modules the spec requires around a code for scanners to lock onto it.
const QUIET_ZONE: u32 = 4;

/// Not currently drawn by the demo -- the canvas is bare. Kept because it is
/// display-agnostic and cheap to wire back in.
#[allow(dead_code)]
pub struct QrImage {
    /// Row-major, `width` entries per row; true is a dark module.
    modules: Vec<bool>,
    width: u32,
    /// Pixels per module.
    scale: u32,
    top_left: Point,
}

#[allow(dead_code)]
impl QrImage {
    /// Encodes `data` at the largest whole-pixel module size that fits `area`,
    /// centred within it, quiet zone included.
    ///
    /// Pass `target.bounding_box()` for `area` to fill the whole display.
    pub fn fit(data: &str, area: Rectangle) -> anyhow::Result<Self> {
        // Low error correction keeps the code small, which matters when a
        // module has to land on whole pixels of a small display.
        let code = QrCode::with_error_correction_level(data.as_bytes(), EcLevel::L)
            .map_err(|e| anyhow::anyhow!("encoding {} bytes as a QR code: {e}", data.len()))?;
        let width = code.width() as u32;
        let modules = code
            .into_colors()
            .into_iter()
            .map(|color| color.select(true, false))
            .collect();

        let span = width + 2 * QUIET_ZONE;
        let scale = (area.size.width.min(area.size.height) / span).max(1);
        let side = (span * scale) as i32;
        let top_left = area.top_left
            + Point::new(
                (area.size.width as i32 - side) / 2,
                (area.size.height as i32 - side) / 2,
            );

        Ok(Self {
            modules,
            width,
            scale,
            top_left,
        })
    }

    /// Side length in pixels, quiet zone included.
    pub fn side(&self) -> u32 {
        (self.width + 2 * QUIET_ZONE) * self.scale
    }
}

impl Drawable for QrImage {
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        // Clear the whole footprint first: the quiet zone has to be blank
        // whatever was underneath, and it lets the loop below skip every light
        // module rather than drawing it.
        Rectangle::new(self.top_left, Size::new(self.side(), self.side()))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(target)?;

        let inset = (QUIET_ZONE * self.scale) as i32;
        let origin = self.top_left + Point::new(inset, inset);
        let dark = PrimitiveStyle::with_fill(BinaryColor::On);

        for (i, is_dark) in self.modules.iter().enumerate() {
            if !is_dark {
                continue;
            }
            let i = i as u32;
            let at = origin
                + Point::new(
                    (i % self.width * self.scale) as i32,
                    (i / self.width * self.scale) as i32,
                );
            Rectangle::new(at, Size::new(self.scale, self.scale))
                .into_styled(dark)
                .draw(target)?;
        }
        Ok(())
    }
}
