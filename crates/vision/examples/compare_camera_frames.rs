use anyhow::{ensure, Context, Result};
use serde_json::json;
use std::path::{Path, PathBuf};
use velocitypnp_vision::{decode_gray_png, encode_gray_png, GrayFrame};

const SEPARATOR_PX: u32 = 8;
const DATUM_CROP_SIZE_PX: u32 = 320;

fn main() -> Result<()> {
    let arguments = std::env::args_os().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 3,
        "usage: compare_camera_frames YUYV.png MJPEG.png OUTPUT_DIRECTORY"
    );
    let yuyv_path = PathBuf::from(&arguments[0]);
    let mjpeg_path = PathBuf::from(&arguments[1]);
    let output_directory = PathBuf::from(&arguments[2]);
    let yuyv = read_gray_png(&yuyv_path)?;
    let mjpeg = read_gray_png(&mjpeg_path)?;
    ensure!(
        yuyv.width == mjpeg.width && yuyv.height == mjpeg.height,
        "camera frames have different dimensions"
    );

    std::fs::create_dir_all(&output_directory)
        .with_context(|| format!("creating {}", output_directory.display()))?;
    write_gray_png(
        &output_directory.join("full-side-by-side.png"),
        &side_by_side(&yuyv, &mjpeg)?,
    )?;
    let crop_size = DATUM_CROP_SIZE_PX.min(yuyv.width).min(yuyv.height);
    let crop_x = (yuyv.width - crop_size) / 2;
    let crop_y = (yuyv.height - crop_size) / 2;
    let yuyv_crop = crop(&yuyv, crop_x, crop_y, crop_size, crop_size)?;
    let mjpeg_crop = crop(&mjpeg, crop_x, crop_y, crop_size, crop_size)?;
    write_gray_png(
        &output_directory.join("datum-crop-side-by-side.png"),
        &side_by_side(&yuyv_crop, &mjpeg_crop)?,
    )?;

    let differences = yuyv
        .pixels
        .iter()
        .zip(&mjpeg.pixels)
        .map(|(left, right)| left.abs_diff(*right))
        .collect::<Vec<_>>();
    let difference = GrayFrame {
        width: yuyv.width,
        height: yuyv.height,
        pixels: differences
            .iter()
            .map(|difference| difference.saturating_mul(4))
            .collect(),
    };
    write_gray_png(
        &output_directory.join("absolute-difference-4x.png"),
        &difference,
    )?;

    let pixel_count = differences.len() as f64;
    let mean_absolute_difference = differences
        .iter()
        .map(|value| f64::from(*value))
        .sum::<f64>()
        / pixel_count;
    let root_mean_square_difference = (differences
        .iter()
        .map(|value| f64::from(*value).powi(2))
        .sum::<f64>()
        / pixel_count)
        .sqrt();
    let mut ordered = differences.clone();
    ordered.sort_unstable();
    let p95_index = ((ordered.len() * 95).div_ceil(100)).saturating_sub(1);
    let report = json!({
        "left": {
            "label": "YUYV 1280x720 at 10 FPS",
            "source": yuyv_path,
        },
        "right": {
            "label": "MJPEG 1280x720 at 30 FPS",
            "source": mjpeg_path,
        },
        "width_px": yuyv.width,
        "height_px": yuyv.height,
        "datum_crop": {
            "x_px": crop_x,
            "y_px": crop_y,
            "width_px": crop_size,
            "height_px": crop_size,
        },
        "raw_pixel_difference": {
            "mean_absolute_luma": mean_absolute_difference,
            "root_mean_square_luma": root_mean_square_difference,
            "p95_absolute_luma": ordered[p95_index],
            "maximum_absolute_luma": ordered[ordered.len() - 1],
        },
        "comparison_images": {
            "full": "full-side-by-side.png",
            "datum_crop": "datum-crop-side-by-side.png",
            "absolute_difference_4x": "absolute-difference-4x.png",
        },
    });
    std::fs::write(
        output_directory.join("comparison.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn read_gray_png(path: &Path) -> Result<GrayFrame> {
    let encoded = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    decode_gray_png(&encoded).with_context(|| format!("decoding {}", path.display()))
}

fn write_gray_png(path: &Path, frame: &GrayFrame) -> Result<()> {
    let encoded = encode_gray_png(frame)?;
    std::fs::write(path, encoded).with_context(|| format!("writing {}", path.display()))
}

fn side_by_side(left: &GrayFrame, right: &GrayFrame) -> Result<GrayFrame> {
    ensure!(
        left.width == right.width && left.height == right.height,
        "side-by-side frames have different dimensions"
    );
    let width = left
        .width
        .checked_mul(2)
        .and_then(|width| width.checked_add(SEPARATOR_PX))
        .context("side-by-side width overflow")?;
    let mut pixels = vec![0; width as usize * left.height as usize];
    for y in 0..left.height as usize {
        let output_row = y * width as usize;
        let input_row = y * left.width as usize;
        let input_end = input_row + left.width as usize;
        pixels[output_row..output_row + left.width as usize]
            .copy_from_slice(&left.pixels[input_row..input_end]);
        let right_start = output_row + left.width as usize + SEPARATOR_PX as usize;
        pixels[right_start..right_start + right.width as usize]
            .copy_from_slice(&right.pixels[input_row..input_end]);
    }
    Ok(GrayFrame {
        width,
        height: left.height,
        pixels,
    })
}

fn crop(frame: &GrayFrame, x: u32, y: u32, width: u32, height: u32) -> Result<GrayFrame> {
    ensure!(
        x.checked_add(width)
            .is_some_and(|right| right <= frame.width)
            && y.checked_add(height)
                .is_some_and(|bottom| bottom <= frame.height),
        "crop is outside the source frame"
    );
    let mut pixels = Vec::with_capacity(width as usize * height as usize);
    for row in y..y + height {
        let start = row as usize * frame.width as usize + x as usize;
        pixels.extend_from_slice(&frame.pixels[start..start + width as usize]);
    }
    Ok(GrayFrame {
        width,
        height,
        pixels,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(width: u32, height: u32, pixels: &[u8]) -> GrayFrame {
        GrayFrame {
            width,
            height,
            pixels: pixels.to_vec(),
        }
    }

    #[test]
    fn side_by_side_keeps_source_pixels_at_native_scale() {
        let left = frame(2, 2, &[1, 2, 3, 4]);
        let right = frame(2, 2, &[5, 6, 7, 8]);

        let combined = side_by_side(&left, &right).unwrap();

        assert_eq!(combined.width, 2 + SEPARATOR_PX + 2);
        assert_eq!(combined.height, 2);
        assert_eq!(&combined.pixels[0..2], &[1, 2]);
        assert_eq!(
            &combined.pixels[(2 + SEPARATOR_PX) as usize..(4 + SEPARATOR_PX) as usize],
            &[5, 6]
        );
    }

    #[test]
    fn crop_keeps_the_requested_native_pixels() {
        let source = frame(4, 3, &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);

        let cropped = crop(&source, 1, 1, 2, 2).unwrap();

        assert_eq!((cropped.width, cropped.height), (2, 2));
        assert_eq!(cropped.pixels, [5, 6, 9, 10]);
    }
}
