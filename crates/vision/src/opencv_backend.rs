use crate::GrayFrame;
use anyhow::{ensure, Context, Result};
use opencv::core::{self, Mat, MatTraitConst, Scalar, CV_32S, CV_64F, CV_8UC1};
use opencv::imgproc;
use opencv::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ThresholdPolarity {
    Dark,
    Bright,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrameStatistics {
    pub mean_luma: f64,
    pub luma_standard_deviation: f64,
    pub laplacian_variance: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ConnectedComponent {
    pub label: i32,
    pub area: usize,
    pub left: u32,
    pub top: u32,
    pub width: u32,
    pub height: u32,
    pub centroid_x: f64,
    pub centroid_y: f64,
}

pub(crate) struct ConnectedComponents {
    pub components: Vec<ConnectedComponent>,
    labels: Mat,
}

impl ConnectedComponents {
    pub fn points(&self, component: ConnectedComponent) -> Result<Vec<[f64; 2]>> {
        let mut points = Vec::with_capacity(component.area);
        for y in component.top..component.top + component.height {
            for x in component.left..component.left + component.width {
                if *self.labels.at_2d::<i32>(y as i32, x as i32)? == component.label {
                    points.push([f64::from(x), f64::from(y)]);
                }
            }
        }
        Ok(points)
    }
}

pub fn gray_frame_to_mat(frame: &GrayFrame) -> Result<Mat> {
    ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize,
        "gray frame buffer does not match its dimensions"
    );
    ensure!(
        frame.width <= i32::MAX as u32 && frame.height <= i32::MAX as u32,
        "gray frame dimensions exceed OpenCV limits"
    );
    let row = Mat::from_slice(&frame.pixels).context("creating OpenCV gray buffer")?;
    let reshaped = row
        .reshape(1, frame.height as i32)
        .context("reshaping OpenCV gray frame")?;
    reshaped
        .try_clone()
        .context("owning OpenCV gray frame storage")
}

pub fn gray_frame_from_mat(mat: &Mat) -> Result<GrayFrame> {
    ensure!(mat.typ() == CV_8UC1, "OpenCV frame must be 8-bit grayscale");
    ensure!(
        mat.rows() >= 0 && mat.cols() >= 0,
        "OpenCV frame size is invalid"
    );
    let owned = mat.try_clone().context("copying OpenCV gray frame")?;
    let pixels = owned
        .data_typed::<u8>()
        .context("reading OpenCV gray pixels")?
        .to_vec();
    Ok(GrayFrame {
        width: owned.cols() as u32,
        height: owned.rows() as u32,
        pixels,
    })
}

pub(crate) fn frame_statistics(frame: &GrayFrame) -> Result<FrameStatistics> {
    ensure!(
        frame.width >= 3 && frame.height >= 3,
        "gray frame is too small"
    );
    let mat = gray_frame_to_mat(frame)?;
    let mut luma_mean = Scalar::default();
    let mut luma_standard_deviation = Scalar::default();
    core::mean_std_dev(
        &mat,
        &mut luma_mean,
        &mut luma_standard_deviation,
        &core::no_array(),
    )?;

    let mut laplacian = Mat::default();
    imgproc::laplacian(
        &mat,
        &mut laplacian,
        CV_64F,
        1,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let interior = Mat::roi(
        &laplacian,
        core::Rect::new(1, 1, mat.cols() - 2, mat.rows() - 2),
    )?;
    let mut laplacian_mean = Scalar::default();
    let mut laplacian_standard_deviation = Scalar::default();
    core::mean_std_dev(
        &interior,
        &mut laplacian_mean,
        &mut laplacian_standard_deviation,
        &core::no_array(),
    )?;

    Ok(FrameStatistics {
        mean_luma: luma_mean[0],
        luma_standard_deviation: luma_standard_deviation[0],
        laplacian_variance: laplacian_standard_deviation[0].powi(2),
    })
}

pub(crate) fn connected_components(
    frame: &GrayFrame,
    threshold: u8,
    polarity: ThresholdPolarity,
) -> Result<ConnectedComponents> {
    let mat = gray_frame_to_mat(frame)?;
    let mut mask = Mat::default();
    let comparison = match polarity {
        ThresholdPolarity::Dark => core::CMP_LE,
        ThresholdPolarity::Bright => core::CMP_GE,
    };
    core::compare(
        &mat,
        &Scalar::all(f64::from(threshold)),
        &mut mask,
        comparison,
    )?;
    let mut labels = Mat::default();
    let mut stats = Mat::default();
    let mut centroids = Mat::default();
    let count = imgproc::connected_components_with_stats(
        &mask,
        &mut labels,
        &mut stats,
        &mut centroids,
        4,
        CV_32S,
    )?;
    let mut components = Vec::with_capacity(count.saturating_sub(1) as usize);
    for label in 1..count {
        components.push(ConnectedComponent {
            label,
            left: *stats.at_2d::<i32>(label, imgproc::CC_STAT_LEFT)? as u32,
            top: *stats.at_2d::<i32>(label, imgproc::CC_STAT_TOP)? as u32,
            width: *stats.at_2d::<i32>(label, imgproc::CC_STAT_WIDTH)? as u32,
            height: *stats.at_2d::<i32>(label, imgproc::CC_STAT_HEIGHT)? as u32,
            area: *stats.at_2d::<i32>(label, imgproc::CC_STAT_AREA)? as usize,
            centroid_x: *centroids.at_2d::<f64>(label, 0)?,
            centroid_y: *centroids.at_2d::<f64>(label, 1)?,
        });
    }
    Ok(ConnectedComponents { components, labels })
}
