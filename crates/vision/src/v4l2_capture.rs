use crate::GrayFrame;
use anyhow::{bail, ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::io;
use std::path::PathBuf;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use v4l::buffer::{Flags as BufferFlags, Type};
use v4l::control::{Control, Description, Flags, Type as ControlType, Value};
use v4l::io::traits::CaptureStream;
use v4l::prelude::MmapStream;
use v4l::video::Capture;
use v4l::{Device, Format, FourCC, Fraction};
use velocitypnp_domain::CameraPixelFormat;

fn yuyv() -> FourCC {
    FourCC::new(b"YUYV")
}

fn mjpg() -> FourCC {
    FourCC::new(b"MJPG")
}

#[derive(Debug, Clone)]
pub struct V4l2CaptureConfig {
    pub device: PathBuf,
    pub width: u32,
    pub height: u32,
    pub pixel_format: CameraPixelFormat,
    pub frames_per_second: u32,
    pub settle_frames: u32,
    pub timeout: Duration,
}

impl V4l2CaptureConfig {
    pub fn lumenpnp(device: impl Into<PathBuf>) -> Self {
        Self {
            device: device.into(),
            width: 1280,
            height: 720,
            pixel_format: CameraPixelFormat::Yuyv,
            frames_per_second: 10,
            settle_frames: 5,
            timeout: Duration::from_secs(3),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CaptureMetadata {
    pub device: PathBuf,
    pub width: u32,
    pub height: u32,
    pub fourcc: String,
    pub stride: u32,
    pub interval_numerator: u32,
    pub interval_denominator: u32,
    pub sequence: u32,
    pub settle_frames: u32,
    #[serde(default)]
    pub timestamp_monotonic: bool,
    #[serde(default)]
    pub timestamp_source: CaptureTimestampSource,
    #[serde(default)]
    pub fresh_after_boundary: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureTimestampSource {
    EndOfFrame,
    StartOfExposure,
    #[default]
    Unknown,
}

impl CaptureTimestampSource {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::EndOfFrame => "end_of_frame",
            Self::StartOfExposure => "start_of_exposure",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureFreshnessBoundary {
    pub exposure_started_after: Duration,
    pub end_of_frame_exposure: Duration,
    pub unknown_source_guard: Duration,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapturedGrayFrame {
    pub frame: GrayFrame,
    pub metadata: CaptureMetadata,
}

enum PersistentCaptureRequest {
    Capture {
        settle_frames: u32,
        response: mpsc::SyncSender<Result<CapturedGrayFrame, String>>,
    },
    CaptureFreshAfter {
        boundary: CaptureFreshnessBoundary,
        fallback_settle_frames: u32,
        maximum_frames: u32,
        response: mpsc::SyncSender<Result<CapturedGrayFrame, String>>,
    },
    Stop,
}

/// A V4L2 stream that stays open across machine moves.
pub struct PersistentGrayCapture {
    requests: mpsc::Sender<PersistentCaptureRequest>,
    worker: Option<thread::JoinHandle<()>>,
}

impl PersistentGrayCapture {
    pub fn open(config: V4l2CaptureConfig) -> Result<Self> {
        validate_capture_config(&config)?;
        let (request_tx, request_rx) = mpsc::channel();
        let (ready_tx, ready_rx) = mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("velocitypnp-v4l2-capture".into())
            .spawn(move || persistent_capture_worker(config, request_rx, ready_tx))
            .context("starting persistent camera capture thread")?;
        match ready_rx.recv() {
            Ok(Ok(())) => Ok(Self {
                requests: request_tx,
                worker: Some(worker),
            }),
            Ok(Err(error)) => {
                let _ = worker.join();
                Err(anyhow::anyhow!(error))
            }
            Err(_) => {
                let _ = worker.join();
                bail!("persistent camera thread stopped during setup")
            }
        }
    }

    pub fn capture(&self, settle_frames: u32) -> Result<CapturedGrayFrame> {
        ensure!(
            settle_frames <= 120,
            "camera settling frame count is excessive"
        );
        let (response_tx, response_rx) = mpsc::sync_channel(1);
        self.requests
            .send(PersistentCaptureRequest::Capture {
                settle_frames,
                response: response_tx,
            })
            .context("requesting a persistent camera frame")?;
        response_rx
            .recv()
            .context("persistent camera thread stopped during capture")?
            .map_err(anyhow::Error::msg)
    }

    pub fn capture_fresh_after(
        &self,
        boundary: CaptureFreshnessBoundary,
        fallback_settle_frames: u32,
        maximum_frames: u32,
    ) -> Result<CapturedGrayFrame> {
        ensure!(
            fallback_settle_frames <= 120,
            "camera fallback settling frame count is excessive"
        );
        ensure!(
            (1..=120).contains(&maximum_frames),
            "fresh camera frame limit must be between 1 and 120"
        );
        let (response_tx, response_rx) = mpsc::sync_channel(1);
        self.requests
            .send(PersistentCaptureRequest::CaptureFreshAfter {
                boundary,
                fallback_settle_frames,
                maximum_frames,
                response: response_tx,
            })
            .context("requesting a fresh persistent camera frame")?;
        response_rx
            .recv()
            .context("persistent camera thread stopped during fresh capture")?
            .map_err(anyhow::Error::msg)
    }
}

pub fn monotonic_timestamp() -> Result<Duration> {
    let mut timestamp = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let result = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut timestamp) };
    if result != 0 {
        return Err(io::Error::last_os_error()).context("reading the monotonic clock");
    }
    ensure!(
        timestamp.tv_sec >= 0 && (0..1_000_000_000).contains(&timestamp.tv_nsec),
        "monotonic clock returned an invalid timestamp"
    );
    Ok(Duration::new(
        timestamp.tv_sec as u64,
        timestamp.tv_nsec as u32,
    ))
}

impl Drop for PersistentGrayCapture {
    fn drop(&mut self) {
        let _ = self.requests.send(PersistentCaptureRequest::Stop);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum CameraControlValue {
    Integer(i64),
    Boolean(bool),
    Text(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CameraControlInfo {
    pub id: u32,
    pub name: String,
    pub kind: String,
    pub minimum: i64,
    pub maximum: i64,
    pub step: u64,
    pub default: i64,
    pub flags: String,
    pub menu_items: Vec<(u32, String)>,
    pub current: Option<CameraControlValue>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CameraControlUpdate {
    pub id: u32,
    pub value: CameraControlValue,
}

pub fn query_camera_controls(device_path: impl Into<PathBuf>) -> Result<Vec<CameraControlInfo>> {
    let device_path = device_path.into();
    let device = Device::with_path(&device_path)
        .with_context(|| format!("opening camera {}", device_path.display()))?;
    query_controls(&device)
}

pub fn apply_camera_controls(
    device_path: impl Into<PathBuf>,
    updates: &[CameraControlUpdate],
) -> Result<Vec<CameraControlInfo>> {
    ensure!(!updates.is_empty(), "camera control update is empty");
    ensure!(
        updates.len() <= 32,
        "camera control update contains too many values"
    );
    let device_path = device_path.into();
    let device = Device::with_path(&device_path)
        .with_context(|| format!("opening camera {}", device_path.display()))?;
    let descriptions = device
        .query_controls()
        .context("querying camera controls")?;
    let mut seen = BTreeSet::new();
    let mut controls = Vec::with_capacity(updates.len());
    for update in updates {
        ensure!(
            seen.insert(update.id),
            "duplicate camera control {}",
            update.id
        );
        let description = descriptions
            .iter()
            .find(|description| description.id == update.id)
            .with_context(|| format!("camera control {} does not exist", update.id))?;
        ensure!(
            !description
                .flags
                .intersects(Flags::DISABLED | Flags::READ_ONLY | Flags::GRABBED),
            "camera control {} ({}) is not writable",
            update.id,
            description.name
        );
        controls.push(Control {
            id: update.id,
            value: validated_control_value(description, &update.value)?,
        });
    }
    // Preserve request ordering because a preceding automatic/manual selector
    // can activate a later exposure or white-balance control.
    for control in controls {
        let id = control.id;
        device
            .set_control(control)
            .with_context(|| format!("setting camera control {id}"))?;
    }
    query_controls(&device)
}

fn query_controls(device: &Device) -> Result<Vec<CameraControlInfo>> {
    device
        .query_controls()
        .context("querying camera controls")?
        .into_iter()
        .map(|description| {
            let current = device
                .control(description.id)
                .ok()
                .and_then(|control| control_value(control.value));
            let menu_items = description
                .items
                .unwrap_or_default()
                .into_iter()
                .map(|(index, item)| (index, item.to_string()))
                .collect();
            Ok(CameraControlInfo {
                id: description.id,
                name: description.name,
                kind: description.typ.to_string(),
                minimum: description.minimum,
                maximum: description.maximum,
                step: description.step,
                default: description.default,
                flags: description.flags.to_string(),
                menu_items,
                current,
            })
        })
        .collect()
}

fn validated_control_value(
    description: &Description,
    requested: &CameraControlValue,
) -> Result<Value> {
    match (description.typ, requested) {
        (ControlType::Boolean, CameraControlValue::Boolean(value)) => Ok(Value::Boolean(*value)),
        (
            ControlType::Integer
            | ControlType::Integer64
            | ControlType::Menu
            | ControlType::IntegerMenu
            | ControlType::Bitmask,
            CameraControlValue::Integer(value),
        ) => {
            ensure!(
                (description.minimum..=description.maximum).contains(value),
                "camera control {} ({}) value {} is outside {}..={}",
                description.id,
                description.name,
                value,
                description.minimum,
                description.maximum
            );
            if description.step > 0 {
                ensure!(
                    (*value - description.minimum) as u64 % description.step == 0,
                    "camera control {} ({}) value {} does not match step {}",
                    description.id,
                    description.name,
                    value,
                    description.step
                );
            }
            if matches!(
                description.typ,
                ControlType::Menu | ControlType::IntegerMenu
            ) {
                let item_exists = description
                    .items
                    .as_ref()
                    .is_none_or(|items| items.iter().any(|(index, _)| i64::from(*index) == *value));
                ensure!(
                    item_exists,
                    "camera control {} ({}) has no menu item {}",
                    description.id,
                    description.name,
                    value
                );
            }
            Ok(Value::Integer(*value))
        }
        _ => bail!(
            "camera control {} ({}) expects {}, not the supplied value type",
            description.id,
            description.name,
            description.typ
        ),
    }
}

fn control_value(value: Value) -> Option<CameraControlValue> {
    match value {
        Value::Integer(value) => Some(CameraControlValue::Integer(value)),
        Value::Boolean(value) => Some(CameraControlValue::Boolean(value)),
        Value::String(value) => Some(CameraControlValue::Text(value)),
        Value::None
        | Value::CompoundU8(_)
        | Value::CompoundU16(_)
        | Value::CompoundU32(_)
        | Value::CompoundPtr(_) => None,
    }
}

fn validate_capture_config(config: &V4l2CaptureConfig) -> Result<()> {
    ensure!(
        config.width > 0 && config.height > 0,
        "camera dimensions must be non-zero"
    );
    if config.pixel_format == CameraPixelFormat::Yuyv {
        ensure!(config.width % 2 == 0, "YUYV capture width must be even");
    }
    ensure!(
        config.frames_per_second > 0,
        "camera frame rate must be non-zero"
    );
    ensure!(
        config.settle_frames <= 120,
        "camera settling frame count is excessive"
    );
    Ok(())
}

fn persistent_capture_worker(
    config: V4l2CaptureConfig,
    requests: mpsc::Receiver<PersistentCaptureRequest>,
    ready: mpsc::SyncSender<Result<(), String>>,
) {
    let setup = (|| -> Result<_> {
        let device = Device::with_path(&config.device)
            .with_context(|| format!("opening camera {}", config.device.display()))?;
        let requested_fourcc = capture_fourcc(config.pixel_format);
        let format = device
            .set_format(&Format::new(config.width, config.height, requested_fourcc))
            .context("setting camera capture format")?;
        if format.fourcc != requested_fourcc
            || format.width != config.width
            || format.height != config.height
        {
            bail!(
                "camera negotiated {} {}x{} instead of {} {}x{}",
                format.fourcc,
                format.width,
                format.height,
                requested_fourcc,
                config.width,
                config.height
            );
        }
        let mut parameters = device.params().context("reading camera frame rate")?;
        parameters.interval = Fraction::new(1, config.frames_per_second);
        let parameters = device
            .set_params(&parameters)
            .context("setting camera frame rate")?;
        validate_frame_interval(parameters.interval, config.frames_per_second)?;
        Ok((device, format, parameters))
    })();
    let (device, format, parameters) = match setup {
        Ok(setup) => setup,
        Err(error) => {
            let _ = ready.send(Err(format!("{error:#}")));
            return;
        }
    };
    let mut stream = match MmapStream::with_buffers(&device, Type::VideoCapture, 4)
        .context("allocating camera capture buffers")
    {
        Ok(stream) => stream,
        Err(error) => {
            let _ = ready.send(Err(format!("{error:#}")));
            return;
        }
    };
    stream.set_timeout(config.timeout);
    if ready.send(Ok(())).is_err() {
        return;
    }

    let mut pending = None;
    loop {
        if pending.is_none() {
            match requests.try_recv() {
                Ok(PersistentCaptureRequest::Stop) | Err(mpsc::TryRecvError::Disconnected) => {
                    break;
                }
                Ok(request) => pending = pending_capture(request),
                Err(mpsc::TryRecvError::Empty) => {}
            }
        }

        let request_was_pending = pending.is_some();
        let (buffer, metadata) = match stream.next() {
            Ok(frame) => frame,
            Err(error) => {
                if let Some(pending) = pending.take() {
                    let _ = pending
                        .response
                        .send(Err(format!("capturing camera frame: {error}")));
                }
                break;
            }
        };

        if pending.is_none() {
            match requests.try_recv() {
                Ok(PersistentCaptureRequest::Stop) | Err(mpsc::TryRecvError::Disconnected) => {
                    break;
                }
                Ok(request) => pending = pending_capture(request),
                Err(mpsc::TryRecvError::Empty) => continue,
            }
        }
        let Some(active) = pending.as_mut() else {
            continue;
        };
        if !request_was_pending && active.freshness.is_none() {
            continue;
        }

        let index = active.assessed_frames;
        active.assessed_frames = active.assessed_frames.saturating_add(1);
        let timestamp_fresh = active.freshness.and_then(|boundary| {
            frame_is_fresh_after(
                metadata.flags,
                metadata.timestamp.sec,
                metadata.timestamp.usec,
                boundary,
            )
        });
        let accepted = timestamp_fresh.unwrap_or(index >= active.settle_frames);
        if accepted {
            let used = (metadata.bytesused as usize).min(buffer.len());
            let capture = capture_to_gray(
                &buffer[..used],
                format.width,
                format.height,
                format.stride,
                config.pixel_format,
            )
            .map(|frame| CapturedGrayFrame {
                frame,
                metadata: CaptureMetadata {
                    device: config.device.clone(),
                    width: format.width,
                    height: format.height,
                    fourcc: format.fourcc.to_string(),
                    stride: format.stride,
                    interval_numerator: parameters.interval.numerator,
                    interval_denominator: parameters.interval.denominator,
                    sequence: metadata.sequence,
                    settle_frames: index,
                    timestamp_monotonic: metadata.flags.contains(BufferFlags::TIMESTAMP_MONOTONIC),
                    timestamp_source: capture_timestamp_source(metadata.flags),
                    fresh_after_boundary: timestamp_fresh == Some(true),
                },
            })
            .map_err(|error| format!("{error:#}"));
            if let Some(pending) = pending.take() {
                let _ = pending.response.send(capture);
            }
        } else if active.assessed_frames >= active.frame_limit {
            if let Some(pending) = pending.take() {
                let _ = pending.response.send(Err(
                    "persistent camera returned no frame after the freshness boundary".into(),
                ));
            }
        }
    }
}

struct PendingCapture {
    settle_frames: u32,
    freshness: Option<CaptureFreshnessBoundary>,
    frame_limit: u32,
    assessed_frames: u32,
    response: mpsc::SyncSender<Result<CapturedGrayFrame, String>>,
}

fn pending_capture(request: PersistentCaptureRequest) -> Option<PendingCapture> {
    match request {
        PersistentCaptureRequest::Capture {
            settle_frames,
            response,
        } => Some(PendingCapture {
            settle_frames,
            freshness: None,
            frame_limit: settle_frames.saturating_add(1),
            assessed_frames: 0,
            response,
        }),
        PersistentCaptureRequest::CaptureFreshAfter {
            boundary,
            fallback_settle_frames,
            maximum_frames,
            response,
        } => Some(PendingCapture {
            settle_frames: fallback_settle_frames,
            freshness: Some(boundary),
            frame_limit: maximum_frames,
            assessed_frames: 0,
            response,
        }),
        PersistentCaptureRequest::Stop => None,
    }
}

fn frame_is_fresh_after(
    flags: BufferFlags,
    seconds: libc::time_t,
    microseconds: libc::time_t,
    boundary: CaptureFreshnessBoundary,
) -> Option<bool> {
    if !flags.contains(BufferFlags::TIMESTAMP_MONOTONIC)
        || seconds < 0
        || !(0..1_000_000).contains(&microseconds)
    {
        return None;
    }
    let timestamp = Duration::new(seconds as u64, microseconds as u32 * 1_000);
    let guard = match capture_timestamp_source(flags) {
        CaptureTimestampSource::EndOfFrame => boundary.end_of_frame_exposure,
        CaptureTimestampSource::StartOfExposure => Duration::ZERO,
        CaptureTimestampSource::Unknown => boundary.unknown_source_guard,
    };
    let not_before = boundary.exposure_started_after.checked_add(guard)?;
    Some(timestamp >= not_before)
}

fn capture_timestamp_source(flags: BufferFlags) -> CaptureTimestampSource {
    match flags.bits() & BufferFlags::TSTAMP_SRC_MASK.bits() {
        value if value == BufferFlags::TSTAMP_SRC_EOF.bits() => CaptureTimestampSource::EndOfFrame,
        value if value == BufferFlags::TSTAMP_SRC_SOE.bits() => {
            CaptureTimestampSource::StartOfExposure
        }
        _ => CaptureTimestampSource::Unknown,
    }
}

pub fn capture_gray_frame(config: &V4l2CaptureConfig) -> Result<CapturedGrayFrame> {
    capture_gray_frames(config, 1)?
        .pop()
        .context("camera returned no frames")
}

pub fn capture_gray_frames(
    config: &V4l2CaptureConfig,
    sample_count: u32,
) -> Result<Vec<CapturedGrayFrame>> {
    ensure!(sample_count > 0, "camera sample count must be non-zero");
    validate_capture_config(config)?;

    let device = Device::with_path(&config.device)
        .with_context(|| format!("opening camera {}", config.device.display()))?;
    let requested_fourcc = capture_fourcc(config.pixel_format);
    let format = device
        .set_format(&Format::new(config.width, config.height, requested_fourcc))
        .context("setting camera capture format")?;
    if format.fourcc != requested_fourcc
        || format.width != config.width
        || format.height != config.height
    {
        bail!(
            "camera negotiated {} {}x{} instead of {} {}x{}",
            format.fourcc,
            format.width,
            format.height,
            requested_fourcc,
            config.width,
            config.height
        );
    }

    let mut parameters = device.params().context("reading camera frame rate")?;
    parameters.interval = Fraction::new(1, config.frames_per_second);
    let parameters = device
        .set_params(&parameters)
        .context("setting camera frame rate")?;
    validate_frame_interval(parameters.interval, config.frames_per_second)?;

    let mut stream = MmapStream::with_buffers(&device, Type::VideoCapture, 4)
        .context("allocating camera capture buffers")?;
    stream.set_timeout(config.timeout);

    let total_frames = config
        .settle_frames
        .checked_add(sample_count)
        .context("camera frame count overflow")?;
    let mut captures = Vec::with_capacity(sample_count as usize);
    for index in 0..total_frames {
        let (buffer, metadata) = stream.next().context("capturing camera frame")?;
        if index < config.settle_frames {
            continue;
        }
        let used = (metadata.bytesused as usize).min(buffer.len());
        captures.push(CapturedGrayFrame {
            frame: capture_to_gray(
                &buffer[..used],
                format.width,
                format.height,
                format.stride,
                config.pixel_format,
            )?,
            metadata: CaptureMetadata {
                device: config.device.clone(),
                width: format.width,
                height: format.height,
                fourcc: format.fourcc.to_string(),
                stride: format.stride,
                interval_numerator: parameters.interval.numerator,
                interval_denominator: parameters.interval.denominator,
                sequence: metadata.sequence,
                settle_frames: config.settle_frames,
                timestamp_monotonic: metadata.flags.contains(BufferFlags::TIMESTAMP_MONOTONIC),
                timestamp_source: capture_timestamp_source(metadata.flags),
                fresh_after_boundary: false,
            },
        });
    }
    Ok(captures)
}

fn capture_fourcc(pixel_format: CameraPixelFormat) -> FourCC {
    match pixel_format {
        CameraPixelFormat::Yuyv => yuyv(),
        CameraPixelFormat::Mjpeg => mjpg(),
    }
}

fn validate_frame_interval(interval: Fraction, frames_per_second: u32) -> Result<()> {
    ensure!(
        interval.numerator > 0 && interval.denominator > 0,
        "camera returned an invalid frame interval"
    );
    ensure!(
        u64::from(interval.denominator)
            == u64::from(interval.numerator) * u64::from(frames_per_second),
        "camera negotiated {}/{} second frames instead of 1/{frames_per_second}",
        interval.numerator,
        interval.denominator
    );
    Ok(())
}

fn capture_to_gray(
    buffer: &[u8],
    width: u32,
    height: u32,
    stride: u32,
    pixel_format: CameraPixelFormat,
) -> Result<GrayFrame> {
    match pixel_format {
        CameraPixelFormat::Yuyv => yuyv_to_gray(buffer, width, height, stride),
        CameraPixelFormat::Mjpeg => mjpeg_to_gray(buffer, width, height),
    }
}

fn mjpeg_to_gray(buffer: &[u8], width: u32, height: u32) -> Result<GrayFrame> {
    use opencv::prelude::*;

    ensure!(!buffer.is_empty(), "MJPEG frame is empty");
    let encoded = opencv::core::Vector::from_slice(buffer);
    let frame = opencv::imgcodecs::imdecode(&encoded, opencv::imgcodecs::IMREAD_GRAYSCALE)
        .context("decoding MJPEG camera frame")?;
    ensure!(!frame.empty(), "MJPEG decoder returned an empty frame");
    ensure!(
        frame.cols() == width as i32 && frame.rows() == height as i32,
        "MJPEG frame is {}x{} instead of {width}x{height}",
        frame.cols(),
        frame.rows()
    );
    ensure!(
        frame.is_continuous(),
        "MJPEG grayscale frame is not contiguous"
    );
    let pixels = frame
        .data_bytes()
        .context("reading decoded MJPEG grayscale pixels")?
        .to_vec();
    ensure!(
        pixels.len() == width as usize * height as usize,
        "MJPEG grayscale frame has an invalid pixel count"
    );
    Ok(GrayFrame {
        width,
        height,
        pixels,
    })
}

fn yuyv_to_gray(buffer: &[u8], width: u32, height: u32, stride: u32) -> Result<GrayFrame> {
    ensure!(width > 0 && height > 0, "frame dimensions must be non-zero");
    ensure!(width % 2 == 0, "YUYV frame width must be even");
    let packed_stride = width.checked_mul(2).context("YUYV stride overflow")?;
    let stride = if stride == 0 { packed_stride } else { stride };
    ensure!(
        stride >= packed_stride,
        "YUYV stride is shorter than one packed row"
    );
    let required = stride
        .checked_mul(height)
        .context("YUYV frame size overflow")? as usize;
    ensure!(
        buffer.len() >= required,
        "YUYV frame is truncated: expected at least {required} bytes, got {}",
        buffer.len()
    );

    let mut pixels = Vec::with_capacity(width as usize * height as usize);
    for y in 0..height as usize {
        let row = y * stride as usize;
        for x in 0..width as usize {
            let pair = row + (x / 2) * 4;
            pixels.push(buffer[pair + if x % 2 == 0 { 0 } else { 2 }]);
        }
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
    use v4l::control::MenuItem;

    fn control_description(typ: ControlType) -> Description {
        Description {
            id: 42,
            typ,
            name: "Test control".into(),
            minimum: 1,
            maximum: 5,
            step: 2,
            default: 1,
            flags: Flags::empty(),
            items: None,
        }
    }

    #[test]
    fn extracts_luma_from_yuyv_with_row_padding() {
        let frame = yuyv_to_gray(
            &[
                10, 100, 20, 150, 30, 101, 40, 151, 0, 0, 50, 102, 60, 152, 70, 103, 80, 153, 0, 0,
            ],
            4,
            2,
            10,
        )
        .unwrap();
        assert_eq!(frame.pixels, [10, 20, 30, 40, 50, 60, 70, 80]);
    }

    #[test]
    fn rejects_truncated_yuyv_frame() {
        assert!(yuyv_to_gray(&[0; 7], 4, 1, 8).is_err());
    }

    #[test]
    fn decodes_mjpeg_to_a_gray_frame() {
        let source =
            opencv::core::Mat::from_slice_2d(&[&[10_u8, 20, 30, 40], &[50_u8, 60, 70, 80]])
                .unwrap();
        let mut encoded = opencv::core::Vector::new();
        opencv::imgcodecs::imencode(
            ".jpg",
            &source,
            &mut encoded,
            &opencv::core::Vector::<i32>::new(),
        )
        .unwrap();

        let frame = mjpeg_to_gray(encoded.as_slice(), 4, 2).unwrap();
        assert_eq!((frame.width, frame.height), (4, 2));
        assert_eq!(frame.pixels.len(), 8);
    }

    #[test]
    fn rejects_an_empty_capture_series_before_opening_the_device() {
        let config = V4l2CaptureConfig::lumenpnp("/dev/null");
        assert!(capture_gray_frames(&config, 0).is_err());
    }

    #[test]
    fn persistent_capture_rejects_invalid_settings_before_opening_the_device() {
        let mut config = V4l2CaptureConfig::lumenpnp("/dev/null");
        config.width = 1279;
        assert!(PersistentGrayCapture::open(config).is_err());
    }

    #[test]
    fn fresh_capture_uses_monotonic_camera_timestamps() {
        let flags = BufferFlags::TIMESTAMP_MONOTONIC;
        let boundary = CaptureFreshnessBoundary {
            exposure_started_after: Duration::new(10, 480_000_000),
            end_of_frame_exposure: Duration::from_millis(20),
            unknown_source_guard: Duration::from_millis(34),
        };
        assert_eq!(
            frame_is_fresh_after(flags, 10, 499_999, boundary),
            Some(false)
        );
        assert_eq!(
            frame_is_fresh_after(flags, 10, 500_000, boundary),
            Some(true)
        );
        assert_eq!(
            frame_is_fresh_after(BufferFlags::empty(), 11, 0, boundary),
            None
        );
        assert_eq!(frame_is_fresh_after(flags, 10, 1_000_000, boundary), None);
    }

    #[test]
    fn fresh_capture_uses_the_timestamp_source() {
        let boundary = CaptureFreshnessBoundary {
            exposure_started_after: Duration::from_secs(10),
            end_of_frame_exposure: Duration::from_millis(20),
            unknown_source_guard: Duration::from_millis(34),
        };
        let monotonic = BufferFlags::TIMESTAMP_MONOTONIC;
        assert_eq!(
            frame_is_fresh_after(monotonic, 10, 19_999, boundary),
            Some(false)
        );
        assert_eq!(
            frame_is_fresh_after(monotonic, 10, 20_000, boundary),
            Some(true)
        );
        assert_eq!(
            frame_is_fresh_after(monotonic | BufferFlags::TSTAMP_SRC_SOE, 10, 0, boundary,),
            Some(true)
        );
        let unknown = BufferFlags::from(monotonic.bits() | 0x0002_0000);
        assert_eq!(
            frame_is_fresh_after(unknown, 10, 33_999, boundary),
            Some(false)
        );
        assert_eq!(
            frame_is_fresh_after(unknown, 10, 34_000, boundary),
            Some(true)
        );
    }

    #[test]
    fn monotonic_clock_returns_a_valid_timestamp() {
        assert!(monotonic_timestamp().unwrap() > Duration::ZERO);
    }

    #[test]
    fn validates_camera_control_range_step_and_type() {
        let description = control_description(ControlType::Integer);
        assert!(validated_control_value(&description, &CameraControlValue::Integer(3)).is_ok());
        assert!(validated_control_value(&description, &CameraControlValue::Integer(2)).is_err());
        assert!(validated_control_value(&description, &CameraControlValue::Integer(7)).is_err());
        assert!(validated_control_value(&description, &CameraControlValue::Boolean(true)).is_err());
    }

    #[test]
    fn validates_camera_menu_membership() {
        let mut description = control_description(ControlType::Menu);
        description.step = 1;
        description.items = Some(vec![
            (1, MenuItem::Name("Manual".into())),
            (3, MenuItem::Name("Automatic".into())),
        ]);
        assert!(validated_control_value(&description, &CameraControlValue::Integer(3)).is_ok());
        assert!(validated_control_value(&description, &CameraControlValue::Integer(2)).is_err());
    }
}
