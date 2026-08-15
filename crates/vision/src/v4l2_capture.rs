use crate::GrayFrame;
use anyhow::{bail, ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::path::PathBuf;
use std::time::Duration;
use v4l::buffer::Type;
use v4l::control::{Control, Description, Flags, Type as ControlType, Value};
use v4l::io::traits::CaptureStream;
use v4l::prelude::MmapStream;
use v4l::video::Capture;
use v4l::{Device, Format, FourCC, Fraction};

fn yuyv() -> FourCC {
    FourCC::new(b"YUYV")
}

#[derive(Debug, Clone)]
pub struct V4l2CaptureConfig {
    pub device: PathBuf,
    pub width: u32,
    pub height: u32,
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
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapturedGrayFrame {
    pub frame: GrayFrame,
    pub metadata: CaptureMetadata,
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
    ensure!(
        config.width > 0 && config.height > 0,
        "camera dimensions must be non-zero"
    );
    ensure!(config.width % 2 == 0, "YUYV capture width must be even");
    ensure!(
        config.frames_per_second > 0,
        "camera frame rate must be non-zero"
    );
    ensure!(
        config.settle_frames <= 120,
        "camera settling frame count is excessive"
    );

    let device = Device::with_path(&config.device)
        .with_context(|| format!("opening camera {}", config.device.display()))?;
    let format = device
        .set_format(&Format::new(config.width, config.height, yuyv()))
        .context("setting camera capture format")?;
    if format.fourcc != yuyv() || format.width != config.width || format.height != config.height {
        bail!(
            "camera negotiated {} {}x{} instead of YUYV {}x{}",
            format.fourcc,
            format.width,
            format.height,
            config.width,
            config.height
        );
    }

    let mut parameters = device.params().context("reading camera frame rate")?;
    parameters.interval = Fraction::new(1, config.frames_per_second);
    let parameters = device
        .set_params(&parameters)
        .context("setting camera frame rate")?;
    if parameters.interval.numerator == 0 || parameters.interval.denominator == 0 {
        bail!("camera returned an invalid frame interval");
    }

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
            frame: yuyv_to_gray(&buffer[..used], format.width, format.height, format.stride)?,
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
            },
        });
    }
    Ok(captures)
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
    fn rejects_an_empty_capture_series_before_opening_the_device() {
        let config = V4l2CaptureConfig::lumenpnp("/dev/null");
        assert!(capture_gray_frames(&config, 0).is_err());
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
