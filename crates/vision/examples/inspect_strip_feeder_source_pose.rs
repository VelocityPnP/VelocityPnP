use anyhow::{Context, Result};
use std::{env, fs};
use uuid::Uuid;
use velocitypnp_domain::{PlanarCameraModel, Pose, StripFeederRegistration};
use velocitypnp_vision::{decode_gray_png, encode_rgb_png, inspect_strip_feeder_source_pose};

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    anyhow::ensure!(
        arguments.len() == 19,
        "usage: inspect_strip_feeder_source_pose REFERENCE CANDIDATE MODEL \
         REGISTRATION OUTPUT QUALIFICATION_UUID REFERENCE_X REFERENCE_Y \
         REFERENCE_POSE_X REFERENCE_POSE_Y REFERENCE_POSE_Z \
         REFERENCE_POSE_ROTATION CANDIDATE_X CANDIDATE_Y CANDIDATE_POSE_X \
         CANDIDATE_POSE_Y CANDIDATE_POSE_Z CANDIDATE_POSE_ROTATION \
         ALLOW_FACE_TEXTURE_CHANGE"
    );
    let number = |index: usize| -> Result<f64> {
        arguments[index]
            .parse()
            .with_context(|| format!("invalid numeric argument {}", index + 1))
    };
    let pose = |index: usize| -> Result<Pose> {
        Ok(Pose {
            x_mm: number(index)?,
            y_mm: number(index + 1)?,
            z_mm: number(index + 2)?,
            rotation_deg: number(index + 3)?,
        })
    };
    let reference = decode_gray_png(&fs::read(&arguments[0])?)?;
    let candidate = decode_gray_png(&fs::read(&arguments[1])?)?;
    let model: PlanarCameraModel = serde_json::from_slice(&fs::read(&arguments[2])?)?;
    let registration: StripFeederRegistration = serde_json::from_slice(&fs::read(&arguments[3])?)?;
    let qualification_id = arguments[5].parse::<Uuid>()?;
    let allow_face_texture_change = arguments[18]
        .parse::<bool>()
        .context("invalid face-texture policy")?;
    let result = inspect_strip_feeder_source_pose(
        &reference,
        pose(8)?,
        [number(6)?, number(7)?],
        qualification_id,
        &candidate,
        pose(14)?,
        [number(12)?, number(13)?],
        &model,
        &registration,
        allow_face_texture_change,
        &arguments[4],
    )?;
    fs::write(&arguments[4], encode_rgb_png(&result.preview)?)?;
    println!("{}", serde_json::to_string_pretty(&result.report)?);
    Ok(())
}
