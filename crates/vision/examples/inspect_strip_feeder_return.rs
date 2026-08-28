use anyhow::{Context, Result};
use std::{env, fs};
use velocitypnp_domain::{PlanarCameraModel, Pose, StripFeederRegistration};
use velocitypnp_vision::{decode_gray_png, encode_rgb_png, inspect_strip_feeder_pocket_return};

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    anyhow::ensure!(
        arguments.len() == 11,
        "usage: inspect_strip_feeder_return BEFORE AFTER MODEL REGISTRATION \
         OUTPUT POCKET_X POCKET_Y POSE_X POSE_Y POSE_Z POSE_ROTATION"
    );
    let number = |index: usize| -> Result<f64> {
        arguments[index]
            .parse()
            .with_context(|| format!("invalid numeric argument {}", index + 1))
    };
    let before = decode_gray_png(&fs::read(&arguments[0])?)?;
    let after = decode_gray_png(&fs::read(&arguments[1])?)?;
    let model: PlanarCameraModel = serde_json::from_slice(&fs::read(&arguments[2])?)?;
    let registration: StripFeederRegistration = serde_json::from_slice(&fs::read(&arguments[3])?)?;
    let camera_pose = Pose {
        x_mm: number(7)?,
        y_mm: number(8)?,
        z_mm: number(9)?,
        rotation_deg: number(10)?,
    };
    let result = inspect_strip_feeder_pocket_return(
        &before,
        camera_pose,
        &after,
        camera_pose,
        &model,
        &registration,
        [number(5)?, number(6)?],
        &arguments[4],
    )?;
    fs::write(&arguments[4], encode_rgb_png(&result.preview)?)?;
    println!("{}", serde_json::to_string_pretty(&result.report)?);
    Ok(())
}
