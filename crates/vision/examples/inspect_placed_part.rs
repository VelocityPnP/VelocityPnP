use anyhow::{bail, Context, Result};
use std::{env, fs};
use velocitypnp_vision::{decode_gray_png, encode_rgb_png, inspect_placed_part};

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    if arguments.len() != 15 {
        bail!(
            "usage: inspect_placed_part BEFORE AFTER OUTPUT CENTER_X CENTER_Y \
             MAP_00 MAP_01 MAP_10 MAP_11 WIDTH_MM LENGTH_MM ROTATION_DEG \
             MAX_OFFSET_MM MAX_ROTATION_DEG ARTIFACT_NAME"
        );
    }
    let before = decode_gray_png(&fs::read(&arguments[0])?)?;
    let after = decode_gray_png(&fs::read(&arguments[1])?)?;
    let number = |index: usize| -> Result<f64> {
        arguments[index]
            .parse::<f64>()
            .with_context(|| format!("argument {} is not a number", index + 1))
    };
    let result = inspect_placed_part(
        &before,
        &after,
        [number(3)?, number(4)?],
        [[number(5)?, number(6)?], [number(7)?, number(8)?]],
        number(9)?,
        number(10)?,
        number(11)?,
        number(12)?,
        number(13)?,
        arguments[14].clone(),
    )?;
    fs::write(&arguments[2], encode_rgb_png(&result.preview)?)?;
    println!("{}", serde_json::to_string_pretty(&result.report)?);
    Ok(())
}
