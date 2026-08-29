use anyhow::{Context, Result};
use std::{env, fs};
use velocitypnp_domain::PartOrientation;
use velocitypnp_vision::{
    decode_gray_png, encode_rgb_png, inspect_held_part_difference_with_orientation,
};

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    anyhow::ensure!(
        arguments.len() == 9 || arguments.len() == 12,
        "usage: inspect_held_part BARE HELD OUTPUT CENTER_X CENTER_Y XX XY YX YY [MODE MIN_CONTRAST ROTATION_OFFSET]"
    );
    let number = |index: usize| -> Result<f64> {
        arguments[index]
            .parse()
            .with_context(|| format!("invalid numeric argument {}", index + 1))
    };
    let bare = decode_gray_png(&fs::read(&arguments[0])?)?;
    let held = decode_gray_png(&fs::read(&arguments[1])?)?;
    let orientation = if arguments.len() == 9 {
        PartOrientation::AxisSymmetric
    } else {
        let minimum_contrast_luma = number(10)?;
        let rotation_offset_deg = number(11)?;
        match arguments[9].as_str() {
            "axis_symmetric" => PartOrientation::AxisSymmetric,
            "feeder_referenced" => PartOrientation::FeederReferenced,
            "brighter_end" => PartOrientation::BrighterEnd {
                minimum_contrast_luma,
                rotation_offset_deg,
            },
            "darker_end" => PartOrientation::DarkerEnd {
                minimum_contrast_luma,
                rotation_offset_deg,
            },
            mode => anyhow::bail!("invalid orientation mode {mode}"),
        }
    };
    let result = inspect_held_part_difference_with_orientation(
        &bare,
        &held,
        [number(3)?, number(4)?],
        [[number(5)?, number(6)?], [number(7)?, number(8)?]],
        1.0,
        0.1,
        12.0,
        orientation,
        arguments[2].clone(),
    )?;
    fs::write(&arguments[2], encode_rgb_png(&result.preview)?)?;
    println!("{}", serde_json::to_string_pretty(&result.report)?);
    Ok(())
}
