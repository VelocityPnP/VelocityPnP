use anyhow::{ensure, Result};
use std::env;
use std::fs;
use std::path::Path;
use velocitypnp_model::SourceComponentModel;
use velocitypnp_vision::decode_gray_png;

fn main() -> Result<()> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 2,
        "usage: inspect_source_component MODEL_PACKAGE SOURCE_CROP_PNG"
    );
    let model = SourceComponentModel::load(Path::new(&arguments[0]))?;
    let image = decode_gray_png(&fs::read(&arguments[1])?)?;
    let prediction = model.predict_gray(image.width, image.height, &image.pixels)?;
    println!("{}", serde_json::to_string_pretty(&prediction)?);
    Ok(())
}
