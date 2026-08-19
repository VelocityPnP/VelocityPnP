use anyhow::{ensure, Context, Result};
use serde_json::json;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::{InstalledPlateGeometryReport, PlanarCameraModel, WorkspaceScanReport};
use velocitypnp_vision::{
    decode_gray_png, measure_workspace_parallax, rescale_planar_camera_model, WorkspaceScanPlan,
};

fn main() -> Result<()> {
    let arguments = env::args_os().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 3,
        "usage: analyze_workspace_parallax SCAN_DIR CALIBRATION_JSON OUTPUT_JSON"
    );
    let scan_directory = PathBuf::from(&arguments[0]);
    let calibration_path = PathBuf::from(&arguments[1]);
    let output_path = PathBuf::from(&arguments[2]);
    let report: WorkspaceScanReport = read_json(&scan_directory.join("manifest.json"))?;
    ensure!(
        report.captured_tiles == report.planned_tiles,
        "workspace scan did not retain every planned tile"
    );
    let calibration: serde_json::Value = read_json(&calibration_path)?;
    let working_model: PlanarCameraModel = serde_json::from_value(
        calibration["artifacts"]["top_camera_planar_model"]["value"]["model"].clone(),
    )
    .context("calibration has no top-camera planar model")?;
    let geometry: InstalledPlateGeometryReport = serde_json::from_value(
        calibration["artifacts"]["installed_plate_geometry"]["value"].clone(),
    )
    .context("calibration has no installed-plate geometry")?;
    let mappings = geometry
        .registrations
        .iter()
        .filter(|registration| registration.passed)
        .filter_map(|registration| registration.pixel_error_to_machine_mm)
        .collect::<Vec<_>>();
    ensure!(
        !mappings.is_empty(),
        "installed-plate geometry has no passing local camera maps"
    );
    let mut mean_mapping = [[0.0; 2]; 2];
    for mapping in &mappings {
        for axis in 0..2 {
            for pixel_axis in 0..2 {
                mean_mapping[axis][pixel_axis] += mapping[axis][pixel_axis];
            }
        }
    }
    for axis in &mut mean_mapping {
        for value in axis {
            *value /= mappings.len() as f64;
        }
    }
    let installed_model =
        rescale_planar_camera_model(&working_model, mean_mapping).map_err(anyhow::Error::msg)?;
    let mut tiles = report.tiles.clone();
    for tile in &mut tiles {
        tile.placement_correction_x_mm = 0.0;
        tile.placement_correction_y_mm = 0.0;
    }
    let plan = WorkspaceScanPlan {
        region: report.region,
        overlap_fraction: report.overlap_fraction,
        output_mm_per_pixel: report.output_mm_per_pixel,
        frame_width_px: report.frame_width_px,
        frame_height_px: report.frame_height_px,
        usable_pixel_bounds: report.usable_pixel_bounds,
        rows: report.rows,
        columns: report.columns,
        tiles,
    };
    let frames = report
        .tiles
        .iter()
        .map(|tile| {
            let path = scan_directory.join(&tile.artifact_name);
            let encoded =
                fs::read(&path).with_context(|| format!("failed to read {}", path.display()))?;
            decode_gray_png(&encoded)
                .with_context(|| format!("failed to decode {}", path.display()))
        })
        .collect::<Result<Vec<_>>>()?;
    let parallax = measure_workspace_parallax(&plan, &working_model, &installed_model, &frames)?;
    fs::write(&output_path, serde_json::to_vec_pretty(&parallax)?)
        .with_context(|| format!("failed to write {}", output_path.display()))?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "source_scan_id": report.id,
            "attempted_pairs": parallax.attempted_pairs,
            "pairs_with_overlap": parallax.pairs_with_overlap,
            "raw_matches": parallax.raw_matches,
            "descriptor_matches": parallax.descriptor_matches,
            "accepted_observations": parallax.accepted_observations,
            "output_json": output_path,
        }))?
    );
    Ok(())
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let data = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_json::from_slice(&data).with_context(|| format!("failed to parse {}", path.display()))
}
