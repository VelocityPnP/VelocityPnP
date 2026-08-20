use anyhow::{ensure, Context, Result};
use serde_json::json;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::{InstalledPlateGeometryReport, PlanarCameraModel, WorkspaceScanReport};
use velocitypnp_vision::{
    build_workspace_height_map, decode_gray_png, encode_gray_png, interpolate_planar_camera_model,
    render_workspace_height_mosaic, rescale_planar_camera_model, WorkspaceMosaicStream,
    WorkspaceParallaxStream, WorkspaceScanPlan,
};

fn main() -> Result<()> {
    let arguments = env::args_os().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 3,
        "usage: build_workspace_height_mosaic SCAN_DIR CALIBRATION_JSON OUTPUT_DIR"
    );
    let scan_directory = PathBuf::from(&arguments[0]);
    let calibration_path = PathBuf::from(&arguments[1]);
    let output_directory = PathBuf::from(&arguments[2]);
    fs::create_dir_all(&output_directory).with_context(|| {
        format!(
            "failed to create output directory {}",
            output_directory.display()
        )
    })?;
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
    let installed_model = installed_camera_model(&calibration, &working_model)?;
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
    let mut parallax_stream =
        WorkspaceParallaxStream::new(plan.clone(), working_model.clone(), installed_model.clone())?;
    for tile in capture_sequence(&plan) {
        parallax_stream.push_frame(
            tile.index,
            read_source_frame(&scan_directory, &tile.artifact_name)?,
        )?;
    }
    let parallax = parallax_stream.finish()?;
    let height_map = build_workspace_height_map(plan.region, &parallax)?;
    let composite = render_workspace_height_mosaic(&plan, &height_map, |fraction| {
        let model = interpolate_planar_camera_model(&working_model, &installed_model, fraction)
            .map_err(anyhow::Error::msg)?;
        let mut stream = WorkspaceMosaicStream::new_without_registration(plan.clone(), model)?;
        for tile in capture_sequence(&plan) {
            stream.push_frame(
                tile.index,
                read_source_frame(&scan_directory, &tile.artifact_name)?,
            )?;
        }
        Ok(stream.finish()?.frame)
    })?;
    write_json(
        &output_directory.join("surface-height-map.json"),
        &height_map,
    )?;
    write_png(
        &output_directory.join("surface-height-map.png"),
        &composite.height_preview,
    )?;
    write_png(
        &output_directory.join("surface-height-confidence.png"),
        &composite.confidence_frame,
    )?;
    write_png(
        &output_directory.join("mosaic-height-corrected.png"),
        &composite.frame,
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "source_scan_id": report.id,
            "accepted_observations": parallax.accepted_observations,
            "height_map_rows": height_map.rows,
            "height_map_columns": height_map.columns,
            "measured_cells": height_map.measured_cells,
            "inferred_cells": height_map.inferred_cells,
            "unknown_cells": height_map.unknown_cells,
            "candidate_fractions": height_map.candidate_fractions,
            "output_directory": output_directory,
        }))?
    );
    Ok(())
}

fn capture_sequence(plan: &WorkspaceScanPlan) -> Vec<&velocitypnp_domain::WorkspaceScanTile> {
    let mut tiles = plan.tiles.iter().collect::<Vec<_>>();
    tiles.sort_by_key(|tile| tile.capture_order);
    tiles
}

fn read_source_frame(
    directory: &Path,
    artifact_name: &str,
) -> Result<velocitypnp_vision::GrayFrame> {
    let path = directory.join(artifact_name);
    let encoded = fs::read(&path).with_context(|| format!("failed to read {}", path.display()))?;
    decode_gray_png(&encoded).with_context(|| format!("failed to decode {}", path.display()))
}

fn installed_camera_model(
    calibration: &serde_json::Value,
    working_model: &PlanarCameraModel,
) -> Result<PlanarCameraModel> {
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
    rescale_planar_camera_model(working_model, mean_mapping).map_err(anyhow::Error::msg)
}

fn write_png(path: &Path, frame: &velocitypnp_vision::GrayFrame) -> Result<()> {
    fs::write(path, encode_gray_png(frame)?)
        .with_context(|| format!("failed to write {}", path.display()))
}

fn write_json<T: serde::Serialize>(path: &Path, value: &T) -> Result<()> {
    fs::write(path, serde_json::to_vec_pretty(value)?)
        .with_context(|| format!("failed to write {}", path.display()))
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let data = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_json::from_slice(&data).with_context(|| format!("failed to parse {}", path.display()))
}
