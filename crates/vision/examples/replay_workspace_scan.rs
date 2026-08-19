use anyhow::{ensure, Context, Result};
use serde_json::json;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::{
    InstalledPlateGeometryReport, PlanarCameraModel, WorkspaceScanReport, WorkspaceSurfaceModel,
};
use velocitypnp_vision::{
    decode_gray_png, encode_gray_png, interpolate_planar_camera_model, rescale_planar_camera_model,
    WorkspaceMosaicStream, WorkspaceScanPlan,
};

fn main() -> Result<()> {
    let arguments = env::args_os().skip(1).collect::<Vec<_>>();
    ensure!(
        (4..=6).contains(&arguments.len()),
        "usage: replay_workspace_scan SCAN_DIR CALIBRATION_JSON OUTPUT_PNG OUTPUT_JSON [recorded|working_plane|installed_plate_planes|surface_fraction:F] [registration|no_registration]"
    );
    let scan_directory = PathBuf::from(&arguments[0]);
    let calibration_path = PathBuf::from(&arguments[1]);
    let output_png_path = PathBuf::from(&arguments[2]);
    let output_json_path = PathBuf::from(&arguments[3]);

    let report: WorkspaceScanReport = read_json(&scan_directory.join("manifest.json"))?;
    ensure!(
        report.captured_tiles == report.planned_tiles,
        "workspace scan did not retain every planned tile"
    );
    let calibration: serde_json::Value = read_json(&calibration_path)?;
    let surface_model_override = arguments
        .get(4)
        .map(|value| value.to_string_lossy().into_owned())
        .unwrap_or_else(|| "recorded".into());
    let surface_fraction = match surface_model_override.as_str() {
        "recorded" => {
            if report.surface_model == WorkspaceSurfaceModel::InstalledPlatePlanes {
                1.0
            } else {
                0.0
            }
        }
        "working_plane" => 0.0,
        "installed_plate_planes" => 1.0,
        value => value
            .strip_prefix("surface_fraction:")
            .context(
                "surface model must be recorded, working_plane, \
                 installed_plate_planes, or surface_fraction:F",
            )?
            .parse::<f64>()
            .context("surface fraction is not a number")?,
    };
    ensure!(
        (-1.0..=2.0).contains(&surface_fraction),
        "surface fraction must be between -1 and 2"
    );
    let registration_mode = arguments
        .get(5)
        .map(|value| value.to_string_lossy().into_owned())
        .unwrap_or_else(|| "registration".into());
    ensure!(
        matches!(
            registration_mode.as_str(),
            "registration" | "no_registration"
        ),
        "registration mode must be registration or no_registration"
    );
    let working_model: PlanarCameraModel = serde_json::from_value(
        calibration["artifacts"]["top_camera_planar_model"]["value"]["model"].clone(),
    )
    .context("calibration has no top-camera planar model")?;
    let model = if surface_fraction == 0.0 {
        working_model
    } else {
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
        let installed_model = rescale_planar_camera_model(&working_model, mean_mapping)
            .map_err(anyhow::Error::msg)?;
        interpolate_planar_camera_model(&working_model, &installed_model, surface_fraction)
            .map_err(anyhow::Error::msg)?
    };
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
    let mut capture_sequence = report.tiles.iter().collect::<Vec<_>>();
    capture_sequence.sort_by_key(|tile| tile.capture_order);
    let mut stream = if registration_mode == "registration" {
        WorkspaceMosaicStream::new(plan, model)?
    } else {
        WorkspaceMosaicStream::new_without_registration(plan, model)?
    };
    for tile in capture_sequence {
        stream.push_frame(tile.index, frames[tile.index as usize].clone())?;
    }
    let mosaic = stream.finish()?;
    let accepted_registrations = mosaic
        .registrations
        .iter()
        .filter(|registration| registration.accepted)
        .count();
    let applied_corrections = mosaic
        .tiles
        .iter()
        .filter(|tile| {
            tile.placement_correction_x_mm != 0.0 || tile.placement_correction_y_mm != 0.0
        })
        .count();

    fs::write(&output_png_path, encode_gray_png(&mosaic.frame)?)
        .with_context(|| format!("failed to write {}", output_png_path.display()))?;
    let replay_report = json!({
        "source_scan_id": report.id,
        "surface_model_override": surface_model_override,
        "surface_fraction": surface_fraction,
        "registration_mode": registration_mode,
        "covered_fraction": mosaic.covered_fraction,
        "nominal_fallback_fraction": mosaic.nominal_fallback_fraction,
        "attempted_registrations": mosaic.registrations.len(),
        "accepted_registrations": accepted_registrations,
        "applied_corrections": applied_corrections,
        "tiles": mosaic.tiles,
        "registrations": mosaic.registrations,
    });
    fs::write(
        &output_json_path,
        serde_json::to_vec_pretty(&replay_report)?,
    )
    .with_context(|| format!("failed to write {}", output_json_path.display()))?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "source_scan_id": report.id,
            "surface_model_override": surface_model_override,
            "surface_fraction": surface_fraction,
            "registration_mode": registration_mode,
            "covered_fraction": mosaic.covered_fraction,
            "nominal_fallback_fraction": mosaic.nominal_fallback_fraction,
            "attempted_registrations": mosaic.registrations.len(),
            "accepted_registrations": accepted_registrations,
            "applied_corrections": applied_corrections,
            "output_png": output_png_path,
            "output_json": output_json_path,
        }))?
    );
    Ok(())
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let data = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_json::from_slice(&data).with_context(|| format!("failed to parse {}", path.display()))
}
