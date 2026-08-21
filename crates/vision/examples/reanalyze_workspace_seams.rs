use anyhow::{ensure, Context, Result};
use serde_json::json;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use velocitypnp_domain::WorkspaceScanReport;
use velocitypnp_vision::{
    decode_gray_png, encode_gray_png, encode_rgb_png, encode_rgba_png,
    reanalyze_workspace_seam_health, WorkspaceScanPlan, WorkspaceSeamHealthReport,
};

fn main() -> Result<()> {
    let arguments = env::args_os().skip(1).collect::<Vec<_>>();
    ensure!(
        arguments.len() == 2,
        "usage: reanalyze_workspace_seams SCAN_DIR OUTPUT_DIR"
    );
    let scan_directory = PathBuf::from(&arguments[0]);
    let output_directory = PathBuf::from(&arguments[1]);
    let scan: WorkspaceScanReport = read_json(&scan_directory.join("manifest.json"))?;
    let prior: WorkspaceSeamHealthReport = read_json(&scan_directory.join("seam-health.json"))?;
    let plan = WorkspaceScanPlan {
        region: scan.region,
        overlap_fraction: scan.overlap_fraction,
        output_mm_per_pixel: scan.output_mm_per_pixel,
        frame_width_px: scan.frame_width_px,
        frame_height_px: scan.frame_height_px,
        usable_pixel_bounds: scan.usable_pixel_bounds,
        rows: scan.rows,
        columns: scan.columns,
        tiles: scan.tiles,
    };
    let mosaic = read_gray(&scan_directory.join("mosaic.png"))?;
    let disagreement = read_gray(&scan_directory.join("mosaic-disagreement.png"))?;
    let overlap_confidence = read_gray(&scan_directory.join("overlap-confidence.png"))?;
    let artifacts = reanalyze_workspace_seam_health(
        &plan,
        &mosaic,
        &disagreement,
        &overlap_confidence,
        &prior,
    )?;

    fs::create_dir_all(&output_directory).with_context(|| {
        format!(
            "failed to create output directory {}",
            output_directory.display()
        )
    })?;
    write(
        &output_directory.join("seam-health.json"),
        &serde_json::to_vec_pretty(&artifacts.report)?,
    )?;
    write(
        &output_directory.join("overlap-confidence.png"),
        &encode_gray_png(&artifacts.overlap_confidence)?,
    )?;
    write(
        &output_directory.join("seam-health-overlay.png"),
        &encode_rgba_png(&artifacts.overlay)?,
    )?;
    write(
        &output_directory.join("mosaic-seam-health.png"),
        &encode_rgb_png(&artifacts.preview)?,
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "source_scan_id": scan.id,
            "total_seams": artifacts.report.total_seams,
            "healthy_seams": artifacts.report.healthy_seams,
            "elevated_seams": artifacts.report.elevated_seams,
            "unresolved_seams": artifacts.report.unresolved_seams,
            "recommended_action": artifacts.report.recommended_action,
            "target_regions": artifacts.report.target_regions,
            "output_directory": output_directory,
        }))?
    );
    Ok(())
}

fn read_gray(path: &Path) -> Result<velocitypnp_vision::GrayFrame> {
    let encoded = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    decode_gray_png(&encoded).with_context(|| format!("failed to decode {}", path.display()))
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let data = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    serde_json::from_slice(&data).with_context(|| format!("failed to parse {}", path.display()))
}

fn write(path: &Path, data: &[u8]) -> Result<()> {
    fs::write(path, data).with_context(|| format!("failed to write {}", path.display()))
}
