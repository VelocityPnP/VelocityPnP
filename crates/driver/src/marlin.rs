use crate::MachineDriver;
use anyhow::{bail, Context, Result};
use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::time::{timeout, Instant};
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use velocitypnp_domain::{HomingAxis, HomingProfile, Pose};

const DISCONNECT_FAILSAFE_COMMANDS: [&str; 5] =
    ["M107 P1", "M107", "M107 P2", "M107 P3", "M150 P0"];
const MIN_MOVE_COMPLETION_TIMEOUT: Duration = Duration::from_secs(60);
const MAX_MOVE_COMPLETION_TIMEOUT: Duration = Duration::from_secs(15 * 60);
const MOVE_TIMEOUT_MARGIN_SECONDS: f64 = 15.0;
const MOVE_TIMEOUT_SAFETY_FACTOR: f64 = 1.5;

pub struct MarlinDriver {
    path: String,
    baud: u32,
    home_pose: Pose,
    homing: HomingProfile,
    emergency_flag: Arc<AtomicBool>,
    stream: Option<BufReader<SerialStream>>,
}

impl MarlinDriver {
    pub fn new(path: impl Into<String>, baud: u32, home_pose: Pose, homing: HomingProfile) -> Self {
        Self {
            path: path.into(),
            baud,
            home_pose,
            homing,
            emergency_flag: Arc::new(AtomicBool::new(false)),
            stream: None,
        }
    }

    async fn command(&mut self, command: &str, command_timeout: Duration) -> Result<Vec<String>> {
        let stream = self
            .stream
            .as_mut()
            .context("Marlin controller is disconnected")?;
        if self.emergency_flag.swap(false, Ordering::AcqRel) {
            stream.get_mut().write_all(b"M112\n").await?;
            stream.get_mut().flush().await?;
            bail!("command {command} interrupted by emergency stop");
        }
        stream
            .get_mut()
            .write_all(format!("{command}\n").as_bytes())
            .await
            .with_context(|| format!("writing Marlin command {command}"))?;
        stream.get_mut().flush().await?;

        let deadline = Instant::now() + command_timeout;
        let mut lines = Vec::new();
        loop {
            if self.emergency_flag.swap(false, Ordering::AcqRel) {
                stream.get_mut().write_all(b"M112\n").await?;
                stream.get_mut().flush().await?;
                bail!("command {command} interrupted by emergency stop");
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                bail!("controller timed out after {command_timeout:?} for {command}");
            }
            let mut line = String::new();
            let count = match timeout(
                remaining.min(Duration::from_millis(20)),
                stream.read_line(&mut line),
            )
            .await
            {
                Ok(result) => result?,
                Err(_) => continue,
            };
            if count == 0 {
                bail!("controller disconnected while waiting for {command}");
            }
            let line = line.trim().to_string();
            if line.starts_with("Error:") || line.starts_with("echo:busy: paused for user") {
                bail!("controller rejected {command}: {line}");
            }
            let done = line.starts_with("ok");
            if !line.is_empty() {
                lines.push(line);
            }
            if done {
                return Ok(lines);
            }
        }
    }

    async fn reported_position(&mut self) -> Result<MarlinPosition> {
        let lines = self.command("M114", Duration::from_secs(5)).await?;
        parse_position_report(&lines)
            .with_context(|| format!("decoding M114 response: {}", lines.join(" | ")))
    }
}

#[async_trait]
impl MachineDriver for MarlinDriver {
    fn simulated(&self) -> bool {
        false
    }

    fn set_emergency_flag(&mut self, flag: Arc<AtomicBool>) {
        self.emergency_flag = flag;
    }

    async fn connect(&mut self) -> Result<String> {
        let serial = tokio_serial::new(&self.path, self.baud)
            .timeout(Duration::from_secs(5))
            .open_native_async()
            .with_context(|| format!("opening {}", self.path))?;
        self.stream = Some(BufReader::new(serial));
        // Controllers commonly reset on CDC open. Drain startup lines by waiting
        // before the first bounded command instead of treating them as replies.
        tokio::time::sleep(Duration::from_secs(3)).await;
        let firmware = self
            .command("M115", Duration::from_secs(5))
            .await?
            .join("\n");
        if !supported_firmware(&firmware) {
            self.stream = None;
            bail!("connected device did not identify as a supported Marlin controller");
        }
        for setup in ["G21", "G90", "M82"] {
            self.command(setup, Duration::from_secs(5)).await?;
        }
        Ok(firmware)
    }

    async fn disconnect(&mut self) -> Result<()> {
        let mut failures = Vec::new();
        if self.stream.is_some() {
            for command in DISCONNECT_FAILSAFE_COMMANDS {
                if let Err(error) = self.command(command, Duration::from_secs(2)).await {
                    failures.push(format!("{command}: {error}"));
                }
            }
        }
        self.stream = None;
        if failures.is_empty() {
            Ok(())
        } else {
            bail!(
                "controller disconnected after actuator fail-safe errors: {}",
                failures.join("; ")
            )
        }
    }

    async fn home(&mut self) -> Result<Pose> {
        let homing = self.homing.clone();
        validate_homing_profile(&homing)?;
        let homing_timeout = Duration::from_secs(homing.timeout_seconds);
        let mut reported = None;
        for stage in &homing.stages {
            self.command(&stage.command, homing_timeout)
                .await
                .with_context(|| format!("executing homing stage {}", stage.id))?;
            self.command("M400", homing_timeout)
                .await
                .with_context(|| format!("waiting for homing stage {}", stage.id))?;
            let stage_position = self
                .reported_position()
                .await
                .with_context(|| format!("reading position after homing stage {}", stage.id))?;
            verify_homed(
                stage_position,
                self.home_pose,
                &stage.verify_axes,
                homing.reported_position_tolerance_mm,
            )
            .with_context(|| format!("verifying homing stage {}", stage.id))?;
            reported = Some(stage_position);
        }
        Ok(reported
            .context("homing profile has no stages")?
            .primary_pose)
    }

    async fn move_to(&mut self, target: Pose, feed_mm_per_min: f64) -> Result<Pose> {
        if !target.finite() || !feed_mm_per_min.is_finite() || feed_mm_per_min <= 0.0 {
            bail!("invalid motion values");
        }
        let start = self
            .reported_position()
            .await
            .context("reading position before motion")?
            .primary_pose;
        let completion_timeout = move_completion_timeout(start, target, feed_mm_per_min);
        let command = format!(
            "G0 X{:.4} Y{:.4} Z{:.4} A{:.4} F{:.0}",
            target.x_mm, target.y_mm, target.z_mm, target.rotation_deg, feed_mm_per_min
        );
        self.command(&command, Duration::from_secs(30)).await?;
        self.command("M400", completion_timeout)
            .await
            .with_context(|| {
                format!(
                    "waiting up to {:.1}s for motion completion",
                    completion_timeout.as_secs_f64()
                )
            })?;
        Ok(target)
    }

    async fn actuate(&mut self, actuator_id: &str, enabled: bool) -> Result<()> {
        for line in actuator_commands(actuator_id, enabled)? {
            self.command(line, Duration::from_secs(5)).await?;
        }
        Ok(())
    }

    async fn emergency_stop(&mut self) -> Result<()> {
        if let Some(stream) = self.stream.as_mut() {
            stream.get_mut().write_all(b"M112\n").await?;
            stream.get_mut().flush().await?;
        }
        Ok(())
    }
}

fn move_completion_timeout(start: Pose, target: Pose, feed_mm_per_min: f64) -> Duration {
    let distance = ((target.x_mm - start.x_mm).powi(2)
        + (target.y_mm - start.y_mm).powi(2)
        + (target.z_mm - start.z_mm).powi(2)
        + (target.rotation_deg - start.rotation_deg).powi(2))
    .sqrt();
    let nominal_seconds = distance * 60.0 / feed_mm_per_min;
    let timeout_seconds =
        nominal_seconds * MOVE_TIMEOUT_SAFETY_FACTOR + MOVE_TIMEOUT_MARGIN_SECONDS;
    Duration::from_secs_f64(timeout_seconds.clamp(
        MIN_MOVE_COMPLETION_TIMEOUT.as_secs_f64(),
        MAX_MOVE_COMPLETION_TIMEOUT.as_secs_f64(),
    ))
}

fn actuator_commands(actuator_id: &str, enabled: bool) -> Result<&'static [&'static str]> {
    match (actuator_id, enabled) {
        ("lights", true) => Ok(&["M150 P255 R255 U255 B255"]),
        ("lights", false) => Ok(&["M150 P0"]),
        ("vacuum_1", true) => Ok(&["M106 P1 S255", "M106"]),
        ("vacuum_1", false) => Ok(&["M107 P1", "M107"]),
        ("vacuum_2", true) => Ok(&["M106 P2 S255", "M106 P3 S255"]),
        ("vacuum_2", false) => Ok(&["M107 P2", "M107 P3"]),
        _ => bail!("actuator {actuator_id} is not supported by the active profile"),
    }
}

fn supported_firmware(firmware: &str) -> bool {
    firmware.contains("FIRMWARE_NAME:Marlin") && firmware.contains("MACHINE_TYPE:LumenPnP")
}

fn validate_homing_profile(homing: &HomingProfile) -> Result<()> {
    if homing.stages.is_empty()
        || homing.timeout_seconds == 0
        || !homing.reported_position_tolerance_mm.is_finite()
        || homing.reported_position_tolerance_mm < 0.0
    {
        bail!("invalid homing configuration");
    }
    let mut verified = [false; 3];
    for stage in &homing.stages {
        if stage.id.trim().is_empty()
            || stage.command.trim().is_empty()
            || stage.verify_axes.is_empty()
        {
            bail!("invalid homing stage configuration");
        }
        for axis in &stage.verify_axes {
            verified[match axis {
                HomingAxis::X => 0,
                HomingAxis::Y => 1,
                HomingAxis::Z => 2,
            }] = true;
        }
    }
    if !verified.into_iter().all(|axis| axis) {
        bail!("homing stages must verify X, Y, and Z");
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct MarlinPosition {
    primary_pose: Pose,
    secondary_rotation_deg: f64,
}

fn parse_position_report(lines: &[String]) -> Result<MarlinPosition> {
    for line in lines {
        // Marlin may append step counts that repeat the axis labels. Only the
        // user-coordinate portion before "Count" describes the current pose.
        let coordinates = line.split("Count").next().unwrap_or(line);
        let mut x = None;
        let mut y = None;
        let mut z = None;
        let mut a = None;
        let mut b = None;
        for field in coordinates.split_ascii_whitespace() {
            let Some((label, value)) = field.split_once(':') else {
                continue;
            };
            let slot = match label {
                "X" => &mut x,
                "Y" => &mut y,
                "Z" => &mut z,
                "A" => &mut a,
                "B" => &mut b,
                _ => continue,
            };
            let parsed = value
                .trim_end_matches(',')
                .parse::<f64>()
                .with_context(|| format!("invalid {label} coordinate {value}"))?;
            if !parsed.is_finite() {
                bail!("non-finite {label} coordinate {value}");
            }
            *slot = Some(parsed);
        }
        if let (Some(x_mm), Some(y_mm), Some(z_mm), Some(rotation_deg), Some(secondary)) =
            (x, y, z, a, b)
        {
            return Ok(MarlinPosition {
                primary_pose: Pose {
                    x_mm,
                    y_mm,
                    z_mm,
                    rotation_deg,
                },
                secondary_rotation_deg: secondary,
            });
        }
    }
    bail!("M114 response did not contain finite X, Y, Z, A, and B coordinates")
}

fn verify_homed(
    reported: MarlinPosition,
    expected: Pose,
    axes: &[HomingAxis],
    tolerance_mm: f64,
) -> Result<()> {
    let failures = axes
        .iter()
        .map(|axis| match axis {
            HomingAxis::X => ("X", (reported.primary_pose.x_mm - expected.x_mm).abs()),
            HomingAxis::Y => ("Y", (reported.primary_pose.y_mm - expected.y_mm).abs()),
            HomingAxis::Z => ("Z", (reported.primary_pose.z_mm - expected.z_mm).abs()),
        })
        .filter(|(_, delta)| *delta > tolerance_mm)
        .map(|(axis, delta)| format!("{axis} delta {delta:.3} mm"))
        .collect::<Vec<_>>();
    if failures.is_empty() {
        Ok(())
    } else {
        bail!(
            "homing verification failed ({}) at X{:.3} Y{:.3} Z{:.3} A{:.3} B{:.3}",
            failures.join(", "),
            reported.primary_pose.x_mm,
            reported.primary_pose.y_mm,
            reported.primary_pose.z_mm,
            reported.primary_pose.rotation_deg,
            reported.secondary_rotation_deg,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn homing_profile() -> HomingProfile {
        HomingProfile {
            method: "sensorless".into(),
            operator_checks: Vec::new(),
            stages: vec![
                velocitypnp_domain::HomingStageProfile {
                    id: "nozzle_z".into(),
                    command: "G28 Z".into(),
                    verify_axes: vec![HomingAxis::Z],
                },
                velocitypnp_domain::HomingStageProfile {
                    id: "gantry_xy".into(),
                    command: "G28 X Y".into(),
                    verify_axes: vec![HomingAxis::X, HomingAxis::Y, HomingAxis::Z],
                },
            ],
            timeout_seconds: 120,
            reported_position_tolerance_mm: 0.05,
            requires_visual_refinement: true,
        }
    }

    #[test]
    fn actuator_commands_match_the_v3_0_4_profile() {
        assert_eq!(
            actuator_commands("lights", true).unwrap(),
            ["M150 P255 R255 U255 B255"]
        );
        assert_eq!(actuator_commands("lights", false).unwrap(), ["M150 P0"]);
        assert_eq!(
            actuator_commands("vacuum_1", true).unwrap(),
            ["M106 P1 S255", "M106"]
        );
        assert_eq!(
            actuator_commands("vacuum_1", false).unwrap(),
            ["M107 P1", "M107"]
        );
        assert_eq!(
            actuator_commands("vacuum_2", true).unwrap(),
            ["M106 P2 S255", "M106 P3 S255"]
        );
        assert_eq!(
            actuator_commands("vacuum_2", false).unwrap(),
            ["M107 P2", "M107 P3"]
        );
        assert!(actuator_commands("unknown", false).is_err());
    }

    #[test]
    fn disconnect_fail_safe_turns_every_validated_output_off() {
        assert_eq!(
            DISCONNECT_FAILSAFE_COMMANDS,
            ["M107 P1", "M107", "M107 P2", "M107 P3", "M150 P0"]
        );
    }

    #[test]
    fn move_timeout_covers_the_observed_conservative_bottom_camera_traverse() {
        let start = Pose {
            x_mm: 0.0,
            y_mm: 0.0,
            z_mm: 31.5,
            rotation_deg: 720.0,
        };
        let target = Pose {
            x_mm: 242.335,
            y_mm: 215.353,
            z_mm: 31.5,
            rotation_deg: 720.0,
        };

        let timeout = move_completion_timeout(start, target, 300.0);
        assert!(timeout > Duration::from_secs(110));
        assert!(timeout < Duration::from_secs(115));
    }

    #[test]
    fn move_timeout_is_bounded_for_short_and_pathological_moves() {
        assert_eq!(
            move_completion_timeout(Pose::ORIGIN, Pose::ORIGIN, 300.0),
            MIN_MOVE_COMPLETION_TIMEOUT
        );
        assert_eq!(
            move_completion_timeout(
                Pose::ORIGIN,
                Pose {
                    rotation_deg: 100_000.0,
                    ..Pose::ORIGIN
                },
                1.0,
            ),
            MAX_MOVE_COMPLETION_TIMEOUT
        );
    }

    #[test]
    fn captured_rev4_firmware_is_recognized() {
        let firmware = include_str!("../tests/fixtures/lumenpnp-v3.0.4-rev4-m115.txt");
        assert!(supported_firmware(firmware));
        assert!(!supported_firmware(
            "FIRMWARE_NAME:Marlin 2.1\nMACHINE_TYPE:3D Printer\nok"
        ));
        assert!(!supported_firmware("MACHINE_TYPE:LumenPnP\nok"));
    }

    #[test]
    fn parses_position_before_marlin_step_counts() {
        let report = parse_position_report(&[
            "X:0.00 Y:0.00 Z:31.50 A:0.00 B:0.00 C:0.00 Count X:0 Y:0 Z:25200 A:0 B:0".into(),
            "ok".into(),
        ])
        .unwrap();
        assert_eq!(
            report,
            MarlinPosition {
                primary_pose: Pose {
                    x_mm: 0.0,
                    y_mm: 0.0,
                    z_mm: 31.5,
                    rotation_deg: 0.0,
                },
                secondary_rotation_deg: 0.0,
            }
        );
    }

    #[test]
    fn parses_attached_rev4_unhomed_position_fixture() {
        let lines = include_str!("../tests/fixtures/lumenpnp-v3.0.4-rev4-m114-unhomed.txt")
            .lines()
            .map(str::to_owned)
            .collect::<Vec<_>>();
        let report = parse_position_report(&lines).unwrap();
        assert_eq!(report.primary_pose.rotation_deg, 720.0);
        assert_eq!(report.secondary_rotation_deg, 720.0);
    }

    #[test]
    fn rejects_incomplete_or_non_finite_position_reports() {
        assert!(parse_position_report(&["X:0 Y:0 Z:31.5 A:0 ok".into()]).is_err());
        assert!(parse_position_report(&["X:nan Y:0 Z:31.5 A:0 B:0".into()]).is_err());
    }

    #[test]
    fn verifies_linear_home_while_preserving_reported_rotations() {
        let expected = Pose {
            x_mm: 0.0,
            y_mm: 0.0,
            z_mm: 31.5,
            rotation_deg: 0.0,
        };
        let mut reported = MarlinPosition {
            primary_pose: expected,
            secondary_rotation_deg: 0.0,
        };
        assert!(verify_homed(reported, expected, &[HomingAxis::X], 0.05).is_ok());

        reported.primary_pose.x_mm = 0.051;
        let error = verify_homed(reported, expected, &[HomingAxis::X], 0.05)
            .unwrap_err()
            .to_string();
        assert!(error.contains("X delta 0.051 mm"));

        reported.primary_pose.x_mm = 0.0;
        reported.primary_pose.rotation_deg = 720.0;
        reported.secondary_rotation_deg = 720.0;
        assert!(verify_homed(
            reported,
            expected,
            &[HomingAxis::X, HomingAxis::Y, HomingAxis::Z],
            0.05,
        )
        .is_ok());
    }

    #[test]
    fn v3_profile_homes_and_verifies_z_before_xy() {
        let homing = homing_profile();
        assert!(validate_homing_profile(&homing).is_ok());
        assert_eq!(homing.stages[0].command, "G28 Z");
        assert_eq!(homing.stages[0].verify_axes, [HomingAxis::Z]);
        assert_eq!(homing.stages[1].command, "G28 X Y");
        assert_eq!(
            homing.stages[1].verify_axes,
            [HomingAxis::X, HomingAxis::Y, HomingAxis::Z]
        );
    }

    #[test]
    fn rejects_homing_profiles_that_do_not_verify_every_linear_axis() {
        let mut homing = homing_profile();
        homing.stages.pop();
        let error = validate_homing_profile(&homing).unwrap_err().to_string();
        assert!(error.contains("verify X, Y, and Z"));
    }
}
