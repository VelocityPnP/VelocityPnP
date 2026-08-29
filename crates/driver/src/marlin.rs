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
const VACUUM_SENSOR_INITIALIZATION_COMMANDS: [&str; 10] = [
    "M260 A112 B1 S1",
    "M260 A109",
    "M260 B48",
    "M260 B27",
    "M260 S1",
    "M260 A112 B2 S1",
    "M260 A109",
    "M260 B48",
    "M260 B27",
    "M260 S1",
];
const MIN_MOVE_COMPLETION_TIMEOUT: Duration = Duration::from_secs(60);
const MAX_MOVE_COMPLETION_TIMEOUT: Duration = Duration::from_secs(15 * 60);
const MOVE_TIMEOUT_MARGIN_SECONDS: f64 = 15.0;
const MOVE_TIMEOUT_SAFETY_FACTOR: f64 = 1.5;
const STOP_RESTART_SETTLE: Duration = Duration::from_secs(1);

pub struct MarlinDriver {
    path: String,
    baud: u32,
    home_pose: Pose,
    homing: HomingProfile,
    rev04_integrated_vacuum_sensors: bool,
    emergency_flag: Arc<AtomicBool>,
    controller_stopped: bool,
    stream: Option<BufReader<SerialStream>>,
}

impl MarlinDriver {
    pub fn new(
        path: impl Into<String>,
        baud: u32,
        home_pose: Pose,
        homing: HomingProfile,
        rev04_integrated_vacuum_sensors: bool,
    ) -> Self {
        Self {
            path: path.into(),
            baud,
            home_pose,
            homing,
            rev04_integrated_vacuum_sensors,
            emergency_flag: Arc::new(AtomicBool::new(false)),
            controller_stopped: false,
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
            self.controller_stopped = true;
            bail!("command {command} interrupted by emergency stop");
        }
        stream
            .get_mut()
            .write_all(format!("{command}\n").as_bytes())
            .await
            .with_context(|| format!("writing Marlin command {command}"))?;

        let deadline = Instant::now() + command_timeout;
        let mut lines = Vec::new();
        loop {
            if self.emergency_flag.swap(false, Ordering::AcqRel) {
                stream.get_mut().write_all(b"M112\n").await?;
                self.controller_stopped = true;
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

    async fn restart_stopped_controller(&mut self) -> Result<()> {
        let stream = self
            .stream
            .as_mut()
            .context("Marlin controller is disconnected")?;
        stream
            .get_mut()
            .write_all(b"M999\n")
            .await
            .context("writing Marlin stop-restart command M999")?;
        tokio::time::sleep(STOP_RESTART_SETTLE).await;
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            let mut line = String::new();
            match timeout(
                remaining.min(Duration::from_millis(100)),
                stream.read_line(&mut line),
            )
            .await
            {
                Ok(Ok(0)) | Err(_) => break,
                Ok(Ok(_)) => {}
                Ok(Err(error)) => return Err(error).context("draining Marlin M999 reply"),
            }
        }
        self.controller_stopped = false;
        Ok(())
    }

    async fn connect_inner(&mut self) -> Result<String> {
        let serial = tokio_serial::new(&self.path, self.baud)
            .timeout(Duration::from_secs(5))
            .open_native_async()
            .with_context(|| format!("opening {}", self.path))?;
        self.stream = Some(BufReader::new(serial));
        // Controllers commonly reset on CDC open. Drain startup lines by waiting
        // before the first bounded command instead of treating them as replies.
        tokio::time::sleep(Duration::from_secs(3)).await;
        self.restart_stopped_controller()
            .await
            .context("normalizing Marlin running state before identification")?;
        let firmware = self
            .command("M115", Duration::from_secs(5))
            .await?
            .join("\n");
        if !supported_firmware(&firmware) {
            bail!("connected device did not identify as a supported Marlin controller");
        }
        for setup in ["G21", "G90", "M82"] {
            self.command(setup, Duration::from_secs(5)).await?;
        }
        if self.rev04_integrated_vacuum_sensors {
            for setup in VACUUM_SENSOR_INITIALIZATION_COMMANDS {
                self.command(setup, Duration::from_secs(5))
                    .await
                    .with_context(|| {
                        format!("initializing integrated vacuum sensors with {setup}")
                    })?;
            }
        }
        Ok(firmware)
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
        let result = self.connect_inner().await;
        if result.is_err() {
            self.stream = None;
            self.controller_stopped = false;
        }
        result
    }

    async fn disconnect(&mut self) -> Result<()> {
        let mut failures = Vec::new();
        if self.stream.is_some() && !self.controller_stopped {
            for command in DISCONNECT_FAILSAFE_COMMANDS {
                if let Err(error) = self.command(command, Duration::from_secs(2)).await {
                    failures.push(format!("{command}: {error}"));
                }
            }
        }
        self.stream = None;
        self.controller_stopped = false;
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

    async fn rotation_position(&mut self, controller_axis: char) -> Result<f64> {
        let reported = self
            .reported_position()
            .await
            .context("reading nozzle rotation position")?;
        rotation_axis_value(reported, controller_axis)
    }

    async fn rotate_nozzle(
        &mut self,
        controller_axis: char,
        target_deg: f64,
        feed_deg_per_min: f64,
    ) -> Result<f64> {
        if !target_deg.is_finite() || !feed_deg_per_min.is_finite() || feed_deg_per_min <= 0.0 {
            bail!("invalid nozzle rotation values");
        }
        let reported = self
            .reported_position()
            .await
            .context("reading position before nozzle rotation")?;
        let start_deg = rotation_axis_value(reported, controller_axis)?;
        let completion_timeout =
            rotation_completion_timeout(start_deg, target_deg, feed_deg_per_min);
        let command = nozzle_rotation_command(controller_axis, target_deg, feed_deg_per_min)?;
        self.command(&command, Duration::from_secs(30)).await?;
        self.command("M400", completion_timeout)
            .await
            .with_context(|| {
                format!(
                    "waiting up to {:.1}s for nozzle rotation",
                    completion_timeout.as_secs_f64()
                )
            })?;
        let actual = rotation_axis_value(
            self.reported_position()
                .await
                .context("reading position after nozzle rotation")?,
            controller_axis,
        )?;
        if (actual - target_deg).abs() > 0.05 {
            bail!("rotation axis {controller_axis} reported {actual:.3}, expected {target_deg:.3}");
        }
        Ok(actual)
    }

    async fn set_xy_coordinates(&mut self, x_mm: f64, y_mm: f64) -> Result<Pose> {
        let command = set_xy_coordinates_command(x_mm, y_mm)?;
        self.command(&command, Duration::from_secs(5)).await?;
        let reported = self
            .reported_position()
            .await
            .context("reading position after XY coordinate reset")?
            .primary_pose;
        if (reported.x_mm - x_mm).abs() > 0.01 || (reported.y_mm - y_mm).abs() > 0.01 {
            bail!(
                "XY coordinate reset reported X{:.3} Y{:.3}, expected X{x_mm:.3} Y{y_mm:.3}",
                reported.x_mm,
                reported.y_mm
            );
        }
        Ok(reported)
    }

    async fn set_z_coordinate(&mut self, z_mm: f64) -> Result<Pose> {
        let command = set_z_coordinate_command(z_mm)?;
        self.command(&command, Duration::from_secs(5)).await?;
        let reported = self
            .reported_position()
            .await
            .context("reading position after Z coordinate reset")?
            .primary_pose;
        if (reported.z_mm - z_mm).abs() > 0.01 {
            bail!(
                "Z coordinate reset reported Z{:.3}, expected Z{z_mm:.3}",
                reported.z_mm
            );
        }
        Ok(reported)
    }

    async fn actuate(&mut self, actuator_id: &str, enabled: bool) -> Result<()> {
        for line in actuator_commands(actuator_id, enabled)? {
            if let Err(error) = self.command(line, Duration::from_secs(5)).await {
                if enabled {
                    for off in actuator_commands(actuator_id, false).unwrap_or_default() {
                        let _ = self.command(off, Duration::from_secs(2)).await;
                    }
                }
                return Err(error)
                    .with_context(|| format!("setting actuator {actuator_id} enabled={enabled}"));
            }
        }
        Ok(())
    }

    async fn read_sensor(&mut self, sensor_id: &str) -> Result<i64> {
        if !self.rev04_integrated_vacuum_sensors {
            bail!("REV04 integrated vacuum sensors are disabled for the active profile");
        }
        let mut response = Vec::new();
        for line in vacuum_sensor_read_commands(sensor_id)? {
            response = self.command(line, Duration::from_secs(5)).await?;
        }
        parse_vacuum_sensor_report(&response)
            .with_context(|| format!("decoding {sensor_id} response: {}", response.join(" | ")))
    }

    async fn emergency_stop(&mut self) -> Result<()> {
        if let Some(stream) = self.stream.as_mut() {
            stream.get_mut().write_all(b"M112\n").await?;
            self.controller_stopped = true;
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

fn rotation_completion_timeout(start_deg: f64, target_deg: f64, feed_deg_per_min: f64) -> Duration {
    let nominal_seconds = (target_deg - start_deg).abs() * 60.0 / feed_deg_per_min;
    let timeout_seconds =
        nominal_seconds * MOVE_TIMEOUT_SAFETY_FACTOR + MOVE_TIMEOUT_MARGIN_SECONDS;
    Duration::from_secs_f64(timeout_seconds.clamp(
        MIN_MOVE_COMPLETION_TIMEOUT.as_secs_f64(),
        MAX_MOVE_COMPLETION_TIMEOUT.as_secs_f64(),
    ))
}

fn nozzle_rotation_command(
    controller_axis: char,
    target_deg: f64,
    feed_deg_per_min: f64,
) -> Result<String> {
    if !matches!(controller_axis, 'A' | 'B') {
        bail!("unsupported nozzle rotation axis {controller_axis}");
    }
    if !target_deg.is_finite() || !feed_deg_per_min.is_finite() || feed_deg_per_min <= 0.0 {
        bail!("invalid nozzle rotation values");
    }
    Ok(format!(
        "G0 {controller_axis}{target_deg:.4} F{feed_deg_per_min:.0}"
    ))
}

fn set_xy_coordinates_command(x_mm: f64, y_mm: f64) -> Result<String> {
    if !x_mm.is_finite() || !y_mm.is_finite() {
        bail!("XY coordinate reset values must be finite");
    }
    Ok(format!("G92 X{x_mm:.4} Y{y_mm:.4}"))
}

fn set_z_coordinate_command(z_mm: f64) -> Result<String> {
    if !z_mm.is_finite() {
        bail!("Z coordinate reset value must be finite");
    }
    Ok(format!("G92 Z{z_mm:.4}"))
}

fn actuator_commands(actuator_id: &str, enabled: bool) -> Result<&'static [&'static str]> {
    match (actuator_id, enabled) {
        ("lights", true) => Ok(&["M150 P255 R255 U255 B255"]),
        ("lights", false) => Ok(&["M150 P0"]),
        ("vacuum_1", true) => Ok(&["M106 P1 S255", "M106"]),
        ("vacuum_1", false) => Ok(&["M107 P1", "M107"]),
        ("vacuum_2", true) => Ok(&["M106 P2 S255", "M106 P3 S255"]),
        ("vacuum_2", false) => Ok(&["M107 P2", "M107 P3"]),
        ("blowoff_1", true) => Ok(&["M106 P1 S255"]),
        ("blowoff_1", false) => Ok(&["M107 P1"]),
        ("blowoff_2", true) => Ok(&["M106 P2 S255"]),
        ("blowoff_2", false) => Ok(&["M107 P2"]),
        _ => bail!("actuator {actuator_id} is not supported by the active profile"),
    }
}

fn vacuum_sensor_read_commands(sensor_id: &str) -> Result<&'static [&'static str]> {
    match sensor_id {
        "vacuum_1" => Ok(&["M260 A112 B1 S1", "M260 A109 B6 S1", "M261 A109 B1 S2"]),
        "vacuum_2" => Ok(&["M260 A112 B2 S1", "M260 A109 B6 S1", "M261 A109 B1 S2"]),
        _ => bail!("sensor {sensor_id} is not supported by the active profile"),
    }
}

fn parse_vacuum_sensor_report(lines: &[String]) -> Result<i64> {
    for line in lines {
        let Some((_, data)) = line.split_once("data:") else {
            continue;
        };
        let token = data
            .trim()
            .split(|character: char| character.is_ascii_whitespace() || character == ',')
            .next()
            .filter(|token| !token.is_empty())
            .context("vacuum sensor data field is empty")?;
        let value = token
            .parse::<i64>()
            .with_context(|| format!("invalid vacuum sensor value {token}"))?;
        if !(-32_768..=32_767).contains(&value) {
            bail!("vacuum sensor value {value} is outside the signed 16-bit range");
        }
        return Ok(value);
    }
    bail!("M261 response did not contain a data field")
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

fn rotation_axis_value(reported: MarlinPosition, controller_axis: char) -> Result<f64> {
    match controller_axis {
        'A' => Ok(reported.primary_pose.rotation_deg),
        'B' => Ok(reported.secondary_rotation_deg),
        _ => bail!("unsupported nozzle rotation axis {controller_axis}"),
    }
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
            stages: vec![velocitypnp_domain::HomingStageProfile {
                id: "machine_xyz".into(),
                command: "G28".into(),
                verify_axes: vec![HomingAxis::X, HomingAxis::Y, HomingAxis::Z],
            }],
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
        assert_eq!(
            actuator_commands("blowoff_1", true).unwrap(),
            ["M106 P1 S255"]
        );
        assert_eq!(actuator_commands("blowoff_1", false).unwrap(), ["M107 P1"]);
        assert_eq!(
            actuator_commands("blowoff_2", true).unwrap(),
            ["M106 P2 S255"]
        );
        assert_eq!(actuator_commands("blowoff_2", false).unwrap(), ["M107 P2"]);
        assert!(actuator_commands("unknown", false).is_err());
    }

    #[test]
    fn vacuum_sensor_commands_match_the_documented_rev04_sequence() {
        assert_eq!(
            &VACUUM_SENSOR_INITIALIZATION_COMMANDS[..5],
            [
                "M260 A112 B1 S1",
                "M260 A109",
                "M260 B48",
                "M260 B27",
                "M260 S1",
            ]
        );
        assert_eq!(
            vacuum_sensor_read_commands("vacuum_1").unwrap(),
            ["M260 A112 B1 S1", "M260 A109 B6 S1", "M261 A109 B1 S2",]
        );
        assert_eq!(
            vacuum_sensor_read_commands("vacuum_2").unwrap()[0],
            "M260 A112 B2 S1"
        );
        assert!(vacuum_sensor_read_commands("unknown").is_err());
    }

    #[test]
    fn parses_decimal_vacuum_sensor_reports() {
        assert_eq!(
            parse_vacuum_sensor_report(&[
                "echo:i2c-reply: from:109 bytes:1 data:-42".into(),
                "ok".into(),
            ])
            .unwrap(),
            -42
        );
        assert_eq!(
            parse_vacuum_sensor_report(&["i2c-reply: data:127, ok".into()]).unwrap(),
            127
        );
        assert!(parse_vacuum_sensor_report(&["ok".into()]).is_err());
        assert!(parse_vacuum_sensor_report(&["i2c-reply: data:nope".into()]).is_err());
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
    fn nozzle_rotation_commands_are_axis_specific_and_bounded_in_time() {
        assert_eq!(
            nozzle_rotation_command('A', 555.0, 600.0).unwrap(),
            "G0 A555.0000 F600"
        );
        assert_eq!(
            nozzle_rotation_command('B', 885.0, 600.0).unwrap(),
            "G0 B885.0000 F600"
        );
        assert!(nozzle_rotation_command('C', 0.0, 600.0).is_err());
        assert!(nozzle_rotation_command('A', f64::NAN, 600.0).is_err());
        assert!(nozzle_rotation_command('A', 0.0, 0.0).is_err());

        let timeout = rotation_completion_timeout(720.0, 555.0, 600.0);
        assert_eq!(timeout, MIN_MOVE_COMPLETION_TIMEOUT);
        assert_eq!(
            rotation_completion_timeout(0.0, 100_000.0, 1.0),
            MAX_MOVE_COMPLETION_TIMEOUT
        );
    }

    #[test]
    fn coordinate_resets_are_bounded_to_the_requested_linear_axes() {
        assert_eq!(
            set_xy_coordinates_command(220.131, 190.271).unwrap(),
            "G92 X220.1310 Y190.2710"
        );
        assert!(set_xy_coordinates_command(f64::NAN, 0.0).is_err());
        assert_eq!(set_z_coordinate_command(3.2).unwrap(), "G92 Z3.2000");
        assert!(set_z_coordinate_command(f64::NAN).is_err());
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
    fn v3_profile_uses_combined_xyz_home_and_verifies_every_linear_axis() {
        let homing = homing_profile();
        assert!(validate_homing_profile(&homing).is_ok());
        assert_eq!(homing.stages.len(), 1);
        assert_eq!(homing.stages[0].command, "G28");
        assert_eq!(
            homing.stages[0].verify_axes,
            [HomingAxis::X, HomingAxis::Y, HomingAxis::Z]
        );
    }

    #[test]
    fn rejects_homing_profiles_that_do_not_verify_every_linear_axis() {
        let mut homing = homing_profile();
        homing.stages[0].verify_axes.pop();
        let error = validate_homing_profile(&homing).unwrap_err().to_string();
        assert!(error.contains("verify X, Y, and Z"));
    }
}
