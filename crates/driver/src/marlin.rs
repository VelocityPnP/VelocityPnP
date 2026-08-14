use crate::MachineDriver;
use anyhow::{bail, Context, Result};
use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::time::{timeout, Instant};
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use velocitypnp_domain::Pose;

pub struct MarlinDriver {
    path: String,
    baud: u32,
    home_pose: Pose,
    emergency_flag: Arc<AtomicBool>,
    stream: Option<BufReader<SerialStream>>,
}

impl MarlinDriver {
    pub fn new(path: impl Into<String>, baud: u32, home_pose: Pose) -> Self {
        Self {
            path: path.into(),
            baud,
            home_pose,
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
        if !firmware.contains("Marlin") && !firmware.contains("LumenPnP") {
            self.stream = None;
            bail!("connected device did not identify as a supported Marlin controller");
        }
        for setup in ["G21", "G90", "M82"] {
            self.command(setup, Duration::from_secs(5)).await?;
        }
        Ok(firmware)
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.stream = None;
        Ok(())
    }

    async fn home(&mut self) -> Result<Pose> {
        self.command("G28", Duration::from_secs(60)).await?;
        self.command("M400", Duration::from_secs(60)).await?;
        Ok(self.home_pose)
    }

    async fn move_to(&mut self, target: Pose, feed_mm_per_min: f64) -> Result<Pose> {
        if !target.finite() || !feed_mm_per_min.is_finite() || feed_mm_per_min <= 0.0 {
            bail!("invalid motion values");
        }
        let command = format!(
            "G0 X{:.4} Y{:.4} Z{:.4} A{:.4} F{:.0}",
            target.x_mm, target.y_mm, target.z_mm, target.rotation_deg, feed_mm_per_min
        );
        self.command(&command, Duration::from_secs(30)).await?;
        self.command("M400", Duration::from_secs(60)).await?;
        Ok(target)
    }

    async fn actuate(&mut self, actuator_id: &str, enabled: bool) -> Result<()> {
        let command = match (actuator_id, enabled) {
            ("lights", true) => "M150 P255 R255 U255 B255",
            ("lights", false) => "M150 P0",
            ("vacuum_1", true) => "M106 P1 S255\nM106",
            ("vacuum_1", false) => "M107 P1\nM107",
            ("vacuum_2", true) => "M106 P2 S255\nM106 P3 S255",
            ("vacuum_2", false) => "M107 P2\nM107 P3",
            _ => bail!("actuator {actuator_id} is not supported by the active profile"),
        };
        for line in command.lines() {
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
