use crate::MachineDriver;
use anyhow::{bail, Result};
use async_trait::async_trait;
use std::time::Duration;
use tokio::time::sleep;
use velocitypnp_domain::{Bounds, Pose};

pub struct SimulatorDriver {
    connected: bool,
    pose: Pose,
    bounds: Bounds,
    safe_z_mm: f64,
}

impl SimulatorDriver {
    pub fn new(bounds: Bounds, safe_z_mm: f64) -> Self {
        Self {
            connected: false,
            pose: Pose::ORIGIN,
            bounds,
            safe_z_mm,
        }
    }

    fn require_connected(&self) -> Result<()> {
        if !self.connected {
            bail!("simulated controller is disconnected");
        }
        Ok(())
    }
}

#[async_trait]
impl MachineDriver for SimulatorDriver {
    fn simulated(&self) -> bool {
        true
    }

    async fn connect(&mut self) -> Result<String> {
        sleep(Duration::from_millis(80)).await;
        self.connected = true;
        Ok("FIRMWARE_NAME:VelocityPnP Simulator MACHINE_TYPE:LumenPnP".into())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.connected = false;
        Ok(())
    }

    async fn home(&mut self) -> Result<Pose> {
        self.require_connected()?;
        sleep(Duration::from_millis(300)).await;
        self.pose = Pose {
            z_mm: self.safe_z_mm,
            ..Pose::ORIGIN
        };
        Ok(self.pose)
    }

    async fn move_to(&mut self, target: Pose, feed_mm_per_min: f64) -> Result<Pose> {
        self.require_connected()?;
        if feed_mm_per_min <= 0.0 || !feed_mm_per_min.is_finite() {
            bail!("feed rate must be finite and positive");
        }
        if !self.bounds.contains(target) {
            bail!("simulator rejected target outside bounds");
        }
        sleep(Duration::from_millis(40)).await;
        self.pose = target;
        Ok(self.pose)
    }

    async fn actuate(&mut self, actuator_id: &str, _enabled: bool) -> Result<()> {
        self.require_connected()?;
        match actuator_id {
            "lights" | "vacuum_1" | "vacuum_2" => Ok(()),
            _ => bail!("unknown simulated actuator {actuator_id}"),
        }
    }

    async fn emergency_stop(&mut self) -> Result<()> {
        self.require_connected()?;
        Ok(())
    }
}
