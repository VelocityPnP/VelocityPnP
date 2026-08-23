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
    vacuum_1_enabled: bool,
    vacuum_2_enabled: bool,
    secondary_rotation_deg: f64,
}

impl SimulatorDriver {
    pub fn new(bounds: Bounds, safe_z_mm: f64) -> Self {
        Self {
            connected: false,
            pose: Pose::ORIGIN,
            bounds,
            safe_z_mm,
            vacuum_1_enabled: false,
            vacuum_2_enabled: false,
            secondary_rotation_deg: 0.0,
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

    async fn rotation_position(&mut self, controller_axis: char) -> Result<f64> {
        self.require_connected()?;
        match controller_axis {
            'A' => Ok(self.pose.rotation_deg),
            'B' => Ok(self.secondary_rotation_deg),
            _ => bail!("simulator has no rotation axis {controller_axis}"),
        }
    }

    async fn rotate_nozzle(
        &mut self,
        controller_axis: char,
        target_deg: f64,
        feed_deg_per_min: f64,
    ) -> Result<f64> {
        self.require_connected()?;
        if !target_deg.is_finite() || !feed_deg_per_min.is_finite() || feed_deg_per_min <= 0.0 {
            bail!("simulator rejected invalid rotation values");
        }
        match controller_axis {
            'A' => self.pose.rotation_deg = target_deg,
            'B' => self.secondary_rotation_deg = target_deg,
            _ => bail!("simulator has no rotation axis {controller_axis}"),
        }
        sleep(Duration::from_millis(20)).await;
        Ok(target_deg)
    }

    async fn set_xy_coordinates(&mut self, x_mm: f64, y_mm: f64) -> Result<Pose> {
        self.require_connected()?;
        if !x_mm.is_finite() || !y_mm.is_finite() {
            bail!("simulator rejected non-finite XY coordinate reset");
        }
        self.pose.x_mm = x_mm;
        self.pose.y_mm = y_mm;
        Ok(self.pose)
    }

    async fn set_z_coordinate(&mut self, z_mm: f64) -> Result<Pose> {
        self.require_connected()?;
        if !z_mm.is_finite() {
            bail!("simulator rejected non-finite Z coordinate reset");
        }
        self.pose.z_mm = z_mm;
        Ok(self.pose)
    }

    async fn actuate(&mut self, actuator_id: &str, enabled: bool) -> Result<()> {
        self.require_connected()?;
        match actuator_id {
            "lights" => Ok(()),
            "vacuum_1" => {
                self.vacuum_1_enabled = enabled;
                Ok(())
            }
            "vacuum_2" => {
                self.vacuum_2_enabled = enabled;
                Ok(())
            }
            _ => bail!("unknown simulated actuator {actuator_id}"),
        }
    }

    async fn read_sensor(&mut self, sensor_id: &str) -> Result<i64> {
        self.require_connected()?;
        match sensor_id {
            "vacuum_1" => Ok(if !self.vacuum_1_enabled {
                0
            } else if self.pose.z_mm <= 3.21 {
                -80
            } else {
                -5
            }),
            "vacuum_2" => Ok(if !self.vacuum_2_enabled {
                0
            } else if self.pose.z_mm >= 59.19 {
                -80
            } else {
                -5
            }),
            _ => bail!("unknown simulated sensor {sensor_id}"),
        }
    }

    async fn emergency_stop(&mut self) -> Result<()> {
        self.require_connected()?;
        Ok(())
    }
}
