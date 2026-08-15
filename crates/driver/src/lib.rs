mod actor;
mod discovery;
mod marlin;
mod photon;
mod simulator;

pub use actor::*;
pub use discovery::*;
pub use marlin::*;
pub use photon::*;
pub use simulator::*;

use anyhow::Result;
use async_trait::async_trait;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use velocitypnp_domain::Pose;

#[async_trait]
pub trait MachineDriver: Send {
    fn simulated(&self) -> bool;
    /// Installs a flag that blocking driver operations must monitor. Physical
    /// drivers use it to issue their controller's immediate stop command.
    fn set_emergency_flag(&mut self, _flag: Arc<AtomicBool>) {}
    async fn connect(&mut self) -> Result<String>;
    async fn disconnect(&mut self) -> Result<()>;
    async fn home(&mut self) -> Result<Pose>;
    async fn move_to(&mut self, target: Pose, feed_mm_per_min: f64) -> Result<Pose>;
    async fn set_xy_coordinates(&mut self, x_mm: f64, y_mm: f64) -> Result<Pose>;
    async fn actuate(&mut self, actuator_id: &str, enabled: bool) -> Result<()>;
    async fn read_sensor(&mut self, sensor_id: &str) -> Result<i64>;
    async fn emergency_stop(&mut self) -> Result<()>;
}
