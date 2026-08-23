use crate::MachineDriver;
use anyhow::{anyhow, bail, Context, Result};
use chrono::{Duration as ChronoDuration, Utc};
use serde_json::json;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc, oneshot, watch};
use uuid::Uuid;
use velocitypnp_domain::{
    ControlLease, JogTarget, MachineEvent, MachineProfile, MachineState, MachineStatus, Pose,
    SensorReading,
};

#[derive(Clone)]
pub struct MachineHandle {
    tx: mpsc::Sender<Command>,
    status: watch::Receiver<MachineStatus>,
    events: broadcast::Sender<MachineEvent>,
    emergency_flag: Arc<AtomicBool>,
}

impl MachineHandle {
    pub fn spawn(mut driver: Box<dyn MachineDriver>, profile: MachineProfile) -> Self {
        let emergency_flag = Arc::new(AtomicBool::new(false));
        driver.set_emergency_flag(emergency_flag.clone());
        let (tx, rx) = mpsc::channel(64);
        let (status_tx, status) = watch::channel(MachineStatus {
            simulated: driver.simulated(),
            profile_id: profile.id.clone(),
            ..MachineStatus::default()
        });
        let (events, _) = broadcast::channel(256);
        tokio::spawn(
            MachineActor {
                driver,
                profile,
                rx,
                status_tx,
                events: events.clone(),
                emergency_flag: emergency_flag.clone(),
                calibration_rotation_references: BTreeMap::new(),
                sequence: 0,
            }
            .run(),
        );
        Self {
            tx,
            status,
            events,
            emergency_flag,
        }
    }

    pub fn status(&self) -> MachineStatus {
        self.status.borrow().clone()
    }

    pub fn subscribe(&self) -> broadcast::Receiver<MachineEvent> {
        self.events.subscribe()
    }

    pub async fn connect(&self) -> Result<String> {
        request(&self.tx, |reply| Command::Connect { reply }).await
    }

    pub async fn disconnect(&self) -> Result<()> {
        request(&self.tx, |reply| Command::Disconnect { reply }).await
    }

    pub async fn complete_preflight(&self, checks_confirmed: bool) -> Result<()> {
        request(&self.tx, |reply| Command::CompletePreflight {
            checks_confirmed,
            reply,
        })
        .await
    }

    pub async fn acquire_lease(
        &self,
        client_name: String,
        duration_seconds: u32,
    ) -> Result<ControlLease> {
        request(&self.tx, |reply| Command::AcquireLease {
            client_name,
            duration_seconds,
            reply,
        })
        .await
    }

    pub async fn renew_lease(&self, lease_id: Uuid, duration_seconds: u32) -> Result<ControlLease> {
        request(&self.tx, |reply| Command::RenewLease {
            lease_id,
            duration_seconds,
            reply,
        })
        .await
    }

    pub async fn home(&self, lease_id: Uuid, checks_confirmed: bool) -> Result<Pose> {
        request(&self.tx, |reply| Command::Home {
            lease_id,
            checks_confirmed,
            reply,
        })
        .await
    }

    pub async fn jog(
        &self,
        lease_id: Uuid,
        target: JogTarget,
        feed_mm_per_min: f64,
    ) -> Result<Pose> {
        request(&self.tx, |reply| Command::Jog {
            lease_id,
            target,
            feed_mm_per_min,
            reply,
        })
        .await
    }

    /// Reads a profile-scoped rotation reference for the automated calibration
    /// runner. This operation is not exposed as unrestricted HTTP motion.
    pub async fn calibration_rotation_reference(
        &self,
        lease_id: Uuid,
        nozzle_id: String,
    ) -> Result<f64> {
        request(&self.tx, |reply| Command::CalibrationRotationReference {
            lease_id,
            nozzle_id,
            reply,
        })
        .await
    }

    /// Moves one nozzle within the bounded offsets in the active calibration
    /// profile. Normal jog rotation remains independently interlocked.
    pub async fn calibration_rotate_nozzle(
        &self,
        lease_id: Uuid,
        nozzle_id: String,
        angle_offset_deg: f64,
    ) -> Result<f64> {
        request(&self.tx, |reply| Command::CalibrationRotateNozzle {
            lease_id,
            nozzle_id,
            angle_offset_deg,
            reply,
        })
        .await
    }

    pub async fn actuate(&self, lease_id: Uuid, actuator_id: String, enabled: bool) -> Result<()> {
        request(&self.tx, |reply| Command::Actuate {
            lease_id,
            actuator_id,
            enabled,
            reply,
        })
        .await
    }

    /// Requests an off transition without a lease. This is intentionally not
    /// exposed by the HTTP API and cannot enable an output.
    pub async fn disable_actuator(&self, actuator_id: String) -> Result<()> {
        request(&self.tx, |reply| Command::DisableActuator {
            actuator_id,
            reply,
        })
        .await
    }

    pub async fn read_sensor(&self, lease_id: Uuid, sensor_id: String) -> Result<SensorReading> {
        request(&self.tx, |reply| Command::ReadSensor {
            lease_id,
            sensor_id,
            reply,
        })
        .await
    }

    pub async fn apply_visual_homing_offset(&self, lease_id: Uuid) -> Result<Pose> {
        request(&self.tx, |reply| Command::ApplyVisualHomingOffset {
            lease_id,
            reply,
        })
        .await
    }

    pub async fn apply_fine_z_homing_offset(
        &self,
        lease_id: Uuid,
        z_mm: f64,
        calibration_revision: Uuid,
        maximum_correction_mm: f64,
    ) -> Result<Pose> {
        request(&self.tx, |reply| Command::ApplyFineZHomingOffset {
            lease_id,
            z_mm,
            calibration_revision,
            maximum_correction_mm,
            reply,
        })
        .await
    }

    pub async fn emergency_stop(&self) -> Result<()> {
        // This reaches a blocking physical driver before the serialized actor
        // can dequeue the explicit state-transition command.
        self.emergency_flag.store(true, Ordering::Release);
        request(&self.tx, |reply| Command::EmergencyStop { reply }).await
    }
}

async fn request<T>(
    tx: &mpsc::Sender<Command>,
    build: impl FnOnce(oneshot::Sender<Result<T>>) -> Command,
) -> Result<T> {
    let (reply_tx, reply_rx) = oneshot::channel();
    tx.send(build(reply_tx))
        .await
        .map_err(|_| anyhow!("machine actor stopped"))?;
    reply_rx
        .await
        .map_err(|_| anyhow!("machine actor dropped response"))?
}

enum Command {
    Connect {
        reply: oneshot::Sender<Result<String>>,
    },
    Disconnect {
        reply: oneshot::Sender<Result<()>>,
    },
    CompletePreflight {
        checks_confirmed: bool,
        reply: oneshot::Sender<Result<()>>,
    },
    AcquireLease {
        client_name: String,
        duration_seconds: u32,
        reply: oneshot::Sender<Result<ControlLease>>,
    },
    RenewLease {
        lease_id: Uuid,
        duration_seconds: u32,
        reply: oneshot::Sender<Result<ControlLease>>,
    },
    Home {
        lease_id: Uuid,
        checks_confirmed: bool,
        reply: oneshot::Sender<Result<Pose>>,
    },
    Jog {
        lease_id: Uuid,
        target: JogTarget,
        feed_mm_per_min: f64,
        reply: oneshot::Sender<Result<Pose>>,
    },
    CalibrationRotationReference {
        lease_id: Uuid,
        nozzle_id: String,
        reply: oneshot::Sender<Result<f64>>,
    },
    CalibrationRotateNozzle {
        lease_id: Uuid,
        nozzle_id: String,
        angle_offset_deg: f64,
        reply: oneshot::Sender<Result<f64>>,
    },
    Actuate {
        lease_id: Uuid,
        actuator_id: String,
        enabled: bool,
        reply: oneshot::Sender<Result<()>>,
    },
    DisableActuator {
        actuator_id: String,
        reply: oneshot::Sender<Result<()>>,
    },
    ReadSensor {
        lease_id: Uuid,
        sensor_id: String,
        reply: oneshot::Sender<Result<SensorReading>>,
    },
    ApplyVisualHomingOffset {
        lease_id: Uuid,
        reply: oneshot::Sender<Result<Pose>>,
    },
    ApplyFineZHomingOffset {
        lease_id: Uuid,
        z_mm: f64,
        calibration_revision: Uuid,
        maximum_correction_mm: f64,
        reply: oneshot::Sender<Result<Pose>>,
    },
    EmergencyStop {
        reply: oneshot::Sender<Result<()>>,
    },
}

struct MachineActor {
    driver: Box<dyn MachineDriver>,
    profile: MachineProfile,
    rx: mpsc::Receiver<Command>,
    status_tx: watch::Sender<MachineStatus>,
    events: broadcast::Sender<MachineEvent>,
    emergency_flag: Arc<AtomicBool>,
    calibration_rotation_references: BTreeMap<String, (Uuid, f64)>,
    sequence: u64,
}

impl MachineActor {
    async fn run(mut self) {
        while let Some(command) = self.rx.recv().await {
            match command {
                Command::Connect { reply } => {
                    let _ = reply.send(self.connect().await);
                }
                Command::Disconnect { reply } => {
                    let result = self.driver.disconnect().await;
                    self.calibration_rotation_references.clear();
                    self.update(|s| {
                        s.state = MachineState::Disconnected;
                        s.homed = false;
                        s.pose = None;
                        s.control_lease = None;
                        s.calibration_revision = None;
                    });
                    self.emit("machine.disconnected", json!({}));
                    let _ = reply.send(result);
                }
                Command::CompletePreflight {
                    checks_confirmed,
                    reply,
                } => {
                    let result = if self.status().state != MachineState::Preflight {
                        bail_result("preflight is only valid immediately after connection")
                    } else if !checks_confirmed {
                        bail_result("operator preflight checks were not confirmed")
                    } else {
                        self.update(|s| s.state = MachineState::Unhomed);
                        self.emit("machine.preflight_completed", json!({}));
                        Ok(())
                    };
                    let _ = reply.send(result);
                }
                Command::AcquireLease {
                    client_name,
                    duration_seconds,
                    reply,
                } => {
                    let _ = reply.send(self.acquire_lease(client_name, duration_seconds));
                }
                Command::RenewLease {
                    lease_id,
                    duration_seconds,
                    reply,
                } => {
                    let _ = reply.send(self.renew_lease(lease_id, duration_seconds));
                }
                Command::Home {
                    lease_id,
                    checks_confirmed,
                    reply,
                } => {
                    let _ = reply.send(self.home(lease_id, checks_confirmed).await);
                }
                Command::Jog {
                    lease_id,
                    target,
                    feed_mm_per_min,
                    reply,
                } => {
                    let _ = reply.send(self.jog(lease_id, target, feed_mm_per_min).await);
                }
                Command::CalibrationRotationReference {
                    lease_id,
                    nozzle_id,
                    reply,
                } => {
                    let _ = reply.send(
                        self.calibration_rotation_reference(lease_id, &nozzle_id)
                            .await,
                    );
                }
                Command::CalibrationRotateNozzle {
                    lease_id,
                    nozzle_id,
                    angle_offset_deg,
                    reply,
                } => {
                    let _ = reply.send(
                        self.calibration_rotate_nozzle(lease_id, &nozzle_id, angle_offset_deg)
                            .await,
                    );
                }
                Command::Actuate {
                    lease_id,
                    actuator_id,
                    enabled,
                    reply,
                } => {
                    let _ = reply.send(self.actuate(lease_id, actuator_id, enabled).await);
                }
                Command::DisableActuator { actuator_id, reply } => {
                    let _ = reply.send(self.disable_actuator(actuator_id).await);
                }
                Command::ReadSensor {
                    lease_id,
                    sensor_id,
                    reply,
                } => {
                    let _ = reply.send(self.read_sensor(lease_id, sensor_id).await);
                }
                Command::ApplyVisualHomingOffset { lease_id, reply } => {
                    let _ = reply.send(self.apply_visual_homing_offset(lease_id).await);
                }
                Command::ApplyFineZHomingOffset {
                    lease_id,
                    z_mm,
                    calibration_revision,
                    maximum_correction_mm,
                    reply,
                } => {
                    let _ = reply.send(
                        self.apply_fine_z_homing_offset(
                            lease_id,
                            z_mm,
                            calibration_revision,
                            maximum_correction_mm,
                        )
                        .await,
                    );
                }
                Command::EmergencyStop { reply } => {
                    let result = self.driver.emergency_stop().await;
                    self.emergency_flag.store(false, Ordering::Release);
                    self.calibration_rotation_references.clear();
                    self.update(|s| {
                        s.state = MachineState::EStopped;
                        s.homed = false;
                        s.control_lease = None;
                        s.calibration_revision = None;
                    });
                    self.emit("machine.emergency_stopped", json!({}));
                    let _ = reply.send(result);
                }
            }
        }
    }

    async fn connect(&mut self) -> Result<String> {
        if self.status().state != MachineState::Disconnected {
            bail!("machine is already connected");
        }
        match self.driver.connect().await {
            Ok(firmware) => {
                self.calibration_rotation_references.clear();
                self.update(|s| {
                    s.state = MachineState::Preflight;
                    s.fault = None;
                    s.calibration_revision = None;
                });
                self.emit("machine.connected", json!({"firmware": firmware}));
                Ok(firmware)
            }
            Err(error) => {
                self.set_fault("connect_failed", error.to_string(), true);
                Err(error)
            }
        }
    }

    fn acquire_lease(
        &mut self,
        client_name: String,
        duration_seconds: u32,
    ) -> Result<ControlLease> {
        if self.status().state == MachineState::Disconnected {
            bail!("connect the machine before acquiring control");
        }
        let now = Utc::now();
        if let Some(existing) = &self.status().control_lease {
            if existing.expires_at > now {
                bail!(
                    "machine is controlled by {} until {}",
                    existing.client_name,
                    existing.expires_at
                );
            }
        }
        let duration_seconds = duration_seconds.clamp(30, 3600);
        let lease = ControlLease {
            id: Uuid::new_v4(),
            client_name,
            acquired_at: now,
            expires_at: now + ChronoDuration::seconds(i64::from(duration_seconds)),
        };
        self.calibration_rotation_references.clear();
        self.update(|s| s.control_lease = Some(lease.clone()));
        self.emit("control.lease_acquired", json!({"lease": lease}));
        Ok(lease)
    }

    fn renew_lease(&mut self, lease_id: Uuid, duration_seconds: u32) -> Result<ControlLease> {
        let now = Utc::now();
        self.status().assert_lease(lease_id, now)?;
        let mut lease = self.status().control_lease.unwrap();
        let duration_seconds = duration_seconds.clamp(30, 3600);
        lease.expires_at = now + ChronoDuration::seconds(i64::from(duration_seconds));
        self.update(|status| status.control_lease = Some(lease.clone()));
        self.emit("control.lease_renewed", json!({"lease": lease}));
        Ok(lease)
    }

    async fn home(&mut self, lease_id: Uuid, checks_confirmed: bool) -> Result<Pose> {
        self.status().assert_lease(lease_id, Utc::now())?;
        self.status().assert_can_home()?;
        if !checks_confirmed {
            bail!("operator homing safety checks were not confirmed");
        }
        self.calibration_rotation_references.clear();
        self.update(|s| s.state = MachineState::Homing);
        self.emit("machine.homing_started", json!({}));
        match self.driver.home().await {
            Ok(pose) => {
                self.update(|s| {
                    s.state = MachineState::Ready;
                    s.homed = true;
                    s.pose = Some(pose);
                    s.calibration_revision = None;
                });
                self.emit("machine.homing_completed", json!({"pose": pose}));
                Ok(pose)
            }
            Err(error) => {
                self.set_fault("homing_failed", error.to_string(), true);
                Err(error)
            }
        }
    }

    async fn jog(
        &mut self,
        lease_id: Uuid,
        target: JogTarget,
        feed_mm_per_min: f64,
    ) -> Result<Pose> {
        let status = self.status();
        status.assert_lease(lease_id, Utc::now())?;
        let current = status.pose.context("jog requires a known current pose")?;
        let target = target
            .resolve(current)
            .context("jog target must contain finite coordinates")?;
        status.assert_can_jog(target, &self.profile.bounds)?;
        let rotation_motion_enabled = self
            .profile
            .capabilities
            .get("rotation_motion")
            .copied()
            .unwrap_or(false);
        if !status.simulated
            && !rotation_motion_enabled
            && (target.rotation_deg - current.rotation_deg).abs() > 1e-6
        {
            bail!(
                "physical nozzle rotation is disabled until rotation motion is hardware-validated"
            );
        }
        let max_feed = self.profile.default_feed_mm_per_min.max(1.0);
        let feed = feed_mm_per_min.clamp(1.0, max_feed);
        match self.driver.move_to(target, feed).await {
            Ok(pose) => {
                self.update(|s| s.pose = Some(pose));
                self.emit("machine.position", json!({"pose": pose}));
                Ok(pose)
            }
            Err(error) => {
                self.set_fault("motion_failed", error.to_string(), true);
                Err(error)
            }
        }
    }

    async fn calibration_rotation_reference(
        &mut self,
        lease_id: Uuid,
        nozzle_id: &str,
    ) -> Result<f64> {
        self.assert_can_calibrate_rotation(lease_id)?;
        if let Some((stored_lease, reference)) = self.calibration_rotation_references.get(nozzle_id)
        {
            if *stored_lease == lease_id {
                return Ok(*reference);
            }
        }
        let controller_axis = self.nozzle_rotation_axis(nozzle_id)?;
        let reference = self.driver.rotation_position(controller_axis).await?;
        self.calibration_rotation_references
            .insert(nozzle_id.into(), (lease_id, reference));
        Ok(reference)
    }

    async fn calibration_rotate_nozzle(
        &mut self,
        lease_id: Uuid,
        nozzle_id: &str,
        angle_offset_deg: f64,
    ) -> Result<f64> {
        self.assert_can_calibrate_rotation(lease_id)?;
        if !angle_offset_deg.is_finite() {
            bail!("nozzle rotation offset must be finite");
        }
        let bottom = self
            .profile
            .bottom_vision
            .as_ref()
            .context("active profile does not define bottom vision")?;
        let geometry = &bottom.head_geometry;
        let max_offset = geometry
            .rotation_angle_offsets_deg
            .iter()
            .map(|angle| angle.abs())
            .fold(0.0, f64::max);
        if angle_offset_deg.abs() > max_offset + 1e-6 {
            bail!("nozzle rotation target exceeds the {max_offset:.1} degree calibration bound");
        }
        let (reference_lease, reference_deg) = self
            .calibration_rotation_references
            .get(nozzle_id)
            .copied()
            .context("capture the nozzle rotation reference before moving it")?;
        if reference_lease != lease_id {
            bail!("nozzle rotation reference belongs to a different control lease");
        }
        let target_deg = reference_deg + angle_offset_deg;
        let feed = geometry.rotation_feed_deg_per_min;
        let controller_axis = self.nozzle_rotation_axis(nozzle_id)?;
        match self
            .driver
            .rotate_nozzle(controller_axis, target_deg, feed)
            .await
        {
            Ok(actual) => {
                if controller_axis == 'A' {
                    self.update(|status| {
                        if let Some(pose) = status.pose.as_mut() {
                            pose.rotation_deg = actual;
                        }
                    });
                }
                self.emit(
                    "machine.nozzle_rotated",
                    json!({
                        "nozzle_id": nozzle_id,
                        "controller_axis": controller_axis,
                        "reference_deg": reference_deg,
                        "angle_offset_deg": angle_offset_deg,
                        "target_deg": target_deg,
                        "actual_deg": actual,
                        "calibration": true,
                    }),
                );
                Ok(actual)
            }
            Err(error) => {
                self.set_fault("rotation_motion_failed", error.to_string(), true);
                Err(error)
            }
        }
    }

    fn assert_can_calibrate_rotation(&self, lease_id: Uuid) -> Result<()> {
        let status = self.status();
        status.assert_lease(lease_id, Utc::now())?;
        if status.state != MachineState::Ready || !status.homed {
            bail!("nozzle rotation calibration requires a homed ready machine");
        }
        if !self
            .profile
            .capabilities
            .get("rotation_calibration")
            .copied()
            .unwrap_or(false)
        {
            bail!("active profile does not permit bounded rotation calibration");
        }
        Ok(())
    }

    fn nozzle_rotation_axis(&self, nozzle_id: &str) -> Result<char> {
        let bottom = self
            .profile
            .bottom_vision
            .as_ref()
            .context("active profile does not define bottom vision")?;
        let nozzle = bottom
            .nozzles
            .iter()
            .find(|nozzle| nozzle.nozzle_id == nozzle_id)
            .with_context(|| format!("unknown nozzle {nozzle_id}"))?;
        let axis = self
            .profile
            .axes
            .iter()
            .find(|axis| axis.name == nozzle.rotation_axis && axis.kind == "rotation")
            .with_context(|| {
                format!(
                    "nozzle {nozzle_id} references missing rotation axis {}",
                    nozzle.rotation_axis
                )
            })?;
        let mut letters = axis.controller_letter.chars();
        let controller_axis = letters
            .next()
            .context("rotation axis has no controller letter")?;
        if letters.next().is_some() || !matches!(controller_axis, 'A' | 'B') {
            bail!("rotation axis controller letter must be A or B");
        }
        Ok(controller_axis)
    }

    async fn actuate(&mut self, lease_id: Uuid, actuator_id: String, enabled: bool) -> Result<()> {
        self.status().assert_lease(lease_id, Utc::now())?;
        if !matches!(
            self.status().state,
            MachineState::Preflight | MachineState::Unhomed | MachineState::Ready
        ) {
            bail!("actuator checks are not allowed in the current machine state");
        }
        if !self
            .profile
            .actuators
            .iter()
            .any(|actuator| actuator.id == actuator_id)
        {
            bail!("actuator {actuator_id} is not present in the active profile");
        }
        self.driver.actuate(&actuator_id, enabled).await?;
        self.emit(
            "machine.actuator",
            json!({"actuator_id": actuator_id, "enabled": enabled}),
        );
        Ok(())
    }

    async fn disable_actuator(&mut self, actuator_id: String) -> Result<()> {
        if !self
            .profile
            .actuators
            .iter()
            .any(|actuator| actuator.id == actuator_id)
        {
            bail!("actuator {actuator_id} is not present in the active profile");
        }
        // Disconnect already commands every validated output off before it
        // releases the driver. A concurrent operation can still request its
        // own cleanup after that point; treat this as an idempotent no-op.
        if self.status().state == MachineState::Disconnected {
            return Ok(());
        }
        if let Err(error) = self.driver.actuate(&actuator_id, false).await {
            let message = format!("failed to disable actuator {actuator_id}: {error}");
            self.set_fault("actuator_disable_failed", message.clone(), true);
            bail!(message);
        }
        self.emit(
            "machine.actuator",
            json!({
                "actuator_id": actuator_id,
                "enabled": false,
                "fail_safe": true,
            }),
        );
        Ok(())
    }

    async fn read_sensor(&mut self, lease_id: Uuid, sensor_id: String) -> Result<SensorReading> {
        self.status().assert_lease(lease_id, Utc::now())?;
        if !matches!(
            self.status().state,
            MachineState::Preflight | MachineState::Unhomed | MachineState::Ready
        ) {
            bail!("sensor reads are not allowed in the current machine state");
        }
        let sensor = self
            .profile
            .sensors
            .iter()
            .find(|sensor| sensor.id == sensor_id)
            .cloned()
            .with_context(|| format!("sensor {sensor_id} is not present in the active profile"))?;
        let reading = SensorReading {
            raw_value: self.driver.read_sensor(&sensor_id).await?,
            sensor_id,
            unit: sensor.unit,
            observed_at: Utc::now(),
        };
        self.emit("machine.sensor", json!({"reading": reading}));
        Ok(reading)
    }

    async fn apply_visual_homing_offset(&mut self, lease_id: Uuid) -> Result<Pose> {
        let status = self.status();
        status.assert_lease(lease_id, Utc::now())?;
        if status.state != MachineState::Ready || !status.homed {
            bail!("visual homing offset requires a homed ready machine");
        }
        let profile = self
            .profile
            .visual_homing
            .as_ref()
            .cloned()
            .context("active profile does not define visual homing")?;
        profile.validate().map_err(anyhow::Error::msg)?;
        if !profile.axis_mapping_validated {
            bail!("visual homing axis mapping has not been hardware-validated");
        }
        let current = status
            .pose
            .context("visual homing offset requires a known pose")?;
        if !self.profile.bounds.contains(Pose {
            x_mm: profile.fiducial_x_mm,
            y_mm: profile.fiducial_y_mm,
            ..current
        }) {
            bail!("visual homing fiducial coordinates are outside machine bounds");
        }
        match self
            .driver
            .set_xy_coordinates(profile.fiducial_x_mm, profile.fiducial_y_mm)
            .await
        {
            Ok(pose) => {
                self.update(|status| status.pose = Some(pose));
                self.emit(
                    "machine.visual_home_applied",
                    json!({
                        "pose": pose,
                        "fiducial_x_mm": profile.fiducial_x_mm,
                        "fiducial_y_mm": profile.fiducial_y_mm,
                    }),
                );
                Ok(pose)
            }
            Err(error) => {
                self.set_fault("visual_home_offset_failed", error.to_string(), true);
                Err(error)
            }
        }
    }

    async fn apply_fine_z_homing_offset(
        &mut self,
        lease_id: Uuid,
        z_mm: f64,
        calibration_revision: Uuid,
        maximum_correction_mm: f64,
    ) -> Result<Pose> {
        let status = self.status();
        status.assert_lease(lease_id, Utc::now())?;
        if status.state != MachineState::Ready || !status.homed {
            bail!("fine Z homing offset requires a homed ready machine");
        }
        let current = status.pose.context("fine Z homing requires a known pose")?;
        if !z_mm.is_finite()
            || !maximum_correction_mm.is_finite()
            || maximum_correction_mm <= 0.0
            || (z_mm - current.z_mm).abs() > maximum_correction_mm
        {
            bail!("fine Z homing correction exceeds its configured bound");
        }
        if !self.profile.bounds.contains(Pose { z_mm, ..current }) {
            bail!("fine Z homing coordinate is outside machine bounds");
        }
        match self.driver.set_z_coordinate(z_mm).await {
            Ok(pose) => {
                self.update(|status| {
                    status.pose = Some(pose);
                    status.calibration_revision = Some(calibration_revision);
                });
                self.emit(
                    "machine.fine_z_home_applied",
                    json!({
                        "pose": pose,
                        "calibration_revision": calibration_revision,
                    }),
                );
                Ok(pose)
            }
            Err(error) => {
                self.set_fault("fine_z_home_offset_failed", error.to_string(), true);
                Err(error)
            }
        }
    }

    fn status(&self) -> MachineStatus {
        self.status_tx.borrow().clone()
    }

    fn update(&self, change: impl FnOnce(&mut MachineStatus)) {
        self.status_tx.send_modify(change);
    }

    fn emit(&mut self, kind: &str, payload: serde_json::Value) {
        self.sequence += 1;
        let _ = self.events.send(MachineEvent {
            sequence: self.sequence,
            at: Utc::now(),
            kind: kind.into(),
            payload,
        });
    }

    fn set_fault(&mut self, code: &str, message: String, recoverable: bool) {
        self.calibration_rotation_references.clear();
        let fault = velocitypnp_domain::Fault {
            code: code.into(),
            message,
            recoverable,
            at: Utc::now(),
        };
        self.update(|s| {
            s.state = MachineState::Fault;
            s.homed = false;
            s.fault = Some(fault.clone());
            s.control_lease = None;
            s.calibration_revision = None;
        });
        self.emit("machine.fault", json!({"fault": fault}));
    }
}

fn bail_result<T>(message: &str) -> Result<T> {
    Err(anyhow!(message.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SimulatorDriver;
    use async_trait::async_trait;
    use std::sync::atomic::AtomicUsize;
    use std::time::Duration;

    fn profile() -> MachineProfile {
        serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap()
    }

    #[tokio::test]
    async fn simulator_requires_preflight_lease_and_home_before_jog() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile,
        );
        handle.connect().await.unwrap();
        assert_eq!(handle.status().state, MachineState::Preflight);
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        let renewed = handle.renew_lease(lease.id, 3600).await.unwrap();
        assert_eq!(renewed.id, lease.id);
        assert!(renewed.expires_at > lease.expires_at);
        handle.home(lease.id, true).await.unwrap();
        let pose = Pose {
            x_mm: 10.0,
            y_mm: 20.0,
            z_mm: 31.5,
            rotation_deg: 0.0,
        };
        assert_eq!(
            handle.jog(lease.id, pose.into(), 100.0).await.unwrap(),
            pose
        );
    }

    struct PhysicalPoseDriver;

    #[async_trait]
    impl MachineDriver for PhysicalPoseDriver {
        fn simulated(&self) -> bool {
            false
        }

        async fn connect(&mut self) -> Result<String> {
            Ok("Marlin test driver".into())
        }

        async fn disconnect(&mut self) -> Result<()> {
            Ok(())
        }

        async fn home(&mut self) -> Result<Pose> {
            Ok(Pose {
                z_mm: 31.5,
                rotation_deg: 720.0,
                ..Pose::ORIGIN
            })
        }

        async fn move_to(&mut self, target: Pose, _feed_mm_per_min: f64) -> Result<Pose> {
            Ok(target)
        }

        async fn set_xy_coordinates(&mut self, x_mm: f64, y_mm: f64) -> Result<Pose> {
            Ok(Pose {
                x_mm,
                y_mm,
                z_mm: 31.5,
                rotation_deg: 720.0,
            })
        }

        async fn actuate(&mut self, _actuator_id: &str, _enabled: bool) -> Result<()> {
            Ok(())
        }

        async fn read_sensor(&mut self, _sensor_id: &str) -> Result<i64> {
            Ok(0)
        }

        async fn emergency_stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    #[tokio::test]
    async fn physical_jog_preserves_rotation_and_rejects_unvalidated_changes() {
        let profile = profile();
        let handle = MachineHandle::spawn(Box::new(PhysicalPoseDriver), profile);
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.home(lease.id, true).await.unwrap();

        let preserved = handle
            .jog(
                lease.id,
                JogTarget {
                    x_mm: 10.0,
                    y_mm: 20.0,
                    z_mm: 31.5,
                    rotation_deg: None,
                },
                100.0,
            )
            .await
            .unwrap();
        assert_eq!(preserved.rotation_deg, 720.0);

        let error = handle
            .jog(
                lease.id,
                JogTarget {
                    rotation_deg: Some(0.0),
                    ..preserved.into()
                },
                100.0,
            )
            .await
            .unwrap_err()
            .to_string();
        assert!(error.contains("rotation is disabled"));
        assert_eq!(handle.status().state, MachineState::Ready);
    }

    #[tokio::test]
    async fn calibration_rotation_is_profile_scoped_and_bounded() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile,
        );
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.home(lease.id, true).await.unwrap();

        let n1_reference = handle
            .calibration_rotation_reference(lease.id, "N1".into())
            .await
            .unwrap();
        let n2_reference = handle
            .calibration_rotation_reference(lease.id, "N2".into())
            .await
            .unwrap();
        assert_eq!(n1_reference, 0.0);
        assert_eq!(n2_reference, 0.0);
        assert_eq!(
            handle
                .calibration_rotate_nozzle(lease.id, "N1".into(), 165.0)
                .await
                .unwrap(),
            165.0
        );
        assert_eq!(
            handle
                .calibration_rotate_nozzle(lease.id, "N2".into(), -165.0)
                .await
                .unwrap(),
            -165.0
        );
        assert_eq!(
            handle
                .calibration_rotation_reference(lease.id, "N1".into())
                .await
                .unwrap(),
            n1_reference
        );
        assert_eq!(handle.status().pose.unwrap().rotation_deg, 165.0);

        let error = handle
            .calibration_rotate_nozzle(lease.id, "N1".into(), 165.1)
            .await
            .unwrap_err()
            .to_string();
        assert!(error.contains("exceeds the 165.0 degree calibration bound"));
        assert_eq!(handle.status().state, MachineState::Ready);
    }

    #[tokio::test]
    async fn estop_revokes_control() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile,
        );
        handle.connect().await.unwrap();
        handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.emergency_stop().await.unwrap();
        assert_eq!(handle.status().state, MachineState::EStopped);
        assert!(handle.status().control_lease.is_none());
        handle.disable_actuator("lights".into()).await.unwrap();
    }

    #[tokio::test]
    async fn simulator_sensor_read_tracks_vacuum_actuator() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile,
        );
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.home(lease.id, true).await.unwrap();
        assert_eq!(
            handle
                .read_sensor(lease.id, "vacuum_1".into())
                .await
                .unwrap()
                .raw_value,
            0
        );
        handle
            .actuate(lease.id, "vacuum_1".into(), true)
            .await
            .unwrap();
        assert_eq!(
            handle
                .read_sensor(lease.id, "vacuum_1".into())
                .await
                .unwrap()
                .raw_value,
            -5
        );
    }

    #[tokio::test]
    async fn fail_safe_disable_is_idempotent_after_disconnect() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile,
        );
        handle.connect().await.unwrap();
        handle.disconnect().await.unwrap();

        handle.disable_actuator("vacuum_1".into()).await.unwrap();
        assert_eq!(handle.status().state, MachineState::Disconnected);
        assert!(handle.status().fault.is_none());
    }

    #[tokio::test]
    async fn visual_offset_requires_validated_mapping_and_resets_only_xy() {
        let mut unvalidated = profile();
        unvalidated
            .visual_homing
            .as_mut()
            .unwrap()
            .axis_mapping_validated = false;
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(
                unvalidated.bounds,
                unvalidated.safe_z_mm,
            )),
            unvalidated,
        );
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.home(lease.id, true).await.unwrap();
        assert!(handle.apply_visual_homing_offset(lease.id).await.is_err());

        let profile = profile();
        assert!(
            profile
                .visual_homing
                .as_ref()
                .unwrap()
                .axis_mapping_validated
        );
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.home(lease.id, true).await.unwrap();
        let pose = handle.apply_visual_homing_offset(lease.id).await.unwrap();
        let expected = profile.visual_homing.unwrap();
        assert_eq!(pose.x_mm, expected.fiducial_x_mm);
        assert_eq!(pose.y_mm, expected.fiducial_y_mm);
        assert_eq!(pose.z_mm, 31.5);
    }

    #[tokio::test]
    async fn fine_z_home_is_bounded_and_scoped_to_the_calibration_revision() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile,
        );
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        handle.home(lease.id, true).await.unwrap();
        handle
            .jog(
                lease.id,
                JogTarget {
                    x_mm: 0.0,
                    y_mm: 0.0,
                    z_mm: 3.3,
                    rotation_deg: None,
                },
                60.0,
            )
            .await
            .unwrap();
        let revision = Uuid::new_v4();
        let pose = handle
            .apply_fine_z_homing_offset(lease.id, 3.2, revision, 0.5)
            .await
            .unwrap();
        assert_eq!(pose.z_mm, 3.2);
        assert_eq!(handle.status().calibration_revision, Some(revision));
        assert!(handle
            .apply_fine_z_homing_offset(lease.id, 4.0, revision, 0.5)
            .await
            .is_err());
        handle.home(lease.id, true).await.unwrap();
        assert_eq!(handle.status().calibration_revision, None);
    }

    struct FailingDisableDriver {
        emergency_calls: Arc<AtomicUsize>,
    }

    #[async_trait]
    impl MachineDriver for FailingDisableDriver {
        fn simulated(&self) -> bool {
            false
        }

        async fn connect(&mut self) -> Result<String> {
            Ok("actuator failure test driver".into())
        }

        async fn disconnect(&mut self) -> Result<()> {
            Ok(())
        }

        async fn home(&mut self) -> Result<Pose> {
            Ok(Pose::ORIGIN)
        }

        async fn move_to(&mut self, target: Pose, _feed_mm_per_min: f64) -> Result<Pose> {
            Ok(target)
        }

        async fn set_xy_coordinates(&mut self, x_mm: f64, y_mm: f64) -> Result<Pose> {
            Ok(Pose {
                x_mm,
                y_mm,
                ..Pose::ORIGIN
            })
        }

        async fn actuate(&mut self, _actuator_id: &str, enabled: bool) -> Result<()> {
            if enabled {
                Ok(())
            } else {
                bail!("controller did not acknowledge the off command")
            }
        }

        async fn read_sensor(&mut self, _sensor_id: &str) -> Result<i64> {
            Ok(0)
        }

        async fn emergency_stop(&mut self) -> Result<()> {
            self.emergency_calls.fetch_add(1, Ordering::AcqRel);
            Ok(())
        }
    }

    #[tokio::test]
    async fn actuator_cleanup_failure_faults_without_an_emergency_stop() {
        let emergency_calls = Arc::new(AtomicUsize::new(0));
        let handle = MachineHandle::spawn(
            Box::new(FailingDisableDriver {
                emergency_calls: emergency_calls.clone(),
            }),
            profile(),
        );
        handle.connect().await.unwrap();

        let error = handle
            .disable_actuator("lights".into())
            .await
            .unwrap_err()
            .to_string();

        assert!(error.contains("failed to disable actuator lights"));
        assert_eq!(handle.status().state, MachineState::Fault);
        assert_eq!(
            handle.status().fault.unwrap().code,
            "actuator_disable_failed"
        );
        assert_eq!(emergency_calls.load(Ordering::Acquire), 0);
    }

    struct InterruptibleDriver {
        emergency_flag: Arc<AtomicBool>,
    }

    #[async_trait]
    impl MachineDriver for InterruptibleDriver {
        fn simulated(&self) -> bool {
            false
        }

        fn set_emergency_flag(&mut self, flag: Arc<AtomicBool>) {
            self.emergency_flag = flag;
        }

        async fn connect(&mut self) -> Result<String> {
            Ok("Marlin test driver".into())
        }

        async fn disconnect(&mut self) -> Result<()> {
            Ok(())
        }

        async fn home(&mut self) -> Result<Pose> {
            loop {
                if self.emergency_flag.swap(false, Ordering::AcqRel) {
                    bail!("homing interrupted by emergency stop");
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }

        async fn move_to(&mut self, target: Pose, _feed_mm_per_min: f64) -> Result<Pose> {
            Ok(target)
        }

        async fn set_xy_coordinates(&mut self, x_mm: f64, y_mm: f64) -> Result<Pose> {
            Ok(Pose {
                x_mm,
                y_mm,
                ..Pose::ORIGIN
            })
        }

        async fn actuate(&mut self, _actuator_id: &str, _enabled: bool) -> Result<()> {
            Ok(())
        }

        async fn read_sensor(&mut self, _sensor_id: &str) -> Result<i64> {
            Ok(0)
        }

        async fn emergency_stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    #[tokio::test]
    async fn estop_interrupts_a_blocking_driver_operation() {
        let profile = profile();
        let handle = MachineHandle::spawn(
            Box::new(InterruptibleDriver {
                emergency_flag: Arc::new(AtomicBool::new(false)),
            }),
            profile,
        );
        handle.connect().await.unwrap();
        handle.complete_preflight(true).await.unwrap();
        let lease = handle.acquire_lease("test".into(), 60).await.unwrap();
        let homing_handle = handle.clone();
        let homing = tokio::spawn(async move { homing_handle.home(lease.id, true).await });
        tokio::time::sleep(Duration::from_millis(20)).await;

        tokio::time::timeout(Duration::from_millis(100), handle.emergency_stop())
            .await
            .expect("emergency stop was stuck behind homing")
            .unwrap();
        assert!(homing.await.unwrap().is_err());
        assert_eq!(handle.status().state, MachineState::EStopped);
    }
}
