use crate::{Pose, Transform2d};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoardSide {
    Top,
    Bottom,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Placement {
    pub reference: String,
    pub value: String,
    pub package: String,
    pub side: BoardSide,
    pub pose: Pose,
    pub enabled: bool,
    pub fiducial: bool,
    pub properties: std::collections::BTreeMap<String, String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardDesign {
    pub id: Uuid,
    pub name: String,
    pub source_format: String,
    pub source_version: Option<u32>,
    pub width_mm: Option<f64>,
    pub height_mm: Option<f64>,
    #[serde(default)]
    pub outline_bounds: Option<BoardOutlineBounds>,
    pub placements: Vec<Placement>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoardOutlineBounds {
    pub min_x_mm: f64,
    pub max_x_mm: f64,
    pub min_y_mm: f64,
    pub max_y_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardInstance {
    pub id: Uuid,
    pub board_id: Uuid,
    pub label: String,
    pub side: BoardSide,
    pub design_to_machine: Option<Transform2d>,
    pub surface_z_mm: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Part {
    pub id: Uuid,
    pub key: String,
    pub manufacturer_part_number: Option<String>,
    pub package: String,
    pub height_mm: Option<f64>,
    pub body_width_mm: Option<f64>,
    pub body_length_mm: Option<f64>,
    pub compatible_tips: Vec<String>,
    pub vision_profile: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Feeder {
    Strip {
        id: Uuid,
        name: String,
        part_key: String,
        first_pick: Pose,
        tape_pitch_mm: f64,
        tape_width_mm: f64,
        tape_rotation_deg: f64,
        remaining: Option<u32>,
    },
    Drag {
        id: Uuid,
        name: String,
        part_key: String,
        pick: Pose,
        feed_distance_mm: f64,
        actuator_id: String,
        remaining: Option<u32>,
    },
    Photon {
        id: Uuid,
        name: String,
        part_key: String,
        hardware_uuid: String,
        slot_address: Option<u8>,
        part_pitch_mm: f64,
        pick: Pose,
    },
}

impl Feeder {
    pub fn id(&self) -> Uuid {
        match self {
            Self::Strip { id, .. } | Self::Drag { id, .. } | Self::Photon { id, .. } => *id,
        }
    }

    pub fn part_key(&self) -> &str {
        match self {
            Self::Strip { part_key, .. }
            | Self::Drag { part_key, .. }
            | Self::Photon { part_key, .. } => part_key,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Job {
    pub id: Uuid,
    pub name: String,
    pub boards: Vec<BoardInstance>,
    pub status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InspectionFinding {
    pub id: Uuid,
    pub board_instance_id: Uuid,
    pub placement_reference: String,
    pub kind: String,
    pub confidence: f64,
    pub measured_offset_mm: Option<f64>,
    pub measured_rotation_deg: Option<f64>,
    pub image_artifact: Option<String>,
    pub disposition: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InspectionStatus {
    AwaitingCapture,
    NeedsReview,
    Passed,
    ReworkRequired,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InspectionReport {
    pub id: Uuid,
    pub job_id: Uuid,
    pub created_at: DateTime<Utc>,
    pub status: InspectionStatus,
    pub findings: Vec<InspectionFinding>,
}
