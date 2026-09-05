"""Permissive training tools for VelocityPnP vision models."""

from .dataset import (
    CLASS_NAMES,
    SourceComponentDataset,
    dataset_summary,
    load_samples,
    polygon_to_pose,
)
from .model import ModelConfig, SourceComponentPoseModel
from .held_part import (
    HELD_PART_CLASS_NAMES,
    HeldPartDataset,
    HeldPartModelConfig,
    HeldPartPoseModel,
    held_part_dataset_summary,
    load_held_part_samples,
)
from .placement_verification import (
    PLACEMENT_CLASS_NAMES,
    PlacementDataset,
    PlacementModelConfig,
    PlacementSample,
    PlacementVerificationModel,
    load_placement_samples,
    placement_dataset_summary,
    placement_sampling_weights,
)

__all__ = [
    "CLASS_NAMES",
    "ModelConfig",
    "SourceComponentDataset",
    "SourceComponentPoseModel",
    "dataset_summary",
    "load_samples",
    "polygon_to_pose",
    "HELD_PART_CLASS_NAMES",
    "HeldPartDataset",
    "HeldPartModelConfig",
    "HeldPartPoseModel",
    "held_part_dataset_summary",
    "load_held_part_samples",
    "PLACEMENT_CLASS_NAMES",
    "PlacementDataset",
    "PlacementModelConfig",
    "PlacementSample",
    "PlacementVerificationModel",
    "load_placement_samples",
    "placement_dataset_summary",
    "placement_sampling_weights",
]
