"""Compact source-component occupancy and pose model."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import torch
from torch import nn
from torch.nn import functional as F


@dataclass(frozen=True)
class ModelConfig:
    """Shape and channel configuration stored with each model artifact."""

    image_size: int = 192
    input_channels: int = 3
    class_count: int = 3
    feature_channels: int = 96

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class ConvBlock(nn.Module):
    def __init__(self, input_channels: int, output_channels: int, stride: int) -> None:
        super().__init__()
        self.layers = nn.Sequential(
            nn.Conv2d(
                input_channels,
                output_channels,
                kernel_size=3,
                stride=stride,
                padding=1,
                bias=False,
            ),
            nn.BatchNorm2d(output_channels),
            nn.SiLU(),
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.layers(inputs)


class ResidualBlock(nn.Module):
    def __init__(self, channels: int) -> None:
        super().__init__()
        self.layers = nn.Sequential(
            ConvBlock(channels, channels, 1),
            nn.Conv2d(channels, channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(channels),
        )
        self.activation = nn.SiLU()

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.activation(inputs + self.layers(inputs))


class SourceComponentPoseModel(nn.Module):
    """Predict occupancy and one normalized oriented box per image."""

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__()
        self.config = config or ModelConfig()
        if self.config.image_size % 8:
            raise ValueError("model image size must be divisible by 8")
        self.stem = nn.Sequential(
            nn.Conv2d(
                self.config.input_channels,
                24,
                kernel_size=5,
                stride=2,
                padding=2,
                bias=False,
            ),
            nn.BatchNorm2d(24),
            nn.SiLU(),
        )
        self.stage_one = nn.Sequential(
            ConvBlock(24, 48, 2),
            ResidualBlock(48),
            ResidualBlock(48),
        )
        self.pose_features = nn.Sequential(
            ConvBlock(48, self.config.feature_channels, 2),
            ResidualBlock(self.config.feature_channels),
            ResidualBlock(self.config.feature_channels),
        )
        self.occupancy_features = nn.Sequential(
            ConvBlock(48, self.config.feature_channels, 2),
            ResidualBlock(self.config.feature_channels),
            ResidualBlock(self.config.feature_channels),
        )
        self.top_down = nn.Conv2d(
            self.config.feature_channels, 64, kernel_size=1, bias=False
        )
        self.lateral = nn.Conv2d(48, 64, kernel_size=1, bias=False)
        self.local_features = nn.Sequential(
            ConvBlock(64, 64, 1),
            ResidualBlock(64),
        )
        self.center_head = nn.Conv2d(64, 1, kernel_size=1)
        self.local_pose_head = nn.Conv2d(64, 4, kernel_size=1)
        with torch.no_grad():
            self.local_pose_head.bias[:2].fill_(-2.0)
        self.classifier = nn.Sequential(
            nn.Linear(self.config.feature_channels, 64),
            nn.SiLU(),
            nn.Dropout(0.1),
            nn.Linear(64, 2),
        )
        feature_size = self.config.image_size // 4
        coordinates = (torch.arange(feature_size, dtype=torch.float32) + 0.5)
        coordinates /= feature_size
        grid_y, grid_x = torch.meshgrid(coordinates, coordinates, indexing="ij")
        self.register_buffer(
            "center_grid",
            torch.stack((grid_x.flatten(), grid_y.flatten()), dim=1),
        )

    def _predict(
        self, inputs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        inputs = (inputs - 0.5) / 0.25
        shallow_features = self.stage_one(self.stem(inputs))
        deep_features = self.pose_features(shallow_features)
        occupancy_features = self.occupancy_features(shallow_features)
        global_features = occupancy_features.mean(dim=(2, 3))
        state_logits = self.classifier(global_features)
        occupied = F.logsigmoid(state_logits[:, 0])
        empty = F.logsigmoid(-state_logits[:, 0])
        flat = occupied + F.logsigmoid(-state_logits[:, 1])
        on_edge = occupied + F.logsigmoid(state_logits[:, 1])
        occupancy_logits = torch.stack((empty, flat, on_edge), dim=1)

        top_down = F.interpolate(
            self.top_down(deep_features), scale_factor=2, mode="nearest"
        )
        local_features = self.local_features(
            top_down + self.lateral(shallow_features)
        )
        center_logits = self.center_head(local_features).flatten(1)
        center_weights = F.softmax(center_logits, dim=1)
        grid = self.center_grid.unsqueeze(0).expand(inputs.shape[0], -1, -1)
        center = torch.bmm(center_weights.unsqueeze(1), grid).squeeze(1)
        raw_pose = torch.bmm(
            self.local_pose_head(local_features).flatten(2),
            center_weights.unsqueeze(2),
        ).squeeze(2)
        size = torch.sigmoid(raw_pose[:, :2])
        angle = F.normalize(raw_pose[:, 2:], dim=1, eps=1e-6)
        pose = torch.cat((center, size, angle), dim=1)
        return occupancy_logits, pose, center_logits, state_logits

    def forward(self, inputs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        occupancy_logits, pose, _, _ = self._predict(inputs)
        return occupancy_logits, pose

    def forward_training(
        self, inputs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        return self._predict(inputs)
