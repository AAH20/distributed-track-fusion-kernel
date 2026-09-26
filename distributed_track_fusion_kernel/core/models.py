"""
Data Models for Distributed Track Fusion Kernel:
Defines track states, covariance matrices, and fusion results across
radar tracking and collaborative warehouse AGV SLAM.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class FusionMode(str, Enum):
    TACTICAL_DEFENSE = "TACTICAL_DEFENSE"
    INDUSTRIAL_AGV_SLAM = "INDUSTRIAL_AGV_SLAM"


@dataclass
class TrackState:
    """Represents a 6-DOF kinematic state track [x, y, z, vx, vy, vz]."""
    track_id: str
    sensor_id: str
    timestamp_ms: float
    state_vector: Tuple[float, float, float, float, float, float]
    # 6-element diagonal covariance representing variance along [x, y, z, vx, vy, vz]
    covariance_diag: Tuple[float, float, float, float, float, float]
    confidence_score: float = 1.0


@dataclass
class FusionResult:
    """Result of Covariance Intersection fusion across multiple sensor tracks."""
    fused_track_id: str
    mode: FusionMode
    state_vector: Tuple[float, float, float, float, float, float]
    covariance_diag: Tuple[float, float, float, float, float, float]
    trace_uncertainty: float
    optimal_omega: float
    data_incest_prevented: bool
    execution_latency_us: float
    contributing_sensors: List[str] = field(default_factory=list)
    metrics: Dict[str, float] = field(default_factory=dict)
