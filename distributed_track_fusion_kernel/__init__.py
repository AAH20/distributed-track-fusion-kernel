"""
Distributed Track Fusion Kernel:
Dual-Use Covariance Intersection & Collaborative Multi-Robot SLAM Fusion Engine.
Zero external dependencies (pure Python standard library).
"""

from .core.models import (
    TrackState,
    FusionResult,
    FusionMode
)
from .core.covariance_intersection import CovarianceIntersectionEngine
from .adapters.defense import TacticalRadarESMAdapter
from .adapters.industrial import WarehouseAGVSLAMAdapter
from .kernel import DistributedTrackFusionKernel

__all__ = [
    "TrackState",
    "FusionResult",
    "FusionMode",
    "CovarianceIntersectionEngine",
    "TacticalRadarESMAdapter",
    "WarehouseAGVSLAMAdapter",
    "DistributedTrackFusionKernel"
]

__version__ = "1.0.0"
