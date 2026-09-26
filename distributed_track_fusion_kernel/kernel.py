"""
Master Distributed Track Fusion Kernel:
Decentralized Covariance Intersection facade for radar tracks and warehouse AGV SLAM.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
from typing import List, Tuple
from .core.models import TrackState, FusionResult, FusionMode
from .core.covariance_intersection import CovarianceIntersectionEngine
from .adapters.defense import TacticalRadarESMAdapter
from .adapters.industrial import WarehouseAGVSLAMAdapter


class DistributedTrackFusionKernel:
    """Master facade for decentralized track fusion without data incest."""

    def __init__(self, tolerance: float = 1e-4):
        self.engine = CovarianceIntersectionEngine(tolerance=tolerance)

    def fuse_tactical_missile_tracks(self, seed: int = 42) -> FusionResult:
        """Executes Covariance Intersection across radar, ESM, and EO/IR missile tracks."""
        tracks = TacticalRadarESMAdapter.create_missile_tracking_scenario(seed=seed)
        return self.engine.fuse_multiple(tracks, mode=FusionMode.TACTICAL_DEFENSE)

    def fuse_warehouse_agv_slam(self, seed: int = 42) -> FusionResult:
        """Executes Covariance Intersection across multi-robot AGV LiDAR and vision poses."""
        tracks = WarehouseAGVSLAMAdapter.create_agv_fleet_scenario(seed=seed)
        return self.engine.fuse_multiple(tracks, mode=FusionMode.INDUSTRIAL_AGV_SLAM)

    def fuse_custom_tracks(
        self,
        tracks: List[TrackState],
        mode: FusionMode = FusionMode.TACTICAL_DEFENSE
    ) -> FusionResult:
        """Fuses an arbitrary list of user-provided track states."""
        return self.engine.fuse_multiple(tracks, mode=mode)
