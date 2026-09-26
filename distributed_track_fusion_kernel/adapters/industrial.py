"""
Warehouse AGV Fleet Collaborative SLAM Adapter:
Generates multi-robot LiDAR and wheel odometry poses across autonomous
guided vehicles in high-dust industrial distribution centers.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
import random
from typing import List
from ..core.models import TrackState


class WarehouseAGVSLAMAdapter:
    """Generates collaborative AGV localization estimates."""

    @staticmethod
    def create_agv_fleet_scenario(seed: int = 42) -> List[TrackState]:
        """Generates 3 collaborative AGV observations of a pallet transport payload."""
        rng = random.Random(seed)

        # True position of target pallet/obstacle: [x, y, z, vx, vy, vz]
        true_pose = (45.20, 12.80, 0.90, 1.20, 0.0, 0.0)

        # AGV 1: On-board 2D LiDAR (High X/Y accuracy, blind Z)
        p1 = TrackState(
            track_id="PALLET-PAYLOAD-404",
            sensor_id="AGV-01-LIDAR",
            timestamp_ms=500.0,
            state_vector=(
                true_pose[0] + rng.gauss(0.0, 0.02),
                true_pose[1] + rng.gauss(0.0, 0.03),
                0.90,
                1.21, 0.0, 0.0
            ),
            covariance_diag=(0.0004, 0.0009, 0.0400, 0.0025, 0.0010, 0.0010)
        )

        # AGV 2: Overhead Stereo Vision Camera (Moderate X/Y accuracy, high Z altitude accuracy)
        p2 = TrackState(
            track_id="PALLET-PAYLOAD-404",
            sensor_id="OVERHEAD-STEREO-CAM",
            timestamp_ms=501.0,
            state_vector=(
                true_pose[0] + rng.gauss(0.0, 0.05),
                true_pose[1] + rng.gauss(0.0, 0.04),
                0.90 + rng.gauss(0.0, 0.01),
                1.19, 0.0, 0.0
            ),
            covariance_diag=(0.0025, 0.0016, 0.0001, 0.0050, 0.0020, 0.0010)
        )

        # AGV 3: Adjacent Collaborative AGV Ultra-Wideband (UWB) Anchor
        p3 = TrackState(
            track_id="PALLET-PAYLOAD-404",
            sensor_id="AGV-02-UWB-ANCHOR",
            timestamp_ms=502.0,
            state_vector=(
                true_pose[0] + rng.gauss(0.0, 0.08),
                true_pose[1] + rng.gauss(0.0, 0.06),
                0.90,
                1.20, 0.0, 0.0
            ),
            covariance_diag=(0.0064, 0.0036, 0.0100, 0.0040, 0.0020, 0.0010)
        )

        return [p1, p2, p3]
