"""
Tactical Radar & ESM Sensor Fusion Adapter:
Generates multi-sensor tracklets across monostatic radar, passive RF ESM,
and airborne electro-optical pods tracking low-observable cruise missiles.
Zero external dependencies (pure Python standard library).
"""

from __future__ import annotations
import random
from typing import List
from ..core.models import TrackState


class TacticalRadarESMAdapter:
    """Generates heterogeneous tactical tracklets with aspect-dependent covariance."""

    @staticmethod
    def create_missile_tracking_scenario(seed: int = 42) -> List[TrackState]:
        """Generates 3 multi-sensor observations of an incoming Mach 2 cruise missile."""
        rng = random.Random(seed)

        # Ground truth: [x, y, z, vx, vy, vz]
        true_state = (12000.0, 4500.0, 150.0, -680.0, 15.0, 0.0)

        # 1. Monostatic Ground Radar: High range accuracy (X), lower cross-range (Y, Z)
        r_x = true_state[0] + rng.gauss(0.0, 15.0)
        r_y = true_state[1] + rng.gauss(0.0, 80.0)
        r_z = true_state[2] + rng.gauss(0.0, 50.0)
        s1 = TrackState(
            track_id="TGT-CRUISE-99",
            sensor_id="GROUND-RADAR-AESA",
            timestamp_ms=1000.0,
            state_vector=(r_x, r_y, r_z, -675.0, 18.0, 0.0),
            covariance_diag=(225.0, 6400.0, 2500.0, 25.0, 100.0, 25.0)  # Variance
        )

        # 2. Airborne ESM Pod: High angular cross-range accuracy (Y), lower range depth (X)
        e_x = true_state[0] + rng.gauss(0.0, 95.0)
        e_y = true_state[1] + rng.gauss(0.0, 12.0)
        e_z = true_state[2] + rng.gauss(0.0, 30.0)
        s2 = TrackState(
            track_id="TGT-CRUISE-99",
            sensor_id="AIRBORNE-ESM-POD",
            timestamp_ms=1001.0,
            state_vector=(e_x, e_y, e_z, -682.0, 14.0, 0.0),
            covariance_diag=(9025.0, 144.0, 900.0, 100.0, 20.0, 25.0)
        )

        # 3. Forward Drone EO/IR Camera: High angular resolution, moderate range estimation
        o_x = true_state[0] + rng.gauss(0.0, 35.0)
        o_y = true_state[1] + rng.gauss(0.0, 25.0)
        o_z = true_state[2] + rng.gauss(0.0, 8.0)
        s3 = TrackState(
            track_id="TGT-CRUISE-99",
            sensor_id="FORWARD-DRONE-EOIR",
            timestamp_ms=1002.0,
            state_vector=(o_x, o_y, o_z, -679.0, 15.0, 0.0),
            covariance_diag=(1225.0, 625.0, 64.0, 50.0, 50.0, 10.0)
        )

        return [s1, s2, s3]
