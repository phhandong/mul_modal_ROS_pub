"""ARS404/ARS408 CAN frame decoding.

The layout is taken from *Standardized ARS Interface*, sections 9.1, 9.2,
10.1, and 10.2. ARS signals use Motorola byte order; the expressions below
retain the byte layout from that document instead of relying on host byte
order.
"""

from dataclasses import dataclass
from typing import Optional


OBJECT_STATUS_ID = 0x60A
OBJECT_GENERAL_ID = 0x60B
CLUSTER_STATUS_ID = 0x600
CLUSTER_GENERAL_ID = 0x701


@dataclass(frozen=True)
class ObjectStatus:
    count: int
    measurement_counter: int
    interface_version: int


@dataclass(frozen=True)
class RadarObject:
    object_id: int
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    rcs_dbm2: float
    dynamic_property: int


@dataclass(frozen=True)
class ClusterStatus:
    near_count: int
    far_count: int
    measurement_counter: int
    interface_version: int

    @property
    def count(self) -> int:
        return self.near_count + self.far_count


def decode_object_status(payload: bytes) -> Optional[ObjectStatus]:
    """Decode Obj_0_Status (0x60A); return None for a short CAN payload."""
    if len(payload) < 5:
        return None
    return ObjectStatus(
        count=payload[0],
        measurement_counter=(payload[2] << 8) | payload[3],
        interface_version=(payload[4] >> 4) & 0x0F,
    )


def decode_object_general(payload: bytes) -> Optional[RadarObject]:
    """Decode Obj_1_General (0x60B), using the documented offsets/scales."""
    if len(payload) < 8:
        return None
    distance_long = (payload[1] << 5) | (payload[2] >> 3)
    distance_lat = ((payload[2] & 0x07) << 8) | payload[3]
    velocity_long = (payload[4] << 2) | (payload[5] >> 6)
    velocity_lat = ((payload[5] & 0x3F) << 3) | (payload[6] >> 5)
    return RadarObject(
        object_id=payload[0],
        x_m=distance_long * 0.2 - 500.0,
        y_m=distance_lat * 0.2 - 204.6,
        vx_mps=velocity_long * 0.25 - 128.0,
        vy_mps=velocity_lat * 0.25 - 64.0,
        rcs_dbm2=payload[7] * 0.5 - 64.0,
        dynamic_property=payload[6] & 0x07,
    )


def decode_cluster_status(payload: bytes) -> Optional[ClusterStatus]:
    """Decode Cluster_0_Status (0x600)."""
    if len(payload) < 5:
        return None
    return ClusterStatus(
        near_count=payload[0],
        far_count=payload[1],
        measurement_counter=(payload[2] << 8) | payload[3],
        interface_version=(payload[4] >> 4) & 0x0F,
    )


def decode_cluster_general(payload: bytes) -> Optional[RadarObject]:
    """Decode Cluster_1_General (0x701)."""
    if len(payload) < 8:
        return None
    distance_long = (payload[1] << 5) | (payload[2] >> 3)
    distance_lat = ((payload[2] & 0x03) << 8) | payload[3]
    velocity_long = (payload[4] << 2) | (payload[5] >> 6)
    velocity_lat = ((payload[5] & 0x3F) << 3) | (payload[6] >> 5)
    return RadarObject(
        object_id=payload[0],
        x_m=distance_long * 0.2 - 500.0,
        y_m=distance_lat * 0.2 - 102.3,
        vx_mps=velocity_long * 0.25 - 128.0,
        vy_mps=velocity_lat * 0.25 - 64.0,
        rcs_dbm2=payload[7] * 0.5 - 64.0,
        dynamic_property=payload[6] & 0x07,
    )
