import logging
from dataclasses import dataclass

from blinking_drone_alg.droneshow_serializer import DronePoint


@dataclass
class DronePointFlaggable:
    time_ms: int
    x: float
    y: float
    z: float
    r: int
    g: int
    b: int
    flag: bool = False


class DroneshowModifier:
    logger = logging.getLogger(__name__)

    @classmethod
    def flag_droneshow(cls, droneshow: list[list[DronePoint]]) -> list[list[DronePointFlaggable]]:
        pass
