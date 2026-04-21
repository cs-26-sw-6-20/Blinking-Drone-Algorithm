import logging
from dataclasses import dataclass

from mixins import WithMixin
from blinking_drone_alg.droneshow_serializer import DronePoint


@dataclass
class DronePointFlaggable(WithMixin, DronePoint):
    flag: bool = False


class DroneshowModifier:
    logger = logging.getLogger(__name__)

    @classmethod
    def flag_droneshow(cls, droneshow: list[list[DronePoint]]) -> list[list[DronePointFlaggable]]:
        pass
