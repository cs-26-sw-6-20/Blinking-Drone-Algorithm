from pathlib import Path

from blinking_drone_alg.droneshow import DronePointFlaggable
from blinking_drone_alg.droneshow_serializer import DroneshowParser, DronePoint


class TestMixins:
    def test_as_convert_drone(self):
        dp = DronePoint(0, 1, 1, 1, 255, 255, 255)

        dpf: DronePointFlaggable = dp.as_(DronePointFlaggable, flag=True)
        assert isinstance(dpf, DronePointFlaggable)
        assert dpf.flag is True
