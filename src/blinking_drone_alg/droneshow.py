import logging
from dataclasses import dataclass

from blinking_drone_alg.droneshow_serializer import DronePoint
import blinking_drone_alg.math_utils


@dataclass
class DronePointFlaggable(DronePoint):
    flag: bool = False


class DroneshowModifier:
    logger = logging.getLogger(__name__)

    @classmethod
    def flag_droneshow(cls, droneshow: list[list[DronePoint]], max_speed: float) -> list[list[DronePointFlaggable]]:
        """
        :param droneshow:
        :param max_speed:
        :return:
        """
        flaggable_list = droneshow

        for drone in range(len(flaggable_list)):
            last_dp = None

            for timeslot in range(len(flaggable_list[drone])):
                dp = flaggable_list[drone][timeslot]

                # First drone is never flagged
                if last_dp is None:
                    last_dp = dp
                    flaggable_list[drone][timeslot] = dp.as_(DronePointFlaggable, flag = False)
                    continue

                last_dp = dp

                dist_travelled = math_utils.distance(last_dp.get_location(), dp.get_location())

                time_elapsed_sec = dp.get_time_sec() - last_dp.get_time_sec()
                velocity = dist_travelled / time_elapsed_sec

                # Going too fast
                if velocity > max_speed:
                    flaggable_list[drone][timeslot] = dp.as_(DronePointFlaggable, flag = True)

        return flaggable_list
