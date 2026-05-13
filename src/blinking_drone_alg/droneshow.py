import logging
from dataclasses import dataclass
import copy
from typing import Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

from blinking_drone_alg import math_utils
from blinking_drone_alg.droneshow_serializer import DronePoint
from blinking_drone_alg.constants import MAX_ALLOWED_DISTANCE, BIG_NUMBER


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
        flaggable_list: list[list[DronePointFlaggable]] = []

        for drone_path in droneshow:
            flaggable_drone = []
            for dp in drone_path:
                flaggable_drone.append(dp.as_(DronePointFlaggable, flag=False))
            flaggable_list.append(flaggable_drone)

        for drone in range(len(flaggable_list)):
            last_dp = None

            for timeslot in range(len(flaggable_list[drone])):
                dp = flaggable_list[drone][timeslot]

                # First drone is never flagged
                if last_dp is None:
                    last_dp = dp
                    flaggable_list[drone][timeslot] = dp.as_(DronePointFlaggable, flag = False)
                    continue


                dist_travelled = math_utils.distance(last_dp.get_location(), dp.get_location())

                time_elapsed_sec = dp.get_time_sec() - last_dp.get_time_sec()
                velocity = dist_travelled / time_elapsed_sec

                # Going too fast
                if velocity > max_speed:
                    dpf = dp.as_(DronePointFlaggable, flag = True)
                    flaggable_list[drone][timeslot] = dpf

                last_dp = dp
        return flaggable_list


    @classmethod
    def create_new_drone(cls, drones: list[list[Optional[DronePointFlaggable]]]) -> int:
        new_drone_index = len(drones)

        drones.append([DronePointFlaggable(time_ms=i*250, x=0.0, y=0.0, z=0.0, r=0, g=0, b=0, flag=False) for i in range(len(drones[0]))])
        for i in range(len(drones[0])):
            drones[new_drone_index][i] = None

        return new_drone_index


    @classmethod
    def dist_calc(cls, flag: list[Optional[DronePointFlaggable]], avail: list[Optional[DronePointFlaggable]], time_index: int) -> float:
        for i in range(time_index):
            if avail[time_index-i-1] is not None:
                total_distance = math_utils.distance(flag[time_index].get_location(), avail[time_index-i-1].get_location())
                diff = i * MAX_ALLOWED_DISTANCE

                if (total_distance - diff) < 0:
                    return 0
                else:
                    return total_distance - diff
        return 0


    # step 1: calculate cost matrix for 1 timeframe to another.
    # step 2: flag all costs that are too high
    # step 3:
    @classmethod
    def distance_matrix_alg(cls, drones: list[list[Optional[DronePointFlaggable]]], flagged_drones: list[int], time_index: int) -> tuple[list[list[float]], list[int]]:
        distance_matrix: list[list[float]] = []

        flag_list_length = len(flagged_drones)
        available_drones = []
        i = 0

        while i < flag_list_length:
            distance_matrix.append([BIG_NUMBER for _ in range(len(drones))])
            for j in range(len(drones)):
                if j not in available_drones:
                    available_drones.append(j)

                distance = DroneshowModifier.dist_calc(drones[flagged_drones[i]], drones[j], time_index)
                if distance <= MAX_ALLOWED_DISTANCE:
                    if j not in flagged_drones and drones[j][time_index] is not None:
                        flagged_drones.append(j)
                        flag_list_length += 1
                    distance_matrix[i][j]  = distance
                else:
                    distance_matrix[i][j]  = BIG_NUMBER
            i +=1

        get_deleted = []

        for i in range(len(distance_matrix[0])):
            skip = False
            for j in range(len(distance_matrix)):
                if distance_matrix[j][i] != BIG_NUMBER:
                    skip = True
                    break
            if not skip:
                get_deleted.append(available_drones[i])

        distance_matrix_np = np.array(distance_matrix)

        for i in range(len(get_deleted), 0,-1):
            distance_matrix_np = np.delete(distance_matrix_np, get_deleted[i-1], 1)

            available_drones.pop(get_deleted[i-1])
        
        mat_diff = len(distance_matrix_np) - len(distance_matrix_np[flagged_drones[0]])

        if mat_diff < 0: #more drones than flags
            for i in range(len(distance_matrix_np),len(distance_matrix_np[0])):
                dummy_flag = [BIG_NUMBER for _ in range(len(distance_matrix_np[0]))]
                np.append(distance_matrix_np, dummy_flag)

        elif mat_diff > 0: #more flags than drones
            for i in range(len(distance_matrix_np[0]),len(distance_matrix_np)):
                available_drones.append(DroneshowModifier.create_new_drone(drones))
                new_drone = [[0] for _ in range(len(distance_matrix_np)) ]
                distance_matrix_np = np.append(distance_matrix_np, new_drone, 1)

        return distance_matrix_np.getlist(), available_drones

    @classmethod
    def hungarian_alg(cls, distance_matrix: list[list[float]]) -> list[tuple[int, int]]:
        cost_matrix = np.array(distance_matrix)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        return list(zip(row_ind, col_ind))

    @classmethod
    def check_solution(cls, drone_matrix: list[list[Optional[DronePointFlaggable]]], sol: list[tuple[int, int]], distance_matrix: list[list[float]], available: list[int], flags: list[int]) -> list[tuple[int, int]]:

        checked_sol: list[tuple[int,int]] = []

        for i in range(len(sol)):
            flagged, avail = sol[i]
            checked_sol.append((available[avail], flags[flagged]))

        for i in range(len(flags)): # TODO: make a test for this.
            flagged, avail = sol[i]

            if distance_matrix[flagged][avail] == BIG_NUMBER:
                new_drone_index = DroneshowModifier.create_new_drone(drone_matrix)
                sol[i] = (new_drone_index,flags[i])
                available.append(new_drone_index)

        return checked_sol

    @classmethod
    def update_matrix(cls, drone_matrix: list[list[Optional[DronePointFlaggable]]], checked_sol: list[tuple[int, int]], time_index: int) -> list[list[Optional[DronePointFlaggable]]]:
        copies: list[tuple[list[Optional[DronePointFlaggable]],list[Optional[DronePointFlaggable]]]] = []

        for sol in checked_sol:
            avail = copy.deepcopy(drone_matrix[sol[0]])
            flag = copy.deepcopy(drone_matrix[sol[1]])

            copies.append((avail, flag))

        for j in range(len(checked_sol)):
            for i in range(time_index, len(drone_matrix[0])):
                _, flagged = checked_sol[j]
                drone_matrix[flagged][i] = None

        for j in range(len(checked_sol)):
            insert_drone = copies[j][1]
            for i in range(time_index, len(drone_matrix[0])):
                drone_matrix[checked_sol[j][0]][i] = insert_drone[i]

        return drone_matrix

    @classmethod
    def extrapolate_drone_path(cls, drone: list[Optional[DronePointFlaggable]], start: DronePoint, end: DronePoint, start_index: int, end_index: int) -> None:
        for k in range(0, end_index - start_index):
            frame = start_index + k

            t = (k + 1) / (end_index - start_index + 1)

            location = DronePoint.interpolate_location(
                start,
                end,
                t)

            drone[frame] = DronePointFlaggable(
                time_ms=250 * frame,
                x=location[0],
                y=location[1],
                z=location[2],
                r=0, g=0, b=0, flag=False)

    @classmethod
    def fill_matrix(cls, drone_matrix: list[list[Optional[DronePointFlaggable]]]) -> list[list[DronePointFlaggable]]:
        get_deleted = []

        for i in range(len(drone_matrix)): # drones
            drone: list[Optional[DronePointFlaggable]] = drone_matrix[i]
            first_none_index: int = 0
            last_known_position: Optional[DronePointFlaggable] = None
            last_position: Optional[DronePointFlaggable] = drone[0]
            current_position: Optional[DronePointFlaggable] = None

            for frame in range(1, len(drone)): # time
                current_position = drone[frame]

                # Hasn't changed
                if (current_position is None) == (last_position is None):
                    last_position = current_position
                    continue

                # It is now none
                if current_position is None:
                    first_none_index = frame
                    last_known_position = last_position
                    last_position = current_position
                    continue

                # it is not none anymore
                if last_known_position is None:
                    last_known_position = current_position

                cls.extrapolate_drone_path(
                    drone,
                    last_known_position,
                    current_position,
                    first_none_index,
                    frame
                )
                last_position = drone[frame]


            # The rest of the drone is unused
            if current_position is None:

                # Drone is completely unused
                if last_known_position is None:
                    get_deleted.append(i)
                    continue

                cls.extrapolate_drone_path(
                    drone,
                    last_known_position,
                    last_known_position,
                    first_none_index,
                    len(drone)
                )
                continue

        drone_matrix_np = np.delete(np.array(drone_matrix), get_deleted, 0)
        return drone_matrix_np.tolist()

    @classmethod
    def not_main(cls, drone_matrix: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        # Create time x Drone Matrix with flags
        drone_matrix_temp: list[list[Optional[DronePointFlaggable]]] = copy.deepcopy(drone_matrix)

        for i in range(len(drone_matrix[0])):
            flagged_drones = []
            for j in range(len(drone_matrix)):
                if drone_matrix[j][i] is not None:
                    if drone_matrix[j][i].flag:
                        flagged_drones.append(j)
            if len(flagged_drones) == 0:
                continue

            distance_matrix, avail_drones = DroneshowModifier.distance_matrix_alg(drone_matrix_temp, flagged_drones, i)
            sol = DroneshowModifier.hungarian_alg(distance_matrix)
            checked_sol = DroneshowModifier.check_solution(drone_matrix_temp, sol, distance_matrix, avail_drones, flagged_drones)
            drone_matrix_temp = DroneshowModifier.update_matrix(drone_matrix_temp, checked_sol, i)

        return DroneshowModifier.fill_matrix(drone_matrix_temp)
