import logging
from dataclasses import dataclass
import copy
import numpy as np
from scipy.optimize import linear_sum_assignment

from blinking_drone_alg import math_utils
from blinking_drone_alg.droneshow_serializer import DronePoint
from blinking_drone_alg.constants import MAX_ALLOWED_DISTANCE, BIG_NUMBER
from blinking_drone_alg.math_utils import distance 



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

                # Going too fast
                time_elapsed_sec = dp.get_time_sec() - last_dp.get_time_sec()
                if time_elapsed_sec != 0 and dist_travelled / time_elapsed_sec > max_speed:
                    flaggable_list[drone][timeslot] = dp.as_(DronePointFlaggable, flag = True)

                last_dp = dp
        return flaggable_list
       

    @classmethod
    def CreateNewDrone(cls, drones: list[list[DronePointFlaggable]]) -> int:
        newDroneIndex = len(drones)   
        
        drones.append([DronePointFlaggable(time_ms=i*250, x=0.0, y=0.0, z=0.0, r=0, g=0, b=0, flag=False) for i in range(len(drones[0]))])
        # drones.append([])
        for i in range(len(drones[0])):
            drones[newDroneIndex][i] = None

        return newDroneIndex 
        

    @classmethod
    def DistCalc(cls, flag: list[DronePointFlaggable] , avail: list[DronePointFlaggable],  time_index: int) -> float: 
        for i in range(time_index):
            if avail[time_index-i-1] != None:
               
                total_distance = distance(flag[time_index].get_location(), avail[time_index-i-1].get_location())
                diff = i * MAX_ALLOWED_DISTANCE
                
                if (total_distance - diff) < 0:
                    return 0
                else:
                    return total_distance - diff
        return 0    
        

    @classmethod
    def DistanceMatrix(cls, drones: list[list[DronePointFlaggable]], flaggedDrones: list[int], time_index: int) -> tuple[list[list[float]], list[int]]:
        
        M_distance: list[list[float]] = []

        flagListLength = len(flaggedDrones)
        availableDrones = []
        i = 0
        while i < flagListLength:
            M_distance.append([BIG_NUMBER for _ in range(len(drones))])
            for j in range(len(drones)):
                if(j not in availableDrones):
                    availableDrones.append(j)
               
                distance = DroneshowModifier.DistCalc(drones[flaggedDrones[i]], drones[j], time_index)
                if distance <= MAX_ALLOWED_DISTANCE:
                    if(j not in flaggedDrones and drones[j][time_index] is not None):
                        flaggedDrones.append(j)
                        flagListLength += 1
                    M_distance[i][j]  = distance
                else:
                    M_distance[i][j]  = BIG_NUMBER
            i +=1     

       
        getDeleted = []
        skip = False


        for i in range(len(M_distance[0])): # drone
            skip = False
            for j in range(len(M_distance)): # flag
                if M_distance[j][i] != BIG_NUMBER:
                    skip = True
                    break
            if (skip != True): 
                getDeleted.append(availableDrones[i])
        

        M_distance = np.array(M_distance)
             
        for i in range(len(getDeleted), 0,-1):
            M_distance = np.delete(M_distance, getDeleted[i-1], 1)

            availableDrones.pop(getDeleted[i-1])  
        
        matDiff = len(M_distance) - len(M_distance[flaggedDrones[0]])

        if matDiff < 0: #more drones than flags
            for i in range(len(M_distance),len(M_distance[0])):
                dummyFlag = [BIG_NUMBER for _ in range(len(M_distance[0]))]
                np.append(M_distance, dummyFlag)

        elif matDiff > 0: #more flags than drones
            for i in range(len(M_distance[0]),len(M_distance)): 
                availableDrones.append(DroneshowModifier.CreateNewDrone(drones))
                newDrone = [[0] for _ in range(len(M_distance)) ]
                M_distance = np.append(M_distance, newDrone, 1)
        
        return M_distance, availableDrones
        

    @classmethod # er distance_matrix en liste eller et array? (se linje 75)
    def HungarianALG(cls, distance_matrix: list[list[float]]) -> list[tuple[int, int]]: 
        cost_matrix = np.array(distance_matrix)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        return list(zip(row_ind, col_ind)) 

      

    @classmethod
    def CheckSolution(cls, sol: list[tuple[int, int]],  M_Distance: list[list[float]], available: list[int], flags: list[int]) -> list[tuple[int, int]]:

        checkedSol: list[tuple[int,int]] = []

        for i in range(len(sol)):
            checkedSol.append((available[sol[i][1]],flags[sol[i][0]]))

        for i in range(len(flags)): #TODO: hvad fuck foregår der??
            if M_Distance[sol[i][0]][sol[i][1]] == BIG_NUMBER:
                newDroneIndex = DroneshowModifier.CreateNewDrone()
                sol[i] = (newDroneIndex,flags[i]) 
                available.append(newDroneIndex)

        return checkedSol
    
    @classmethod
    def UpdateMatrix(cls, M_Drones: list[list[DronePointFlaggable]], checkedSol: list[tuple[int, int]] , time_index: int) -> list[list[DronePointFlaggable]]:
        
        avail = []
        flag = []

        copies: list[tuple[DronePointFlaggable,DronePointFlaggable]] = []

        for sol in checkedSol:
            avail = copy.deepcopy(M_Drones[sol[0]])
            flag = copy.deepcopy(M_Drones[sol[1]])
          
            copies.append((avail, flag))     
       
        for j in range(len(checkedSol)):
            for i in range(time_index, len(M_Drones[0])):
                M_Drones[checkedSol[j][1]][i] = None
        for j in range(len(checkedSol)):
            insert_drone = copies[j][1]
            for i in range(time_index, len(M_Drones[0])):
                M_Drones[checkedSol[j][0]][i] = insert_drone[i]

        return M_Drones
    
    @classmethod
    def fill_matrix(cls, drone_matrix: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        
        get_deleted = []

        for i in range(len(drone_matrix)): # drones
            time_index = 0
            while time_index < len(drone_matrix[0]): # time
                case = 0
                if (drone_matrix[i][time_index]) is None:
                    start_dp = time_index-1
                    none_index = time_index
                    target_dp = 0
                    for t in range(none_index+1, len(drone_matrix[0])):
                        target_dp = t
                        if (drone_matrix[i][t]) is not None:
                            if none_index == 0: # Case 1
                                case = 1
                                break
                            else:
                                case = 2
                                break
                        else:
                            if none_index == 0:
                                case = 3
                            else:
                                case = 4
                    if case == 1:
                        for k in range(none_index, target_dp):
                            drone_matrix[i][k] = DronePointFlaggable(time_ms=250 * k, x=drone_matrix[i][target_dp].x, y=drone_matrix[i][target_dp].y, z=drone_matrix[i][target_dp].z, r=0, g=0, b=0, flag=False)
                        time_index = target_dp
                    elif case == 2:
                        rx = (drone_matrix[i][start_dp].x - drone_matrix[i][target_dp].x) / (target_dp - start_dp)
                        ry = (drone_matrix[i][start_dp].y - drone_matrix[i][target_dp].y) / (target_dp - start_dp)
                        rz = (drone_matrix[i][start_dp].z - drone_matrix[i][target_dp].z) / (target_dp - start_dp)
                        for k in range(none_index, target_dp):
                            drone_matrix[i][k] = DronePointFlaggable(time_ms=250 * k, x=drone_matrix[i][k - 1].x - rx, y=drone_matrix[i][k - 1].y - ry, z=drone_matrix[i][k - 1].z - rz, r=0, g=0, b=0, flag=False)
                        time_index = target_dp
                    elif case == 3:
                        get_deleted.append(i)
                    elif case == 4:
                        for k in range(none_index, len(drone_matrix[0])):
                            drone_matrix[i][k] = DronePointFlaggable(time_ms=250 * k, x=drone_matrix[i][start_dp].x, y=drone_matrix[i][start_dp].y, z=drone_matrix[i][start_dp].z, r=0, g=0, b=0, flag=False)
                time_index += 1


        drone_matrix = np.delete(drone_matrix, get_deleted, 0)
        return drone_matrix

   
    @classmethod
    def not_main(cls, drone_matrix: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        # Create time x Drone Matrix with flags

        for i in range(len(drone_matrix[0])):
            flagged_drones = []
            for j in range(len(drone_matrix)):
                if drone_matrix[j][i] is not None:
                    if drone_matrix[j][i].flag:
                        flagged_drones.append(j)
            if len(flagged_drones) == 0:
                continue
            
            distance_matrix, avail_drones = DroneshowModifier.DistanceMatrix(drone_matrix, flagged_drones, i)
            sol = DroneshowModifier.HungarianALG(distance_matrix)
            checked_sol = DroneshowModifier.CheckSolution(sol, distance_matrix, avail_drones, flagged_drones)
            drone_matrix = DroneshowModifier.UpdateMatrix(drone_matrix, checked_sol, i)
       
        drone_matrix = DroneshowModifier.fill_matrix(drone_matrix)
        return drone_matrix
        # Make MDrones into CSV files
        # Count amount of drones
        # Return CSV files and global drone counter value