import logging
from dataclasses import dataclass
from shutil import copy
from turtle import distance
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

                # Going too fast
                time_elapsed_sec = dp.get_time_sec() - last_dp.get_time_sec()
                if dist_travelled / time_elapsed_sec > max_speed:
                    flaggable_list[drone][timeslot] = dp.as_(DronePointFlaggable, flag = True)

        return flaggable_list
       

    @classmethod
    def CreateNewDrone(cls):
        pass 

    @classmethod
    def DistCalc(cls, flag: list[DronePointFlaggable] , avail: list[DronePointFlaggable],  time_index: int) -> float: 
        for i in range(time_index):
            if avail[time_index-i] != None:
                total_distance = distance(flag[i].get_location(), avail[time_index-i].get_location())
                diff = i * MAX_ALLOWED_DISTANCE
                return total_distance - diff
        return 0    
        

    @classmethod
    def DistanceMatrix(cls, drones: list[list[DronePointFlaggable]], flaggedDrones: list[int], time_index: int) -> list[list[float]]:
        
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
                    if(j not in flaggedDrones):
                        flaggedDrones.append(j)
                        flagListLength += 1
                    print(distance)
                    M_distance[i][j]  = distance
                else:
                    M_distance[i][j]  = BIG_NUMBER
            i +=1     

       
        getDeleted = []
        skip = False

        print("M_distance", len(M_distance))
        print("M_distance[0]", len(M_distance[0]))

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
            for i in range(len(M_distance)+1,len(M_distance[0])):
                np.append(M_distance, [BIG_NUMBER for _ in range(len(M_distance[0]))])

        elif matDiff > 0: #more flags than drones
            for i in range(len(M_distance[0])+1,len(M_distance)): # ---------------------------------------------------- fix squareness of matrix
                availableDrones.append(DroneshowModifier.CreateNewDrone())
                
                for j in range(len(M_distance)):
                    M_distance[j][i] = 0
        return M_distance, availableDrones
        
    @classmethod 
    def MinimumColValue(cls, distance_matrix: list[list[float]]) -> list[float]:
        distance_matrix = np.array(distance_matrix)
        return np.min(distance_matrix, axis=0) # axis 0 is row



    @classmethod # er distance_matrix en liste eller et array? (se linje 75)
    def HungarianALG(cls, distance_matrix: list[list[float]]) -> list[tuple[int, int]]: 
        cost_matrix = np.array(distance_matrix)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

       # print("Optimal assignment:", list(zip(row_ind, col_ind)))
        return list(zip(row_ind, col_ind))

        

    @classmethod
    def CheckSolution(cls, sol: list[tuple[int, int]],  minColVal: list[float], available: list[int], flags: list[int]) -> list[tuple[int, int]]:
        print("minimum column value:", minColVal)
        print("solution", sol)
        print("flags", flags)
        print("avail", available)

        checkedSol: list[tuple[int,int]] = []

        for i in range(len(sol)):
            checkedSol.append((available[sol[i][0]],flags[sol[i][1]]))

        for i in range(len(flags)):
            if minColVal[i] == BIG_NUMBER:
                newDroneIndex = DroneshowModifier.CreateNewDrone()
                sol[i] = (newDroneIndex,flags[i]) 
                available.append(newDroneIndex)

        print("checked solution", checkedSol)
        return checkedSol
    
    @classmethod
    def UpdateMatrix(cls, M_Drones: list[list[DronePointFlaggable]], checkedSol: list[tuple[int, int]] , time_index: int) -> list[list[DronePointFlaggable]]:
        
        for pair in checkedSol:
            insert_drone = pair[0]
            for i in range(time_index, len(M_Drones[0])):
                print("insert drone", insert_drone)
                print("i", i)
                print("pair", pair)
                print("pair0", pair[0])
                print("pair1", pair[1])
                M_Drones[pair[1]][i] = insert_drone
                M_Drones[pair[0]][i] = None
        
        return M_Drones
    
    @classmethod
    def FillMatrix(M_Drones: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        for i in range(len(M_Drones)): # drone
            timeInterval = 1
            for j in range(len(M_Drones[0])): # time
                if M_Drones[j][i] == None :
                    if timeInterval == 1:
                        t = j+1
                        while t < len(M_Drones[0]): 
                            timeInterval = t-j-1
                            if M_Drones[t][i] is not None:
                                target = M_Drones[t][i]
                                if j == 0:
                                    start = target
                                    M_Drones[j][i].x = M_Drones[t][i].x 
                                    M_Drones[j][i].y = M_Drones[t][i].y 
                                    M_Drones[j][i].z = M_Drones[t][i].z 
                                    M_Drones[j][i].r = 0
                                    M_Drones[j][i].g = 0
                                    M_Drones[j][i].b = 0
                                    M_Drones[j][i].flag = False
                                else:
                                    start = M_Drones[j-1][i]
                                rx = (start.x - target.x)/timeInterval
                                ry = (start.y - target.y)/timeInterval
                                rz = (start.z - target.z)/timeInterval
                                t = len(M_Drones[0])
                            t += 1
                    if t == len(M_Drones[0])+1:
                        M_Drones[j][i].x = M_Drones[j-1][i].x + rx
                        M_Drones[j][i].y = M_Drones[j-1][i].y + ry
                        M_Drones[j][i].z = M_Drones[j-1][i].z + rz
                        M_Drones[j][i].r = 0
                        M_Drones[j][i].g = 0
                        M_Drones[j][i].b = 0
                        M_Drones[j][i].flag = False

                        timeInterval -= 1
                    elif t == len(M_Drones[0]):
                        M_Drones[j][i].x = M_Drones[j-1][i].x
                        M_Drones[j][i].y = M_Drones[j-1][i].y
                        M_Drones[j][i].z = M_Drones[j-1][i].z
                        M_Drones[j][i].r = 0
                        M_Drones[j][i].g = 0
                        M_Drones[j][i].b = 0
                        M_Drones[j][i].flag = False
                    else:
                        print("not good lmao") # vores version af en error message
        return M_Drones


   
    @classmethod
    def not_main(cls, M_Drones: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        # Create time x Drone Matrix with flags

        for i in range(len(M_Drones[0])):
            flaggedDrones = []
            for j in range(len(M_Drones)):
                if M_Drones[j][i].flag:
                    flaggedDrones.append(j)
                    print("Flag found")
            if (flaggedDrones == []):
                continue
            print("Flagged Drones:", flaggedDrones)
            print("time:", i)
            
            M_DistanceAndAvailDrones = DroneshowModifier.DistanceMatrix(M_Drones, flaggedDrones, i)
            minColVal = DroneshowModifier.MinimumColValue(M_DistanceAndAvailDrones[0])
            sol = DroneshowModifier.HungarianALG(M_DistanceAndAvailDrones[0])
            checkedSol = DroneshowModifier.CheckSolution(sol, minColVal, M_DistanceAndAvailDrones[1], flaggedDrones) 
            M_Drones = DroneshowModifier.UpdateMatrix(M_Drones, checkedSol, i)
        M_Drones = DroneshowModifier.FillMatrix(M_Drones)
        return M_Drones
        # Make MDrones into CSV files
        # Count amount of drones
        # Return CSV files and global drone counter value