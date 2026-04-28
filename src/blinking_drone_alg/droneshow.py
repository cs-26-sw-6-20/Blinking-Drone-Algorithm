import logging
from dataclasses import dataclass
from shutil import copy
from turtle import distance
import numpy as np
from scipy.optimize import linear_sum_assignment

from blinking_drone_alg.droneshow_serializer import DronePoint
from blinking_drone_alg.constants import MAX_ALLOWED_DISTANCE, BIG_NUMBER
from blinking_drone_alg.math_utils import distance 



@dataclass
class DronePointFlaggable:
    time_ms: float
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
       

    @classmethod
    def CreateNewDrone(cls):
        pass 

    @classmethod
    def DistCalc(cls, flag: list[DronePointFlaggable] , avail: list[DronePointFlaggable],  time_index: int) -> float: 
        
        print("Time index:", time_index)
        for i in range(time_index):
            print("Din")
            if avail[time_index-i] != None:
                print("Mor")
                total_distance = distance(flag[i].get_location(), avail[time_index-i].get_location())
                diff = i * MAX_ALLOWED_DISTANCE
                return total_distance - diff
        return 0    
        

    @classmethod
    def DistanceMatrix(cls, drones: list[list[DronePointFlaggable]], flaggedDrones: list[int], time_index: int) -> list[list[float]]:
        
        M_distance: list[list[float]] = []

        for i in range(len(flaggedDrones)):
            M_distance.append([BIG_NUMBER for _ in range(len(drones))])
            for j in range(len(drones)):
                    distance = DroneshowModifier.DistCalc(drones[flaggedDrones[i]], drones[j], time_index)
                    if distance <= MAX_ALLOWED_DISTANCE:
                        if(j not in flaggedDrones):
                            flaggedDrones.append(j)
                        print(distance)
                        M_distance[i][j]  = distance
                    else:
                        M_distance[i][j]  = BIG_NUMBER


        print("Initial Distance Matrix:", M_distance)
        for i in range(len(M_distance)): # drones
            for j in range(len(distance[0])): # flags
                if M_distance[j][i] != BIG_NUMBER:
                    break
                
            M_distance = np.array(M_distance)
            M_distance = np.delete(M_distance, i, 1)

        print("Distance Matrix:", M_distance)
        matDiff = len(M_distance) - len(M_distance[flaggedDrones[0]])

        if matDiff < 0: #more drones than flags
            for i in range(len(M_distance)+1,len(M_distance[0])):
                for j in range(len(M_distance[0])):
                    M_distance[i][j] = BIG_NUMBER

        elif matDiff > 0: #more flags than drones
            for i in range(len(M_distance[0])+1,len(M_distance)):
                DroneshowModifier.CreateNewDrone()
                for j in range(len(M_distance)):
                    M_distance[j][i] = 0

        return M_distance
        
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
    def CheckSolution(cls, sol: list[tuple[int, int]],  minColVal: list[float], flags: list[int]) -> list[tuple[int, int]]:
        for i in range(len(flags)):
            if minColVal[i] == BIG_NUMBER:
                sol[i] = (DroneshowModifier.CreateNewDrone(),flags[i])
        return sol
    
    @classmethod
    def UpdateMatrix(cls, M_Drones: list[list[DronePointFlaggable]], checkedSol: list[tuple[int, int]] , time_index: int) -> list[list[DronePointFlaggable]]:
        copies = []
        
        for pair in checkedSol:
            copies.append(copy.copy(pair[1])) # copy all flag-values

            insert_drone = copies[pair[0]]

            for i in range(time_index, len(M_Drones[0])):
                M_Drones[pair[1]][i] = insert_drone[i] 
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
            
            MDistance = DroneshowModifier.DistanceMatrix(M_Drones, flaggedDrones, i)
            minColVal = DroneshowModifier.MinimumColValue(MDistance)
            sol = DroneshowModifier.HungarianALG(MDistance)
            checkedSol = DroneshowModifier.CheckSolution(sol, minColVal, flaggedDrones) 
            MDrones = DroneshowModifier.UpdateMatrix(MDrones, checkedSol, i)
        MDrones = DroneshowModifier.FillMatrix(MDrones)
        return MDrones
        # Make MDrones into CSV files
        # Count amount of drones
        # Return CSV files and global drone counter value