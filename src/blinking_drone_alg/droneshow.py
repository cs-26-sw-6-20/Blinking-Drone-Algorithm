import logging
from dataclasses import dataclass
from shutil import copy
from turtle import distance
import numpy as np

from blinking_drone_alg.droneshow_serializer import DronePoint
from blinking_drone_alg.main_alg import MAX_ALLOWED_DISTANCE, BIG_NUMBER
from math_utils import distance 
from droneshow_serializer import DronePointFlaggable


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
       

    @classmethod
    def CreateNewDrone(cls):
        pass 

    @classmethod
    def DistCalc(cls, flag: list[DronePointFlaggable] , avail: list[DronePointFlaggable],  time_index: float) -> float: 
        
        for i in range(1, time_index):
            if avail[time_index-i] != None:
                total_distance = distance(flag[i].getLocation(), avail[time_index-i].getLocation())
                diff = i * MAX_ALLOWED_DISTANCE
                return total_distance - diff
        

    @classmethod
    def DistanceMatrix(cls, drones: list[list[DronePointFlaggable]], firstFlag: int, time_index: float) -> list[list[float]]:
        
        flaggedDrones:list[int]  = [firstFlag]

        M_distance:list[list[float]] = []

        for i in flaggedDrones:
            for j in range(len(drones)):
                    distance = DroneshowModifier.DistCalc(drones[i], drones[j], time_index)
                    if distance <= MAX_ALLOWED_DISTANCE:
                        flaggedDrones.append(j)
                        M_distance[i][j]  = distance
                    else:
                        if drones[j].flag:
                            flaggedDrones.append(j)                            
                        M_distance[i][j]  = BIG_NUMBER


        # --------------------------------------------- vi er kommet hertil
        for drone in range(len(M_distance)):
            for flag in range(len(distance[0])):
                if M_distance[drone][flag] != BIG_NUMBER:
                    break
                
            M_distance = np.array(M_distance)
            M_distance = np.delete(M_distance, drone, 0)
            
        matDiff = len(M_distance) - len(M_distance[0])

        if matDiff < 0: #more drones than flags
            for i in range(len(M_distance)+1,len(M_distance) + matDiff ):
                for j in range(0,len(M_distance[0]) ):
                    M_distance[i][j] = BIG_NUMBER

        elif matDiff > 0: #more flags than drones
            for i in range(len(M_distance[0])+1,len(M_distance[0]) + matDiff ):
                DroneshowModifier.CreateNewDrone()
                for j in range(0,len(M_distance)):
                    M_distance[i][j] = 0

        return M_distance
        

    @classmethod # er distance_matrix en liste eller et array? (se linje 75)
    def HungarianALG(cls, distance_matrix: list[list[float]]) -> tuple[list[tuple[int, int]], list[float]]: 
        
        distance_matrix = np.array(distance_matrix)

        minColVal = np.min(distance_matrix, axis=1) # axis 1 er column
        minRowVal = np.min(distance_matrix, axis=0) # axis 0 er row
        #column reduction
        for column in range(len(distance_matrix)):
            min = minColVal[column]
            for i in range(len(column)):
                distance_matrix[column][i] -= min

        #row reduction T-T
        for row in range(len(distance_matrix)):
            min = minRowVal[row]
            for i in range(len(row)):
                distance_matrix[i][row] -= min
        
        # track 0's .... to be continued 
            
        
        
        

    @classmethod
    def CheckSolution(cls, sol,  minColVal, flags):
        for i in range(len(flags)):
            if minColVal[i] == BIG_NUMBER:
                sol[i] = (DroneshowModifier.CreateNewDrone(),flags[i])

        return sol
    
    @classmethod
    def UpdateMatrix(cls, M_Drones, checkedSol , time_index):
        copies = []
        
        for i in checkedSol:
            copies.append(copy.copy(i[1])) # copy all flag-values
        
            insert_drone = copies[i]

            for j in range(time_index, len(i[0])):
                i[0][j] = insert_drone[j]
                i[1][j] = None
        
        return M_Drones
    
    @classmethod
    def FillMatrix(M_Drones: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        for drone in range(len(M_Drones)):
            timeInterval = 1
            for time in range(len(M_Drones[0])):
                if M_Drones[drone][time] == None:
                    if timeInterval == 1:
                        start = M_Drones[drone][time-1]
                        t = 0
                        while t < len(M_Drones[0]): 
                            timeInterval = t-time-1
                            if M_Drones[drone][t] is not None:
                                target = M_Drones[drone][t]
                                rx = (start.x - target.x)/timeInterval
                                ry = (start.y - target.y)/timeInterval
                                rz = (start.z - target.z)/timeInterval
                                t = len(M_Drones[0])
                            t += 1
                    if t == len(M_Drones[0]+1):
                        M_Drones[drone][time].x = M_Drones[drone-1][time-1].x + rx
                        M_Drones[drone][time].y = M_Drones[drone-1][time-1].y + ry
                        M_Drones[drone][time].z = M_Drones[drone-1][time-1].z + rz
                        timeInterval -= 1
                    elif t == len(M_Drones[0]):
                        M_Drones[drone][time].x = M_Drones[drone-1][time-1].x
                        M_Drones[drone][time].y = M_Drones[drone-1][time-1].y
                        M_Drones[drone][time].z = M_Drones[drone-1][time-1].z
                    else:
                        print("not good lmao")
        return M_Drones