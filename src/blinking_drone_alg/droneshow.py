import logging
from dataclasses import dataclass
from turtle import distance
import numpy as np

from blinking_drone_alg.droneshow_serializer import DronePoint
from blinking_drone_alg.main_alg import MAX_ALLOWED_DISTANCE, BIG_NUMBER


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
    def DistCalc(cls, Flag, Avail, time_index):
        for i in range(0,time_index +1):
            if Avail[time_index+1-i] != None:
                distance = np.sqrt((Flag.x - Avail[time_index+1-i].x)**2 + (Flag.y - Avail[time_index+1-i].y)**2 + (Flag.z - Avail[time_index+1-i].z)**2)
                total_distance = 
                diff = i *

       

    @classmethod
    def CreateNewDrone(cls):
        pass 


    @classmethod
    def DistanceMatrix(cls, drones, firstFlag, time_index):
        
        flaggedDrones  = []
        flaggedDrones.append(firstFlag)

        M_distance = []

        for i in flaggedDrones:
            for j in drones:
                    distance = DroneshowModifier.DistCalc(i, j, time_index)
                    if distance <= MAX_ALLOWED_DISTANCE:
                        flaggedDrones.append(drones[j])
                        M_distance[i][j]  = distance
                    else:
                        if drones[j].flag:
                            flaggedDrones.append(drones[j])                            
                        M_distance[i][j]  = BIG_NUMBER
                        
        for drone in M_distance:
            for flag in M_distance:
                if M_distance[drone][flag] != BIG_NUMBER:
                    break
                
            M_distance = np.array(M_distance)
            M_distance = np.delete(M_distance, drone, 0)
            
        matDiff = len(M_distance) - len(M_distance[0])

        if matDiff < 0: #more drones than flags
            for i in range(len(M_distance)+1,len(M_distance) + matDiff +1):
                for j in range(0,len(M_distance[0])+1 ):
                    M_distance[i][j] = BIG_NUMBER

        elif matDiff > 0: #more flags than drones
            for i in range(len(M_distance[0])+1,len(M_distance[0]) + matDiff +1):
                DroneshowModifier.CreateNewDrone()
                for j in range(0,len(M_distance)):
                    M_distance[i][j] = 0

        return M_distance
        

    @classmethod
    def HungarianALG(cls, distance_matrix):
        pass

    @classmethod
    def CheckSolution(cls, sol):
        pass