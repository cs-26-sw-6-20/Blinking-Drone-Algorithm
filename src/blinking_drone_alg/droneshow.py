import logging
from dataclasses import dataclass
from shutil import copy
from turtle import distance
import numpy as np
from scipy.optimize import linear_sum_assignment
import copy

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
    def CreateNewDrone(cls, drones: list[list[DronePointFlaggable]]) -> int:
        newDroneIndex = len(drones)   
        
        drones.append([DronePointFlaggable(time_ms=i*250, x=0.0, y=0.0, z=0.0, r=0, g=0, b=0, flag=False) for i in range(len(drones[0]))])
        # drones.append([])
        for i in range(len(drones[0])):
            drones[newDroneIndex][i] = None

        return newDroneIndex # -------------------------------------------------------------------------------------------------------------------------- nye droner skal have None værdier

        

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
            for i in range(len(M_distance)+1,len(M_distance[0])):
                np.append(M_distance, [BIG_NUMBER for _ in range(len(M_distance[0]))])

        elif matDiff > 0: #more flags than drones
            for i in range(len(M_distance[0]),len(M_distance)): # ---------------------------------------------------- fix squareness of matrix
                availableDrones.append(DroneshowModifier.CreateNewDrone(drones))
                newDrone = [[0] for _ in range(len(M_distance)) ]
                M_distance =  np.append(M_distance, newDrone, 1)
        
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
    def FillMatrix(cls, M_Drones: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        
        getDeleted = []

        for i in range(len(M_Drones)): # drones
            for j in range(len(M_Drones[0])): # time
                if (M_Drones[i][j]) is None:
                    for t in range(j+1,len(M_Drones[0])):
                        if (M_Drones[i][t]) is not None:
                            if j == 0:
                                case = 1
                                break
                            else:
                                case = 2
                                break
                        else:
                            if j == 0:
                                case = 3
                            else:
                                case = 4
                    if case == 1:
                        for k in range(j, t):
                            M_Drones[i][k] = DronePointFlaggable(time_ms=250*k, x=M_Drones[i][t].x, y=M_Drones[i][t].y, z=M_Drones[i][t].z, r=0, g=0, b=0, flag=False)
                        j = t
                        case = 0
                    elif case == 2:
                        rx = (M_Drones[i][j-1].x - M_Drones[i][t].x)/(t-j+1)
                        ry = (M_Drones[i][j-1].y - M_Drones[i][t].y)/(t-j+1)
                        rz = (M_Drones[i][j-1].z - M_Drones[i][t].z)/(t-j+1)
                        for k in range(j, t):
                            M_Drones[i][k] = DronePointFlaggable(time_ms=250*k, x=M_Drones[i][k-1].x-rx, y=M_Drones[i][k-1].y-ry, z=M_Drones[i][k-1].z-rz, r=0, g=0, b=0, flag=False)
                        j = t
                        case = 0
                    elif case == 3:
                        getDeleted.append(i)
                        case = 0
                    elif case == 4:
                        for k in range(j, len(M_Drones[0])):
                            M_Drones[i][k] = DronePointFlaggable(time_ms=250*k, x=M_Drones[i][j-1].x, y=M_Drones[i][j-1].y, z=M_Drones[i][j-1].z, r=0, g=0, b=0, flag=False)
                        case = 0
                    else:
                        case = 0        

        M_Drones = np.delete(M_Drones, getDeleted, 0)   
        return M_Drones            
















        """ 
        getDeleted = []
        
        for i in range(len(M_Drones)): # drone
            timeInterval = 1
            for j in range(len(M_Drones[0])): # time
                if M_Drones[i][j] == None :
                    if timeInterval == 1:
                        t = j+1
                        while t < len(M_Drones[0]): 
                            timeInterval = t-j-1
                            if M_Drones[i][t] is not None:
                                target = M_Drones[i][t]
                                if j == 0:
                                    start = target
                                    M_Drones[i][j] = DronePointFlaggable(time_ms=0, x=M_Drones[i][t].x, y=M_Drones[i][t].y, z=M_Drones[i][t].z, r=0, g=0, b=0, flag=False)
                                else:
                                    start = M_Drones[i][j-1]
                                 
                                rx = (start.x - target.x)/(timeInterval+2)
                                ry = (start.y - target.y)/(timeInterval+2)
                                rz = (start.z - target.z)/(timeInterval+2)
                                
                                t = len(M_Drones[0])
                            t += 1
                    if t == len(M_Drones[0])+1:
                        M_Drones[i][j] = DronePointFlaggable(time_ms=j*250, x=M_Drones[i][j-1].x-rx, y=M_Drones[i][j-1].y-ry, z=M_Drones[i][j-1].z-rz, r=0, g=0, b=0, flag=False)
                        
                        timeInterval -= 1
                    
                    elif t == len(M_Drones[0]):
                        if j != 0:
                            M_Drones[i][j] = DronePointFlaggable(time_ms=j*250, x=M_Drones[i][j-1].x, y=M_Drones[i][j-1].y, z=M_Drones[i][j-1].z, r=0, g=0, b=0, flag=False)
                        else:
                            getDeleted.append(i)
                            print("Deleting drone", i)
                    else:
                        print("Fill Matrix calculations failed") # vores version af en error message
                      
            M_Drones = np.delete(M_Drones, getDeleted, 0)            
        return M_Drones """


   
    @classmethod
    def not_main(cls, M_Drones: list[list[DronePointFlaggable]]) -> list[list[DronePointFlaggable]]:
        # Create time x Drone Matrix with flags

        for i in range(len(M_Drones[0])):
            flaggedDrones = []
            for j in range(len(M_Drones)):
                if M_Drones[j][i] is not None:
                    if M_Drones[j][i].flag:
                        flaggedDrones.append(j)
            if (flaggedDrones == []):
                continue
            
            M_DistanceAndAvailDrones = DroneshowModifier.DistanceMatrix(M_Drones, flaggedDrones, i)
            sol = DroneshowModifier.HungarianALG(M_DistanceAndAvailDrones[0])
            checkedSol = DroneshowModifier.CheckSolution(sol, M_DistanceAndAvailDrones[0], M_DistanceAndAvailDrones[1], flaggedDrones) 
            M_Drones = DroneshowModifier.UpdateMatrix(M_Drones, checkedSol, i)
            
       
        M_Drones = DroneshowModifier.FillMatrix(M_Drones)
        return M_Drones
        # Make MDrones into CSV files
        # Count amount of drones
        # Return CSV files and global drone counter value