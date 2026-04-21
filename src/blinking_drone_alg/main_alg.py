####################
# Megamind-gruppen #
####################
DRONE_COUNTER = 0
TOP_SPEED = 5 #m/s
TIME_INTERVAL = 0.250
MAX_ALLOWED_DISTANCE =  TOP_SPEED*TIME_INTERVAL
BIG_NUMBER = 1000000 #big number go brrr

"""
[
    [(x, y, color, flag), ...], # drone 1
    [(x, y, color, flag), ...], # drone 2
]
"""

def main():
   # Create time x Drone Matrix with flags
   
    for i in time:
        firstFlag = NULL
        for j in drone:
            if drone[j].time[i].flag:
                firstFlag = drone[j] 
                break
        if (firstFlag == NULL):
            continue
        sol = HungarianALG(DistanceMatrix(drone, firstFlag, i))
        checkedSol = CheckSolution( sol )
        MDrones = UpdateMatrix(MDrones, checkedSol, i)
    MDrones = FillMatrix(MDrones)
    # Make MDrones into CSV files
    # Return CSV files and global drone counter value


def create_time_x_drone_matrix_with_flag():
    pass

if __name__ == '__main__':
    main()
