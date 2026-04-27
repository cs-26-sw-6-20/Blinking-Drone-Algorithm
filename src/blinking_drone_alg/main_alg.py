
from blinking_drone_alg.droneshow import DroneshowModifier


####################
# Megamind-gruppen #
####################
TOP_SPEED = 5 #m/s
TIME_INTERVAL = 0.250
MAX_ALLOWED_DISTANCE =  TOP_SPEED * TIME_INTERVAL
BIG_NUMBER = 1000000 #big number go brrR

"""
[
    [(x, y, color, flag), ...], # drone 1
    [(x, y, color, flag), ...], # drone 2
]
"""

def main():
   
   DroneshowModifier.not_main()



def create_time_x_drone_matrix_with_flag():
    pass

if __name__ == '__main__':
    main()
