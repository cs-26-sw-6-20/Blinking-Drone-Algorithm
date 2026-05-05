from blinking_drone_alg.droneshow import DronePointFlaggable, DroneshowModifier
from blinking_drone_alg.constants import MAX_ALLOWED_DISTANCE, BIG_NUMBER, TOP_SPEED, TIME_INTERVAL
from blinking_drone_alg.droneshow_serializer import DroneshowParser

####################
# Megamind-gruppen #
####################

"""
[
    [(x, y, color, flag), ...], # drone 1
    [(x, y, color, flag), ...], # drone 2
]
"""

def main():
   
    archive_path = "C:/Users/Alija/Blinking-Drone-Algorithm/dronesjov.zip"
    
    max_speed = 0
    
    droneshowPoints = DroneshowParser.load_droneshow_from_archive(archive_path)
    flagged_drones = DroneshowModifier.flag_droneshow(droneshowPoints,max_speed)  
    print(flagged_drones)
    MDrone = DroneshowModifier.not_main(flagged_drones)
    
    
    print(f"Original number of drones: {len(droneshowPoints)}")
    print(f"Final number of drones: {len(MDrone)}")
   


def create_time_x_drone_matrix_with_flag():
    pass

if __name__ == '__main__':
    main()
