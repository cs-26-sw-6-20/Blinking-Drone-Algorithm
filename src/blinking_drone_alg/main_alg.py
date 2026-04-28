from blinking_drone_alg.droneshow import DronePointFlaggable, DroneshowModifier
from blinking_drone_alg.constants import MAX_ALLOWED_DISTANCE, BIG_NUMBER, TOP_SPEED, TIME_INTERVAL

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
   # need to supply with M_drones: list[list[DronePointFlaggable]]
    m = [
    [DronePointFlaggable(time_ms=000, x=1.0, y=1.0, z=0.0, r=0, g=0, b=0, flag=False),
    DronePointFlaggable(time_ms=250, x=3.0, y=1.5, z=0.0, r=0, g=0, b=0, flag=True),
    DronePointFlaggable(time_ms=500, x=4.0, y=0.5, z=0.0, r=0, g=0, b=0, flag=True),
    DronePointFlaggable(time_ms=750, x=5.0, y=0.5, z=0.0, r=0, g=0, b=0, flag=False)],

    [DronePointFlaggable(time_ms=000, x=2.0, y=1.5, z=0.0, r=1, g=1, b=1, flag=False),
    DronePointFlaggable(time_ms=250, x=1.0, y=2.0, z=0.0, r=1, g=1, b=1, flag=False),
    DronePointFlaggable(time_ms=500, x=3.0, y=2.0, z=0.0, r=1, g=1, b=1, flag=True),
    DronePointFlaggable(time_ms=750, x=4.0, y=2.5, z=0.0, r=1, g=1, b=1, flag=False)],

    [DronePointFlaggable(time_ms=000, x=3.0, y=2.5, z=0.0, r=2, g=2, b=2, flag=False),
    DronePointFlaggable(time_ms=250, x=3.0, y=3.5, z=0.0, r=2, g=2, b=2, flag=False),
    DronePointFlaggable(time_ms=500, x=2.5, y=4.5, z=0.0, r=2, g=2, b=2, flag=False),
    DronePointFlaggable(time_ms=750, x=3.0, y=5.0, z=0.0, r=2, g=2, b=2, flag=False)],

    [DronePointFlaggable(time_ms=000, x=4.5, y=1.0, z=0.0, r=3, g=3, b=3, flag=False),
    DronePointFlaggable(time_ms=250, x=5.0, y=3.0, z=0.0, r=3, g=3, b=3, flag=True),
    DronePointFlaggable(time_ms=500, x=5.0, y=4.0, z=0.0, r=3, g=3, b=3, flag=False),
    DronePointFlaggable(time_ms=750, x=5.5, y=5.0, z=0.0, r=3, g=3, b=3, flag=False)],
]
    
    DroneshowModifier.not_main(m)



def create_time_x_drone_matrix_with_flag():
    pass

if __name__ == '__main__':
    main()
