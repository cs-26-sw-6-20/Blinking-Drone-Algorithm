#import pytest
from blinking_drone_alg.droneshow import DroneshowModifier, DronePointFlaggable

input = [
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


output = [
    [DronePointFlaggable(time_ms=000, x=1.0, y=1.0, z=0.0, r=0, g=0, b=0, flag=False),
    DronePointFlaggable(time_ms=250, x=1.0, y=2.0, z=0.0, r=1, g=1, b=1, flag=False),
    DronePointFlaggable(time_ms=500, x=1.0, y=2.0, z=0.0, r=0, g=0, b=0, flag=False),
    DronePointFlaggable(time_ms=750, x=1.0, y=2.0, z=0.0, r=0, g=0, b=0, flag=False)],

    [DronePointFlaggable(time_ms=000, x=2.0, y=1.5, z=0.0, r=1, g=1, b=1, flag=False),
    DronePointFlaggable(time_ms=250, x=3.0, y=1.5, z=0.0, r=0, g=0, b=0, flag=True),
    DronePointFlaggable(time_ms=500, x=3.0, y=2.0, z=0.0, r=1, g=1, b=1, flag=True),
    DronePointFlaggable(time_ms=750, x=4.0, y=2.5, z=0.0, r=1, g=1, b=1, flag=False)],

    [DronePointFlaggable(time_ms=000, x=3.0, y=2.5, z=0.0, r=2, g=2, b=2, flag=False),
    DronePointFlaggable(time_ms=250, x=3.0, y=3.5, z=0.0, r=2, g=2, b=2, flag=False),
    DronePointFlaggable(time_ms=500, x=2.5, y=4.5, z=0.0, r=2, g=2, b=2, flag=False),
    DronePointFlaggable(time_ms=750, x=3.0, y=5.0, z=0.0, r=2, g=2, b=2, flag=False)],

    [DronePointFlaggable(time_ms=000, x=4.5, y=1.0, z=0.0, r=3, g=3, b=3, flag=False),
    DronePointFlaggable(time_ms=250, x=4.25, y=0.75, z=0.0, r=0, g=0, b=0, flag=True),
    DronePointFlaggable(time_ms=500, x=4.0, y=0.5, z=0.0, r=0, g=0, b=0, flag=True),
    DronePointFlaggable(time_ms=750, x=5.0, y=0.5, z=0.0, r=0, g=0, b=0, flag=False)],
    
    [DronePointFlaggable(time_ms=000, x=5.0, y=3.0, z=0.0, r=0, g=0, b=0, flag=False),
    DronePointFlaggable(time_ms=250, x=5.0, y=3.0, z=0.0, r=3, g=3, b=3, flag=True),
    DronePointFlaggable(time_ms=500, x=5.0, y=4.0, z=0.0, r=3, g=3, b=3, flag=False),
    DronePointFlaggable(time_ms=750, x=5.5, y=5.0, z=0.0, r=3, g=3, b=3, flag=False)]
]

class TestMain:
    def test_not_main(self):
        result = DroneshowModifier.not_main(input)

        assert result is not None

        assert len(result) == len(output)
        