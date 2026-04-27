import pytest
from math import sqrt
from blinking_drone_alg.droneshow import  DronePointFlaggable, DroneshowModifier



@pytest.fixture
def flag_drone():
    return [
    DronePointFlaggable(time_ms=0.0, x=1.0, y=2.0, z=3.0, r=0, g=0, b=0, flag=False),
    DronePointFlaggable(time_ms=0.25, x=100.0, y=2.0, z=3.0, r=0, g=0, b=0, flag=False),
    DronePointFlaggable(time_ms=0.5, x=1.0, y=2.0, z=3.0, r=0, g=0, b=0, flag=True)]


pytest.fixture
def available_drone_close():
    return [
        DronePointFlaggable(time_ms=0.0, x=1.0, y=2.0, z=3.0, r=0, g=0, b=0, flag=False),
        DronePointFlaggable(time_ms=0.25, x=4.0, y=5.0, z=6.0, r=0, g=0, b=0, flag=False),
        DronePointFlaggable(time_ms=0.5, x=7.0, y=8.0, z=9.0, r=0, g=0, b=0, flag=False)]

pytest.fixture
def avail_drone_none():
    return [
        DronePointFlaggable(time_ms=0.0, x=4.0, y=5.0, z=6.0, r=0, g=0, b=0, flag=False),
        DronePointFlaggable(None),
        DronePointFlaggable(None)]

pytest.fixture
def avail_drone_all_none():
    return  [
        DronePointFlaggable(None),
        DronePointFlaggable(None),
        DronePointFlaggable(None)]


class TestDistCalc:
    def test_dist_calc(self):
       result =  DronePointFlaggable.distCalc(flag_drone(), available_drone_close(), 2)
       assert result == sqrt(27) 

  
        
