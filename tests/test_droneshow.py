import pytest
from pathlib import Path

from blinking_drone_alg.drone_parser import DroneParser


@pytest.fixture(params=[
    "demo-droneshow-5seconds.zip",
])
def droneshow_data(request):
    fixture_path = (
        Path(__file__).parent / "fixtures" / request.param
    ).as_posix()

    return DroneParser.load_droneshow_from_archive(fixture_path)


class TestDroneshow:
    def test_load_droneshow_zip(self, droneshow_data):
        # If fixture loads, test passes
        assert droneshow_data is not None
