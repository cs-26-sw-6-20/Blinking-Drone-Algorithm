import pytest
from pathlib import Path

from blinking_drone_alg.drone_parser import DroneParser

class TestDroneshowArchiveLoad:
    def test_load_droneshow_zip(self):
        fixture_path = (Path(__file__).parent / "fixtures" / "demo-droneshow-5seconds.zip").as_posix()
        # Throws error if there's any issue
        DroneParser.load_droneshow_from_archive(fixture_path)

    @pytest.mark.parametrize(
        "filename, expected_drones",
        [
            ("demo-droneshow-5seconds.zip", 64),
        ],
    )
    def test_amount_of_drones_correct(self, filename, expected_drones):
        path = (Path(__file__).parent / "fixtures" / filename).as_posix()

        droneshow = DroneParser.load_droneshow_from_archive(path)

        assert len(droneshow) == expected_drones
