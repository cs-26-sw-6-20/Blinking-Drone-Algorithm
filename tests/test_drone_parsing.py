import pytest
from pathlib import Path

from blinking_drone_alg.drone_parser import DroneParser

class TestDroneParsing:
    def test_load_csv_without_error(self):
        fixture_path = (Path(__file__).parent / 'fixtures' / 'demo_single_drone.csv').as_posix()

        # Throws error if any issue
        DroneParser.load_csv(fixture_path)
