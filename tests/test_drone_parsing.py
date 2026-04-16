import pytest
from pathlib import Path

from blinking_drone_alg.drone_parser import DroneParser

class TestDroneParsing:
    def test_load_csv_without_error(self):
        fixture_path = (Path(__file__).parent / 'fixtures' / 'demo_single_drone.csv').as_posix()

        # Throws error if any issue
        DroneParser.load_csv(fixture_path)

    def test_read_rows_correctly(self):
        """
        Test a couple of the rows against hard-coded values.
        """
        fixture_path = (Path(__file__).parent / 'fixtures' / 'demo_single_drone.csv').as_posix()

        drone_series = DroneParser.load_csv(fixture_path)

        #assert drone_series[0] == ???
        pass

    def test_rows_correct_order(self):
        """
        Validate that the rows are sequential in the list.
        """
        pass
