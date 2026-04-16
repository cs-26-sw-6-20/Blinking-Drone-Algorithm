import csv
from dataclasses import dataclass
import zipfile
import tempfile


@dataclass
class DronePoint:
    time_ms: int
    x: float
    y: float
    z: float
    r: int
    g: int
    b: int

class DroneParser:
    @staticmethod
    def load_csv(file_path: str) -> list[DronePoint]:
        drone_series = []
        with open(file_path, newline="") as f:
            reader = csv.DictReader(f)

            for row in reader:
                drone_series.append(
                    DronePoint(
                        int(row["Time [msec]"]),
                        float(row["x [m]"]),
                        float(row["y [m]"]),
                        float(row["z [m]"]),
                        int(row["Red"]),
                        int(row["Green"]),
                        int(row["Blue"]),
                    )
                )

        return drone_series

    @staticmethod
    def load_droneshow_from_archive(zip_path: str) -> list[list[DronePoint]]:
        extract_to = tempfile.mkdtemp(prefix="droneshow_")
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(extract_to)
        print("unzipped to", extract_to)

        list_of_files = [] # TODO
        return [DroneParser.load_csv(csv_path) for csv_path in list_of_files]