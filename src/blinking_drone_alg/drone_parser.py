import csv
from dataclasses import dataclass


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

