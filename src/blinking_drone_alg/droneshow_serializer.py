import csv
from dataclasses import dataclass
import zipfile
import tempfile
from pathlib import Path
import logging

from mixins import WithMixin

@dataclass
class DronePoint(WithMixin):
    time_ms: int
    x: float
    y: float
    z: float
    r: int
    g: int
    b: int

    def get_location(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def get_time_sec(self) -> float:
        return self.time_ms / 1000.0


class DroneshowParser:
    logger = logging.getLogger(__name__)

    @classmethod
    def load_csv(cls, file_path: Path | str) -> list[DronePoint]:
        drone_series = []
        with open(file_path, newline="") as f:
            reader = csv.DictReader(f)
            cls.logger.info("loaded csv", file_path)

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

    @classmethod
    def load_droneshow_from_archive(cls, zip_path: Path | str) -> list[list[DronePoint]]:
        extract_to = tempfile.mkdtemp(prefix="droneshow_")
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(extract_to)
        cls.logger.info("unzipped to", extract_to)

        drone_csv_list = list(Path(extract_to).rglob("*.csv"))
        return [DroneshowParser.load_csv(csv_path) for csv_path in drone_csv_list]


class DroneshowGenerator:
    logger = logging.getLogger(__name__)

    @classmethod
    def save_csv(cls, drone: DronePoint, file_path: Path | str) -> None:
        pass

    @classmethod
    def save_droneshow_to_archive(cls, droneshow: list[list[DronePoint]], zip_path: Path | str) -> None:
        pass
