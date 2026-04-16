import os
import zipfile
import tempfile
from pathlib import Path

base = Path(__file__).resolve().parent
zip_path = base / "demo-droneshow-5seconds.zip"

extract_to = tempfile.mkdtemp(prefix="droneshow_")
with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    zip_ref.extractall(extract_to)
print("unzipped to", extract_to)