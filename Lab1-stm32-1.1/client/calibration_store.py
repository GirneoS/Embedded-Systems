"""Board-specific calibration, saved on PC rather than wearing MCU flash."""
import json
import re
from datetime import datetime
from pathlib import Path

from protocol import CalibrationProfile


class CalibrationStore:
    def __init__(self, directory: Path):
        self.directory = directory

    def path(self, uid: str) -> Path:
        if not re.fullmatch(r"[0-9A-Fa-f]{24}", uid):
            raise ValueError("Неверный UID стенда")
        return self.directory / (uid.upper() + ".json")

    def load(self, uid: str, model: str):
        path = self.path(uid)
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("Повреждён файл профиля")
        if data.get("version") != 1 or data.get("uid") != uid.upper() or data.get("model") != model:
            raise ValueError("Профиль относится к другому стенду или датчику")
        return CalibrationProfile(tuple(data["values"]))

    def save(self, uid: str, model: str, profile: CalibrationProfile) -> Path:
        path = self.path(uid)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps({"version": 1, "uid": uid.upper(), "model": model,
                                        "saved_at": datetime.now().astimezone().isoformat(),
                                        "values": profile.values}, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        return path

    def remove(self, uid: str):
        self.path(uid).unlink(missing_ok=True)
