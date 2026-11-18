"""UART wire protocol shared conceptually with Core/Src/wire_protocol.c."""
from dataclasses import dataclass
from typing import List, Tuple

MAX_BODY = 160
MAX_FRAME = MAX_BODY + 7  # $, *, four CRC digits, LF
FLAG_SAMPLE = 1
FLAG_HEADING = 2
FLAG_SATURATED = 4
FLAG_OLED = 8
FLAG_CALIBRATED = 16
FLAG_CALIBRATING = 32


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xffff, no reflection/xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode(*fields: object) -> bytes:
    parts = [str(field) for field in fields]
    if any(not part or any(c in part for c in ",*$\r\n") for part in parts):
        raise ValueError("Недопустимое поле протокола")
    body = ",".join(parts).encode("ascii")
    if not body or len(body) > MAX_BODY:
        raise ValueError("Слишком длинное сообщение")
    return b"$" + body + f"*{crc16(body):04X}\n".encode("ascii")


def decode(frame: bytes) -> Tuple[str, ...]:
    frame = frame.removesuffix(b"\n").removesuffix(b"\r")
    if not frame.startswith(b"$") or frame.count(b"*") != 1:
        raise ValueError("Неверный формат кадра")
    body, checksum = frame[1:].split(b"*")
    if not body or len(body) > MAX_BODY or len(checksum) != 4:
        raise ValueError("Неверная длина кадра")
    if any(c not in b"0123456789abcdefABCDEF" for c in checksum):
        raise ValueError("Неверный формат CRC")
    if crc16(body) != int(checksum, 16):
        raise ValueError("Ошибка CRC")
    parts = tuple(body.decode("ascii").split(","))
    if len(parts) < 2 or any(not part for part in parts):
        raise ValueError("Пустое поле кадра")
    return parts


class StreamDecoder:
    """Handles split/combined frames and resynchronizes at the next '$'."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.errors: List[str] = []

    def feed(self, data: bytes) -> List[Tuple[str, ...]]:
        result = []
        for byte in data:
            if byte == ord("$"):
                self.buffer.clear()
                self.buffer.append(byte)
            elif not self.buffer:
                continue
            elif byte == ord("\n"):
                try:
                    result.append(decode(bytes(self.buffer)))
                except (ValueError, UnicodeError) as error:
                    self.errors.append(str(error))
                self.buffer.clear()
            elif len(self.buffer) >= MAX_FRAME or byte > 127 or byte == 0:
                self.errors.append("Повреждённый или слишком длинный кадр")
                self.buffer.clear()
            else:
                self.buffer.append(byte)
        return result


@dataclass(frozen=True)
class Telemetry:
    tick_ms: int
    field: Tuple[int, int, int]  # 0.01 microtesla
    heading: int  # 0.01 degree clockwise from screen up, -1 if undefined
    gain: Tuple[int, int, int]
    flags: int

    @classmethod
    def from_fields(cls, fields: Tuple[str, ...]) -> "Telemetry":
        if len(fields) != 11 or fields[:2] != ("0", "DATA"):
            raise ValueError("Неверный пакет DATA")
        values = tuple(int(part) for part in fields[2:])
        tick, x, y, z, heading, gx, gy, gz, flags = values
        if not 0 <= tick <= 0xFFFFFFFF or not 0 <= flags <= 63:
            raise ValueError("Неверные счётчик или флаги")
        if not all(-2147483648 <= value <= 2147483647 for value in (x, y, z)):
            raise ValueError("Значение поля вне диапазона")
        if not all(0 <= value <= 200 for value in (gx, gy, gz)):
            raise ValueError("Коэффициент вне диапазона")
        if heading != -1 and not 0 <= heading < 36000:
            raise ValueError("Угол вне диапазона")
        if bool(flags & FLAG_HEADING) != (heading >= 0):
            raise ValueError("Противоречивое направление")
        if flags & FLAG_HEADING and (not flags & FLAG_SAMPLE or flags & FLAG_SATURATED):
            raise ValueError("Недостоверное направление")
        if flags & FLAG_HEADING and flags & FLAG_CALIBRATING:
            raise ValueError("Направление во время калибровки")
        return cls(tick, (x, y, z), heading, (gx, gy, gz), flags)


@dataclass(frozen=True)
class CalibrationStatus:
    stage: int
    samples: int
    coverage: int
    ready: bool

    @classmethod
    def parse(cls, fields) -> "CalibrationStatus":
        if len(fields) != 4:
            raise ValueError("Неверный статус калибровки")
        stage, samples, coverage, ready = map(int, fields)
        if (not 0 <= stage <= 4 or not 0 <= samples <= 0xFFFFFFFF or
                not 0 <= coverage <= 16 or ready not in (0, 1) or
                (stage == 0 and ready) or (stage == 4 and not ready)):
            raise ValueError("Неверные параметры калибровки")
        return cls(stage, samples, coverage, bool(ready))


@dataclass(frozen=True)
class CalibrationProfile:
    # bias in 0.001 sensor counts, symmetric matrix in 0.000001 units,
    # reference in 0.01 degree, handedness +1/-1. Same validation as firmware.
    values: Tuple[int, ...]

    def __post_init__(self):
        if len(self.values) != 7 or any(type(v) is not int for v in self.values):
            raise ValueError("Неверный профиль калибровки")
        bx, by, a, b, c, north, direction = self.values
        determinant = (a * c - b * b) / 1e12
        if (abs(bx) > 32768000 or abs(by) > 32768000 or
                not 200000 <= a <= 5000000 or not 200000 <= c <= 5000000 or
                not 0.8 <= determinant <= 1.2 or not 0 <= north < 36000 or
                direction not in (-1, 1)):
            raise ValueError("Профиль калибровки вне диапазона")
