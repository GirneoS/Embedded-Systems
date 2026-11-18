"""In-process stand emulator for --demo and serial integration tests."""
import math
import time
from protocol import CalibrationProfile, StreamDecoder, encode


class DemoSerial:
    def __init__(self, *args, **kwargs) -> None:
        self.buffer = bytearray()
        self.decoder = StreamDecoder()
        self.gains = (100, 100, 100)
        self.started = time.monotonic()
        self.next_sample = self.started
        self.timeout = kwargs.get("timeout", 0.02)
        self.stage = 0
        self.ready = False
        self.collect_started = self.started
        self.samples = 0
        self.profile = (0, 0, 1000000, 0, 1000000, 0, 1)
        self.candidate = self.profile

    def status(self, seq=0):
        prefix = (seq, "OK", "CAL") if int(seq) else (0, "CALSTAT")
        return encode(*prefix, self.stage, self.samples, 16 if self.stage >= 2 else 0,
                      int(self.ready))

    def calibration(self, seq, fields):
        op = fields[2]
        if op == "LOAD" and len(fields) == 10:
            try:
                profile = CalibrationProfile(tuple(map(int, fields[3:])))
            except ValueError:
                return encode(seq, "ERR", "CAL_PROFILE")
            self.profile = profile.values
            self.ready, self.stage = True, 4
        elif op == "READ":
            return encode(seq, "OK", "CALDATA", *self.profile) if self.ready else encode(seq, "ERR", "CAL_STATE")
        elif op == "START":
            if self.stage not in (1, 2, 3):
                self.stage, self.samples, self.collect_started = 1, 0, time.monotonic()
                self.candidate = (0, 0, 1000000, 0, 1000000, 0, 1)
        elif op == "FINISH":
            if self.stage == 1 and self.samples >= 100:
                self.stage = 2
            elif self.stage not in (2, 3):
                return encode(seq, "ERR", "CAL_SAMPLES")
        elif op == "NORTH":
            if self.stage == 2:
                north = round(math.degrees((time.monotonic() - self.started) * .35) * 100) % 36000
                self.candidate = (*self.candidate[:5], north, -1)
                self.stage = 3
            elif self.stage != 3:
                return encode(seq, "ERR", "CAL_STATE")
        elif op == "TURN":
            if self.stage in (3, 4):
                if self.stage == 3:
                    self.profile = self.candidate
                self.ready, self.stage = True, 4
            else:
                return encode(seq, "ERR", "CAL_STATE")
        elif op == "CANCEL":
            self.stage = 4 if self.ready else 0
        elif op == "RESET":
            self.ready, self.stage, self.samples = False, 0, 0
        elif op != "GET":
            return encode(seq, "ERR", "BAD_COMMAND")
        return self.status(seq)

    @property
    def in_waiting(self) -> int:
        return len(self.buffer)

    def write(self, data: bytes) -> int:
        for fields in self.decoder.feed(data):
            seq, command = fields[:2]
            if command == "PING":
                self.buffer.extend(encode(seq, "PONG", 2, "DEMO", 29, "000000000000000000000001"))
            elif command == "CAL" and len(fields) >= 3:
                self.buffer.extend(self.calibration(seq, fields))
            elif command == "GET":
                self.buffer.extend(encode(seq, "OK", "GET", *self.gains))
            elif command == "SET" and len(fields) == 5:
                try:
                    gains = tuple(int(value) for value in fields[2:])
                except ValueError:
                    gains = (-1, -1, -1)
                if all(0 <= value <= 200 for value in gains):
                    self.gains = gains
                    self.buffer.extend(encode(seq, "OK", "SET", *self.gains))
                else:
                    self.buffer.extend(encode(seq, "ERR", "RANGE"))
        return len(data)

    def read(self, count: int) -> bytes:
        now = time.monotonic()
        if now >= self.next_sample:
            seconds = now - self.started
            raw = (30 * math.sin(seconds * 0.35), 30 * math.cos(seconds * 0.35), 42)
            field = tuple(round(value * gain) for value, gain in zip(raw, self.gains))
            heading = (round(math.degrees(math.atan2(field[0], field[1])) * 100) % 36000
                       if self.ready and self.stage == 4 and math.hypot(*field[:2]) >= 50 else -1)
            if heading >= 0:
                heading = (self.profile[6] * (heading - self.profile[5])) % 36000
            if self.stage == 1:
                self.samples = int((now - self.collect_started) * 25)
            flags = 9 | (2 if heading >= 0 else 0) | (16 if self.ready else 0) | (32 if 1 <= self.stage <= 3 else 0)
            self.buffer.extend(encode(0, "DATA", int(seconds * 1000) & 0xFFFFFFFF,
                                      *field, heading, *self.gains, flags))
            self.buffer.extend(self.status())
            self.next_sample = now + 0.2
        if not self.buffer:
            time.sleep(self.timeout)
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result

    def close(self) -> None:
        pass
