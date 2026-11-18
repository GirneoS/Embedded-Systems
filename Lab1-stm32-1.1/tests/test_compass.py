import ctypes
import json
import math
import queue
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "client"))
from protocol import CalibrationProfile, StreamDecoder, Telemetry, crc16, decode, encode
from calibration_store import CalibrationStore
from serial_link import SerialWorker
from simulator import DemoSerial


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_and_corruption(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        self.assertEqual(decode(encode(1, "SET", 0, 100, 200)), ("1", "SET", "0", "100", "200"))
        with self.assertRaises(ValueError):
            decode(encode(1, "PING").replace(b"PING", b"PONG"))

    def test_stream_fragmentation_and_recovery(self):
        stream = StreamDecoder()
        frame = encode(4, "PING")
        self.assertEqual(stream.feed(b"garbage" + frame[:5]), [])
        self.assertEqual(stream.feed(frame[5:] + encode(5, "GET")), [("4", "PING"), ("5", "GET")])
        self.assertEqual(stream.feed(b"$" + b"x" * 500 + encode(6, "PING")), [("6", "PING")])
        self.assertTrue(stream.errors)

    def test_telemetry_rejects_invalid_direction_and_ranges(self):
        sample = Telemetry.from_fields(decode(encode(0, "DATA", 3, 10, -10, 0, -1, 0, 200, 100, 9)))
        self.assertEqual(sample.gain, (0, 200, 100))
        for heading, gain, flags in [(36000, 100, 11), (-1, 201, 9), (-1, 100, 11), (0, 100, 15)]:
            with self.assertRaises(ValueError):
                Telemetry.from_fields(decode(encode(0, "DATA", 3, 10, 20, 30, heading, gain, 100, 100, flags)))


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("cc")
        if not compiler:
            raise unittest.SkipTest("A native C compiler is needed for the HAL simulation")
        cls.temp = tempfile.TemporaryDirectory(prefix="sdk-firmware-test-")
        library = Path(cls.temp.name) / "firmware.so"
        sources = [ROOT / "Core/Src" / name for name in
                   ("compass.c", "compass_math.c", "compass_calibration.c", "wire_protocol.c", "magnetometer.c", "oled.c")]
        subprocess.run([compiler, "-shared", "-fPIC", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(ROOT / "tests/stubs"), "-I" + str(ROOT / "Core/Inc"),
                        *map(str, sources), str(ROOT / "tests/fake_hal.c"), "-lm", "-o", str(library)], check=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.Fake_Output.restype = ctypes.c_char_p
        cls.lib.Fake_Display.restype = ctypes.POINTER(ctypes.c_ubyte)
        cls.lib.Wire_Crc16.restype = ctypes.c_uint16
        cls.lib.Compass_Heading.argtypes = [ctypes.POINTER(ctypes.c_int32)]
        cls.lib.Calibration_Add.argtypes = [ctypes.POINTER(ctypes.c_int16)]
        cls.lib.Calibration_Finish.restype = ctypes.c_char_p
        cls.lib.Calibration_SetNorth.restype = ctypes.c_char_p
        cls.lib.Calibration_SetTurn.restype = ctypes.c_char_p

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.lib.Fake_Reset(0x1D, 0x49)
        self.lib.Fake_Raw(0x1D, 0x08, 1000, 2000, -3000)

    def frames(self):
        data = self.lib.Fake_Output()
        self.lib.Fake_ClearOutput()
        stream = StreamDecoder()
        frames = stream.feed(data)
        self.assertEqual(stream.errors, [])
        return frames

    def command(self, sequence, *fields):
        frame = encode(sequence, *fields)
        # Feed separate pieces, exercising the real firmware byte ISR and parser.
        self.lib.Fake_Receive(frame[:3], 3)
        self.lib.Fake_Run(1)
        self.lib.Fake_Receive(frame[3:], len(frame) - 3)
        self.lib.Fake_Run(10)
        return self.frames()

    def next_sample(self):
        self.lib.Fake_Advance(250)
        self.lib.Fake_Run(10)
        frames = self.frames()
        return Telemetry.from_fields(next(fields for fields in reversed(frames) if fields[1] == "DATA"))

    def test_real_sensor_registers_reset_and_page_transfer(self):
        self.assertEqual(self.lib.Fake_Register(0x1D, 0x24), 0x70)
        self.assertEqual(self.lib.Fake_Register(0x1D, 0x25), 0x60)
        self.assertEqual(self.lib.Fake_Register(0x70, 1), 0xA5)
        self.assertEqual(self.lib.Fake_Register(0x70, 3), 0xFE)
        self.assertEqual(self.lib.Fake_ResetCount(), 1)
        self.lib.Fake_Run(10)
        self.assertTrue(any(self.lib.Fake_Display()[i] for i in range(1024)))

    def test_ping_scaling_atomic_validation_and_persistence(self):
        self.assertIn(("1", "PONG", "2", "LSM9DS0", "29", "1234567890ABCDEF01020304"), self.command(1, "PING"))
        self.command(40, "CAL", "LOAD", 0, 0, 1000000, 0, 1000000, 0, 1)
        self.assertIn(("2", "OK", "SET", "0", "200", "50"), self.command(2, "SET", 0, 200, 50))
        sample = self.next_sample()
        self.assertEqual(sample.field, (0, 19200, -7200))
        self.assertEqual(sample.heading, 0)
        self.assertIn(("3", "ERR", "RANGE"), self.command(3, "SET", 123, 201, 50))
        self.assertIn(("4", "OK", "GET", "0", "200", "50"), self.command(4, "GET"))
        self.assertEqual(self.next_sample().gain, (0, 200, 50))
        self.assertIn(("5", "OK", "SET", "0", "0", "0"), self.command(5, "SET", 0, 0, 0))
        self.assertEqual(self.next_sample().heading, -1)

    def test_bad_crc_overflow_and_uart_error_recover(self):
        bad = encode(10, "SET", 50, 50, 50).replace(b"SET", b"GET")
        self.lib.Fake_Receive(bad, len(bad))
        self.lib.Fake_Run(10)
        self.assertIn(("0", "ERR", "BAD_FRAME"), self.frames())
        self.assertIn(("11", "OK", "GET", "100", "100", "100"), self.command(11, "GET"))
        self.lib.Fake_Receive(b"x" * 600, 600)
        self.lib.Fake_Run(10)
        self.assertIn(("0", "ERR", "RX_OVERFLOW"), self.frames())
        self.lib.Fake_UartError()
        self.lib.Fake_Run(10)
        self.assertIn(("0", "ERR", "UART_ERROR"), self.frames())
        self.assertIn(("12", "PONG", "2", "LSM9DS0", "29", "1234567890ABCDEF01020304"), self.command(12, "PING"))

    def test_heading_quadrants_scale_limits_and_crc_match(self):
        for field, expected in [((0, 1000, 0), 0), ((1000, 0, 0), 9000),
                                ((0, -1000, 0), 18000), ((-1000, 0, 0), 27000),
                                ((1, 1, 0), -1), ((0, 0, 1000), -1)]:
            self.assertEqual(self.lib.Compass_Heading((ctypes.c_int32 * 3)(*field)), expected)
        raw = (ctypes.c_int16 * 3)(-32768, 32767, -1)
        gains = (ctypes.c_uint16 * 3)(200, 200, 200)
        field = (ctypes.c_int32 * 3)()
        self.lib.Compass_Scale(raw, 58, gains, field)
        self.assertEqual(tuple(field), (-380109, 380097, -12))
        body = b"65535,SET,0,100,200"
        self.assertEqual(self.lib.Wire_Crc16(body, len(body)), crc16(body))

    def test_lsm9ds1_detection_saturation_and_missing_devices(self):
        self.lib.Fake_Reset(0x1C, 0x3D)
        self.lib.Fake_Raw(0x1C, 0x28, 1000, 2000, -3000)
        self.assertEqual(self.lib.Fake_Register(0x1C, 0x20), 0xF8)
        self.assertIn(("1", "PONG", "2", "LSM9DS1", "28", "1234567890ABCDEF01020304"), self.command(1, "PING"))
        self.assertEqual(self.next_sample().field, (5800, 11600, -17400))
        self.lib.Fake_Raw(0x1C, 0x28, 32767, 10, 20)
        saturated = self.next_sample()
        self.assertTrue(saturated.flags & 4)
        self.assertEqual(saturated.heading, -1)
        self.lib.Fake_Present(0x1C, 0)
        self.lib.Fake_Present(0x3C, 0)
        self.next_sample()  # OLED failure is detected while flushing this frame.
        sample = self.next_sample()
        self.assertFalse(sample.flags & 1)
        self.assertFalse(sample.flags & 8)
        self.assertIn(("2", "OK", "SET", "20", "30", "40"), self.command(2, "SET", 20, 30, 40))

    def test_healthy_sensor_is_not_reinitialized_after_two_seconds(self):
        for _ in range(20):
            self.next_sample()
        # Reinitializing would lose a fresh sample for at least one publication.
        self.assertTrue(self.next_sample().flags & 1)

    @staticmethod
    def distorted(cw_degrees, handedness=1):
        angle = math.radians(37 + handedness * cw_degrees)
        x, y = 650 * math.sin(angle), 650 * math.cos(angle)
        return (ctypes.c_int16 * 3)(round(1600 + 1.4*x + .25*y),
                                    round(-1100 + .25*x + .85*y), -750)

    def acquire_ellipse(self, handedness=1, sweep=360):
        self.lib.Calibration_Start()
        for i in range(400):
            # Deliberately nonuniform speed: covariance-only correction would
            # bias the answer, but the fitted ellipse must stay the same.
            self.lib.Calibration_Add(self.distorted(sweep * (i / 399)**1.3, handedness))

    def hold(self, angle, handedness=1):
        for _ in range(12):
            self.lib.Calibration_Add(self.distorted(angle, handedness))

    def test_calibration_removes_offset_cross_axis_distortion_and_learns_mounting(self):
        for handedness in (-1, 1):
            with self.subTest(handedness=handedness):
                self.lib.Calibration_Init()
                self.acquire_ellipse(handedness)
                self.assertIsNone(self.lib.Calibration_Finish(48))
                self.assertGreaterEqual(self.lib.Calibration_Coverage(), 14)
                self.assertEqual(self.lib.Calibration_SetNorth(), b"CAL_STILL")
                self.hold(0, handedness)
                self.assertIsNone(self.lib.Calibration_SetNorth())
                self.hold(20, handedness)
                self.assertEqual(self.lib.Calibration_SetTurn(), b"CAL_TURN90")
                self.hold(90, handedness)
                self.assertIsNone(self.lib.Calibration_SetTurn())
                fields = (ctypes.c_int32 * 3)()
                gains = (ctypes.c_uint16 * 3)(100, 100, 100)
                magnitudes = []
                for degrees in range(0, 360, 15):
                    self.lib.Calibration_Field(self.distorted(degrees, handedness), 48, gains, fields)
                    actual = self.lib.Calibration_Heading(fields)
                    expected = (-degrees * 100) % 36000
                    error = (actual - expected + 18000) % 36000 - 18000
                    self.assertLess(abs(error), 35)  # <0.35 degree, including rounding
                    self.assertEqual(fields[2], -3600)  # Horizontal cal preserves Z.
                    magnitudes.append(math.hypot(*fields[:2]))
                self.assertLess(max(magnitudes) / min(magnitudes), 1.005)

    def test_incomplete_calibration_is_rejected_and_cancel_keeps_old_profile(self):
        self.assertEqual(self.next_sample().heading, -1)  # Never claim north before calibration.
        self.command(1, "CAL", "LOAD", 0, 0, 1000000, 0, 1000000, 0, 1)
        old = self.next_sample().heading
        self.acquire_ellipse(sweep=150)
        self.assertEqual(self.lib.Calibration_Finish(48), b"CAL_COVERAGE")
        self.assertEqual(self.next_sample().heading, -1)
        self.command(2, "CAL", "CANCEL")
        self.assertEqual(self.next_sample().heading, old)
        self.acquire_ellipse()
        self.lib.Calibration_Start()  # A retry must not clear the acquisition.
        self.assertEqual(self.lib.Calibration_Samples(), 400)
        self.assertIsNone(self.lib.Calibration_Finish(48))
        self.assertIsNone(self.lib.Calibration_Finish(48))
        # A changing field cannot be used as a stationary north reference.
        for i in range(12):
            self.lib.Calibration_Add(self.distorted(i * 4))
        self.assertEqual(self.lib.Calibration_SetNorth(), b"CAL_STILL")

    def test_calibration_wire_roundtrip_validation_reset_and_sensor_byte_order(self):
        profile = (1245000, -900000, 1250000, 0, 800000, 1234, -1)
        self.assertIn(("1", "OK", "CAL", "4", "0", "0", "1"),
                      self.command(1, "CAL", "LOAD", *profile))
        self.assertIn(("2", "OK", "CALDATA", *map(str, profile)), self.command(2, "CAL", "READ"))
        self.assertIn(("3", "ERR", "CAL_PROFILE"),
                      self.command(3, "CAL", "LOAD", 0, 0, 1, 0, 1, 0, 1))
        self.assertIn(("4", "OK", "CALDATA", *map(str, profile)), self.command(4, "CAL", "READ"))
        self.command(5, "CAL", "RESET")
        sample = self.next_sample()
        self.assertEqual(sample.heading, -1)
        self.assertFalse(sample.flags & 16)
        self.lib.Fake_Reset(0x1C, 0x3D)
        self.lib.Fake_Raw(0x1C, 0x28, -32768, 32767, -1)
        self.assertEqual(self.next_sample().field, (-190054, 190049, -6))


class CalibrationStoreTests(unittest.TestCase):
    def test_profiles_are_atomic_validated_and_bound_to_uid_and_model(self):
        with tempfile.TemporaryDirectory() as temp:
            store = CalibrationStore(Path(temp))
            first, second = "111111111111111111111111", "222222222222222222222222"
            profile = CalibrationProfile((125, -250, 1000000, 0, 1000000, 1234, -1))
            store.save(first, "LSM9DS1", profile)
            self.assertEqual(store.load(first, "LSM9DS1"), profile)
            self.assertIsNone(store.load(second, "LSM9DS1"))
            with self.assertRaises(ValueError):
                store.load(first, "LSM9DS0")
            with self.assertRaises(ValueError):
                store.path("../other")
            store.path(first).write_text('{"version":1,"uid":"' + first +
                                         '","model":"LSM9DS1","values":[0,0,1,0,1,0,1]}')
            with self.assertRaises(ValueError):
                store.load(first, "LSM9DS1")
            store.remove(first)
            self.assertIsNone(store.load(first, "LSM9DS1"))


class TransportTests(unittest.TestCase):
    def wait_for(self, events, kind, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            event, payload = events.get(timeout=max(0.01, deadline - time.monotonic()))
            if event == "error":
                self.fail(payload)
            if event == kind:
                return payload
        self.fail(f"No {kind} event")

    def test_handshake_set_ack_telemetry_and_lossless_log(self):
        with tempfile.TemporaryDirectory() as temp:
            events = queue.Queue()
            path = Path(temp) / "uart.jsonl"
            worker = SerialWorker("DEMO", 38400, path, events, DemoSerial,
                                  profile_dir=Path(temp) / "profiles")
            worker.start()
            try:
                self.wait_for(events, "connected")
                self.assertEqual(self.wait_for(events, "ack"), ("GET", (100, 100, 100)))
                self.assertTrue(worker.submit("SET", 0, 200, 50))
                self.assertEqual(self.wait_for(events, "ack"), ("SET", (0, 200, 50)))
                sample = self.wait_for(events, "sample")
                self.assertEqual(sample.gain, (0, 200, 50))
                self.assertEqual(sample.field[0], 0)
            finally:
                worker.stop()
                worker.join(1)
            self.assertFalse(worker.is_alive())
            entries = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(all("timestamp" in entry for entry in entries))
            tx = b"".join(bytes.fromhex(entry["hex"]) for entry in entries
                           if entry.get("kind") == "bytes" and entry["direction"] == "TX")
            frames = StreamDecoder().feed(tx)
            self.assertEqual(frames[0], ("1", "PING"))
            self.assertTrue(any(frame[1] == "SET" for frame in frames))
            self.assertTrue(any(entry.get("direction") == "RX" and entry["kind"] == "bytes"
                                for entry in entries))

    def test_profile_restore_reconnect_and_reset(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / "profiles"
            store = CalibrationStore(directory)
            uid = "000000000000000000000001"
            profile = CalibrationProfile((0, 0, 1000000, 0, 1000000, 1234, -1))
            store.save(uid, "DEMO", profile)
            for reset in (False, True):
                events = queue.Queue()
                worker = SerialWorker("DEMO", 38400, Path(temp) / "uart.jsonl", events,
                                      DemoSerial, profile_dir=directory)
                worker.start()
                try:
                    self.wait_for(events, "connected")
                    while self.wait_for(events, "calibration").stage != 4:
                        pass
                    self.wait_for(events, "profile")  # CAL READ saves the restored profile.
                    self.assertEqual(store.load(uid, "DEMO"), profile)
                    if reset:
                        self.assertTrue(worker.submit("CAL", "RESET"))
                        while self.wait_for(events, "cal_ack") != "RESET":
                            pass
                        self.assertIsNone(store.load(uid, "DEMO"))
                finally:
                    worker.stop()
                    worker.join(1)


if __name__ == "__main__":
    unittest.main()
