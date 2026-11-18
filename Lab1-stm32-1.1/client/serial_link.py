"""Background serial I/O; Tk widgets are only accessed by the main thread."""
import json
import queue
import threading
import time
import re
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

from protocol import CalibrationProfile, CalibrationStatus, StreamDecoder, Telemetry, encode
from calibration_store import CalibrationStore


class UartLog:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("a", encoding="utf-8")

    def record(self, **data: object) -> None:
        entry = {"timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                 **data}
        self.file.write(json.dumps(entry, ensure_ascii=False) + "\n")
        self.file.flush()

    def raw(self, direction: str, data: bytes) -> None:
        self.record(kind="bytes", direction=direction, hex=data.hex(),
                    text=data.decode("ascii", errors="backslashreplace"))

    def close(self) -> None:
        self.file.close()


@dataclass
class Pending:
    sequence: int
    command: Tuple[object, ...]
    frame: bytes
    sent_at: float = 0.0
    attempts: int = 0


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int, log_path: Path,
                 events: queue.Queue, serial_factory=None, profile_dir=None) -> None:
        super().__init__(daemon=True, name="sdk-uart")
        self.port_name = port
        self.baud = baud
        self.log_path = log_path
        self.events = events
        self.serial_factory = serial_factory
        self.commands: queue.Queue = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self.sequence = 0
        self.online = False
        self.uid = None
        self.model = ""
        self.cal_supported = False
        self.internal = deque()
        self.store = CalibrationStore(Path(profile_dir) if profile_dir is not None else
                                      Path(__file__).resolve().parent.parent / "calibration_profiles")
        self.profile_checked = False
        self.profile_requested = False

    def _calibration_status(self, status: CalibrationStatus) -> None:
        self.events.put(("calibration", status))
        if not self.uid or self.model == "NONE":
            return
        if not self.profile_checked:
            self.profile_checked = True
            try:
                saved = self.store.load(self.uid, self.model)
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.events.put(("warning", f"Не удалось загрузить калибровку: {error}"))
                saved = None
            if status.stage == 0 and saved is not None:
                self.internal.append(("CAL", "LOAD", *saved.values))
                self.events.put(("profile", "Восстановление калибровки этого стенда…"))
        if status.stage == 4:
            if not self.profile_requested:
                self.profile_requested = True
                self.internal.append(("CAL", "READ"))
        else:
            self.profile_requested = False

    def _calibration_reply(self, fields, pending) -> bool:
        if len(fields) == 7 and fields[1:3] == ("OK", "CAL"):
            status = CalibrationStatus.parse(fields[3:])
            # Delete the PC copy before processing the reset status, so it
            # cannot be reloaded accidentally in this connection.
            if pending.command[1] == "RESET" and self.uid:
                try:
                    self.store.remove(self.uid)
                except OSError as error:
                    self.events.put(("warning", f"Не удалось удалить профиль: {error}"))
                self.profile_checked = True
                self.events.put(("profile", "Калибровка сброшена на стенде и ПК."))
            self._calibration_status(status)
        elif (len(fields) == 10 and fields[1:3] == ("OK", "CALDATA") and
              pending.command[1] == "READ"):
            profile = CalibrationProfile(tuple(map(int, fields[3:])))
            try:
                path = self.store.save(self.uid, self.model, profile)
            except (OSError, ValueError) as error:
                self.events.put(("warning", f"Калибровка работает, но не сохранена: {error}"))
            else:
                self.events.put(("profile", f"Профиль сохранён: {path}"))
        else:
            return False
        self.events.put(("cal_ack", pending.command[1]))
        return True

    def submit(self, *command: object) -> bool:
        if not self.online or self.stop_event.is_set():
            return False
        try:
            self.commands.put_nowait(tuple(command))
            return True
        except queue.Full:
            return False

    def stop(self) -> None:
        self.stop_event.set()

    def _request(self, command: Tuple[object, ...]) -> Pending:
        self.sequence = self.sequence % 65535 + 1
        return Pending(self.sequence, command, encode(self.sequence, *command))

    def _send(self, port, log: UartLog, pending: Pending) -> None:
        offset = 0
        while offset < len(pending.frame):
            sent = port.write(pending.frame[offset:])
            if not sent:
                raise OSError("COM-порт не принимает данные")
            log.raw("TX", pending.frame[offset:offset + sent])
            offset += sent
        log.record(kind="message", direction="TX", sequence=pending.sequence,
                   fields=list(pending.command))
        pending.sent_at = time.monotonic()
        pending.attempts += 1

    def run(self) -> None:
        port = None
        log: Optional[UartLog] = None
        try:
            log = UartLog(self.log_path)
            log.record(kind="session", event="open", port=self.port_name, baud=self.baud)
            if self.serial_factory is None:
                import serial
                factory = serial.serial_for_url
            else:
                factory = self.serial_factory
            port = factory(self.port_name, baudrate=self.baud, timeout=0.02,
                           write_timeout=0.3, rtscts=False, dsrdtr=False)
            self.events.put(("status", "Порт открыт. Проверка PING…"))
            # The first transmitted command after opening the port is always PING.
            pending: Optional[Pending] = self._request(("PING",))
            self._send(port, log, pending)
            decoder = StreamDecoder()
            last_frame = time.monotonic()
            last_ping = last_frame
            while not self.stop_event.is_set():
                data = port.read(max(1, min(port.in_waiting, 1024)))
                if data:
                    log.raw("RX", data)  # Preserve every byte, even invalid frames.
                    for fields in decoder.feed(data):
                        log.record(kind="message", direction="RX", fields=list(fields))
                        now = time.monotonic()
                        if fields[:2] == ("0", "DATA"):
                            try:
                                sample = Telemetry.from_fields(fields)
                            except ValueError as error:
                                self.events.put(("warning", str(error)))
                            else:
                                last_frame = now
                                if self.online:
                                    self.events.put(("sample", sample))
                            continue
                        if self.online and self.cal_supported and fields[:2] == ("0", "CALSTAT"):
                            try:
                                self._calibration_status(CalibrationStatus.parse(fields[2:]))
                            except ValueError as error:
                                self.events.put(("warning", str(error)))
                            continue
                        if fields[:2] == ("0", "ERR"):
                            self.events.put(("rejected", ",".join(fields[2:])))
                            continue
                        if pending is None or fields[0] != str(pending.sequence):
                            continue
                        if fields[1] == "ERR":
                            self.events.put(("rejected", ",".join(fields[2:])))
                            if not self.online:
                                raise OSError("Стенд отклонил PING")
                            pending = None
                            continue
                        if pending.command[0] == "PING":
                            legacy = len(fields) == 5 and fields[1:3] == ("PONG", "1")
                            current = (len(fields) == 6 and fields[1:3] == ("PONG", "2") and
                                       re.fullmatch(r"[0-9A-Fa-f]{24}", fields[5]))
                            if not legacy and not current:
                                continue
                            try:
                                address = int(fields[4])
                            except ValueError:
                                continue
                            if not 0 <= address <= 127:
                                continue
                            if self.online and current and fields[5].upper() != self.uid:
                                raise OSError("Сменился UID стенда: переподключите клиент")
                            if self.online and fields[3] != "NONE" and fields[3] != self.model:
                                self.model = fields[3]
                                self.profile_checked = self.profile_requested = False
                                self.events.put(("connected", (self.model, str(address))))
                            last_frame = now
                            last_ping = now
                            self.events.put(("pong", None))
                            if not self.online:
                                self.model = fields[3]
                                self.cal_supported = bool(current)
                                self.uid = fields[5].upper() if current else None
                                self.online = True
                                self.events.put(("connected", (fields[3], str(address))))
                                self.events.put(("capabilities", (self.cal_supported, self.uid)))
                                if self.cal_supported:
                                    self.internal.appendleft(("CAL", "GET"))
                                else:
                                    self.events.put(("profile", "Для калибровки обновите прошивку стенда (протокол 2)."))
                                pending = self._request(("GET",))
                                self._send(port, log, pending)
                                continue
                        elif pending.command[0] == "CAL":
                            try:
                                valid = self._calibration_reply(fields, pending)
                            except (ValueError, TypeError) as error:
                                self.events.put(("warning", str(error)))
                                continue
                            if not valid:
                                continue
                            last_frame = now
                        else:
                            if len(fields) != 6 or fields[1:3] != ("OK", pending.command[0]):
                                continue
                            try:
                                gains = tuple(int(value) for value in fields[3:])
                            except ValueError:
                                continue
                            if not all(0 <= value <= 200 for value in gains):
                                continue
                            last_frame = now
                            self.events.put(("ack", (pending.command[0], gains)))
                        pending = None
                    for error in decoder.errors:
                        log.record(kind="decode_error", detail=error)
                        self.events.put(("warning", error))
                    decoder.errors.clear()
                now = time.monotonic()
                if pending is not None and now - pending.sent_at >= 1.0:
                    if pending.attempts >= 3:
                        raise TimeoutError(f"Нет ответа на {pending.command[0]} после 3 попыток")
                    self._send(port, log, pending)
                if self.online and now - last_frame > 3.5:
                    raise TimeoutError("Стенд перестал отвечать")
                if pending is None:
                    if self.internal:
                        command = self.internal.popleft()
                    else:
                        try:
                            command = self.commands.get_nowait()
                        except queue.Empty:
                            command = ("PING",) if now - last_ping >= 2.0 else None
                    if command is not None:
                        pending = self._request(command)
                        self._send(port, log, pending)
        except Exception as error:
            # Includes missing pyserial, serial disconnects, and log-file errors.
            if log is not None:
                try:
                    log.record(kind="error", detail=str(error))
                except OSError:
                    pass
            self.events.put(("error", f"{type(error).__name__}: {error}"))
        finally:
            self.online = False
            if port is not None:
                try:
                    port.close()
                except Exception as error:
                    self.events.put(("error", f"Ошибка закрытия порта: {error}"))
            if log is not None:
                try:
                    log.record(kind="session", event="close")
                except OSError:
                    pass
                finally:
                    try:
                        log.close()
                    except OSError:
                        pass
            self.events.put(("closed", None))
