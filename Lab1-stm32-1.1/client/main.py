#!/usr/bin/env python3
"""Graphical magnetometer client. Usage: python main.py COM5."""
import argparse
import math
import queue
import sys
import time
from datetime import datetime
from pathlib import Path

from protocol import (FLAG_CALIBRATED, FLAG_CALIBRATING, FLAG_HEADING,
                      FLAG_OLED, FLAG_SAMPLE, FLAG_SATURATED)
from serial_link import SerialWorker


def main() -> None:
    parser = argparse.ArgumentParser(description="Магнитометр SDK-1.1: GUI, UART и журнал")
    parser.add_argument("port", nargs="?", help="COM5 (Windows) или /dev/cu.usbserial-… (macOS)")
    parser.add_argument("--baud", type=int, default=38400, help="Скорость UART, по умолчанию 38400")
    parser.add_argument("--log", type=Path, help="Путь к журналу JSONL (по умолчанию logs/)")
    parser.add_argument("--demo", action="store_true", help="Показать GUI с эмулятором без стенда")
    args = parser.parse_args()
    if not args.demo and not args.port:
        parser.error("задайте COM-порт аргументом или используйте --demo")
    if args.baud <= 0:
        parser.error("--baud должен быть положительным")

    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as error:
        hint = ("Установите Homebrew-пакет python-tk@3.14 и запускайте клиент "
                "через /opt/homebrew/bin/python3.14."
                if sys.platform == "darwin" else
                "Установите Tcl/Tk и модуль tkinter для используемого Python.")
        parser.exit(2, f"Ошибка: Tkinter недоступен ({error}).\n{hint}\n"
                      f"Текущий Python: {sys.executable}\n")
    # Apple's Tcl/Tk 8.5 can abort the process inside Tk(), before Python can
    # raise an exception. Reject it before initializing the native GUI.
    if sys.platform == "darwin" and tk.TkVersion < 8.6:
        parser.exit(2, "Ошибка: системный Tcl/Tk 8.5 несовместим с этим GUI на macOS.\n"
                      "Используйте Homebrew Python 3.14 с пакетом python-tk@3.14 "
                      "или Python с python.org.\n"
                      f"Текущий Python: {sys.executable}\n")

    class Application:
        def __init__(self, root) -> None:
            self.root = root
            self.worker = None
            self.events = queue.Queue()
            self.last_sample = None
            self.sample_time = 0.0
            self.error_seen = False
            self.closing = False
            self.pending_set = False
            self.controls_ready = False
            self.model = ""
            self.cal_supported = False
            self.cal_stage = 0
            self.cal_pending = False
            self.cal_title = tk.StringVar(value="Калибровка: ожидание подключения")
            self.cal_instruction = tk.StringVar()
            self.cal_progress = tk.StringVar(value="Образцы: 0")
            self.cal_profile = tk.StringVar(value="Профиль ещё не сохранён")
            self.values = [tk.StringVar(value="—") for _ in range(3)]
            self.requested = [tk.StringVar(value="100") for _ in range(3)]
            self.applied = tk.StringVar(value="На стенде: ожидание подключения")
            self.status = tk.StringVar(value="Открытие порта…")
            self.notice = tk.StringVar(value="")
            self.heading = tk.StringVar(value="Направление: —")
            self.log_label = tk.StringVar()
            root.title("SDK-1.1 · Магнитометр" + (" · ДЕМО" if args.demo else ""))
            root.geometry("860x820")
            root.minsize(800, 760)
            root.configure(bg="#f1f5f9")
            style = ttk.Style(root)
            if "clam" in style.theme_names():
                style.theme_use("clam")
            style.configure("TFrame", background="#f1f5f9")
            style.configure("TLabel", background="#f1f5f9", foreground="#173047")
            style.configure("Title.TLabel", font=("Helvetica", 23, "bold"))
            style.configure("Value.TLabel", font=("Helvetica", 26, "bold"))
            style.configure("TButton", padding=(12, 8))
            style.configure("TLabelframe", background="#f1f5f9")
            style.configure("TLabelframe.Label", background="#f1f5f9", font=("Helvetica", 12, "bold"))

            content = ttk.Frame(root, padding=24)
            content.pack(fill="both", expand=True)
            content.columnconfigure(0, weight=1)
            content.rowconfigure(3, weight=1)
            ttk.Label(content, text="Магнитное поле", style="Title.TLabel").grid(row=0, sticky="w")
            subtitle = f"SDK-1.1  /  {'ЭМУЛЯТОР' if args.demo else args.port}  /  {args.baud} 8N1"
            ttk.Label(content, text=subtitle).grid(row=1, sticky="w", pady=(5, 12))
            ttk.Label(content, textvariable=self.status, wraplength=720).grid(row=2, sticky="w")

            self.tabs = ttk.Notebook(content)
            self.tabs.grid(row=3, sticky="nsew", pady=(12, 0))
            measurements = ttk.Frame(self.tabs, padding=8)
            measurements.columnconfigure(0, weight=1)
            self.tabs.add(measurements, text="Измерения")
            calibration = ttk.Frame(self.tabs, padding=16)
            self.tabs.add(calibration, text="Калибровка")
            self.build_calibration(calibration)
            readings = ttk.Frame(measurements)
            readings.grid(row=0, sticky="nsew", pady=(0, 6))
            readings.columnconfigure(0, weight=1)
            axes = ttk.Frame(readings)
            axes.grid(row=0, column=0, sticky="nsew", padx=(0, 24))
            for index, name in enumerate("XYZ"):
                ttk.Label(axes, text=f"Ось {name}", font=("Helvetica", 12, "bold")).grid(
                    row=index * 2, column=0, sticky="w", pady=(8, 0))
                ttk.Label(axes, textvariable=self.values[index], style="Value.TLabel").grid(
                    row=index * 2 + 1, column=0, sticky="w", pady=(2, 10))
                ttk.Label(axes, text="мкТл").grid(row=index * 2 + 1, column=1, padx=(14, 0))
            self.canvas = tk.Canvas(readings, width=292, height=292, bg="#102739",
                                    highlightthickness=0)
            self.canvas.grid(row=0, column=1, sticky="e")
            ttk.Label(measurements, textvariable=self.heading).grid(row=1, sticky="e")

            settings = ttk.LabelFrame(measurements, text="Масштабные коэффициенты", padding=10)
            settings.grid(row=2, sticky="ew", pady=(8, 5))
            settings.columnconfigure(1, weight=1)
            self.scales = []
            for index, name in enumerate("XYZ"):
                ttk.Label(settings, text=name).grid(row=index, column=0, padx=(0, 12))
                scale = ttk.Scale(settings, from_=0, to=200,
                                  command=lambda value, i=index: self.requested[i].set(str(round(float(value)))))
                scale.set(100)
                scale.grid(row=index, column=1, sticky="ew", padx=(0, 12), pady=4)
                self.scales.append(scale)
                entry = ttk.Spinbox(settings, from_=0, to=200, increment=1, width=6,
                                    textvariable=self.requested[index])
                entry.grid(row=index, column=2)
                ttk.Label(settings, text="%").grid(row=index, column=3, padx=(6, 0))
            actions = ttk.Frame(settings)
            actions.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(10, 0))
            self.apply_button = ttk.Button(actions, text="Применить X, Y, Z", command=self.apply)
            self.apply_button.pack(side="left")
            ttk.Button(actions, text="Ввести 100%", command=self.defaults).pack(side="left", padx=8)
            ttk.Label(measurements, textvariable=self.applied).grid(row=3, sticky="w", pady=(0, 3))
            ttk.Label(content, textvariable=self.notice, foreground="#a4441b", wraplength=720).grid(
                row=4, sticky="w", pady=(5, 0))
            bottom = ttk.Frame(content)
            bottom.grid(row=5, sticky="ew", pady=(8, 0))
            self.ping_button = ttk.Button(bottom, text="PING", command=self.ping)
            self.ping_button.pack(side="left")
            self.reconnect_button = ttk.Button(bottom, text="Переподключить", command=self.connect)
            self.reconnect_button.pack(side="left", padx=8)
            ttk.Label(content, textvariable=self.log_label, wraplength=720,
                      font=("Helvetica", 9)).grid(row=6, sticky="w", pady=(8, 0))
            self.draw_compass(None)
            self.update_controls()
            root.protocol("WM_DELETE_WINDOW", self.close)
            self.connect()
            root.after(40, self.poll)

        def build_calibration(self, frame) -> None:
            frame.columnconfigure(0, weight=1)
            ttk.Label(frame, textvariable=self.cal_title, font=("Helvetica", 17, "bold"),
                      wraplength=700).grid(row=0, sticky="w", pady=(0, 14))
            ttk.Label(frame, text="Держите стенд горизонтально. Уберите магнит, телефон и "
                      "корпус ПК подальше. Положение кабелей и окружение при калибровке "
                      "и измерениях должны совпадать.", wraplength=700).grid(row=1, sticky="w")
            ttk.Label(frame, textvariable=self.cal_instruction, wraplength=700,
                      font=("Helvetica", 13)).grid(row=2, sticky="w", pady=20)
            ttk.Label(frame, textvariable=self.cal_progress).grid(row=3, sticky="w")
            actions = ttk.Frame(frame)
            actions.grid(row=4, sticky="w", pady=16)
            self.cal_buttons = {}
            for row, (operation, label) in enumerate([
                ("START", "1. Начать сбор"), ("FINISH", "2. Завершить обороты"),
                ("NORTH", "3. Верх OLED направлен на север"),
                ("TURN", "4. Поворот на 90° по часовой выполнен")]):
                button = ttk.Button(actions, text=label,
                                    command=lambda op=operation: self.cal_command(op))
                button.grid(row=row, column=0, sticky="ew", pady=4)
                self.cal_buttons[operation] = button
            recovery = ttk.Frame(frame)
            recovery.grid(row=5, sticky="w", pady=(0, 14))
            for operation, label in [("CANCEL", "Отменить"), ("RESET", "Сбросить калибровку")]:
                button = ttk.Button(recovery, text=label,
                                    command=lambda op=operation: self.cal_command(op))
                button.pack(side="left", padx=(0, 8))
                self.cal_buttons[operation] = button
            ttk.Label(frame, textvariable=self.cal_profile, wraplength=700).grid(row=6, sticky="w")
            if args.demo:
                ttk.Label(frame, text="ДЕМО: повороты и привязка к северу имитируются.",
                          wraplength=700).grid(row=7, sticky="w", pady=12)
            self.set_cal_instruction()

        def set_cal_instruction(self) -> None:
            instructions = [
                "Нужна калибровка. Нажмите «Начать сбор», затем медленно поверните стенд "
                "два полных оборота вокруг вертикальной оси за 20–30 секунд.",
                "Сбор данных: сделайте два полных горизонтальных оборота. Сохраняйте "
                "наклон и расстояние до окружающих предметов. Затем нажмите «Завершить обороты».",
                "Направьте верх OLED (сторона с отметкой 0) на известный магнитный север. "
                "Определите его независимым компасом вдали от электроники. Удерживайте "
                "стенд неподвижно не меньше секунды и нажмите кнопку 3.",
                "Из положения «верх OLED на север» поверните весь стенд на 90° по часовой "
                "стрелке, если смотреть сверху. Верх OLED теперь направлен на восток. "
                "Удерживайте секунду и нажмите кнопку 4.",
                "Калибровка готова. При повороте стенда на 90° по часовой стрелке угол "
                "стрелки относительно верха OLED должен уменьшаться на 90°. Клиент показывает "
                "тот же относительный угол; его окно находится в системе координат стенда."
            ]
            self.cal_instruction.set(instructions[self.cal_stage])

        def cal_command(self, operation) -> None:
            if self.worker.submit("CAL", operation):
                self.cal_pending = True
                self.notice.set("Ожидание ответа на команду калибровки…")
            self.update_controls()

        def update_controls(self) -> None:
            online = self.worker is not None and self.worker.online
            self.apply_button.configure(state="normal" if online and self.controls_ready and
                                        not self.pending_set else "disabled")
            self.ping_button.configure(state="normal" if online else "disabled")
            running = self.worker is not None and self.worker.is_alive()
            self.reconnect_button.configure(state="disabled" if running else "normal")
            allowed = {"START": (0, 4), "FINISH": (1,), "NORTH": (2,),
                       "TURN": (3,), "CANCEL": (1, 2, 3), "RESET": (0, 4)}
            for operation, button in self.cal_buttons.items():
                button.configure(state="normal" if online and self.cal_supported and
                                 not self.cal_pending and self.cal_stage in allowed[operation]
                                 else "disabled")

        def connect(self) -> None:
            if self.worker is not None and self.worker.is_alive():
                return
            # Drop any unprocessed events from the previous connection.
            self.events = queue.Queue()
            self.last_sample = None
            self.sample_time = 0.0
            self.error_seen = self.pending_set = self.controls_ready = False
            self.cal_supported = self.cal_pending = False
            self.cal_stage = 0
            self.cal_title.set("Калибровка: ожидание подключения")
            self.cal_profile.set("Профиль ещё не загружен")
            self.set_cal_instruction()
            self.status.set("Открытие порта…")
            self.notice.set("")
            self.applied.set("На стенде: ожидание подтверждения")
            self.clear_readings()
            log_path = args.log or (Path(__file__).resolve().parent.parent / "logs" /
                                    ("uart-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".jsonl"))
            self.log_label.set(f"Журнал: {log_path.resolve()}")
            factory = None
            if args.demo:
                from simulator import DemoSerial
                factory = DemoSerial
            self.worker = SerialWorker("DEMO" if args.demo else args.port, args.baud,
                                       log_path, self.events, factory)
            self.worker.start()
            self.update_controls()

        def defaults(self) -> None:
            for index in range(3):
                self.requested[index].set("100")
                self.scales[index].set(100)

        def apply(self) -> None:
            try:
                texts = tuple(variable.get().strip() for variable in self.requested)
                if not all(text.isascii() and text.isdigit() for text in texts):
                    raise ValueError()
                gains = tuple(int(text) for text in texts)
                if not all(0 <= value <= 200 for value in gains):
                    raise ValueError()
            except ValueError:
                self.notice.set("Введите целые коэффициенты от 0 до 200% для всех трёх осей.")
                return
            if self.worker.submit("SET", *gains):
                self.pending_set = True
                self.notice.set("Коэффициенты отправлены. Ожидание подтверждения…")
                for index, value in enumerate(gains):
                    self.scales[index].set(value)
            else:
                self.notice.set("Нет подключения или очередь команд заполнена.")
            self.update_controls()

        def ping(self) -> None:
            if self.worker.submit("PING"):
                self.notice.set("PING поставлен в очередь; ответ обновит статус подключения.")

        def draw_compass(self, heading) -> None:
            canvas = self.canvas
            canvas.delete("all")
            center, radius = 146, 103
            canvas.create_oval(center - radius, center - radius, center + radius,
                               center + radius, outline="#456074", width=2)
            for degree in range(0, 360, 30):
                angle = math.radians(degree)
                canvas.create_line(center + 95 * math.sin(angle), center - 95 * math.cos(angle),
                                   center + 103 * math.sin(angle), center - 103 * math.cos(angle),
                                   fill="#7190a4", width=2)
            canvas.create_text(center, 20, text="ВЕРХ ЭКРАНА · 0°", fill="#c7d9e5",
                               font=("Helvetica", 10, "bold"))
            canvas.create_text(271, center, text="90°", fill="#90aabd", font=("Helvetica", 9))
            canvas.create_text(center, 272, text="180°", fill="#90aabd", font=("Helvetica", 9))
            canvas.create_text(21, center, text="270°", fill="#90aabd", font=("Helvetica", 9))
            if heading is not None:
                angle = math.radians(heading / 100)
                dx, dy = math.sin(angle), -math.cos(angle)
                canvas.create_line(center - 25 * dx, center - 25 * dy,
                                   center + 80 * dx, center + 80 * dy, fill="#49d8bd",
                                   width=5, arrow="last", arrowshape=(18, 22, 8))
                canvas.create_text(center + 99 * dx, center + 99 * dy, text="N",
                                   fill="#71f2d8", font=("Helvetica", 13, "bold"))
            else:
                canvas.create_text(center, center, text="—", fill="#90aabd", font=("Helvetica", 32))
            canvas.create_oval(center - 4, center - 4, center + 4, center + 4,
                               fill="#dce9f1", outline="")

        def clear_readings(self) -> None:
            for value in self.values:
                value.set("—")
            self.heading.set("Направление: —")
            self.draw_compass(None)

        def show_sample(self, sample) -> None:
            self.last_sample = sample
            self.sample_time = time.monotonic()
            for variable, field in zip(self.values, sample.field):
                variable.set(f"{field / 100:+.2f}" if sample.flags & FLAG_SAMPLE else "—")
            heading = sample.heading if sample.flags & FLAG_HEADING else None
            self.draw_compass(heading)
            self.heading.set(f"Направление поля: {heading / 100:.2f}°" if heading is not None
                             else "Направление поля не определено")
            self.applied.set("На стенде: " + "   ".join(
                f"{axis} = {gain}%" for axis, gain in zip("XYZ", sample.gain)))
            issues = []
            if not sample.flags & FLAG_SAMPLE:
                issues.append("нет свежих данных магнитометра")
            elif sample.flags & FLAG_SATURATED:
                issues.append("насыщение датчика: удалите магнит")
            elif sample.flags & FLAG_CALIBRATING:
                issues.append("калибровка — стрелка временно скрыта")
            elif self.cal_supported and not sample.flags & FLAG_CALIBRATED:
                issues.append("откройте вкладку «Калибровка»")
            elif not sample.flags & FLAG_HEADING:
                issues.append("горизонтальное поле близко к нулю")
            if not sample.flags & FLAG_OLED:
                issues.append("ошибка OLED / PCA9538")
            self.status.set(f"Подключено · {self.model}" + (" · " + "; ".join(issues) if issues else " · данные поступают"))

        def poll(self) -> None:
            for _ in range(100):
                try:
                    kind, payload = self.events.get_nowait()
                except queue.Empty:
                    break
                if kind == "sample":
                    self.show_sample(payload)
                elif kind == "status":
                    self.status.set(payload)
                elif kind == "connected":
                    self.model = payload[0]
                    self.status.set(f"PING → PONG · {self.model} · I²C 0x{int(payload[1]):02X}")
                elif kind == "capabilities":
                    self.cal_supported, uid = payload
                    self.cal_profile.set(f"Стенд UID: {uid}" if uid else "Старая прошивка: обновите стенд.")
                    if not self.cal_supported:
                        self.cal_title.set("Калибровка требует новой прошивки")
                elif kind == "calibration":
                    previous = self.cal_stage
                    self.cal_stage = payload.stage
                    self.cal_title.set(["Нужна калибровка", "Сбор полного оборота",
                                        "Привязка к северу", "Определение направления осей",
                                        "Калибровка готова"][payload.stage])
                    self.cal_progress.set(f"Образцы: {payload.samples}" +
                                          (f" · секторы: {payload.coverage}/16" if payload.coverage else ""))
                    self.set_cal_instruction()
                    if previous != payload.stage and payload.stage in (2, 3, 4):
                        self.notice.set("Шаг калибровки выполнен. Следуйте инструкции во вкладке.")
                elif kind == "cal_ack":
                    self.cal_pending = False
                elif kind == "profile":
                    self.cal_profile.set(payload)
                elif kind == "ack":
                    command, gains = payload
                    self.controls_ready = True
                    self.applied.set("На стенде: " + "   ".join(
                        f"{axis} = {gain}%" for axis, gain in zip("XYZ", gains)))
                    if command == "GET":
                        for index, value in enumerate(gains):
                            self.requested[index].set(str(value))
                            self.scales[index].set(value)
                    else:
                        self.pending_set = False
                        self.notice.set("Стенд подтвердил новые коэффициенты.")
                elif kind == "pong":
                    self.notice.set("Получен PONG: связь со стендом работает.")
                elif kind == "rejected":
                    self.pending_set = False
                    self.cal_pending = False
                    explanations = {
                        "CAL_SAMPLES": "Мало образцов: продолжайте медленные полные обороты.",
                        "CAL_RANGE": "Мало изменения поля: держите стенд горизонтально и поверните полностью.",
                        "CAL_COVERAGE": "Не охвачен полный круг. Продолжайте обороты и повторите шаг 2.",
                        "CAL_DISTORTION": "Поле менялось или слишком искажено. Отмените сбор, уберите металл/магниты и начните снова.",
                        "CAL_STILL": "Держите стенд неподвижно хотя бы секунду, затем повторите нажатие.",
                        "CAL_TURN90": "Нужен поворот на 90° по часовой стрелке от положения на север.",
                        "CAL_ABORTED": "Калибровка отменена из-за таймаута, потери датчика или насыщения.",
                        "CAL_SENSOR": "Нет свежих данных датчика или он насыщен.",
                    }
                    self.notice.set(explanations.get(payload, f"Стенд отклонил команду: {payload}"))
                elif kind == "warning":
                    self.notice.set(f"Ошибка протокола: {payload}")
                elif kind == "error":
                    self.error_seen = True
                    self.status.set(f"Ошибка подключения: {payload}")
                    self.pending_set = self.controls_ready = False
                    self.clear_readings()
                elif kind == "closed":
                    self.pending_set = self.controls_ready = False
                    self.clear_readings()
                    if not self.error_seen:
                        self.status.set("Соединение закрыто")
            if self.last_sample is not None and time.monotonic() - self.sample_time > 1.0:
                self.clear_readings()
                if self.worker.online:
                    self.status.set("Нет свежих измерений больше 1 секунды")
            self.update_controls()
            if self.closing and not self.worker.is_alive():
                self.root.destroy()
            else:
                self.root.after(40, self.poll)

        def close(self) -> None:
            self.closing = True
            self.worker.stop()
            self.status.set("Закрытие соединения…")

    root = tk.Tk()
    Application(root)
    root.mainloop()


if __name__ == "__main__":
    main()
