#!/usr/bin/env python3
"""Part 5: manual control GUI for the TEC experiment.

Features:
- PWM slider and text box synchronized
- temperature, PWM, direction, and elapsed-time readouts
- temperature and PWM plots
- CSV logging and serial commands
"""

import csv
import re
import sys
from collections import deque
from pathlib import Path

import serial
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from pyqtgraph import PlotWidget

PORT = "/dev/tty.usbmodem1101"
BAUD = 115200
WINDOW_SECONDS = 60.0
PLOT_INTERVAL_MS = 100
OUTPUT_FILE = Path(__file__).resolve().parents[2] / "data" / "module_03" / "tec_control_data.csv"

MEASUREMENT_PATTERN = re.compile(
    r"Temperature \(C\):\s*([-+]?\d*\.?\d+)\s*,\s*Time \(s\):\s*([-+]?\d*\.?\d+)\s*,\s*PWM:\s*(\d+)\s*,\s*Heat/Cool:\s*([01])"
)


class ControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TEC Manual Control")
        self.resize(1100, 700)

        self.serial_port = serial.Serial(PORT, BAUD, timeout=0.1)
        self.temp_history = deque()
        self.time_history = deque()
        self.pwm_history = deque()

        OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(OUTPUT_FILE, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time_s", "temperature_C", "pwm", "heat_cool"])

        self.current_pwm = 0
        self.current_direction = "HEAT"

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        root_layout = QVBoxLayout(central_widget)

        controls_layout = QHBoxLayout()
        self.pwm_slider = QSlider(Qt.Horizontal)
        self.pwm_slider.setRange(0, 255)
        self.pwm_slider.valueChanged.connect(self.on_slider_changed)

        self.pwm_edit = QLineEdit("0")
        self.pwm_edit.setFixedWidth(80)
        self.pwm_edit.returnPressed.connect(self.on_text_entry)

        self.direction_label = QLabel("Direction: HEAT")
        self.temperature_label = QLabel("Temperature (C): --")
        self.time_label = QLabel("Time (s): --")
        self.pwm_label = QLabel("PWM: 0")

        controls_layout.addWidget(QLabel("PWM"))
        controls_layout.addWidget(self.pwm_slider)
        controls_layout.addWidget(self.pwm_edit)
        controls_layout.addWidget(self.direction_label)
        controls_layout.addWidget(self.temperature_label)
        controls_layout.addWidget(self.time_label)
        controls_layout.addWidget(self.pwm_label)

        root_layout.addLayout(controls_layout)

        self.temp_plot = PlotWidget(title="Temperature vs Time")
        self.pwm_plot = PlotWidget(title="PWM vs Time")

        plot_layout = QHBoxLayout()
        plot_layout.addWidget(self.temp_plot)
        plot_layout.addWidget(self.pwm_plot)
        root_layout.addLayout(plot_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_serial)
        self.timer.start(PLOT_INTERVAL_MS)

        self.send_command()

    def clamp_pwm(self, value):
        return max(0, min(255, int(value)))

    def on_slider_changed(self, value):
        self.current_pwm = self.clamp_pwm(value)
        self.pwm_edit.setText(str(self.current_pwm))
        self.pwm_label.setText(f"PWM: {self.current_pwm}")
        self.send_command()

    def on_text_entry(self):
        try:
            value = self.clamp_pwm(self.pwm_edit.text())
        except ValueError:
            value = 0
        self.current_pwm = value
        self.pwm_slider.setValue(value)
        self.pwm_label.setText(f"PWM: {value}")
        self.send_command()

    def send_command(self):
        command = f"SET PWM {self.current_pwm} DIR {self.current_direction}\n"
        if self.serial_port.is_open:
            self.serial_port.write(command.encode("utf-8"))

    def poll_serial(self):
        while self.serial_port.in_waiting:
            raw_line = self.serial_port.readline().decode("utf-8", errors="replace")
            if not raw_line:
                continue

            parsed = parse_measurement(raw_line)
            if parsed is None:
                continue

            self.temperature_label.setText(f"Temperature (C): {parsed['temperature_C']:.2f}")
            self.time_label.setText(f"Time (s): {parsed['time_s']:.2f}")
            self.pwm_label.setText(f"PWM: {parsed['pwm']}")

            direction_text = "HEAT" if parsed["heat_cool"] == 1 else "COOL"
            self.direction_label.setText(f"Direction: {direction_text}")
            self.current_direction = direction_text

            self.time_history.append(parsed["time_s"])
            self.temp_history.append(parsed["temperature_C"])
            self.pwm_history.append(parsed["pwm"])
            self.csv_writer.writerow(
                [
                    parsed["time_s"],
                    parsed["temperature_C"],
                    parsed["pwm"],
                    parsed["heat_cool"],
                ]
            )
            self.csv_file.flush()

            if len(self.time_history) >= 2:
                self.temp_plot.plot(list(self.time_history), list(self.temp_history), clear=True, pen="r")
                self.pwm_plot.plot(list(self.time_history), list(self.pwm_history), clear=True, pen="b")


def parse_measurement(line: str):
    match = MEASUREMENT_PATTERN.search(line)
    if not match:
        return None

    temperature_c, time_s, pwm_value, heat_cool = match.groups()
    return {
        "time_s": float(time_s),
        "temperature_C": float(temperature_c),
        "pwm": int(pwm_value),
        "heat_cool": int(heat_cool),
    }


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ControlWindow()
    window.show()
    sys.exit(app.exec())
