#!/usr/bin/env python3
"""Module 5 TEC calibration GUI with manual direction and PWM control."""

import csv
import math
import os
import queue
import re
import stat
import sys
import threading
from collections import deque
from pathlib import Path

import serial
from PySide6 import QtCore, QtWidgets


def expose_macos_qt_plugins():
    if sys.platform != "darwin":
        return

    plugin_dir = Path(
        QtCore.QLibraryInfo.path(QtCore.QLibraryInfo.LibraryPath.PluginsPath)
    ) / "platforms"
    for path in (plugin_dir, *plugin_dir.glob("*.dylib")):
        flags = path.stat().st_flags
        if flags & stat.UF_HIDDEN:
            os.chflags(path, flags & ~stat.UF_HIDDEN)


expose_macos_qt_plugins()

import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter


SERIAL_PORT = "/dev/cu.usbmodem1101"
BAUD_RATE = 9600
WINDOW_SECONDS = 60.0
PLOT_UPDATE_MS = 200
CSV_PATH = Path(__file__).resolve().parents[1] / "data" / "module_05" / "tec_calibration.csv"
STRIP_CHART_DIR = Path(__file__).resolve().parent / "strip charts"
KP_PWM_PER_C = 10.0

HEAT_COLOR = "#c43b32"
COOL_COLOR = "#2475a8"
MIN_SAFE_TEMPERATURE_C = 10.0
MAX_SAFE_TEMPERATURE_C = 60.0

MEASUREMENT_RE = re.compile(
    r"Temperature \(C\):\s*(?P<temperature>-?\d+(?:\.\d+)?|nan)\s*,\s*"
    r"Time \(s\):\s*(?P<time>-?\d+(?:\.\d+)?)\s*,\s*"
    r"PWM:\s*(?P<pwm>\d+)\s*,\s*(?:[^,\r\n]*,\s*)*"
    r"Heat/Cool:\s*(?P<heat_cool>[01])"
)


def parse_measurement(line: str):
    match = MEASUREMENT_RE.search(line)
    if match is None or match.group("temperature").lower() == "nan":
        return None

    temperature = float(match.group("temperature"))
    timestamp = float(match.group("time"))
    if not math.isfinite(temperature) or not math.isfinite(timestamp):
        return None

    return {
        "time_s": timestamp,
        "temperature_C": temperature,
        "pwm": int(match.group("pwm")),
        "heat_cool": int(match.group("heat_cool")),
    }


def calculate_proportional_command(
    temperature_c: float,
    setpoint_c: float,
    kp_pwm_per_c: float,
):
    signed_pwm = kp_pwm_per_c * (setpoint_c - temperature_c)
    direction = "HEAT" if signed_pwm >= 0 else "COOL"
    pwm = min(255, round(abs(signed_pwm)))
    return pwm, direction


def next_strip_chart_trial_number(output_dir: Path) -> int:
    trial_numbers = []
    for path in output_dir.iterdir() if output_dir.exists() else ():
        folder_match = re.fullmatch(r"trial_(\d+)", path.name)
        file_match = re.fullmatch(r"trial_(\d+)_.*", path.name)
        match = folder_match or file_match
        if match is not None:
            trial_numbers.append(int(match.group(1)))
    return max(trial_numbers, default=0) + 1


def analyze_strip_chart(records):
    if not records:
        return None

    temperatures = [item["temperature_C"] for item in records]
    times = [item["time_s"] for item in records]
    late_start = max(0, math.floor(len(records) * 0.8))
    steady_temperature = sum(temperatures[late_start:]) / len(temperatures[late_start:])
    start_temperature = temperatures[0]
    setpoint = records[0]["setpoint_C"]
    approach_direction = 1 if setpoint >= start_temperature else -1

    extrema = []
    for index in range(1, len(records) - 1):
        previous = temperatures[index - 1]
        current = temperatures[index]
        following = temperatures[index + 1]
        if current > previous and current > following:
            extrema.append((index, "maximum"))
        elif current < previous and current < following:
            extrema.append((index, "minimum"))

    peak_kind = "maximum" if approach_direction > 0 else "minimum"
    first_peak = next(
        (index for index, kind in extrema if kind == peak_kind),
        None,
    )
    overshoot = (
        approach_direction * (temperatures[first_peak] - steady_temperature)
        if first_peak is not None
        else None
    )

    crossings = []
    if first_peak is not None:
        for index in range(first_peak, len(records) - 1):
            first_delta = temperatures[index] - steady_temperature
            next_delta = temperatures[index + 1] - steady_temperature
            if first_delta * next_delta < 0:
                fraction = -first_delta / (next_delta - first_delta)
                crossing_time = times[index] + fraction * (times[index + 1] - times[index])
                crossings.append(crossing_time)

    later_peaks = [
        {
            "time_s": times[index],
            "temperature_C": temperatures[index],
            "deviation_C": abs(temperatures[index] - steady_temperature),
            "kind": kind,
        }
        for index, kind in extrema
        if first_peak is not None and index > first_peak
    ]
    peak_deviations = [peak["deviation_C"] for peak in later_peaks]
    peaks_decay = bool(peak_deviations) and all(
        later <= earlier for earlier, later in zip(peak_deviations, peak_deviations[1:])
    )

    signed_pwm = [
        item["pwm"] if item["heat_cool"] == 1 else -item["pwm"]
        for item in records
    ]
    saturation_intervals = []
    saturation_start = None
    for index, item in enumerate(records):
        saturated = item["pwm"] >= 255
        if saturated and saturation_start is None:
            saturation_start = index
        if saturation_start is not None and (
            not saturated or index == len(records) - 1
        ):
            saturation_end = index if saturated else index - 1
            saturation_intervals.append(
                {
                    "start_time_s": times[saturation_start],
                    "end_time_s": times[saturation_end],
                    "duration_s": times[saturation_end] - times[saturation_start],
                }
            )
            saturation_start = None

    gains = [item["kp_pwm_per_C"] for item in records]
    return {
        "start_temperature_C": start_temperature,
        "setpoint_C": setpoint,
        "steady_temperature_C": steady_temperature,
        "steady_window_start_s": times[late_start],
        "first_peak": (
            {
                "time_s": times[first_peak],
                "temperature_C": temperatures[first_peak],
            }
            if first_peak is not None
            else None
        ),
        "overshoot_C": overshoot,
        "later_crossings_s": crossings,
        "later_peaks": later_peaks,
        "later_peaks_decay": peaks_decay,
        "signed_pwm_min": min(signed_pwm),
        "signed_pwm_max": max(signed_pwm),
        "saturation_intervals": saturation_intervals,
        "kp_min": min(gains),
        "kp_max": max(gains),
    }


def format_strip_chart_report(trial_number: int, analysis):
    first_peak = analysis["first_peak"]
    if first_peak is None:
        peak_description = "No local peak detected."
        overshoot_description = "Not available (no local peak detected)."
    else:
        peak_description = (
            f"{first_peak['temperature_C']:.2f} °C at "
            f"{first_peak['time_s']:.2f} s"
        )
        overshoot_description = f"{analysis['overshoot_C']:.2f} °C relative to T_ss"

    if analysis["kp_min"] == analysis["kp_max"]:
        gain_description = f"{analysis['kp_min']:g} PWM/°C"
    else:
        gain_description = (
            f"{analysis['kp_min']:g}–{analysis['kp_max']:g} PWM/°C "
            "(varied during run)"
        )

    crossings = (
        ", ".join(f"{time_s:.2f} s" for time_s in analysis["later_crossings_s"])
        or "none observed"
    )
    peaks = analysis["later_peaks"]
    if peaks:
        peak_description_later = "; ".join(
            f"{peak['kind']} {peak['temperature_C']:.2f} °C at "
            f"{peak['time_s']:.2f} s (|T − T_ss|={peak['deviation_C']:.2f} °C)"
            for peak in peaks
        )
        if len(peaks) == 1:
            decay_description = "only one later peak; decay trend cannot be determined"
        elif analysis["later_peaks_decay"]:
            decay_description = "later peak deviations are non-increasing"
        else:
            decay_description = "later peak deviations are not monotonically decreasing"
        peaks_description = f"{peak_description_later}; {decay_description}"
    else:
        peaks_description = "No later local peaks detected."

    intervals = analysis["saturation_intervals"]
    saturation_description = (
        "; ".join(
            f"{interval['start_time_s']:.2f}–{interval['end_time_s']:.2f} s "
            f"({interval['duration_s']:.2f} s)"
            for interval in intervals
        )
        if intervals
        else "none observed"
    )

    return "\n".join(
        [
            f"Trial {trial_number:03d} strip-chart analysis",
            f"Starting temperature: {analysis['start_temperature_C']:.2f} °C",
            f"Setpoint: {analysis['setpoint_C']:.2f} °C",
            f"K_p: {gain_description}",
            (
                f"T_ss: {analysis['steady_temperature_C']:.2f} °C "
                f"(mean of final 20% of samples, from "
                f"{analysis['steady_window_start_s']:.2f} s)"
            ),
            f"First peak: {peak_description}",
            f"Overshoot relative to T_ss: {overshoot_description}",
            f"Later crossings of T_ss: {crossings}",
            f"Later peaks: {peaks_description}",
            (
                "Signed PWM (heat positive, cool negative): "
                f"{analysis['signed_pwm_min']:+d} to {analysis['signed_pwm_max']:+d} counts"
            ),
            f"PWM saturation (|PWM| = 255): {saturation_description}",
        ]
    )


def create_strip_chart(records, trial_number: int, output_dir: Path):
    if not records:
        return None

    analysis = analyze_strip_chart(records)
    trial_dir = output_dir / f"trial_{trial_number:03d}"
    trial_dir.mkdir(parents=True, exist_ok=False)
    chart = pg.PlotWidget()
    chart.setTitle(
        f"Trial {trial_number:03d}: T_start={analysis['start_temperature_C']:.2f} °C, "
        f"T_set={analysis['setpoint_C']:.2f} °C, "
        f"T_ss={analysis['steady_temperature_C']:.2f} °C, "
        f"K_p={analysis['kp_min']:g} PWM/°C"
    )
    chart.setLabel("left", "Temperature", units="°C")
    chart.setLabel("bottom", "Time", units="s")
    chart.showGrid(x=True, y=True, alpha=0.25)
    chart.addLegend()

    times = [item["time_s"] for item in records]
    heating = [
        item["temperature_C"] if item["heat_cool"] == 1 else math.nan
        for item in records
    ]
    cooling = [
        item["temperature_C"] if item["heat_cool"] == 0 else math.nan
        for item in records
    ]
    chart.plot(times, heating, pen=pg.mkPen(HEAT_COLOR, width=2), name="Heating")
    chart.plot(times, cooling, pen=pg.mkPen(COOL_COLOR, width=2), name="Cooling")
    chart.resize(1200, 700)

    chart_path = trial_dir / "strip_chart.png"
    ImageExporter(chart.plotItem).export(str(chart_path))

    data_path = trial_dir / "trace.csv"
    with data_path.open("w", newline="") as data_file:
        writer = csv.writer(data_file)
        writer.writerow(
            [
                "time_s",
                "temperature_C",
                "setpoint_C",
                "kp_pwm_per_C",
                "pwm",
                "heat_cool",
                "signed_pwm_count",
            ]
        )
        writer.writerows(
            [
                item["time_s"],
                item["temperature_C"],
                item["setpoint_C"],
                item["kp_pwm_per_C"],
                item["pwm"],
                item["heat_cool"],
                item["pwm"] if item["heat_cool"] == 1 else -item["pwm"],
            ]
            for item in records
        )

    report_path = trial_dir / "analysis.txt"
    report_path.write_text(
        format_strip_chart_report(trial_number, analysis) + "\n",
        encoding="utf-8",
    )
    chart.close()
    return chart_path


class SerialWorker(QtCore.QThread):
    line_received = QtCore.Signal(str)
    status_changed = QtCore.Signal(str, bool)

    def __init__(self, port: str, baud: int):
        super().__init__()
        self.port = port
        self.baud = baud
        self.commands = queue.Queue()
        self.stop_requested = threading.Event()

    def send_command(self, pwm: int, direction: str):
        self.commands.put(f"SET PWM {pwm} DIR {direction}\n")

    def stop(self):
        self.send_command(0, "HEAT")
        self.stop_requested.set()
        self.wait(2500)

    def run(self):
        try:
            with serial.Serial(
                self.port,
                self.baud,
                timeout=0.1,
                write_timeout=1.0,
            ) as connection:
                self.stop_requested.wait(2.0)
                if self.stop_requested.is_set():
                    return
                connection.reset_input_buffer()
                self.status_changed.emit(f"Connected: {self.port} at {self.baud} baud", True)

                while True:
                    while True:
                        try:
                            command = self.commands.get_nowait()
                        except queue.Empty:
                            break
                        connection.write(command.encode("ascii"))

                    if self.stop_requested.is_set():
                        break

                    raw_line = connection.readline()
                    if raw_line:
                        self.line_received.emit(raw_line.decode("utf-8", errors="replace").strip())
        except (serial.SerialException, OSError) as exc:
            self.status_changed.emit(f"Serial unavailable: {exc}", False)


class CalibrationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Module 5 TEC Calibration")
        self.resize(1050, 760)
        self.records = deque(maxlen=6000)
        self.run_records = []
        self.strip_chart_trial_number = next_strip_chart_trial_number(STRIP_CHART_DIR)
        self.csv_file = None
        self.closing = False
        self.serial_ready = False
        self.data_collection_paused = False
        self.latest_temperature_c = None
        self.temperature_cutoff_tripped = False

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        controls = QtWidgets.QHBoxLayout()
        self.heat_button = QtWidgets.QRadioButton("Heat")
        self.cool_button = QtWidgets.QRadioButton("Cool")
        self.heat_button.setChecked(True)
        self.heat_button.setStyleSheet(f"color: {HEAT_COLOR}; font-weight: 600")
        self.cool_button.setStyleSheet(f"color: {COOL_COLOR}; font-weight: 600")
        direction_group = QtWidgets.QButtonGroup(self)
        direction_group.setExclusive(True)
        direction_group.addButton(self.heat_button)
        direction_group.addButton(self.cool_button)

        controls.addWidget(QtWidgets.QLabel("Direction"))
        controls.addWidget(self.heat_button)
        controls.addWidget(self.cool_button)
        controls.addSpacing(16)
        controls.addWidget(QtWidgets.QLabel("PWM"))

        self.pwm_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.pwm_slider.setRange(0, 255)
        self.pwm_slider.setValue(0)
        self.pwm_slider.setMinimumWidth(220)
        controls.addWidget(self.pwm_slider, 1)

        self.pwm_value = QtWidgets.QLabel("0")
        self.pwm_value.setMinimumWidth(36)
        self.pwm_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
        controls.addWidget(self.pwm_value)

        self.stop_button = QtWidgets.QPushButton("PWM 0")
        controls.addWidget(self.stop_button)
        controls.addStretch(1)
        self.pause_collection_button = QtWidgets.QPushButton("Pause")
        self.pause_collection_button.setCheckable(True)
        self.pause_collection_button.setFixedWidth(72)
        self.pause_collection_button.setToolTip(
            "Pause saving and plotting measurements; temperature safety and control remain active."
        )
        controls.addWidget(self.pause_collection_button)
        layout.addLayout(controls)

        control_mode = QtWidgets.QHBoxLayout()
        self.proportional_checkbox = QtWidgets.QCheckBox(
            f"P-only control (Kp = {KP_PWM_PER_C:g} PWM/°C)"
        )
        self.proportional_checkbox.setChecked(True)
        self.setpoint_value = QtWidgets.QDoubleSpinBox()
        self.setpoint_value.setRange(-50.0, 150.0)
        self.setpoint_value.setDecimals(1)
        self.setpoint_value.setSuffix(" °C")
        self.setpoint_value.setValue(33.0)
        self.setpoint_value.setEnabled(False)
        self.kp_value = QtWidgets.QDoubleSpinBox()
        self.kp_value.setRange(0.0, 100.0)
        self.kp_value.setSingleStep(0.5)
        self.kp_value.setDecimals(1)
        self.kp_value.setSuffix(" PWM/°C")
        self.kp_value.setValue(KP_PWM_PER_C)
        self.kp_value.setEnabled(False)
        control_mode.addWidget(self.proportional_checkbox)
        control_mode.addSpacing(16)
        control_mode.addWidget(QtWidgets.QLabel("Setpoint"))
        control_mode.addWidget(self.setpoint_value)
        control_mode.addSpacing(16)
        control_mode.addWidget(QtWidgets.QLabel("Kp"))
        control_mode.addWidget(self.kp_value)
        control_mode.addStretch(1)
        layout.addLayout(control_mode)

        readouts = QtWidgets.QHBoxLayout()
        self.temperature_value = QtWidgets.QLabel("Temperature: -- °C")
        self.time_value = QtWidgets.QLabel("Time: -- s")
        self.measured_pwm_value = QtWidgets.QLabel("Measured PWM: --")
        self.measured_direction = QtWidgets.QLabel("Direction: --")
        self.error_value = QtWidgets.QLabel("Error (T_set − T): -- °C")
        readouts.addWidget(self.temperature_value)
        readouts.addWidget(self.time_value)
        readouts.addWidget(self.measured_pwm_value)
        readouts.addWidget(self.measured_direction)
        readouts.addWidget(self.error_value)
        layout.addLayout(readouts)

        safety_controls = QtWidgets.QHBoxLayout()
        self.temperature_safety_value = QtWidgets.QLabel(
            f"Temperature safety cutoff: {MIN_SAFE_TEMPERATURE_C:g}–"
            f"{MAX_SAFE_TEMPERATURE_C:g} °C"
        )
        self.reset_cutoff_button = QtWidgets.QPushButton("Reset temperature cutoff")
        self.reset_cutoff_button.setEnabled(False)
        safety_controls.addWidget(self.temperature_safety_value)
        safety_controls.addStretch(1)
        safety_controls.addWidget(self.reset_cutoff_button)
        layout.addLayout(safety_controls)

        self.temperature_plot = pg.PlotWidget(title="Temperature vs Time")
        self.temperature_plot.setLabel("left", "Temperature", units="°C")
        self.temperature_plot.setLabel("bottom", "Time", units="s")
        self.temperature_plot.showGrid(x=True, y=True, alpha=0.25)
        self.temperature_plot.enableAutoRange(axis="y", enable=True)
        self.temperature_plot.addLegend()
        self.heat_temperature_curve = self.temperature_plot.plot(
            pen=pg.mkPen(HEAT_COLOR, width=2), name="Heating"
        )
        self.cool_temperature_curve = self.temperature_plot.plot(
            pen=pg.mkPen(COOL_COLOR, width=2), name="Cooling"
        )
        layout.addWidget(self.temperature_plot, 3)

        self.pwm_plot = pg.PlotWidget(title="PWM vs Time")
        self.pwm_plot.setLabel("left", "PWM", units="count")
        self.pwm_plot.setLabel("bottom", "Time", units="s")
        self.pwm_plot.setYRange(0, 255, padding=0)
        self.pwm_plot.showGrid(x=True, y=True, alpha=0.25)
        self.pwm_plot.addLegend()
        self.heat_pwm_curve = self.pwm_plot.plot(
            pen=pg.mkPen(HEAT_COLOR, width=2), name="Heating"
        )
        self.cool_pwm_curve = self.pwm_plot.plot(
            pen=pg.mkPen(COOL_COLOR, width=2), name="Cooling"
        )
        layout.addWidget(self.pwm_plot, 2)

        self.status_label = QtWidgets.QLabel(f"Connecting to {SERIAL_PORT}...")
        self.statusBar().addWidget(self.status_label, 1)

        self.command_timer = QtCore.QTimer(self)
        self.command_timer.setSingleShot(True)
        self.command_timer.setInterval(120)
        self.command_timer.timeout.connect(self.send_current_command)

        self.pwm_slider.valueChanged.connect(self.on_pwm_changed)
        self.heat_button.toggled.connect(self.on_direction_changed)
        self.cool_button.toggled.connect(self.on_direction_changed)
        self.proportional_checkbox.toggled.connect(self.on_proportional_toggled)
        self.setpoint_value.valueChanged.connect(self.on_setpoint_changed)
        self.kp_value.valueChanged.connect(self.on_kp_changed)
        self.stop_button.clicked.connect(self.stop_output)
        self.pause_collection_button.toggled.connect(self.on_collection_paused)
        self.reset_cutoff_button.clicked.connect(self.reset_temperature_cutoff)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(PLOT_UPDATE_MS)

        self.serial_worker = SerialWorker(SERIAL_PORT, BAUD_RATE)
        self.serial_worker.line_received.connect(self.on_serial_line)
        self.serial_worker.status_changed.connect(self.on_serial_status)
        self.serial_worker.start()
        self.set_controls_enabled(False)

    def set_controls_enabled(self, enabled: bool):
        controls_available = enabled and not self.temperature_cutoff_tripped
        manual_enabled = controls_available and not self.proportional_checkbox.isChecked()
        self.heat_button.setEnabled(manual_enabled)
        self.cool_button.setEnabled(manual_enabled)
        self.pwm_slider.setEnabled(manual_enabled)
        self.proportional_checkbox.setEnabled(controls_available)
        self.setpoint_value.setEnabled(
            controls_available and self.proportional_checkbox.isChecked()
        )
        self.kp_value.setEnabled(
            controls_available and self.proportional_checkbox.isChecked()
        )
        self.stop_button.setEnabled(enabled)
        temperature_is_safe = (
            self.latest_temperature_c is not None
            and MIN_SAFE_TEMPERATURE_C <= self.latest_temperature_c <= MAX_SAFE_TEMPERATURE_C
        )
        self.reset_cutoff_button.setEnabled(
            enabled and self.temperature_cutoff_tripped and temperature_is_safe
        )

    def current_direction(self) -> str:
        return "HEAT" if self.heat_button.isChecked() else "COOL"

    @QtCore.Slot(bool)
    def on_collection_paused(self, paused: bool):
        self.data_collection_paused = paused
        self.pause_collection_button.setText("Resume" if paused else "Pause")

    @QtCore.Slot(int)
    def on_pwm_changed(self, value: int):
        self.pwm_value.setText(str(value))
        self.command_timer.start()

    @QtCore.Slot(bool)
    def on_direction_changed(self, checked: bool):
        if checked:
            self.send_current_command()

    @QtCore.Slot()
    def stop_output(self):
        if self.temperature_cutoff_tripped:
            if self.serial_ready and self.serial_worker.isRunning():
                self.serial_worker.send_command(0, "HEAT")
            return
        self.pwm_slider.setValue(0)
        self.command_timer.stop()
        if self.proportional_checkbox.isChecked():
            self.proportional_checkbox.setChecked(False)
            return
        self.send_current_command()

    @QtCore.Slot(bool)
    def on_proportional_toggled(self, enabled: bool):
        self.command_timer.stop()
        controls_available = self.serial_ready and not self.temperature_cutoff_tripped
        self.setpoint_value.setEnabled(enabled and controls_available)
        self.kp_value.setEnabled(enabled and controls_available)
        self.heat_button.setEnabled(controls_available and not enabled)
        self.cool_button.setEnabled(controls_available and not enabled)
        self.pwm_slider.setEnabled(controls_available and not enabled)
        if enabled:
            self.send_proportional_command(self.setpoint_value.value())
        else:
            self.send_current_command()

    @QtCore.Slot(float)
    def on_setpoint_changed(self, value: float):
        self.update_error_readout(value)
        if self.proportional_checkbox.isChecked():
            self.send_proportional_command(value)

    @QtCore.Slot(float)
    def on_kp_changed(self, value: float):
        if self.proportional_checkbox.isChecked():
            self.send_proportional_command(kp_pwm_per_c=value)

    @QtCore.Slot()
    def send_current_command(self):
        if (
            not self.temperature_cutoff_tripped
            and not self.proportional_checkbox.isChecked()
            and self.serial_ready
            and self.serial_worker.isRunning()
        ):
            self.serial_worker.send_command(self.pwm_slider.value(), self.current_direction())

    def update_error_readout(self, setpoint_c: float | None = None):
        if self.latest_temperature_c is None:
            self.error_value.setText("Error (T_set − T): -- °C")
            return

        target = self.setpoint_value.value() if setpoint_c is None else setpoint_c
        error = target - self.latest_temperature_c
        self.error_value.setText(f"Error (T_set − T): {error:.2f} °C")

    def send_proportional_command(
        self,
        setpoint_c: float | None = None,
        kp_pwm_per_c: float | None = None,
    ):
        if self.temperature_cutoff_tripped or not self.proportional_checkbox.isChecked():
            return
        if not self.serial_ready or not self.serial_worker.isRunning():
            return

        if self.latest_temperature_c is None:
            self.serial_worker.send_command(0, "HEAT")
            return

        pwm, direction = calculate_proportional_command(
            self.latest_temperature_c,
            self.setpoint_value.value() if setpoint_c is None else setpoint_c,
            self.kp_value.value() if kp_pwm_per_c is None else kp_pwm_per_c,
        )
        self.serial_worker.send_command(pwm, direction)

    def trip_temperature_cutoff(self, temperature_c: float):
        if not self.temperature_cutoff_tripped:
            self.temperature_cutoff_tripped = True
            self.temperature_safety_value.setText(
                f"TEMPERATURE CUTOFF at {temperature_c:.2f} °C "
                f"(safe range {MIN_SAFE_TEMPERATURE_C:g}–{MAX_SAFE_TEMPERATURE_C:g} °C)"
            )
            if self.serial_ready and self.serial_worker.isRunning():
                self.serial_worker.send_command(0, "HEAT")
        self.set_controls_enabled(self.serial_ready)

    @QtCore.Slot()
    def reset_temperature_cutoff(self):
        if not self.temperature_cutoff_tripped or self.latest_temperature_c is None:
            return
        if not MIN_SAFE_TEMPERATURE_C <= self.latest_temperature_c <= MAX_SAFE_TEMPERATURE_C:
            return

        self.temperature_cutoff_tripped = False
        self.temperature_safety_value.setText(
            f"Temperature safety cutoff: {MIN_SAFE_TEMPERATURE_C:g}–"
            f"{MAX_SAFE_TEMPERATURE_C:g} °C"
        )
        self.set_controls_enabled(self.serial_ready)
        if self.proportional_checkbox.isChecked():
            self.send_proportional_command()
        else:
            self.send_current_command()

    @QtCore.Slot(str, bool)
    def on_serial_status(self, message: str, ready: bool):
        if self.closing:
            return
        self.serial_ready = ready
        self.status_label.setText(message)
        self.set_controls_enabled(ready)
        if ready:
            if self.proportional_checkbox.isChecked():
                self.send_proportional_command(self.setpoint_value.value())
            else:
                self.send_current_command()

    @QtCore.Slot(str)
    def on_serial_line(self, line: str):
        if self.closing:
            return
        measurement = parse_measurement(line)
        if measurement is None:
            return

        self.latest_temperature_c = measurement["temperature_C"]
        self.update_error_readout()
        temperature_c = measurement["temperature_C"]
        if temperature_c < MIN_SAFE_TEMPERATURE_C or temperature_c > MAX_SAFE_TEMPERATURE_C:
            self.trip_temperature_cutoff(temperature_c)
        else:
            self.set_controls_enabled(self.serial_ready)
        if self.proportional_checkbox.isChecked():
            self.send_proportional_command(self.setpoint_value.value())

        if not self.data_collection_paused:
            measurement["setpoint_C"] = self.setpoint_value.value()
            measurement["kp_pwm_per_C"] = self.kp_value.value()
            self.records.append(measurement)
            self.run_records.append(measurement.copy())
        direction = "HEAT" if measurement["heat_cool"] == 1 else "COOL"
        color = HEAT_COLOR if measurement["heat_cool"] == 1 else COOL_COLOR
        self.temperature_value.setText(f"Temperature: {measurement['temperature_C']:.2f} °C")
        self.time_value.setText(f"Time: {measurement['time_s']:.2f} s")
        self.measured_pwm_value.setText(f"Measured PWM: {measurement['pwm']}")
        self.measured_direction.setText(f"Direction: {direction}")
        self.measured_direction.setStyleSheet(f"color: {color}; font-weight: 600")
        if not self.data_collection_paused:
            self.write_csv_row(measurement)

    def write_csv_row(self, measurement: dict):
        if self.csv_file is None:
            CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
            is_empty = not CSV_PATH.exists() or CSV_PATH.stat().st_size == 0
            self.csv_file = CSV_PATH.open("a", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            if is_empty:
                self.csv_writer.writerow(["time_s", "temperature_C", "pwm", "heat_cool"])

        self.csv_writer.writerow(
            [
                measurement["time_s"],
                measurement["temperature_C"],
                measurement["pwm"],
                measurement["heat_cool"],
            ]
        )
        self.csv_file.flush()

    def update_plots(self):
        if not self.records:
            return

        latest_time = self.records[-1]["time_s"]
        visible = [
            item for item in self.records
            if latest_time - WINDOW_SECONDS <= item["time_s"] <= latest_time
        ]
        times = [item["time_s"] for item in visible]
        heat_temperatures = [
            item["temperature_C"] if item["heat_cool"] == 1 else math.nan
            for item in visible
        ]
        cool_temperatures = [
            item["temperature_C"] if item["heat_cool"] == 0 else math.nan
            for item in visible
        ]
        heat_pwm = [item["pwm"] if item["heat_cool"] == 1 else math.nan for item in visible]
        cool_pwm = [item["pwm"] if item["heat_cool"] == 0 else math.nan for item in visible]
        self.heat_temperature_curve.setData(times, heat_temperatures)
        self.cool_temperature_curve.setData(times, cool_temperatures)
        self.heat_pwm_curve.setData(times, heat_pwm)
        self.cool_pwm_curve.setData(times, cool_pwm)

        start_time = max(0.0, latest_time - WINDOW_SECONDS)
        self.temperature_plot.setXRange(start_time, latest_time, padding=0)
        self.pwm_plot.setXRange(start_time, latest_time, padding=0)

    def closeEvent(self, event):
        self.closing = True
        self.command_timer.stop()
        self.plot_timer.stop()
        self.serial_worker.stop()
        if self.csv_file is not None:
            self.csv_file.close()
        create_strip_chart(
            self.run_records,
            self.strip_chart_trial_number,
            STRIP_CHART_DIR,
        )
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = CalibrationWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()