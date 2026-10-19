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
from datetime import datetime
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


SERIAL_PORT = "/dev/cu.usbmodem1101"
BAUD_RATE = 9600
WINDOW_SECONDS = 60.0
PLOT_UPDATE_MS = 200
CSV_PATH = Path(__file__).resolve().parents[1] / "data" / "module_05" / "tec_calibration.csv"
KP_PWM_PER_C = 10.0
GAIN_TRIAL_VALUES = (0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 20.0, 30.0)

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
        self.csv_file = None
        self.closing = False
        self.serial_ready = False
        self.latest_temperature_c = None
        self.temperature_cutoff_tripped = False
        self.active_gain_trial = None
        self.gain_trial_results = []
        self.completed_gain_trials = set()

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

        experiment_group = QtWidgets.QGroupBox("Droop-versus-gain experiment")
        experiment_layout = QtWidgets.QGridLayout(experiment_group)
        self.trial_gain_value = QtWidgets.QComboBox()
        for gain in GAIN_TRIAL_VALUES:
            self.trial_gain_value.addItem(f"{gain:g} PWM/°C", gain)
        self.ambient_value = QtWidgets.QDoubleSpinBox()
        self.ambient_value.setRange(-50.0, 150.0)
        self.ambient_value.setDecimals(1)
        self.ambient_value.setSuffix(" °C")
        self.ambient_value.setValue(25.0)
        self.susceptibility_value = QtWidgets.QDoubleSpinBox()
        self.susceptibility_value.setRange(0.0, 100.0)
        self.susceptibility_value.setDecimals(4)
        self.susceptibility_value.setSingleStep(0.001)
        self.susceptibility_value.setSuffix(" °C/PWM")
        self.trial_prediction = QtWidgets.QLabel()
        self.trial_outcome = QtWidgets.QComboBox()
        self.trial_outcome.addItems(["Settled", "Failed to settle"])
        self.start_trial_button = QtWidgets.QPushButton("Start gain trial")
        self.record_trial_button = QtWidgets.QPushButton("Record result and trace")
        self.record_trial_button.setEnabled(False)
        self.show_droop_button = QtWidgets.QPushButton("Plot droop vs Kp")
        self.trial_status = QtWidgets.QLabel("No gain trial recorded.")

        experiment_layout.addWidget(QtWidgets.QLabel("Trial Kp"), 0, 0)
        experiment_layout.addWidget(self.trial_gain_value, 0, 1)
        experiment_layout.addWidget(QtWidgets.QLabel("Ambient temperature"), 0, 2)
        experiment_layout.addWidget(self.ambient_value, 0, 3)
        experiment_layout.addWidget(QtWidgets.QLabel("Module 4 susceptibility"), 1, 0)
        experiment_layout.addWidget(self.susceptibility_value, 1, 1)
        experiment_layout.addWidget(self.trial_prediction, 1, 2, 1, 2)
        experiment_layout.addWidget(self.trial_outcome, 2, 0)
        experiment_layout.addWidget(self.start_trial_button, 2, 1)
        experiment_layout.addWidget(self.record_trial_button, 2, 2)
        experiment_layout.addWidget(self.show_droop_button, 2, 3)
        experiment_layout.addWidget(self.trial_status, 3, 0, 1, 4)
        layout.addWidget(experiment_group)
        self.update_trial_prediction()

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
        self.reset_cutoff_button.clicked.connect(self.reset_temperature_cutoff)
        self.trial_gain_value.currentIndexChanged.connect(self.update_trial_prediction)
        self.ambient_value.valueChanged.connect(self.update_trial_prediction)
        self.setpoint_value.valueChanged.connect(self.update_trial_prediction)
        self.susceptibility_value.valueChanged.connect(self.update_trial_prediction)
        self.start_trial_button.clicked.connect(self.start_gain_trial)
        self.record_trial_button.clicked.connect(self.record_gain_trial)
        self.show_droop_button.clicked.connect(self.show_droop_plot)

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
        trial_idle = self.active_gain_trial is None
        self.proportional_checkbox.setEnabled(controls_available and trial_idle)
        self.setpoint_value.setEnabled(
            controls_available and trial_idle and self.proportional_checkbox.isChecked()
        )
        self.kp_value.setEnabled(
            controls_available and trial_idle and self.proportional_checkbox.isChecked()
        )
        self.stop_button.setEnabled(enabled)
        temperature_is_safe = (
            self.latest_temperature_c is not None
            and MIN_SAFE_TEMPERATURE_C <= self.latest_temperature_c <= MAX_SAFE_TEMPERATURE_C
        )
        self.reset_cutoff_button.setEnabled(
            enabled and self.temperature_cutoff_tripped and temperature_is_safe
        )
        self.trial_gain_value.setEnabled(trial_idle)
        self.ambient_value.setEnabled(trial_idle)
        self.susceptibility_value.setEnabled(trial_idle)
        self.update_trial_prediction()

    def current_direction(self) -> str:
        return "HEAT" if self.heat_button.isChecked() else "COOL"

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

    @QtCore.Slot()
    def update_trial_prediction(self):
        gain = float(self.trial_gain_value.currentData())
        initial_error = self.setpoint_value.value() - self.ambient_value.value()
        initial_pwm = gain * abs(initial_error)
        prediction = f"Predicted P₀: {initial_pwm:.1f} PWM"
        susceptibility = self.susceptibility_value.value()
        if susceptibility > 0:
            predicted_droop = initial_error / (1 + susceptibility * gain)
            prediction += f"    Model droop: {predicted_droop:.2f} °C"
        self.trial_prediction.setText(prediction)
        self.start_trial_button.setEnabled(
            self.active_gain_trial is None
            and gain not in self.completed_gain_trials
            and self.serial_ready
            and self.latest_temperature_c is not None
            and not self.temperature_cutoff_tripped
        )

    @QtCore.Slot()
    def start_gain_trial(self):
        if (
            not self.serial_ready
            or not self.serial_worker.isRunning()
            or self.latest_temperature_c is None
            or self.temperature_cutoff_tripped
            or self.active_gain_trial is not None
        ):
            return

        gain = float(self.trial_gain_value.currentData())
        if gain in self.completed_gain_trials:
            self.trial_status.setText(f"Kp={gain:g} has already been recorded in this test.")
            return
        setpoint = self.setpoint_value.value()
        ambient = self.ambient_value.value()
        initial_pwm = gain * abs(setpoint - ambient)

        self.command_timer.stop()
        self.pwm_slider.setValue(0)
        self.command_timer.stop()
        self.heat_button.setChecked(True)
        if self.proportional_checkbox.isChecked():
            self.proportional_checkbox.setChecked(False)
        else:
            self.serial_worker.send_command(0, "HEAT")
        self.kp_value.setValue(gain)
        self.proportional_checkbox.setChecked(True)

        self.active_gain_trial = {
            "gain": gain,
            "initial_pwm": initial_pwm,
            "setpoint": setpoint,
            "ambient": ambient,
            "initial_error": setpoint - ambient,
            "susceptibility": self.susceptibility_value.value(),
            "start_time_s": self.records[-1]["time_s"] if self.records else 0.0,
            "started_at": datetime.now().strftime("%Y%m%d_%H%M%S_%f"),
        }
        self.start_trial_button.setEnabled(False)
        self.record_trial_button.setEnabled(True)
        self.set_controls_enabled(self.serial_ready)
        self.trial_status.setText(
            f"Running Kp={gain:g}; predicted initial PWM P₀={initial_pwm:.1f}. "
            "Wait for settling or mark failure to settle."
        )

    @QtCore.Slot()
    def record_gain_trial(self):
        trial = self.active_gain_trial
        if trial is None or not self.records:
            return

        trace = [
            item for item in self.records
            if item["time_s"] > trial["start_time_s"]
        ]
        if not trace:
            self.trial_status.setText("No trace measurements available; trial remains active.")
            return

        final_measurement = trace[-1]
        final_temperature = final_measurement["temperature_C"]
        final_error = trial["setpoint"] - final_temperature
        result = {
            **trial,
            "outcome": self.trial_outcome.currentText(),
            "final_temperature": final_temperature,
            "error": final_error,
            "pwm": final_measurement["pwm"],
        }

        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        trace_path = CSV_PATH.parent / f"kp_trial_{trial['started_at']}_trace.csv"
        with trace_path.open("w", newline="") as trace_file:
            writer = csv.writer(trace_file)
            writer.writerow(["time_s", "temperature_C", "pwm", "heat_cool"])
            writer.writerows(
                [
                    item["time_s"],
                    item["temperature_C"],
                    item["pwm"],
                    item["heat_cool"],
                ]
                for item in trace
            )

        results_path = CSV_PATH.parent / "kp_droop_results.csv"
        is_empty = not results_path.exists() or results_path.stat().st_size == 0
        with results_path.open("a", newline="") as results_file:
            writer = csv.writer(results_file)
            if is_empty:
                writer.writerow(
                    [
                        "started_at",
                        "kp_pwm_per_c",
                        "predicted_initial_pwm",
                        "setpoint_C",
                        "ambient_C",
                        "initial_error_C",
                        "module4_susceptibility_C_per_pwm",
                        "outcome",
                        "final_temperature_C",
                        "error_C",
                        "measured_pwm",
                        "trace_file",
                    ]
                )
            writer.writerow(
                [
                    result["started_at"],
                    result["gain"],
                    result["initial_pwm"],
                    result["setpoint"],
                    result["ambient"],
                    result["initial_error"],
                    result["susceptibility"],
                    result["outcome"],
                    result["final_temperature"],
                    result["error"],
                    result["pwm"],
                    trace_path.name,
                ]
            )

        self.gain_trial_results.append(result)
        self.completed_gain_trials.add(result["gain"])
        self.active_gain_trial = None
        self.record_trial_button.setEnabled(False)
        self.set_controls_enabled(self.serial_ready)
        next_gain_index = next(
            (
                index for index, gain in enumerate(GAIN_TRIAL_VALUES)
                if gain not in self.completed_gain_trials
            ),
            None,
        )
        if next_gain_index is not None:
            self.trial_gain_value.setCurrentIndex(next_gain_index)
        self.update_trial_prediction()
        if next_gain_index is None:
            self.trial_status.setText(
                f"All {len(GAIN_TRIAL_VALUES)} gains recorded. Last trial: "
                f"{result['outcome'].lower()}, final temperature {final_temperature:.2f} °C, "
                f"error {final_error:.2f} °C, PWM {result['pwm']}. "
                f"Trace: {trace_path.name}"
            )
        else:
            self.trial_status.setText(
                f"Saved {result['outcome'].lower()} trial: final temperature "
                f"{final_temperature:.2f} °C, error {final_error:.2f} °C, "
                f"PWM {result['pwm']}. Trace: {trace_path.name}"
            )

    @QtCore.Slot()
    def show_droop_plot(self):
        if not self.gain_trial_results:
            self.trial_status.setText("Record at least one gain trial before plotting droop.")
            return

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Measured droop versus proportional gain")
        dialog.resize(800, 500)
        dialog_layout = QtWidgets.QVBoxLayout(dialog)
        plot = pg.PlotWidget()
        plot.setLabel("bottom", "Proportional gain, Kp", units="PWM/°C")
        plot.setLabel("left", "Droop (setpoint − final temperature)", units="°C")
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend()

        for outcome, symbol, color in (
            ("Settled", "o", "#2475a8"),
            ("Failed to settle", "x", "#c43b32"),
        ):
            selected = [
                result for result in self.gain_trial_results
                if result["outcome"] == outcome
            ]
            if selected:
                selected.sort(key=lambda result: result["gain"])
                plot.plot(
                    [result["gain"] for result in selected],
                    [result["error"] for result in selected],
                    pen=None,
                    symbol=symbol,
                    symbolBrush=color,
                    symbolPen=color,
                    name=f"Measured ({outcome.lower()})",
                )

        modeled = [
            result for result in self.gain_trial_results
            if result["susceptibility"] > 0
        ]
        if modeled:
            modeled.sort(key=lambda result: result["gain"])
            plot.plot(
                [result["gain"] for result in modeled],
                [
                    result["initial_error"]
                    / (1 + result["susceptibility"] * result["gain"])
                    for result in modeled
                ],
                pen=pg.mkPen("#6b4c9a", width=2),
                symbol="s",
                symbolBrush="#6b4c9a",
                name="Module 4 model",
            )

        dialog_layout.addWidget(plot)
        dialog.exec()

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
        self.update_trial_prediction()
        self.update_error_readout()
        temperature_c = measurement["temperature_C"]
        if temperature_c < MIN_SAFE_TEMPERATURE_C or temperature_c > MAX_SAFE_TEMPERATURE_C:
            self.trip_temperature_cutoff(temperature_c)
        else:
            self.set_controls_enabled(self.serial_ready)
        if self.proportional_checkbox.isChecked():
            self.send_proportional_command(self.setpoint_value.value())

        self.records.append(measurement)
        direction = "HEAT" if measurement["heat_cool"] == 1 else "COOL"
        color = HEAT_COLOR if measurement["heat_cool"] == 1 else COOL_COLOR
        self.temperature_value.setText(f"Temperature: {measurement['temperature_C']:.2f} °C")
        self.time_value.setText(f"Time: {measurement['time_s']:.2f} s")
        self.measured_pwm_value.setText(f"Measured PWM: {measurement['pwm']}")
        self.measured_direction.setText(f"Direction: {direction}")
        self.measured_direction.setStyleSheet(f"color: {color}; font-weight: 600")
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
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = CalibrationWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()