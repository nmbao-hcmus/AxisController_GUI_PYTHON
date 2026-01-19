# app.py
import os
import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox,QFileDialog, QDialog, QVBoxLayout, QTextEdit, QPushButton,  QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QGraphicsLineItem
from PyQt6.QtCore import QThread, pyqtSignal, QObject,Qt
from PyQt6.QtGui import QPen, QColor,QPainter
from PyQt6.uic import loadUi

# CLASS: SerialWorker
# Handles all serial communication in a separate thread to prevent UI freezing.
class SerialWorker(QObject):
    """
    Worker thread for handling serial port reading.
    Emits a signal with the data received.
    """
    data_received = pyqtSignal(str)

    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self._is_running = True

    def run(self):
        """Main loop for the worker thread."""
        while self._is_running and self.serial_port.is_open:
            try:
                # Add errors='ignore' to the decode() function
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.data_received.emit(line)
            except serial.SerialException:
                # Port might have been disconnected
                break
        print("Serial worker thread finished.")

    def stop(self):
        """Stops the worker thread."""
        self._is_running = False
# ------------------Preset Widgets------------------




# ----------------- GCODE CLEANER WINDOW -----------------
import re
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QFileDialog, QCheckBox, QTextEdit, QMessageBox, QSplitter
)
from PyQt6.QtCore import Qt
def process_gcode_text(content: str, remove_s=True, remove_m=True, remove_f=True, remove_z=True):
    lines = content.splitlines()
    output_lines = []

    for line in lines:
        line = line.strip()
        if not line:
            continue
        if not re.search(r'\bG0\b|\bG1\b|X|Y|Z', line):
            continue
        if remove_s:
            line = re.sub(r'S\d*\.?\d*', '', line)
        if remove_m:
            line = re.sub(r'M\d*', '', line)
        if remove_f:
            line = re.sub(r'F\d*\.?\d*', '', line)
        if remove_z:
            line = re.sub(r'Z-?\d*\.?\d*', '', line)

        if not re.search(r'X|Y|Z', line):
            continue
        if not re.search(r'\bG0\b|\bG1\b', line) and re.search(r'X|Y', line):
            line = 'G0 ' + line

        x_match = re.search(r'X(-?\d+\.?\d*)', line)
        y_match = re.search(r'Y(-?\d+\.?\d*)', line)
        z_match = re.search(r'Z(-?\d+\.?\d*)', line) if not remove_z else None

        x_val = f"X{x_match.group(1)}" if x_match else ""
        y_val = f"Y{y_match.group(1)}" if y_match else ""
        z_val = f"Z{z_match.group(1)}" if z_match else ""

        g_code_match = re.search(r'\b(G0|G1)\b', line)
        g_code = g_code_match.group(1) if g_code_match else "G0"

        new_line = f"{g_code}"
        if x_val:
            new_line += f" {x_val}"
        if y_val:
            new_line += f" {y_val}"
        if z_val:
            new_line += f" {z_val}"

        output_lines.append(new_line.strip())

    return "\n".join(output_lines)

class GCodeCleanerApp(QWidget):
    def __init__(self, main_window=None):
        super().__init__()
        self.main_window = main_window  # để log ngược về main
        self.setWindowTitle("🧩 GCode Cleaner - So sánh trước / sau xử lý")
        self.setGeometry(200, 150, 950, 600)
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()
        hl_input = QHBoxLayout()
        hl_input.addWidget(QLabel("File đầu vào:"))
        self.in_edit = QLineEdit()
        btn_in = QPushButton("Chọn...")
        btn_in.clicked.connect(self.select_input)
        hl_input.addWidget(self.in_edit)
        hl_input.addWidget(btn_in)
        main_layout.addLayout(hl_input)

        opt_layout = QHBoxLayout()
        self.cb_s = QCheckBox("Bỏ S"); self.cb_s.setChecked(True)
        self.cb_m = QCheckBox("Bỏ M"); self.cb_m.setChecked(True)
        self.cb_f = QCheckBox("Bỏ F"); self.cb_f.setChecked(True)
        self.cb_z = QCheckBox("Bỏ Z"); self.cb_z.setChecked(True)

        for cb in (self.cb_s, self.cb_m, self.cb_f, self.cb_z):
            cb.stateChanged.connect(self.auto_reprocess)
            opt_layout.addWidget(cb)
        main_layout.addLayout(opt_layout)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        self.text_before = QTextEdit(); self.text_after = QTextEdit()
        self.text_before.setReadOnly(True); self.text_after.setReadOnly(True)
        self.text_before.setPlaceholderText("📄 Nội dung gốc (trước khi xử lý)")
        self.text_after.setPlaceholderText("⚙️ Kết quả sau xử lý")
        splitter.addWidget(self.text_before); splitter.addWidget(self.text_after)
        splitter.setSizes([450, 450])
        main_layout.addWidget(splitter)

        btn_save = QPushButton("💾 Lưu kết quả...")
        btn_save.clicked.connect(self.save_result)
        btn_save.setStyleSheet("background-color:#0078D7; color:white; font-weight:bold; height:35px;")
        main_layout.addWidget(btn_save)

        self.setLayout(main_layout)

    # ---- Các hành động ----
    def select_input(self):
        path, _ = QFileDialog.getOpenFileName(self, "Chọn file G-code", "", "")
        if not path: return
        self.load_file(path)

    def load_file(self, path):
        self.in_edit.setText(path)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
            self.text_before.setPlainText(content)
            processed = self.process_current(content)
            self.text_after.setPlainText(processed)
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", str(e))

    def process_current(self, content: str):
        return process_gcode_text(
            content,
            remove_s=self.cb_s.isChecked(),
            remove_m=self.cb_m.isChecked(),
            remove_f=self.cb_f.isChecked(),
            remove_z=self.cb_z.isChecked(),
        )

    def auto_reprocess(self):
        content = self.text_before.toPlainText()
        if not content.strip():
            return
        processed = self.process_current(content)
        self.text_after.setPlainText(processed)

    def save_result(self):
        result = self.text_after.toPlainText().strip()
        if not result:
            QMessageBox.warning(self, "Cảnh báo", "Không có nội dung để lưu!")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Lưu file kết quả", "", "")
        if path:
            try:
                with open(path, 'w', encoding='utf-8') as f:
                    f.write(result)
                QMessageBox.information(self, "Thành công", f"Đã lưu kết quả tại:\n{path}")
                if self.main_window:
                    self.main_window.logTextEdit.append("--- Đã chuyển đổi G-code thành công ---")
            except Exception as e:
                QMessageBox.critical(self, "Lỗi", str(e))

# --- MAIN WINDOW ---
# CLASS: MainWindow
# The main application window and logic.
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # Load the UI file created with Qt Designer
        loadUi("Main_v7.ui", self) 
        #Debug variables
        self.x_test = 0
        self.y_test = 0

        # --- Trạng thái macro ---
        self.macro_running = False
        self.macro_commands = []
        self.is_waiting_next = False  # Thêm dòng này để chống gửi đè lệnh
        self.machine_state = "Idle"   # có thể là "Idle" hoặc "RunGCode"

        # --- GCode Previous Position Command ---
        self.target_x = None
        self.target_y = None
        self.target_z = None

        # --- XYZ Visualizer ---
        # (x, y) ─────────────┐ X+
        # │                   │
        # │      (ellipse)    │  
        # │                   │
        # └────────────────────
        #  Y+    

        # ===== Kích thước vùng làm việc =====
        self.work_width = 100000
        self.work_height = 100000
        # Enable trace line of tool movement
        self.enableTrace = 1  # 1: vẽ đường đi, 0: không vẽ
        # === Tạo scene ===
        self.scene = QGraphicsScene(0, -100, self.work_width, self.work_height)
        self.xyGraphicsView.setScene(self.scene)
        self.xyGraphicsView.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.xyGraphicsView.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

        # === Vẽ khung ngoài ===
        border_pen = QPen(Qt.GlobalColor.black, 1.5)
        self.scene.addRect(0, -100,  self.work_width,  self.work_height, border_pen)

        # === Vẽ trục X và Y ===
        axis_pen = QPen(Qt.GlobalColor.darkGray, 0.8)
        self.scene.addLine(0, 0,  self.work_width, 0, axis_pen)   # Trục X
        self.scene.addLine(0, 0, 0, -self.work_height, axis_pen)  # Trục Y

        # === Vẽ vạch chia (mỗi 10 đơn vị) ===
        grid_pen = QPen(QColor(220, 220, 220))
        grid_pen.setWidth(0)
        spacing = 10
        for x in range(0, 251, spacing):
            self.scene.addLine(x, 0, x, -100, grid_pen)
        for y in range(0, 101, spacing):
            self.scene.addLine(0, -y, 250, -y, grid_pen)

        # === Thêm chấm đỏ (đại diện vị trí đầu tool) ===
        self.tool_dot = QGraphicsEllipseItem(-3, -3, 6, 6)
        self.tool_dot.setBrush(QColor("red"))
        self.scene.addItem(self.tool_dot)

        # Biến lưu vị trí trước đó (để vẽ đường đi)
        self.last_pos = None

        # Serial communication attributes
        self.serial_port = None
        self.serial_thread = None
        self.serial_worker = None
        #for macro running
        self.command_queue = []     # This will hold our sequence of commands
        self.previous_state = "Unknown" # To detect state changes

        # Set focus policy to capture key events
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._populate_ports()
        self._connect_signals()
    def _populate_ports(self):
        """Find and list all available serial ports."""
        self.portComboBox.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.portComboBox.addItem(port.device)
    def _connect_signals(self):
        """Connect all UI widget signals to their corresponding slots."""
        self.connectButton.clicked.connect(self.toggle_serial_connection)
        self.refreshButton.clicked.connect(self._populate_ports)
        self.sendButton.clicked.connect(self.send_gcode_from_input)
        # --- Set Current Position Zero ---
        self.SetX0Button.clicked.connect(lambda: self.send_command("SET X\n"))
        self.SetY0Button.clicked.connect(lambda: self.send_command("SET Y\n"))
        self.SetZ0Button.clicked.connect(lambda: self.send_command("SET Z\n"))

        # --- Jogging Buttons ---
        self.jogXPlusButton.clicked.connect(lambda: self.send_command(f"G0 X{ float(self.xPosLabel.text()) + float(self.txt_input_intervalDistance.text()) } {self._get_feed_command_string()}\n"))
        self.jogXMinusButton.clicked.connect(lambda: self.send_command(f"G0 X{float( self.xPosLabel.text()) - float(self.txt_input_intervalDistance.text()) } {self._get_feed_command_string()}\n"))
        self.jogYPlusButton.clicked.connect(lambda: self.send_command(f"G0 Y{float( self.yPosLabel.text()) + float( self.txt_input_intervalDistance.text()) } {self._get_feed_command_string()}\n"))
        self.jogYMinusButton.clicked.connect(lambda: self.send_command(f"G0 Y{float( self.yPosLabel.text()) -  float(self.txt_input_intervalDistance.text()) } {self._get_feed_command_string()}\n"))
        self.jogZPlusButton.clicked.connect(lambda: self.send_command(f"G0 Z{float(self.zPosLabel.text()) +   float(self.txt_input_intervalDistance.text()) } {self._get_feed_command_string()}\n"))
        self.jogZMinusButton.clicked.connect(lambda: self.send_command(f"G0 Z{float(self.zPosLabel.text()) - float(self.txt_input_intervalDistance.text()) } {self._get_feed_command_string()}\n"))
        
        # ---  Set F Nhanh/Chậm ---
        self.slowButton.clicked.connect(lambda: self.txt_input_send_F.setText("10000"))
        self.fastButton.clicked.connect(lambda: self.txt_input_send_F.setText("27000")) 
        
        # --- Go to Preset Position Buttons ---
        # These buttons trigger the corresponding GOTO command on the STM32
        self.gotoAButton.clicked.connect(lambda: self.send_command("GOTO A\n")) # Assumes firmware handles "GOTO"
        self.gotoBButton.clicked.connect(lambda: self.send_command("GOTO B\n"))
        self.gotoCButton.clicked.connect(lambda: self.send_command("GOTO C\n"))
        self.gotoDButton.clicked.connect(lambda: self.send_command("GOTO D\n"))
        
        # --- Get Position ---
        self.getAButton.clicked.connect(lambda:self.send_command("RETURN A\n")) 
        self.getBButton.clicked.connect(lambda:self.send_command("RETURN B\n"))
        self.getCButton.clicked.connect(lambda:self.send_command("RETURN C\n"))
        self.getDButton.clicked.connect(lambda:self.send_command("RETURN D\n"))

        # --- Set Position Buttons ---
        self.setAButton.clicked.connect(self.set_preset_position_A)
        self.setBButton.clicked.connect(self.set_preset_position_B)
        self.setCButton.clicked.connect(self.set_preset_position_C)
        self.setDButton.clicked.connect(self.set_preset_position_D)

        # --- Get Current Position into GOTO ---
        self.CurrentPosButton.clicked.connect(self.get_current_position_into_goto)
        # --- Go to Manual Positioning ---
        self.btn_manual_goto.clicked.connect(self.send_manual_goto_command)

        # --- Stop operation ---
        self.Stopbtn.clicked.connect(lambda: self.send_command("STOP")) 
        
        # -- Import GCode Button (Deleted Button)---
        # self.importButton.clicked.connect(self.import_gcode_file)
        # --- Run GCode ---
        self.RunGCodeButton.clicked.connect(self.run_imported_gcode_file)
        # --- Macros ---
        self.runMacroButton.clicked.connect(self.random_move)  # For testing

        # --- Visualize XY View ---
        self.trace_button.clicked.connect(self.toggle_trace)  # For enabling/disabling trace lines
        self.clearGraphButton.clicked.connect(self.clear_xy_graph)
        # --- GCode Cleaner (Deleted Button)---
        # self.processGCodeButton.clicked.connect(self.open_gcode_cleaner)

        # ===== Toolbar =====
        self.actionImport.triggered.connect(self.import_gcode_file)
        self.actionProccessGCode.triggered.connect(self.open_gcode_cleaner)
        self.actionClearGraph.triggered.connect(self.clear_xy_graph)
        self.actionEnableDisable_Trace.triggered.connect(self.toggle_trace)

    def toggle_serial_connection(self):
        """Connects or disconnects from the selected serial port."""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
    def connect_serial(self):
        """Establishes a connection and starts the reading thread."""
        port_name = self.portComboBox.currentText()
        if not port_name:
            QMessageBox.warning(self, "Connection Error", "No serial port selected.")
            return

        try:
            self.serial_port = serial.Serial(port_name, 9600, timeout=1)
            
            # Create and start the worker thread
            self.serial_thread = QThread()
            self.serial_worker = SerialWorker(self.serial_port)
            self.serial_worker.moveToThread(self.serial_thread)
            
            self.serial_thread.started.connect(self.serial_worker.run)
            self.serial_worker.data_received.connect(self.update_ui)
            
            self.serial_thread.start()
            
            self.connectButton.setText("Disconnect")
            self.statusLabel.setText(f"Connected to {port_name}")
            # Set style for "Connected"
            self.statusLabel.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 12pt;")
            print(f"Successfully connected to {port_name}")

        except serial.SerialException as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to open port {port_name}:\n{e}")
            self.serial_port = None
    def disconnect_serial(self):
        """Stops the thread and closes the serial connection."""
        if self.serial_worker:
            self.serial_worker.stop()
        if self.serial_thread:
            self.serial_thread.quit()
            self.serial_thread.wait() # Wait for the thread to finish
        if self.serial_port:
            self.serial_port.close()
        
        self.serial_port = None
        self.serial_thread = None
        self.serial_worker = None
        
        self.connectButton.setText("Connect")
        self.statusLabel.setText("Disconnected")
        print("Disconnected from serial port.")
        self.statusLabel.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 12pt;")
    def send_command(self, command):
        """Sends a command string to the STM32."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(f"{command}\n".encode('utf-8'))
            self.logTextEdit.append(f"--> {command}")
        else:
            QMessageBox.warning(self, "Not Connected", "Please connect to a serial port first.")
    def send_gcode_from_input(self):
        """Sends the G-code command from the input line edit."""
        command = self.gcodeInput.text().strip()
        if command:
            self.send_command(command)
            self.gcodeInput.clear()
    def _get_feed_command_string(self):
        """
        Lấy giá trị F từ txt_input_send_F, kiểm tra và trả về chuỗi lệnh.
        """
        f = self.txt_input_send_F.text().strip()
        try:
            if f != "":
                float(f) # Kiểm tra xem có phải là số không
                return f" F{f}"
        except ValueError:
            pass # Bỏ qua nếu F không hợp lệ
        return "" # Trả về chuỗi rỗng nếu không có F hoặc F không hợp lệ
    def send_manual_goto_command(self):
        x = self.txt_input_goto_X.text()
        y = self.txt_input_goto_Y.text()
        z = self.txt_input_goto_Z.text()
        command = f"G0 "
        # Validate each axis value by attempting to convert to float; ignore invalid entries
        try:
            if x.strip() != "":
                float(x)
                command += f"X{x} "
        except ValueError:
            pass
        try:
            if y.strip() != "":
                float(y)
                command += f"Y{y} "
        except ValueError:
            pass
        try:
            if z.strip() != "":
                float(z)
                command += f"Z{z} "
        
        except ValueError:
            pass
        # Thêm giá trị F từ txt_input_send_F (theo yêu cầu)
        command += self._get_feed_command_string()
        
        if command == "G0 ":
            QMessageBox.warning(self, "Invalid Input", "Please enter at least one valid coordinate.")
            return
        else:
            command += "\n"
            # command = f"G0 X{x} Y{y} Z{z} F{f}\n"
            self.send_command(command)
    def set_preset_position_A(self):
        """Sends a SET command to update position A with the current coordinates."""
        x = self.txt_input_set_X.text()
        y = self.txt_input_set_Y.text()
        z = self.txt_input_set_Z.text()
        try:
            if x.strip() != "" and y.strip() != "" and z.strip() != "":
                float(x)
                float(y)
                float(z)
                command = f"SET A X{x} Y{y} Z{z}\n"
                self.send_command(command)
                return
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid SET X, Y, and Z coordinates.")
            pass
    def set_preset_position_B(self):
        """Sends a SET command to update position B with the current coordinates."""
        x = self.txt_input_set_X.text()
        y = self.txt_input_set_Y.text()
        z = self.txt_input_set_Z.text()
        try:
            if x.strip() != "" and y.strip() != "" and z.strip() != "":
                float(x)
                float(y)
                float(z)
                command = f"SET B X{x} Y{y} Z{z}\n"
                self.send_command(command)
                return
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid SET X, Y, and Z coordinates.")
            pass
    def set_preset_position_C(self):
        """Sends a SET command to update position C with the current coordinates."""
        x = self.txt_input_set_X.text()
        y = self.txt_input_set_Y.text()
        z = self.txt_input_set_Z.text()
        try:
            if x.strip() != "" and y.strip() != "" and z.strip() != "":
                float(x)
                float(y)
                float(z)
                command = f"SET C X{x} Y{y} Z{z}\n"
                self.send_command(command)
                return
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid SET X, Y, and Z coordinates.")
            pass
    def set_preset_position_D(self):
        """Sends a SET command to update position D with the current coordinates."""
        x = self.txt_input_set_X.text()
        y = self.txt_input_set_Y.text()
        z = self.txt_input_set_Z.text()
        try:
            if x.strip() != "" and y.strip() != "" and z.strip() != "":
                float(x)
                float(y)
                float(z)
                command = f"SET D X{x} Y{y} Z{z}\n"
                self.send_command(command)
                return
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid SET X, Y, and Z coordinates.")
            pass
    def get_current_position_into_goto(self):
        """Gets the current position from the labels and fills the GOTO input fields."""
        self.txt_input_goto_X.setText(self.xPosLabel.text())
        self.txt_input_goto_Y.setText(self.yPosLabel.text())
        self.txt_input_goto_Z.setText(self.zPosLabel.text())
    def import_gcode_file(self):
        """Open a G-code file and show its content in a non-blocking viewer window."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open G-code File",
            "",
            "G-code Files (*.gcode *.nc *.txt *.tap);;All Files (*)"
        )

        if not file_path:
            return  # user canceled

        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                content = file.read()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open file:\n{e}")
            return

        # --- Log into main window ---
        self.logTextEdit.append(f"--- Imported File: {file_path} ---")
        self.ImportedDirectory.setText(f"Imported File: {file_path}")
        # --- Create floating, non-blocking G-code viewer window ---
        viewer = QDialog(self)
        viewer.setWindowTitle(f"G-code Viewer - {file_path.split('/')[-1]}")
        viewer.resize(700, 600)
        viewer.setModal(False)  # <-- non-blocking window
        layout = QVBoxLayout(viewer)
        text_view = QTextEdit()
        text_view.setReadOnly(True)
        text_view.setPlainText(content)
        layout.addWidget(text_view)
        close_button = QPushButton("Close Viewer")
        layout.addWidget(close_button)
        close_button.clicked.connect(viewer.close)
        # Show without blocking main UI
        viewer.show()
    def run_imported_gcode_file(self):
        """
        Runs the imported G-code file line-by-line using the same macro system.
        """
        file_path = self.ImportedDirectory.text().replace("Imported File: ", "").strip()
        if not file_path or not os.path.exists(file_path):
            QMessageBox.warning(self, "No File", "Please import a G-code file first.")
            return

        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                lines = file.readlines()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open file:\n{e}")
            return

        # --- Clean lines: remove comments, empty lines ---
        commands = []
        for line in lines:
            line = line.strip()
            if not line or line.startswith(';') or line.startswith('('):
                continue  # skip comments or blank lines
            commands.append(line)

        if not commands:
            QMessageBox.warning(self, "Empty File", "The selected G-code file has no valid commands.")
            return

        # --- Prevent multiple runs at once ---
        if self.command_queue:
            QMessageBox.warning(self, "Busy", "A macro or file execution is already in progress.")
            return

        # --- Initialize queue ---
        self.command_queue = commands.copy()
        self.logTextEdit.append(f"--- Running G-code file ({len(commands)} commands) ---")
        self.is_waiting_next = False
        self.macro_running = True
        # --- Start running line by line ---
        self.run_next_macro_command()
    
    def run_next_macro_command(self):
        """Chạy lệnh kế tiếp trong macro"""
        if self.is_waiting_next:
            return  # đang chờ lệnh trước hoàn tất

        # self.logTextEdit.append(f"➡️ Run next macro command check. Queue length: {len(self.command_queue)}")
        # self.logTextEdit.append(f"➡️ Run next macro command check. Queue length: {not self.macro_running} {not self.macro_commands} {not self.command_queue}")
        if not self.macro_running  or not self.command_queue:
            self.macro_running = False
            self.logTextEdit.append("✅ Macro hoàn tất.")
            return

        command = self.command_queue.pop(0)
        self.send_command(command)
        self.logTextEdit.append(f"➡️ Gửi lệnh: {command}")

        # Nếu là lệnh di chuyển (G0/G1) → chờ hoàn tất
        if "G0" in command or "G1" in command:
            self.is_waiting_next = True
            self.machine_state = "RunGCode"
            self.target_x = self._extract_coord(command, "X")
            self.target_y = self._extract_coord(command, "Y")
            self.target_z = self._extract_coord(command, "Z")
            # self.logTextEdit.append(f"🎯 Mục tiêu: X={self.target_x}, Y={self.target_y}, Z={self.target_z}")

    # def update_tool_position(self, x, y):
    #     """Cập nhật vị trí đầu tool CNC trên khung XY"""
    #     y = -y
    #     if self.last_pos is not None and self.enableTrace == 1:
    #         line_pen = QPen(QColor(0, 150, 255), 1)
    #         self.scene.addLine(self.last_pos[0], self.last_pos[1], x, y, line_pen)
    #     self.tool_dot.setPos(x, y)
    #     self.last_pos = (x, y)

    def update_tool_position(self, x, y):
        """Cập nhật vị trí đầu tool CNC trên khung XY"""
        # Lưu ý: Trục Y trong Qt Graphics Scene hướng xuống dưới, nên CNC Y+ thì Qt phải là Y-
        y = -y 
        
        # Vẽ đường trace
        if self.last_pos is not None and self.enableTrace == 1:
            # SỬA: Tăng độ dày nét vẽ (width) lên, ví dụ 50 hoặc 100 tùy mức zoom
            line_pen = QPen(QColor(0, 150, 255), 80) 
            self.scene.addLine(self.last_pos[0], self.last_pos[1], x, y, line_pen)
            
        self.tool_dot.setPos(x, y)
        self.last_pos = (x, y)

    def random_move(self):
        """Tạo vị trí ngẫu nhiên để demo"""
        import random
        # x = random.uniform(0, self.work_width)
        # y = random.uniform(0, self.work_height)
        self.x_test += 1
        self.y_test += 1
        if self.x_test > self.work_width:
            self.x_test = 0
        if self.y_test > self.work_height:
            self.y_test = 0
        x = self.x_test
        y = self.y_test
        self.logTextEdit.append(f"Moving tool to X={x:.2f}, Y={y:.2f}")
        self.update_tool_position(x, y)

    def update_ui(self, data):
        # Parse GRBL-style status reports: <State|WPos:X,Y,Z>
        if data.startswith('<') and data.endswith('>'):
            parts = data[1:-1].split('|')
            #Identify state change for macro continuation
            state_str = "Unknown"
            if len(parts) >= 1:
                state_str = parts[0]
                self.stateLabel.setText(parts[0]) # Update state (Idle, Run, etc.)
                self.stateLabel.setText(state_str)
                self.machine_state = state_str

                # Nếu máy vừa chuyển sang Idle
                # if state_str == "Idle" and self.is_waiting_next:
                #     self.is_waiting_next = False     
                #     self.logTextEdit.append("✅ Máy đã trở về trạng thái Idle, chạy tiếp...")           
            if len(parts) >= 2 and parts[1].startswith("WPos:"):
                coords_str = parts[1][5:] # Get "X,Y,Z"
                coords = coords_str.split(',')
                if len(coords) == 3:
                    self.xPosLabel.setText(coords[0])
                    self.yPosLabel.setText(coords[1])
                    self.zPosLabel.setText(coords[2])
                    try:
                        current_x = float(coords[0])
                        current_y = float(coords[1])
                        current_z = float(coords[2])
                        self.update_tool_position(current_x, current_y)
                    except ValueError:
                        pass
                    # self.logTextEdit.append(f"<-- Position Update: X={current_x}, Y={current_y}, Z={current_z}")
                    # self.logTextEdit.append(f"<-- Machine State: {self.is_waiting_next} {self.target_x} {self.target_y} {self.target_z} ")
                    # Kiểm tra nếu đạt đến vị trí mục tiêu
                    if self.is_waiting_next and (self.target_x is not None or self.target_y is not None or self.target_z is not None):
                        if self._position_reached(current_x, current_y, current_z):
                            self.is_waiting_next = False
                            self.machine_state = "Idle"
                            # self.logTextEdit.append("✅ Đã đến vị trí mục tiêu, chạy tiếp...")
                            self.run_next_macro_command()

        else:
            # Regular log message
            self.logTextEdit.append(f"<-- {data}")
    def _extract_coord(self, line: str, axis: str):
        """Tách giá trị tọa độ trục (X/Y/Z) từ lệnh G-code"""
        match = re.search(rf'{axis}(-?\d*\.?\d*)', line)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
        return None
    def _position_reached(self, cx, cy, cz, tol=0.002):
        """Kiểm tra xem vị trí hiện tại đã tới vị trí mục tiêu chưa"""
        def close(a, b):
            return b is None or abs(a - b) <= tol

        return close(cx, self.target_x) and close(cy, self.target_y) and close(cz, self.target_z)
    def open_gcode_cleaner(self):
        """Mở cửa sổ xử lý G-code, tự động load file nếu đã import."""
        self.gcode_cleaner_window = GCodeCleanerApp(main_window=self)
        self.gcode_cleaner_window.show()

        imported_path = self.ImportedDirectory.text().replace("Imported File: ", "").strip()
        if imported_path and os.path.exists(imported_path):
            self.gcode_cleaner_window.load_file(imported_path)
    def closeEvent(self, event):
        """Ensure disconnection on window close."""
        self.disconnect_serial()
        event.accept()
    
    def toggle_trace(self):
        """Bật/tắt vẽ đường đi của đầu tool"""
        if self.enableTrace == 1:
            self.enableTrace = 0
            self.logTextEdit.append("🚫 Đã tắt vẽ đường đi của đầu tool.")
            self.trace_button.setText("Enable Trace")
            self.trace_button.setStyleSheet("background-color: rgb(0, 150, 255); color: rgb(255, 255, 255);")
            self.actionEnableDisable_Trace.setText("Enable Trace")
        else:
            self.enableTrace = 1
            self.logTextEdit.append("✅ Đã bật vẽ đường đi của đầu tool.")
            self.trace_button.setText("Disable Trace")
            self.actionEnableDisable_Trace.setText("Disable Trace")
            self.trace_button.setStyleSheet("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255);")
    def clear_xy_graph(self):
        self.scene.clear()  
        self.draw_workspace()
        self.last_pos = None
        self.logTextEdit.append("🧭 Workspace cleared and redrawn.")
    def draw_workspace(self):
        """Vẽ khung và lưới làm việc 100000x100000"""
        # Xóa scene cũ nếu có
        if hasattr(self, 'scene'):
            self.scene.clear()

        # === Tạo scene ===
        # SỬA: Scene bao trùm từ X=0 đến Max, Y từ -Max đến 0 (do trục Y trong Qt ngược với CNC)
        self.scene = QGraphicsScene(0, -self.work_height, self.work_width, self.work_height)
        self.xyGraphicsView.setScene(self.scene)
        self.xyGraphicsView.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Zoom view để thấy toàn bộ không gian
        self.xyGraphicsView.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

        # === Vẽ khung ngoài (Biên giới hạn) ===
        border_pen = QPen(Qt.GlobalColor.black, 200) # Tăng độ dày nét vẽ biên lên để dễ nhìn ở tỷ lệ lớn
        self.scene.addRect(0, -self.work_height, self.work_width, self.work_height, border_pen)

        # === Vẽ trục X và Y (Gốc tọa độ) ===
        axis_pen = QPen(Qt.GlobalColor.darkGray, 100) # Tăng độ dày trục
        self.scene.addLine(0, 0, self.work_width, 0, axis_pen)           # Trục X
        self.scene.addLine(0, 0, 0, -self.work_height, axis_pen)         # Trục Y

        # === Vẽ vạch chia lưới (Grid) ===
        # QUAN TRỌNG: Với kích thước 100.000, spacing phải lớn (ví dụ 1000 hoặc 5000)
        # Nếu để spacing=10 máy sẽ bị TREO vì vẽ quá nhiều đường.
        spacing = 2000  # Mỗi ô lưới là 2000 đơn vị
        
        grid_pen = QPen(QColor(220, 220, 220))
        grid_pen.setWidth(0) # 0 nghĩa là 1 pixel cosmetic (luôn mảnh)

        # Vẽ lưới dọc
        for x in range(0, self.work_width + 1, spacing):
            self.scene.addLine(x, 0, x, -self.work_height, grid_pen)
            
        # Vẽ lưới ngang
        for y in range(0, self.work_height + 1, spacing):
            self.scene.addLine(0, -y, self.work_width, -y, grid_pen)

        # === Thêm chấm đỏ (đại diện vị trí đầu tool) ===
        # Vì không gian lớn, chấm đỏ cần to hơn để nhìn thấy
        dot_size = 400 # Kích thước chấm đỏ
        self.tool_dot = QGraphicsEllipseItem(-dot_size/2, -dot_size/2, dot_size, dot_size)
        self.tool_dot.setBrush(QColor("red"))
        self.tool_dot.setPen(QPen(Qt.GlobalColor.NoPen))
        self.scene.addItem(self.tool_dot)

        # Biến lưu vị trí trước đó (để vẽ đường đi)
        self.last_pos = None

        # Label trục (Cho chữ to lên)
        font = self.scene.addText("X →").font()
        font.setPointSize(500) # Cỡ chữ to
        
        txt_x = self.scene.addText("X →")
        txt_x.setFont(font)
        txt_x.setScale(20) # Scale chữ to lên
        txt_x.setPos(self.work_width/2, 500)
        
        txt_y = self.scene.addText("↑ Y")
        txt_y.setFont(font)
        txt_y.setScale(20)
        txt_y.setPos(-3000, -self.work_height/2)
    # def draw_workspace(self):
    #     """Vẽ khung và lưới làm việc 250x100"""
    #     width = self.work_width
    #     height = self.work_height
    #     grid = 25

    #      # === Tạo scene ===
    #     self.scene = QGraphicsScene(0, -100, self.work_width, self.work_height)
    #     self.xyGraphicsView.setScene(self.scene)
    #     self.xyGraphicsView.setRenderHint(QPainter.RenderHint.Antialiasing)
    #     self.xyGraphicsView.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    #     # === Vẽ khung ngoài ===
    #     border_pen = QPen(Qt.GlobalColor.black, 1.5)
    #     self.scene.addRect(0, -100,  self.work_width,  self.work_height, border_pen)

    #     # === Vẽ trục X và Y ===
    #     axis_pen = QPen(Qt.GlobalColor.darkGray, 0.8)
    #     self.scene.addLine(0, 0,  self.work_width, 0, axis_pen)   # Trục X
    #     self.scene.addLine(0, 0, 0, -self.work_height, axis_pen)  # Trục Y

    #     # === Vẽ vạch chia (mỗi 10 đơn vị) ===
    #     grid_pen = QPen(QColor(220, 220, 220))
    #     grid_pen.setWidth(0)
    #     spacing = 10
    #     for x in range(0, 251, spacing):
    #         self.scene.addLine(x, 0, x, -100, grid_pen)
    #     for y in range(0, 101, spacing):
    #         self.scene.addLine(0, -y, 250, -y, grid_pen)

    #     # === Thêm chấm đỏ (đại diện vị trí đầu tool) ===
    #     self.tool_dot = QGraphicsEllipseItem(-3, -3, 6, 6)
    #     self.tool_dot.setBrush(QColor("red"))
    #     self.scene.addItem(self.tool_dot)

    #     # Biến lưu vị trí trước đó (để vẽ đường đi)
    #     self.last_pos = None

    #     # Label trục
    #     self.scene.addText("X →").setPos(width/2 - 15, height + 5)
    #     self.scene.addText("↑ Y").setPos(-20, height/2 - 10)      
    
    
    
    # ===============================================================
    # 🕹️ Điều khiển bàn phím: WASD + ZX cho trục XYZ
    # ===============================================================
    def keyPressEvent(self, event):
        key = event.key()
        step_xy = step_z = float( self.txt_input_intervalDistance.text())   # bước di chuyển XYZ (mm)

        # Đọc tọa độ hiện tại từ label
        try:
            x = float(self.xPosLabel.text())
            y = float(self.yPosLabel.text())
            z = float(self.zPosLabel.text())
        except ValueError:
            x, y, z = 0.0, 0.0, 0.0

        # --- Xử lý theo phím ---
        if key == Qt.Key.Key_W:       # ↑ Y+
            y += step_xy
        elif key == Qt.Key.Key_S:     # ↓ Y-
            y -= step_xy
        elif key == Qt.Key.Key_A:     # ← X-
            x -= step_xy
        elif key == Qt.Key.Key_D:     # → X+
            x += step_xy
        elif key == Qt.Key.Key_Z:     # Z+
            z += step_z
        elif key == Qt.Key.Key_X:     # Z-
            z -= step_z
        else:
            return  # phím khác thì bỏ qua
        # Lấy giá trị F từ hàm trợ giúp
        f_val_str = self._get_feed_command_string()

        # --- Gửi lệnh G-code đến máy ---
        cmd = f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f}{f_val_str}"
        self.send_command(cmd)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())