# app.py
import os
import sys
import serial
import serial.tools.list_ports
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QMessageBox, QFileDialog, 
                             QDialog, QVBoxLayout, QTextEdit, QPushButton, 
                             QGraphicsScene, QGraphicsEllipseItem, QWidget, 
                             QHBoxLayout, QLabel, QLineEdit, QCheckBox, QSplitter)
from PyQt6.QtCore import QThread, pyqtSignal, QObject, Qt
from PyQt6.QtGui import QPen, QColor, QPainter
from PyQt6.uic import loadUi

# ==========================================
# CLASS: SerialWorker
# Xử lý giao tiếp Serial trên luồng riêng biệt
# ==========================================
class SerialWorker(QObject):
    """
    Worker thread xử lý việc đọc dữ liệu từ cổng Serial.
    Emit tín hiệu khi nhận được dữ liệu (xuống dòng).
    """
    data_received = pyqtSignal(str)

    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self._is_running = True

    def run(self):
        """Vòng lặp chính của worker."""
        while self._is_running and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
            except serial.SerialException:
                break
            except Exception:
                continue
        print("Serial worker thread finished.")

    def stop(self):
        self._is_running = False

# ==========================================
# HELPER: GCODE CLEANER LOGIC
# ==========================================
def process_gcode_text(content: str, remove_s=True, remove_m=True, remove_f=True, remove_z=True):
    lines = content.splitlines()
    output_lines = []

    for line in lines:
        line = line.strip()
        if not line: continue
        if not re.search(r'\bG0\b|\bG1\b|X|Y|Z', line): continue
        
        if remove_s: line = re.sub(r'S\d*\.?\d*', '', line)
        if remove_m: line = re.sub(r'M\d*', '', line)
        if remove_f: line = re.sub(r'F\d*\.?\d*', '', line)
        if remove_z: line = re.sub(r'Z-?\d*\.?\d*', '', line)

        if not re.search(r'X|Y|Z', line): continue
        if not re.search(r'\bG0\b|\bG1\b', line) and re.search(r'X|Y', line):
            line = 'G0 ' + line

        x_match = re.search(r'X(-?\d+\.?\d*)', line)
        y_match = re.search(r'Y(-?\d+\.?\d*)', line)
        z_match = re.search(r'Z(-?\d+\.?\d*)', line) if not remove_z else None

        parts = []
        g_code_match = re.search(r'\b(G0|G1)\b', line)
        parts.append(g_code_match.group(1) if g_code_match else "G0")
        
        if x_match: parts.append(f"X{x_match.group(1)}")
        if y_match: parts.append(f"Y{y_match.group(1)}")
        if z_match: parts.append(f"Z{z_match.group(1)}")

        output_lines.append(" ".join(parts))

    return "\n".join(output_lines)

# ==========================================
# CLASS: GCodeCleanerApp (Cửa sổ phụ)
# ==========================================
class GCodeCleanerApp(QWidget):
    def __init__(self, main_window=None):
        super().__init__()
        self.main_window = main_window
        self.setWindowTitle("🧩 GCode Cleaner")
        self.setGeometry(200, 150, 950, 600)
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()
        hl_input = QHBoxLayout()
        hl_input.addWidget(QLabel("File đầu vào:"))
        self.in_edit = QLineEdit()
        btn_in = QPushButton("Chọn...")
        btn_in.clicked.connect(self.select_input)
        hl_input.addWidget(self.in_edit); hl_input.addWidget(btn_in)
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
        splitter.addWidget(self.text_before); splitter.addWidget(self.text_after)
        main_layout.addWidget(splitter)

        btn_save = QPushButton("💾 Lưu kết quả...")
        btn_save.clicked.connect(self.save_result)
        main_layout.addWidget(btn_save)
        self.setLayout(main_layout)

    def select_input(self):
        path, _ = QFileDialog.getOpenFileName(self, "Chọn file G-code", "", "")
        if path: self.load_file(path)

    def load_file(self, path):
        self.in_edit.setText(path)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
            self.text_before.setPlainText(content)
            self.text_after.setPlainText(self.process_current(content))
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", str(e))

    def process_current(self, content):
        return process_gcode_text(content, self.cb_s.isChecked(), self.cb_m.isChecked(), self.cb_f.isChecked(), self.cb_z.isChecked())

    def auto_reprocess(self):
        self.text_after.setPlainText(self.process_current(self.text_before.toPlainText()))

    def save_result(self):
        path, _ = QFileDialog.getSaveFileName(self, "Lưu file", "", "")
        if path:
            try:
                with open(path, 'w', encoding='utf-8') as f:
                    f.write(self.text_after.toPlainText())
                QMessageBox.information(self, "OK", "Đã lưu!")
            except Exception as e:
                QMessageBox.critical(self, "Lỗi", str(e))

# ==========================================
# CLASS: MainWindow (Chương trình chính)
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("Main_v8_OK.ui", self)
        
        # --- Variables for MAIN Controller ---
        self.serial_port = None
        self.serial_thread = None
        self.serial_worker = None

        # --- Variables for KIM Controller ---
        self.serial_port_kim = None
        self.serial_thread_kim = None
        self.serial_worker_kim = None

        # --- GCode Execution State ---
        self.macro_running = False
        self.command_queue = []
        self.is_waiting_next = False
        self.machine_state = "Idle"
        self.target_x = None; self.target_y = None; self.target_z = None

        # --- Visualizer Settings ---
        self.work_width = 100000
        self.work_height = 100000
        self.enableTrace = 1
        self.last_pos = None

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        # self.draw_workspace() # Khởi tạo vùng vẽ
        self._populate_ports()
        self._connect_signals()

    def _populate_ports(self):
        """Lấy danh sách cổng COM và đưa vào cả 2 ComboBox"""
        self.portComboBox.clear()
        self.portComboBoxKim.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.portComboBox.addItem(port.device)
            self.portComboBoxKim.addItem(port.device)

    def _connect_signals(self):
        # === MAIN CONTROLLER SIGNALS ===
        self.connectButton.clicked.connect(self.toggle_serial_connection)
        self.refreshButton.clicked.connect(self._populate_ports)
        self.sendButton.clicked.connect(self.send_gcode_from_input)
        
        # Set Zero
        self.SetX0Button.clicked.connect(lambda: self.send_command("SET X\n"))
        self.SetY0Button.clicked.connect(lambda: self.send_command("SET Y\n"))
        self.SetZ0Button.clicked.connect(lambda: self.send_command("SET Z\n"))

        # Jogging Main
        self.jogXPlusButton.clicked.connect(lambda: self.send_jog_command("X", 1))
        self.jogXMinusButton.clicked.connect(lambda: self.send_jog_command("X", -1))
        self.jogYPlusButton.clicked.connect(lambda: self.send_jog_command("Y", 1))
        self.jogYMinusButton.clicked.connect(lambda: self.send_jog_command("Y", -1))
        self.jogZPlusButton.clicked.connect(lambda: self.send_jog_command("Z", 1))
        self.jogZMinusButton.clicked.connect(lambda: self.send_jog_command("Z", -1))

        # Speed Main
        self.slowButton.clicked.connect(lambda: self.txt_input_send_F.setText("1000"))
        self.fastButton.clicked.connect(lambda: self.txt_input_send_F.setText("5000"))

        # Manual Goto Main
        self.btn_manual_goto.clicked.connect(self.send_manual_goto_command)
        self.CurrentPosButton.clicked.connect(self.get_current_position_into_goto)
        self.Stopbtn.clicked.connect(lambda: self.send_command("STOP"))

        # Presets Main
        self.setAButton.clicked.connect(lambda: self.set_preset("A"))
        self.setBButton.clicked.connect(lambda: self.set_preset("B"))
        self.setCButton.clicked.connect(lambda: self.set_preset("C"))
        self.setDButton.clicked.connect(lambda: self.set_preset("D"))
        self.gotoAButton.clicked.connect(lambda: self.send_command("GOTO A\n"))
        self.gotoBButton.clicked.connect(lambda: self.send_command("GOTO B\n"))
        self.gotoCButton.clicked.connect(lambda: self.send_command("GOTO C\n"))
        self.gotoDButton.clicked.connect(lambda: self.send_command("GOTO D\n"))
        self.getAButton.clicked.connect(lambda: self.send_command("RETURN A\n"))
        self.getBButton.clicked.connect(lambda: self.send_command("RETURN B\n"))
        self.getCButton.clicked.connect(lambda: self.send_command("RETURN C\n"))
        self.getDButton.clicked.connect(lambda: self.send_command("RETURN D\n"))


        # === KIM CONTROLLER SIGNALS (New) ===
        self.connectButtonKim.clicked.connect(self.toggle_serial_connection_kim)
        self.refreshButtonKim.clicked.connect(self._populate_ports) # Refresh chung cho cả 2
        
        # Set Zero Kim
        self.SetX0KimButton.clicked.connect(lambda: self.send_command_kim("SET X\n"))
        self.SetY0KimButton.clicked.connect(lambda: self.send_command_kim("SET Y\n"))
        self.SetZ0KimButton.clicked.connect(lambda: self.send_command_kim("SET Z\n"))

        # Jogging Kim
        self.jogXPlusKimButton.clicked.connect(lambda: self.send_jog_command_kim("X", 1))
        self.jogXMinusKimButton.clicked.connect(lambda: self.send_jog_command_kim("X", -1))
        self.jogYPlusKimButton.clicked.connect(lambda: self.send_jog_command_kim("Y", 1))
        self.jogYMinusKimButton.clicked.connect(lambda: self.send_jog_command_kim("Y", -1))
        self.jogZKimPlusButton.clicked.connect(lambda: self.send_jog_command_kim("Z", 1))
        self.jogZMinusKimButton.clicked.connect(lambda: self.send_jog_command_kim("Z", -1))

        # Speed Kim
        self.slowKimButton.clicked.connect(lambda: self.txt_input_send_FKim.setText("1000"))
        self.fastKimButton.clicked.connect(lambda: self.txt_input_send_FKim.setText("5000"))

        # Manual Goto Kim
        self.btn_manual_gotoKim.clicked.connect(self.send_manual_goto_command_kim)
        self.CurrentPosKimButton.clicked.connect(self.get_current_position_into_goto_kim)
        self.StopKimbtn.clicked.connect(lambda: self.send_command_kim("STOP"))

        # === GENERAL TOOLS ===
        self.RunGCodeButton.clicked.connect(self.run_imported_gcode_file)
        self.actionImport.triggered.connect(self.import_gcode_file)
        self.actionProccessGCode.triggered.connect(self.open_gcode_cleaner)
        self.actionClearGraph.triggered.connect(self.clear_xy_graph)
        self.actionEnableDisable_Trace.triggered.connect(self.toggle_trace)

    # ==========================================
    # LOGIC KẾT NỐI - MAIN CONTROLLER
    # ==========================================
    def toggle_serial_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port_name = self.portComboBox.currentText()
        if not port_name: return
        try:
            self.serial_port = serial.Serial(port_name, 9600, timeout=1)
            self.serial_thread = QThread()
            self.serial_worker = SerialWorker(self.serial_port)
            self.serial_worker.moveToThread(self.serial_thread)
            self.serial_thread.started.connect(self.serial_worker.run)
            self.serial_worker.data_received.connect(self.update_ui) # Link to Main UI Update
            self.serial_thread.start()
            
            self.connectButton.setText("Disconnect")
            self.statusLabel.setText(f"Connected {port_name}")
            self.statusLabel.setStyleSheet("color: #2ecc71; font-weight: bold;")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi kết nối", str(e))

    def disconnect_serial(self):
        if self.serial_worker: self.serial_worker.stop()
        if self.serial_thread: self.serial_thread.quit(); self.serial_thread.wait()
        if self.serial_port: self.serial_port.close()
        
        self.serial_port = None
        self.connectButton.setText("Connect")
        self.statusLabel.setText("Disconnected")
        self.statusLabel.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def send_command(self, command):
        """Gửi lệnh tới Main Controller"""
        if self.serial_port and self.serial_port.is_open:
            if not command.endswith('\n'): command += '\n'
            self.serial_port.write(command.encode('utf-8'))
            self.logTextEdit.append(f"Main --> {command.strip()}")
        else:
            QMessageBox.warning(self, "Lỗi", "Main Controller chưa kết nối!")

    # ==========================================
    # LOGIC KẾT NỐI - KIM CONTROLLER (New)
    # ==========================================
    def toggle_serial_connection_kim(self):
        if self.serial_port_kim and self.serial_port_kim.is_open:
            self.disconnect_serial_kim()
        else:
            self.connect_serial_kim()

    def connect_serial_kim(self):
        port_name = self.portComboBoxKim.currentText()
        if not port_name: return
        try:
            # Mở cổng COM riêng cho Kim
            self.serial_port_kim = serial.Serial(port_name, 9600, timeout=1)
            self.serial_thread_kim = QThread()
            self.serial_worker_kim = SerialWorker(self.serial_port_kim)
            self.serial_worker_kim.moveToThread(self.serial_thread_kim)
            self.serial_thread_kim.started.connect(self.serial_worker_kim.run)
            self.serial_worker_kim.data_received.connect(self.update_ui_kim) # Link to Kim UI Update
            self.serial_thread_kim.start()
            
            self.connectButtonKim.setText("Disconnect")
            self.statusKimLabel.setText(f"Connected {port_name}")
            self.statusKimLabel.setStyleSheet("color: #2ecc71; font-weight: bold;")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi kết nối Kim", str(e))

    def disconnect_serial_kim(self):
        if self.serial_worker_kim: self.serial_worker_kim.stop()
        if self.serial_thread_kim: self.serial_thread_kim.quit(); self.serial_thread_kim.wait()
        if self.serial_port_kim: self.serial_port_kim.close()
        
        self.serial_port_kim = None
        self.connectButtonKim.setText("Connect")
        self.statusKimLabel.setText("Disconnected")
        self.statusKimLabel.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def send_command_kim(self, command):
        """Gửi lệnh tới Kim Controller"""
        if self.serial_port_kim and self.serial_port_kim.is_open:
            if not command.endswith('\n'): command += '\n'
            self.serial_port_kim.write(command.encode('utf-8'))
            self.logTextEdit.append(f"Kim --> {command.strip()}")
        else:
            QMessageBox.warning(self, "Lỗi", "Kim Controller chưa kết nối!")

    # ==========================================
    # LOGIC JOGGING & GOTO - MAIN
    # ==========================================
    def _get_feed(self):
        f = self.txt_input_send_F.text().strip()
        try:
            float(f)
            return f" F{f}"
        except: return ""

    def send_jog_command(self, axis, direction):
        try:
            dist = float(self.txt_input_intervalDistance.text())
            current = 0.0
            if axis == "X": current = float(self.xPosLabel.text())
            elif axis == "Y": current = float(self.yPosLabel.text())
            elif axis == "Z": current = float(self.zPosLabel.text())
            
            target = current + (dist * direction)
            cmd = f"G0 {axis}{target:.3f}{self._get_feed()}\n"
            self.send_command(cmd)
        except ValueError:
            pass

    def send_manual_goto_command(self):
        cmd = "G0 "
        has_coord = False
        for axis, txt in [("X", self.txt_input_goto_X), ("Y", self.txt_input_goto_Y), ("Z", self.txt_input_goto_Z)]:
            if txt.text().strip():
                cmd += f"{axis}{txt.text()} "
                has_coord = True
        
        if has_coord:
            cmd += self._get_feed()
            self.send_command(cmd)

    def get_current_position_into_goto(self):
        self.txt_input_goto_X.setText(self.xPosLabel.text())
        self.txt_input_goto_Y.setText(self.yPosLabel.text())
        self.txt_input_goto_Z.setText(self.zPosLabel.text())

    def set_preset(self, label):
        x, y, z = self.txt_input_set_X.text(), self.txt_input_set_Y.text(), self.txt_input_set_Z.text()
        if x and y and z:
            self.send_command(f"SET {label} X{x} Y{y} Z{z}\n")

    # ==========================================
    # LOGIC JOGGING & GOTO - KIM
    # ==========================================
    def _get_feed_kim(self):
        f = self.txt_input_send_FKim.text().strip()
        try:
            float(f)
            return f" F{f}"
        except: return ""

    def send_jog_command_kim(self, axis, direction):
        try:
            dist = float(self.txt_input_intervalDistanceKim.text())
            current = 0.0
            # Đọc từ Label của Kim
            if axis == "X": current = float(self.xPosKimLabel.text())
            elif axis == "Y": current = float(self.yPosKimLabel.text())
            elif axis == "Z": current = float(self.zPosKimLabel.text())
            
            target = current + (dist * direction)
            cmd = f"G0 {axis}{target:.3f}{self._get_feed_kim()}\n"
            self.send_command_kim(cmd)
        except ValueError:
            pass

    def send_manual_goto_command_kim(self):
        cmd = "G0 "
        has_coord = False
        # Đọc từ TextBox Goto của Kim
        for axis, txt in [("X", self.txt_input_goto_XKim), ("Y", self.txt_input_goto_YKim), ("Z", self.txt_input_goto_ZKim)]:
            if txt.text().strip():
                cmd += f"{axis}{txt.text()} "
                has_coord = True
        
        if has_coord:
            cmd += self._get_feed_kim()
            self.send_command_kim(cmd)

    def get_current_position_into_goto_kim(self):
        self.txt_input_goto_XKim.setText(self.xPosKimLabel.text())
        self.txt_input_goto_YKim.setText(self.yPosKimLabel.text())
        self.txt_input_goto_ZKim.setText(self.zPosKimLabel.text())

    # ==========================================
    # UPDATE UI HANDLERS
    # ==========================================
    def update_ui(self, data):
        """Xử lý phản hồi từ Main Controller"""
        if data.startswith('<') and data.endswith('>'):
            parts = data[1:-1].split('|')
            if len(parts) >= 1:
                self.machine_state = parts[0]
                self.stateLabel.setText(self.machine_state)

            if len(parts) >= 2 and parts[1].startswith("WPos:"):
                coords = parts[1][5:].split(',')
                if len(coords) == 3:
                    try:
                        cx, cy, cz = float(coords[0]), float(coords[1]), float(coords[2])
                        self.xPosLabel.setText(f"{cx:.3f}")
                        self.yPosLabel.setText(f"{cy:.3f}")
                        self.zPosLabel.setText(f"{cz:.3f}")
                        self.update_tool_position(cx, cy) # Chỉ vẽ Visualizer cho Main
                        
                        # Logic chạy Gcode file (Main Only)
                        if self.is_waiting_next and (self.target_x or self.target_y or self.target_z):
                             if self._position_reached(cx, cy, cz):
                                self.is_waiting_next = False
                                self.run_next_macro_command()
                    except ValueError: pass
        else:
            self.logTextEdit.append(f"<-- Main: {data}")

    def update_ui_kim(self, data):
        """Xử lý phản hồi từ Kim Controller (Tương tự Main nhưng update Label Kim)"""
        if data.startswith('<') and data.endswith('>'):
            parts = data[1:-1].split('|')
            if len(parts) >= 1:
                self.stateKimLabel.setText(parts[0])

            if len(parts) >= 2 and parts[1].startswith("WPos:"):
                coords = parts[1][5:].split(',')
                if len(coords) == 4:
                    try:
                        self.xPosKimLabel.setText(f"{float(coords[0]):.3f}")
                        self.yPosKimLabel.setText(f"{float(coords[1]):.3f}")
                        self.zPosKimLabel.setText(f"{float(coords[2]):.3f}")
                    except ValueError: pass
        else:
            self.logTextEdit.append(f"<-- Kim: {data}")

    # # ==========================================
    # # VISUALIZER & TOOLS
    # # ==========================================
    # def draw_workspace(self):
    #     if hasattr(self, 'scene'): self.scene.clear()
    #     self.scene = QGraphicsScene(0, -self.work_height, self.work_width, self.work_height)
    #     self.xyGraphicsView.setScene(self.scene)
    #     self.xyGraphicsView.setRenderHint(QPainter.RenderHint.Antialiasing)
    #     self.xyGraphicsView.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    #     border_pen = QPen(Qt.GlobalColor.black, 200)
    #     self.scene.addRect(0, -self.work_height, self.work_width, self.work_height, border_pen)
        
    #     spacing = 2000
    #     grid_pen = QPen(QColor(220, 220, 220)); grid_pen.setWidth(0)
    #     for x in range(0, self.work_width + 1, spacing):
    #         self.scene.addLine(x, 0, x, -self.work_height, grid_pen)
    #     for y in range(0, self.work_height + 1, spacing):
    #         self.scene.addLine(0, -y, self.work_width, -y, grid_pen)

    #     dot_size = 400
    #     self.tool_dot = QGraphicsEllipseItem(-dot_size/2, -dot_size/2, dot_size, dot_size)
    #     self.tool_dot.setBrush(QColor("red"))
    #     self.tool_dot.setPen(QPen(Qt.GlobalColor.NoPen))
    #     self.scene.addItem(self.tool_dot)

    #     font = self.scene.addText("X").font(); font.setPointSize(500)
    #     txt = self.scene.addText("X ->"); txt.setFont(font); txt.setScale(20); txt.setPos(self.work_width/2, 500)
    #     txt = self.scene.addText("^ Y"); txt.setFont(font); txt.setScale(20); txt.setPos(-3000, -self.work_height/2)

    def update_tool_position(self, x, y):
        y = -y # Đảo trục Y cho Visualizer
        if self.last_pos is not None and self.enableTrace == 1:
            line_pen = QPen(QColor(0, 150, 255), 80)
            # self.scene.addLine(self.last_pos[0], self.last_pos[1], x, y, line_pen)
        # self.tool_dot.setPos(x, y)
        self.last_pos = (x, y)

    def toggle_trace(self):
        self.enableTrace = 1 if self.enableTrace == 0 else 0
        self.logTextEdit.append(f"Trace {'Enabled' if self.enableTrace else 'Disabled'}")

    def clear_xy_graph(self):
        self.scene.clear()
        self.draw_workspace()
        self.last_pos = None

    # ==========================================
    # GCODE EXECUTION (MAIN ONLY)
    # ==========================================
    def send_gcode_from_input(self):
        cmd = self.gcodeInput.text().strip()
        if cmd: self.send_command(cmd); self.gcodeInput.clear()

    def import_gcode_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open G-code", "", "G-code (*.gcode *.nc *.txt *.tap)")
        if path:
            self.ImportedDirectory.setText(f"Imported File: {path}")
            self.logTextEdit.append(f"Loaded: {path}")

    def run_imported_gcode_file(self):
        path = self.ImportedDirectory.text().replace("Imported File: ", "").strip()
        if not path or not os.path.exists(path): return QMessageBox.warning(self, "Lỗi", "Chưa import file!")
        
        if self.command_queue: return QMessageBox.warning(self, "Busy", "Đang chạy file khác!")
        
        try:
            with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
            self.command_queue = [l.strip() for l in lines if l.strip() and not l.startswith(';') and not l.startswith('(')]
            self.is_waiting_next = False
            self.macro_running = True
            self.logTextEdit.append(f"--- Bắt đầu chạy {len(self.command_queue)} dòng lệnh ---")
            self.run_next_macro_command()
        except Exception as e: QMessageBox.critical(self, "Lỗi", str(e))

    def run_next_macro_command(self):
        if self.is_waiting_next: return
        if not self.macro_running or not self.command_queue:
            self.macro_running = False
            self.logTextEdit.append("✅ Hoàn tất G-code.")
            return

        cmd = self.command_queue.pop(0)
        self.send_command(cmd)
        
        if "G0" in cmd or "G1" in cmd:
            self.is_waiting_next = True
            self.target_x = self._extract_coord(cmd, "X")
            self.target_y = self._extract_coord(cmd, "Y")
            self.target_z = self._extract_coord(cmd, "Z")

    def _extract_coord(self, line, axis):
        match = re.search(rf'{axis}(-?\d*\.?\d*)', line)
        return float(match.group(1)) if match else None

    def _position_reached(self, cx, cy, cz, tol=0.01):
        def close(a, b): return b is None or abs(a - b) <= tol
        return close(cx, self.target_x) and close(cy, self.target_y) and close(cz, self.target_z)

    def open_gcode_cleaner(self):
        self.cleaner = GCodeCleanerApp(self)
        self.cleaner.show()
        path = self.ImportedDirectory.text().replace("Imported File: ", "").strip()
        if path and os.path.exists(path): self.cleaner.load_file(path)
    
    def keyPressEvent(self, event):
        # Giữ lại điều khiển phím cho Main (Kim không dùng phím để tránh nhầm lẫn)
        key = event.key()
        try: dist = float(self.txt_input_intervalDistance.text())
        except: dist = 1.0
        
        try: x, y, z = float(self.xPosLabel.text()), float(self.yPosLabel.text()), float(self.zPosLabel.text())
        except: x, y, z = 0.0, 0.0, 0.0

        if key == Qt.Key.Key_W: y += dist
        elif key == Qt.Key.Key_S: y -= dist
        elif key == Qt.Key.Key_A: x -= dist
        elif key == Qt.Key.Key_D: x += dist
        elif key == Qt.Key.Key_Z: z += dist
        elif key == Qt.Key.Key_X: z -= dist
        else: return

        self.send_command(f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f}{self._get_feed()}")

    def closeEvent(self, event):
        self.disconnect_serial()
        self.disconnect_serial_kim()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())