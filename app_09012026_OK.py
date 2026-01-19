import sys
import os
import serial
import serial.tools.list_ports
import re
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QMessageBox, QDialog, 
                             QVBoxLayout, QTextBrowser, QDialogButtonBox)
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt6.uic import loadUi


# --- Thêm class xử lý cửa sổ Preset ---
class SetPresetDialog(QDialog):
    def __init__(self, main_window):
        super().__init__()
        loadUi("SetPresetPosition.ui", self)
        self.main_window = main_window
        
        # Kết nối nút bấm
        self.btn_GetInfo.clicked.connect(self.get_all_info)
        self.btn_SetPosition.clicked.connect(self.set_position)
        
        # Kết nối signal nhận dữ liệu từ MainWindow
        self.main_window.preset_data_received.connect(self.update_ui_position)
 
    def get_all_info(self):
        # Gửi lệnh lấy thông tin lần lượt A B C D
        for char in ['A', 'B', 'C', 'D']:
            self.main_window.send_command_table(f"RETURN {char}\n")
            time.sleep(0.1) # Delay nhỏ để tránh nghẽn buffer
        # self.main_window.send_command_table(f"RETURN A\n")
    def set_position(self):
        target = self.combo_Target.currentText()
        x = self.input_Set_X.text().strip() or "0"
        y = self.input_Set_Y.text().strip() or "0"
        z = self.input_Set_Z.text().strip() or "0"
        
        # Gửi lệnh SET
        cmd = f"SET {target}X{x}Y{y}Z{z}\n"
        self.main_window.send_command_table(cmd)
        
        # Gửi lệnh Get Info lại cho trục vừa set để cập nhật UI
        time.sleep(0.1)
        self.main_window.send_command_table(f"RETURN {target}\n")

    def update_ui_position(self, pos_char, x, y, z):
        # Hàm này được gọi khi MainWindow nhận được tin nhắn dạng %A...
        if pos_char == 'A':
            self.line_A_X.setText(f"{x:.3f}")
            self.line_A_Y.setText(f"{y:.3f}")
            self.line_A_Z.setText(f"{z:.3f}")
        elif pos_char == 'B':
            self.line_B_X.setText(f"{x:.3f}")
            self.line_B_Y.setText(f"{y:.3f}")
            self.line_B_Z.setText(f"{z:.3f}")
        elif pos_char == 'C':
            self.line_C_X.setText(f"{x:.3f}")
            self.line_C_Y.setText(f"{y:.3f}")
            self.line_C_Z.setText(f"{z:.3f}")
        elif pos_char == 'D':
            self.line_D_X.setText(f"{x:.3f}")
            self.line_D_Y.setText(f"{y:.3f}")
            self.line_D_Z.setText(f"{z:.3f}")



# --- Worker Thread để xử lý Serial ---
class SerialThread(QThread):
    data_received = pyqtSignal(str) # Signal gửi dữ liệu về UI
    status_changed = pyqtSignal(bool) # Signal trạng thái kết nối

    def __init__(self, port, baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = False

    def run(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.is_running = True
            self.status_changed.emit(True)
            
            while self.is_running:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        # Đọc từng dòng dữ liệu
                        line = self.serial_port.readline().decode('utf-8').strip()
                        # # Xuất dữ liệu về UI
                        # print(f"Received from {self.port}: {line}")
                        if line:
                            self.data_received.emit(line)
                    except Exception as e:
                        print(f"Error reading {self.port}: {e}")
                        break
                time.sleep(0.01) # Tránh chiếm dụng CPU quá mức
                
        except serial.SerialException as e:
            print(f"Failed to connect {self.port}: {e}")
        finally:
            self.stop()

    def send_data(self, data):
        if self.serial_port and self.serial_port.is_open:
            try:
                # Đảm bảo gửi dạng bytes
                if isinstance(data, str):
                    data = data.encode('utf-8')
                self.serial_port.write(data)
                print(f"Sent to {self.port}: {data}")
            except Exception as e:
                print(f"Send error: {e}")

    def stop(self):
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.status_changed.emit(False)

# --- Main Window ---
class MainWindow(QMainWindow):
    # Khai báo Signal mới
    preset_data_received = pyqtSignal(str, float, float, float) # char, x, y, z
    def __init__(self):
        super().__init__()
        loadUi("Main_vTest.ui", self)  # Đảm bảo file .ui nằm cùng thư mục
        self.setWindowTitle("STM32 G-Code Controller System")
        self.connectButton.setFixedWidth(69)       
        self.connectButtonKim.setFixedWidth(69)
        # Khởi tạo các biến lưu trữ COM Thread
        self.thread_table = None
        self.thread_kim = None
        # (Nối thêm vào CSS hiện có trong file UI)
        current_style = self.styleSheet()
        popup_style_fix = """
            QMessageBox, QDialog { background-color: #f5f5f5; }
            QMessageBox QLabel, QDialog QLabel { color: #000000; font-weight: normal; font-size: 10pt; }
            QMessageBox QPushButton, QDialog QPushButton { background-color: #e0e0e0; color: black; border: 1px solid #aaa; }
            QMessageBox QPushButton:hover, QDialog QPushButton:hover { background-color: #d0d0d0; }
        """
        self.setStyleSheet(current_style + popup_style_fix)
        # Biến lưu tọa độ hiện tại (Cập nhật liên tục từ COM)
        self.curr_pos_table = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.curr_pos_kim = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'v': 0.0} # z là X1, v là Y1 trên UI

        
        # --- KẾT NỐI ACTION ABOUT ---
        self.actionAbout.triggered.connect(self.show_about_dialog)
        # --- KẾT NỐI ACTION PRESET DIALOG ---
        self.actionSet_Preset_Position.triggered.connect(self.show_preset_dialog)

        # --- SETUP KẾT NỐI ---
        self.refresh_ports()
        self.refreshButton.clicked.connect(self.refresh_ports)
        self.refreshButtonKim.clicked.connect(self.refresh_ports)

        self.connectButton.clicked.connect(self.toggle_table_connection)
        self.connectButtonKim.clicked.connect(self.toggle_kim_connection)

        # --- SETUP ZEROING ---
        # Bàn
        self.SetX0Button.clicked.connect(lambda: self.send_command_table("SET X\n"))
        self.SetY0Button.clicked.connect(lambda: self.send_command_table("SET Y\n"))
        self.SetZ0Button.clicked.connect(lambda: self.send_command_table("SET Z\n"))
        # Kim
        self.SetX0KimButton.clicked.connect(lambda: self.send_command_kim("SET X\n"))
        self.SetY0KimButton.clicked.connect(lambda: self.send_command_kim("SET Y\n"))
        self.SetX1KimButton.clicked.connect(lambda: self.send_command_kim("SET Z\n")) # X1 -> Z
        self.SetY1KimButton.clicked.connect(lambda: self.send_command_kim("SET V\n")) # Y1 -> V

        # --- SETUP STOP ---
        self.Stopbtn.clicked.connect(lambda: self.send_command_table("STOP\n"))
        self.StopKimbtn.clicked.connect(lambda: self.send_command_kim("STOP\n"))

        # --- SETUP JOGGING (BÀN) ---
        self.jogXMinusButton.clicked.connect(lambda: self.jog_table('X', -1))
        self.jogXPlusButton.clicked.connect(lambda: self.jog_table('X', 1))
        self.jogYMinusButton.clicked.connect(lambda: self.jog_table('Y', -1))
        self.jogYPlusButton.clicked.connect(lambda: self.jog_table('Y', 1))
        self.jogZMinusButton.clicked.connect(lambda: self.jog_table('Z', -1))
        self.jogZPlusButton.clicked.connect(lambda: self.jog_table('Z', 1))

        # --- SETUP JOGGING (KIM) ---
        # X, Y thường
        self.jogXMinusKimButton.clicked.connect(lambda: self.jog_kim('X', -1))
        self.jogXPlusKimButton.clicked.connect(lambda: self.jog_kim('X', 1))
        self.jogYMinusKimButton.clicked.connect(lambda: self.jog_kim('Y', -1))
        self.jogYPlusKimButton.clicked.connect(lambda: self.jog_kim('Y', 1))
        # X1 (Gửi Z), Y1 (Gửi V)
        self.jogX1MinusKimButton.clicked.connect(lambda: self.jog_kim('Z', -1)) # X1 maps to Z
        self.jogX1PlusKimButton.clicked.connect(lambda: self.jog_kim('Z', 1))
        self.jogY1MinusKimButton.clicked.connect(lambda: self.jog_kim('V', -1)) # Y1 maps to V
        self.jogY1PlusKimButton.clicked.connect(lambda: self.jog_kim('V', 1))

        # --- SETP GOTO PRESETS ---
        self.gotoAButton.clicked.connect(lambda: self.send_command_table("GOTO A\n"))
        self.gotoBButton.clicked.connect(lambda: self.send_command_table("GOTO B\n"))
        self.gotoCButton.clicked.connect(lambda: self.send_command_table("GOTO C\n"))
        self.gotoDButton.clicked.connect(lambda: self.send_command_table("GOTO D\n"))




        # --- SETUP SPEED PRESETS ---
        self.slowButton.clicked.connect(lambda: self.txt_input_send_F.setText("1000"))
        self.fastButton.clicked.connect(lambda: self.txt_input_send_F.setText("5000"))
        self.slowKimButton.clicked.connect(lambda: self.txt_input_send_FKim.setText("1000"))
        self.fastKimButton.clicked.connect(lambda: self.txt_input_send_FKim.setText("2000"))

        # --- SETUP CURRENT POS FILL ---
        self.CurrentPosButton.clicked.connect(self.fill_current_pos_table)
        self.CurrentPosKimButton.clicked.connect(self.fill_current_pos_kim)

        # --- SETUP MANUAL GOTO ---
        self.btn_manual_goto.clicked.connect(self.manual_goto_table)
        self.btn_manual_gotoKim.clicked.connect(self.manual_goto_kim)

        # --- SETUP MANUAL SEND ---
        self.sendTableButton.clicked.connect(self.manual_send_table)
        self.sendKimButton.clicked.connect(self.manual_send_kim)
    # --- Bật toàn màn hình rồi tắt ngay để refresh layout ---
    def force_refresh_ui(self):
        # 1. Lưu lại trạng thái cửa sổ hiện tại (đang Maximize hay Normal?)
        # Giúp trả về đúng trạng thái ban đầu của người dùng
        was_maximized = self.isMaximized()

        # 2. Bật chế độ toàn màn hình
        self.showFullScreen()
        
        # 3. Bắt buộc Qt xử lý các sự kiện vẽ lại ngay lập tức (quan trọng)
        # Nếu không có dòng này, code chạy quá nhanh mắt thường sẽ không thấy gì
        QApplication.processEvents()

        # 4. Khôi phục lại trạng thái cũ
        if was_maximized:
            self.showMaximized()
        else:
            self.showNormal()
# --- HÀM HIỂN THỊ HƯỚNG DẪN CHI TIẾT ---
    def show_about_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Hướng Dẫn Sử Dụng")
        dialog.resize(600, 500) # Kích thước cửa sổ hướng dẫn

        layout = QVBoxLayout()

        # Nội dung hướng dẫn dạng HTML
        help_text = """
        <h2 style="color: #2980b9;">HƯỚNG DẪN SỬ DỤNG PHẦN MỀM ĐIỀU KHIỂN CNC</h2>
        <p>Phần mềm hỗ trợ điều khiển song song 2 cơ cấu: <b>Bàn Máy (Table)</b> và <b>Kim (Needle)</b> thông qua giao tiếp G-code.</p>
        
        <hr>
        
        <h3 style="color: #c0392b;">1. Kết Nối (Connection)</h3>
        <ul>
            <li><b>Bước 1:</b> Cắm dây cáp USB cho cả 2 mạch điều khiển.</li>
            <li><b>Bước 2:</b> Nhấn nút <b>Refresh</b> để cập nhật danh sách cổng COM.</li>
            <li><b>Bước 3:</b> Chọn cổng COM tương ứng cho <b>Bàn</b> và <b>Kim</b>. 
                <i>(Lưu ý: Không chọn cùng 1 cổng cho cả 2).</i></li>
            <li><b>Bước 4:</b> Nhấn <b>Connect</b>. Trạng thái sẽ chuyển sang <span style="color:green;">Connected</span>.</li>
        </ul>

        <h3 style="color: #27ae60;">2. Chế Độ Jog (Điều khiển bước)</h3>
        <ul>
            <li>Nhập bước di chuyển vào ô <b>Distance</b> (Ví dụ: 1mm).</li>
            <li>Nhấn các nút <b>X+, X-, Y+, Y-...</b> để di chuyển trục tương ứng.</li>
            <li><b>Slow/Fast:</b> Nhấn để chọn nhanh tốc độ di chuyển mặc định.</li>
        </ul>

        <h3 style="color: #8e44ad;">3. Chế Độ Manual Goto (Đi tới tọa độ)</h3>
        <ul>
            <li>Nhấn nút <b>CurPos</b> (Lấy vị trí) để điền tọa độ hiện tại vào các ô nhập liệu.</li>
            <li>Chỉnh sửa tọa độ mong muốn vào các ô X, Y, Z...</li>
            <li>Nhập tốc độ vào ô <b>F</b>.</li>
            <li>Nhấn <b>Go!</b> để máy chạy tới vị trí đã nhập.</li>
        </ul>

        <h3 style="color: #d35400;">4. Thiết Lập Gốc (Zeroing)</h3>
        <ul>
            <li>Di chuyển máy bằng tay đến vị trí mong muốn làm gốc.</li>
            <li>Nhấn nút <b>Set X0, Set Y0...</b> để báo cho mạch điều khiển biết đây là vị trí 0.</li>
        </ul>

        <h3 style="color: red;">5. Lưu Ý An Toàn</h3>
        <ul>
            <li>Nút <b>STOP</b> màu đỏ dùng để dừng khẩn cấp mọi chuyển động.</li>
            <li>Nếu phần mềm báo "Warning: Lộn cổng", hãy ngắt kết nối và đổi lại cổng COM giữa Bàn và Kim.</li>
        </ul>
        """

        text_browser = QTextBrowser()
        text_browser.setHtml(help_text)
        text_browser.setOpenExternalLinks(True)
        # Ép style riêng cho bảng hướng dẫn này để dễ đọc
        text_browser.setStyleSheet("background-color: white; color: black; font-size: 11pt; padding: 10px;")

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(dialog.accept)

        layout.addWidget(text_browser)
        layout.addWidget(buttons)
        dialog.setLayout(layout)
        
        dialog.exec()
    # --- HÀM HIỂN THỊ PRESET DIALOG ---
    def show_preset_dialog(self):
        dialog = SetPresetDialog(self)
        dialog.exec()
    
    
    # --- PORT HANDLING ---
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        
        # Lưu lại lựa chọn hiện tại nếu có
        curr_table = self.portComboBox.currentText()
        curr_kim = self.portComboBoxKim.currentText()

        self.portComboBox.clear()
        self.portComboBoxKim.clear()
        
        self.portComboBox.addItems(ports)
        self.portComboBoxKim.addItems(ports)

        # Khôi phục lựa chọn nếu vẫn còn tồn tại
        if curr_table in ports: self.portComboBox.setCurrentText(curr_table)
        if curr_kim in ports: self.portComboBoxKim.setCurrentText(curr_kim)

    # --- CONNECTION LOGIC ---
    def toggle_table_connection(self):
        if self.thread_table is None: # Đang disconnect -> Connect
            port = self.portComboBox.currentText()
            if not port: return
            
            # Kiểm tra trùng port
            if self.thread_kim and self.thread_kim.port == port:
                QMessageBox.warning(self, "Lỗi", "Cổng COM này đang được dùng cho Kim!")
                return

            self.thread_table = SerialThread(port)
            self.thread_table.data_received.connect(self.parse_data_table)
            self.thread_table.status_changed.connect(self.update_status_table)
            self.thread_table.start()
            self.connectButton.setText("Disconnect")
        else: # Đang connect -> Disconnect
            self.thread_table.stop()
            self.thread_table = None
            self.connectButton.setText("Connect")
        # self.force_refresh_ui()

    def toggle_kim_connection(self):
        if self.thread_kim is None:
            port = self.portComboBoxKim.currentText()
            if not port: return

            if self.thread_table and self.thread_table.port == port:
                QMessageBox.warning(self, "Lỗi", "Cổng COM này đang được dùng cho Bàn!")
                return

            self.thread_kim = SerialThread(port)
            self.thread_kim.data_received.connect(self.parse_data_kim)
            self.thread_kim.status_changed.connect(self.update_status_kim)
            self.thread_kim.start()
            self.connectButtonKim.setText("Disconnect")
        else:
            self.thread_kim.stop()
            self.thread_kim = None
            self.connectButtonKim.setText("Connect")
        # self.force_refresh_ui()

    def update_status_table(self, connected):
        if connected:
            self.statusLabel.setText("Connected")
            self.statusLabel.setStyleSheet("color: green; ")
        else:
            self.statusLabel.setText("Disconnected")
            self.statusLabel.setStyleSheet("color: red; ")

    def update_status_kim(self, connected):
        if connected:
            self.statusKimLabel.setText("Connected")
            self.statusKimLabel.setStyleSheet("color: green; ")
        else:
            self.statusKimLabel.setText("Disconnected")
            self.statusKimLabel.setStyleSheet("color: red; ")

    # --- SEND COMMANDS ---
    def send_command_table(self, cmd):
        if self.thread_table:
            self.thread_table.send_data(cmd)
            self.log_to_screen(f"Bàn -> {cmd.strip()}")
        else:
            QMessageBox.warning(self, "Lỗi", "Chưa kết nối COM Bàn")

    def send_command_kim(self, cmd):
        if self.thread_kim:
            self.thread_kim.send_data(cmd)
            self.log_to_screen(f"Kim -> {cmd.strip()}")
        else:
            QMessageBox.warning(self, "Lỗi", "Chưa kết nối COM Kim")

    # --- JOGGING LOGIC ---
    def get_feed(self, input_widget):
        try:
            return float(input_widget.text())
        except ValueError:
            return 1000.0 # Default speed

    def get_interval(self, input_widget):
        try:
            return float(input_widget.text())
        except ValueError:
            return 1.0 # Default interval

    def jog_table(self, axis, direction):
        # Target = Current + (Interval * Direction)
        interval = self.get_interval(self.txt_input_intervalDistance)
        feed = self.get_feed(self.txt_input_send_F)
        
        current_val = self.curr_pos_table.get(axis.lower(), 0.0)
        target = current_val + (interval * direction)
        
        cmd = f"G0 {axis}{target:.3f} F{feed}\n"
        self.send_command_table(cmd)

    def jog_kim(self, axis, direction):
        # Axis truyền vào: 'X', 'Y', 'Z' (là X1), 'V' (là Y1)
        interval = self.get_interval(self.txt_input_intervalDistanceKim)
        feed = self.get_feed(self.txt_input_send_FKim)

        key_map = axis.lower() # key trong dict curr_pos_kim
        current_val = self.curr_pos_kim.get(key_map, 0.0)
        target = current_val + (interval * direction)

        cmd = f"G0 {axis}{target:.3f} F{feed}\n"
        self.send_command_kim(cmd)

    # --- MANUAL GOTO & FILL LOGIC ---
    def fill_current_pos_table(self):
        self.txt_input_goto_X.setText(f"{self.curr_pos_table['x']:.3f}")
        self.txt_input_goto_Y.setText(f"{self.curr_pos_table['y']:.3f}")
        self.txt_input_goto_Z.setText(f"{self.curr_pos_table['z']:.3f}")
        self.txt_input_goto_F.setText(self.txt_input_send_F.text())

    def fill_current_pos_kim(self):
        self.txt_input_goto_XKim.setText(f"{self.curr_pos_kim['x']:.3f}")
        self.txt_input_goto_YKim.setText(f"{self.curr_pos_kim['y']:.3f}")
        self.txt_input_goto_X1Kim.setText(f"{self.curr_pos_kim['z']:.3f}") # Z là X1
        self.txt_input_goto_Y1Kim.setText(f"{self.curr_pos_kim['v']:.3f}") # V là Y1
        self.txt_input_goto_FKim.setText(self.txt_input_send_FKim.text())

    def get_value_or_default(self, widget, default_val):
        text = widget.text().strip()
        if not text:
            return default_val
        try:
            return float(text)
        except ValueError:
            return default_val

    def manual_goto_table(self):
        x = self.get_value_or_default(self.txt_input_goto_X, self.curr_pos_table['x'])
        y = self.get_value_or_default(self.txt_input_goto_Y, self.curr_pos_table['y'])
        z = self.get_value_or_default(self.txt_input_goto_Z, self.curr_pos_table['z'])
        f = self.get_value_or_default(self.txt_input_goto_F, 1000.0)
        
        cmd = f"G0 X{x:.3f}Y{y:.3f}Z{z:.3f}F{f}\n"
        self.send_command_table(cmd)

    def manual_goto_kim(self):
        x = self.get_value_or_default(self.txt_input_goto_XKim, self.curr_pos_kim['x'])
        y = self.get_value_or_default(self.txt_input_goto_YKim, self.curr_pos_kim['y'])
        z = self.get_value_or_default(self.txt_input_goto_X1Kim, self.curr_pos_kim['z']) # Z map to X1
        v = self.get_value_or_default(self.txt_input_goto_Y1Kim, self.curr_pos_kim['v']) # V map to Y1
        f = self.get_value_or_default(self.txt_input_goto_FKim, 1000.0)

        cmd = f"G0 X{x:.3f}Y{y:.3f}Z{z:.3f}V{v:.3f}F{f}\n"
        self.send_command_kim(cmd)

    def manual_send_table(self):
        cmd = self.gcodeInput.text()
        if cmd:
            self.send_command_table(cmd + "\n")

    def manual_send_kim(self):
        cmd = self.gcodeInput.text()
        if cmd:
            self.send_command_kim(cmd + "\n")

    # --- PARSING DATA (REGEX) ---
    # Regex cho Bàn: <Idle|WPos:0.000,0.000,0.000>
    # Regex cho Kim: <Idle|WPos:0.000,0.000,0.000,0.000>
    
    def parse_data_table(self, data):
        # Xử lý lộn cổng (Heuristic Check)
        # Nếu bàn nhận được 4 tọa độ -> Có thể là Kim
        if data.count(',') >= 3 and "WPos" in data:
             # Logic cảnh báo nhẹ (Log only)
             self.log_to_screen("Warning: COM Bàn nhận dữ liệu dạng 4 trức (Có thể lộn cổng Kim?)")
             pass 
        # Check Regex cho Preset (A X...Y...Z...) //Vị trí A B C D
        match_preset = re.search(r"([ABCD])\s*X([\d\.\-]+)Y([\d\.\-]+)Z([\d\.\-]+)", data)
        if match_preset:
            char = match_preset.group(1)
            x = float(match_preset.group(2))
            y = float(match_preset.group(3))
            z = float(match_preset.group(4))
            self.preset_data_received.emit(char, x, y, z)
            print(f"Preset Data Received: {char} X:{x} Y:{y} Z:{z}")
            return # Đã xử lý xong, thoát hàm
        match = re.search(r"<([^|]+)\|WPos:([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)(?:,[\d\.\-]+)?>", data)
        if match:
            state = match.group(1)
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))

            # Update Variables
            self.curr_pos_table = {'x': x, 'y': y, 'z': z}

            # Update UI
            self.stateLabel.setText(state)
            self.xPosLabel.setText(f"{x:.3f}")
            self.yPosLabel.setText(f"{y:.3f}")
            self.zPosLabel.setText(f"{z:.3f}")
        else:
            self.log_to_screen(f"Unrecognized data from Table: {data}")

    def parse_data_kim(self, data):
        # Regex bắt 4 giá trị: X, Y, Z, V
        match = re.search(r"<([^|]+)\|WPos:([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)>", data)
        if match:
            state = match.group(1)
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))
            v = float(match.group(5))

            # Update Variables
            self.curr_pos_kim = {'x': x, 'y': y, 'z': z, 'v': v}

            # Update UI (Mapping: Z -> X1 label, V -> Y1 label)
            self.stateKimLabel.setText(state)
            self.xPosKimLabel.setText(f"{x:.3f}")
            self.yPosKimLabel.setText(f"{y:.3f}")
            self.x1PosKimLabel.setText(f"{z:.3f}") # X1 hiển thị Z
            self.y1PosKimLabel.setText(f"{v:.3f}") # Y1 hiển thị V
        
        # Xử lý lộn cổng: Nếu data chỉ có 3 tọa độ (thiếu V) -> Cảnh báo
        elif re.search(r"<([^|]+)\|WPos:([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)>", data):
             self.log_to_screen("Warning: COM Kim nhận dữ liệu dạng 3 trục (Có thể lộn cổng Bàn?)")

    def log_to_screen(self, msg):
        self.logTextEdit.append(msg)

# --- Entry Point ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())