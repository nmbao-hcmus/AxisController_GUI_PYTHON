# Da Mo camera va hien thi len UI

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

import cv2
import numpy as np
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt
import sys
import cv2
import numpy as np
from PyQt6.QtWidgets import (QWidget, QLabel, QVBoxLayout, QApplication, 
                             QDialog, QMessageBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QPoint, QRect
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QCursor
from PyQt6.uic import loadUi

# --- PHẦN 1: THREAD ĐỌC CAMERA ---
# =========================================================
# 1. THREAD CAMERA
# =========================================================
class CameraThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self, index=0):
        super().__init__()
        self.index = index
        self._run_flag = True

    def run(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        while self._run_flag:
            ret, frame = cap.read()
            if ret:
                self.change_pixmap_signal.emit(frame)
            else:
                break
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# =========================================================
# 2. DIALOG CÀI ĐẶT (SETTING)
# =========================================================
class SettingCameraDialog(QDialog):
    settings_changed = pyqtSignal(dict) # Bắn tín hiệu khi có thay đổi

    def __init__(self, current_settings=None):
        super().__init__()
        loadUi("SettingCamera.ui", self)
        
        # Cấu hình mặc định
        self.settings = {
            "brightness": 0,
            "contrast": 100, # 100 = 1.0x
            "sharpness": 0,
            "grayscale": False,
            "binary_mode": False,
            "threshold": 127
        }
        
        # Nếu có setting cũ truyền vào thì load lên
        if current_settings:
            self.settings.update(current_settings)
            self.update_ui_from_settings()

        # Kết nối sự kiện UI
        self.slider_Brightness.valueChanged.connect(self.on_change)
        self.slider_Contrast.valueChanged.connect(self.on_change)
        self.slider_Sharpness.valueChanged.connect(self.on_change)
        self.slider_Threshold.valueChanged.connect(self.on_change)
        self.chk_Grayscale.toggled.connect(self.on_change)
        self.chk_Binary.toggled.connect(self.on_binary_toggled)
        
        self.btn_SaveJson.clicked.connect(self.export_json)
        self.btn_LoadJson.clicked.connect(self.import_json)
        
        # Set trạng thái ban đầu cho UI
        self.update_ui_from_settings()

    def update_ui_from_settings(self):
        self.slider_Brightness.setValue(self.settings["brightness"])
        self.slider_Contrast.setValue(self.settings["contrast"])
        self.slider_Sharpness.setValue(self.settings["sharpness"])
        self.chk_Grayscale.setChecked(self.settings["grayscale"])
        self.chk_Binary.setChecked(self.settings["binary_mode"])
        self.slider_Threshold.setValue(self.settings["threshold"])
        self.slider_Threshold.setEnabled(self.settings["binary_mode"])

    def on_binary_toggled(self, checked):
        self.slider_Threshold.setEnabled(checked)
        self.on_change()

    def on_change(self):
        # Cập nhật dictionary từ UI
        self.settings["brightness"] = self.slider_Brightness.value()
        self.settings["contrast"] = self.slider_Contrast.value()
        self.settings["sharpness"] = self.slider_Sharpness.value()
        self.settings["grayscale"] = self.chk_Grayscale.isChecked()
        self.settings["binary_mode"] = self.chk_Binary.isChecked()
        self.settings["threshold"] = self.slider_Threshold.value()
        
        # Bắn tín hiệu về Window cha để áp dụng ngay lập tức
        self.settings_changed.emit(self.settings)

    def export_json(self):
        file_path, _ = QFileDialog.getSaveFileName(self, "Export Settings", "", "JSON Files (*.json)")
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(self.settings, f, indent=4)
                QMessageBox.information(self, "Success", "Đã xuất file JSON thành công!")
            except Exception as e:
                QMessageBox.warning(self, "Error", str(e))

    def import_json(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Import Settings", "", "JSON Files (*.json)")
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                    self.settings.update(data)
                    self.update_ui_from_settings()
                    self.settings_changed.emit(self.settings) # Áp dụng ngay sau khi load
                QMessageBox.information(self, "Success", "Đã nạp cài đặt thành công!")
            except Exception as e:
                QMessageBox.warning(self, "Error", str(e))

# =========================================================
# 3. WIDGET HIỂN THỊ (Giữ nguyên logic Zoom/Pan)
# =========================================================
class CameraViewerWidget(QLabel):
    pos_selected = pyqtSignal(int, int) 
    
    def __init__(self, minimap_label):
        super().__init__()
        self.setMouseTracking(True)
        self.minimap_label = minimap_label
        
        self.current_frame = None
        self.zoom_levels = [1.0, 1.5, 2.0, 3.0]
        self.zoom_idx = 0
        self.offset_x = 0 
        self.offset_y = 0
        
        self.mode_goto = False 
        self.is_panning = False
        self.last_mouse_pos = QPoint()

    def set_frame(self, frame):
        self.current_frame = frame
        self.update()

    def toggle_zoom(self):
        self.zoom_idx = (self.zoom_idx + 1) % len(self.zoom_levels)
        self.offset_x = 0
        self.offset_y = 0
        self.update()

    def set_goto_mode(self, active):
        self.mode_goto = active
        self.setCursor(Qt.CursorShape.CrossCursor if active else Qt.CursorShape.ArrowCursor)

    def paintEvent(self, event):
        if self.current_frame is None:
            painter = QPainter(self)
            painter.fillRect(self.rect(), Qt.GlobalColor.black)
            return

        painter = QPainter(self)
        img_h, img_w = self.current_frame.shape[:2]
        zoom = self.zoom_levels[self.zoom_idx]
        
        view_w = int(img_w / zoom)
        view_h = int(img_h / zoom)

        max_x = max(0, img_w - view_w)
        max_y = max(0, img_h - view_h)
        self.offset_x = max(0, min(self.offset_x, max_x))
        self.offset_y = max(0, min(self.offset_y, max_y))
        
        cropped = self.current_frame[int(self.offset_y):int(self.offset_y)+view_h, 
                                     int(self.offset_x):int(self.offset_x)+view_w]

        if cropped.size > 0:
            # Lưu ý: frame đã được xử lý (có thể là Gray hoặc BGR)
            # Nếu là ảnh xám (2 chiều), cần convert sang RGB để hiển thị
            if len(cropped.shape) == 2: 
                rgb_frame = cv2.cvtColor(cropped, cv2.COLOR_GRAY2RGB)
            else:
                rgb_frame = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
                
            h, w, ch = rgb_frame.shape
            q_img = QImage(rgb_frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
            painter.drawImage(self.rect(), q_img)

        self.draw_minimap(img_w, img_h, view_w, view_h)

    def draw_minimap(self, img_w, img_h, view_w, view_h):
        if self.minimap_label is None: return

        if len(self.current_frame.shape) == 2:
            rgb_full = cv2.cvtColor(self.current_frame, cv2.COLOR_GRAY2RGB)
        else:
            rgb_full = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
            
        h, w, ch = rgb_full.shape
        q_full = QImage(rgb_full.data, w, h, ch * w, QImage.Format.Format_RGB888)
        pix_minimap = QPixmap.fromImage(q_full).scaled(self.minimap_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
        
        painter = QPainter(pix_minimap)
        painter.setPen(QPen(QColor("red"), 2))
        
        scale_x = pix_minimap.width() / img_w
        scale_y = pix_minimap.height() / img_h
        
        rect_x = int(self.offset_x * scale_x)
        rect_y = int(self.offset_y * scale_y)
        rect_w = int(view_w * scale_x)
        rect_h = int(view_h * scale_y)
        
        painter.drawRect(rect_x, rect_y, rect_w, rect_h)
        painter.end()
        self.minimap_label.setPixmap(pix_minimap)

    # (Các hàm mouse event giữ nguyên như code trước)
    def mousePressEvent(self, event):
        if self.current_frame is None: return
        if event.button() == Qt.MouseButton.LeftButton:
            if self.mode_goto:
                widget_w = self.width()
                widget_h = self.height()
                zoom = self.zoom_levels[self.zoom_idx]
                img_h, img_w = self.current_frame.shape[:2]
                view_w = int(img_w / zoom)
                view_h = int(img_h / zoom)
                click_x = event.pos().x()
                click_y = event.pos().y()
                roi_x = (click_x / widget_w) * view_w
                roi_y = (click_y / widget_h) * view_h
                real_x = self.offset_x + roi_x
                real_y = self.offset_y + roi_y
                center_x = img_w / 2
                center_y = img_h / 2
                delta_x = real_x - center_x
                delta_y = center_y - real_y
                self.pos_selected.emit(int(delta_x), int(delta_y))
                self.set_goto_mode(False)
            elif self.zoom_idx > 0: 
                self.is_panning = True
                self.last_mouse_pos = event.pos()
                self.setCursor(Qt.CursorShape.ClosedHandCursor)
    def mouseMoveEvent(self, event):
        if self.is_panning and self.current_frame is not None:
            delta = event.pos() - self.last_mouse_pos
            self.last_mouse_pos = event.pos()
            zoom = self.zoom_levels[self.zoom_idx]
            img_h, img_w = self.current_frame.shape[:2]
            scale_x = (img_w / zoom) / self.width()
            scale_y = (img_h / zoom) / self.height()
            self.offset_x -= delta.x() * scale_x
            self.offset_y -= delta.y() * scale_y
            self.update()
    def mouseReleaseEvent(self, event):
        if self.is_panning:
            self.is_panning = False
            if not self.mode_goto:
                self.setCursor(Qt.CursorShape.ArrowCursor)

# =========================================================
# 4. WINDOW ĐIỀU KHIỂN CHÍNH
# =========================================================
class ControlCameraWindow(QWidget): 
    send_coordinates = pyqtSignal(int, int)

    def __init__(self):
        super().__init__()
        loadUi("ControlCamera.ui", self)
        self.setWindowTitle("Camera Controller")
        
        self.thread_cam = None
        self.settings_dialog = None
        
        # Dictionary lưu cài đặt hiện tại
        self.current_settings = {
            "brightness": 0, "contrast": 100, 
            "sharpness": 0, "grayscale": False,
            "binary_mode": False, "threshold": 127
        }

        # Setup Viewer
        self.viewer = CameraViewerWidget(self.label_Minimap) 
        layout = QVBoxLayout(self.frame_View)
        layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.viewer)
        
        # Kết nối nút bấm
        self.btn_Pin.clicked.connect(self.toggle_pin_window)
        self.btn_StartCamera.clicked.connect(self.toggle_camera)
        self.btn_Zoom.clicked.connect(self.action_zoom)
        self.btn_GoTo.clicked.connect(self.action_goto)
        
        # --- KẾT NỐI NÚT SETTING MỚI ---
        self.btn_Settings.clicked.connect(self.open_settings)
        
        self.viewer.pos_selected.connect(self.handle_pos_from_viewer)

    def open_settings(self):
        # Mở dialog settings, truyền vào cài đặt hiện tại
        if self.settings_dialog is None:
            self.settings_dialog = SettingCameraDialog(self.current_settings)
            self.settings_dialog.settings_changed.connect(self.update_settings)
        
        # Ghim settings luôn ở trên cùng để dễ chỉnh
        self.settings_dialog.setWindowFlags(Qt.WindowType.WindowStaysOnTopHint)
        self.settings_dialog.show()
        self.settings_dialog.raise_()

    def update_settings(self, new_settings):
        # Hàm này được gọi khi Slider bên Setting thay đổi
        self.current_settings = new_settings
        # Việc áp dụng sẽ diễn ra tự động trong hàm process_frame

    def process_frame(self, frame):
        """Hàm xử lý ảnh dựa trên Setting trước khi hiển thị"""
        
        # 1. Chỉnh Brightness / Contrast
        # CT: new_img = alpha*img + beta
        alpha = self.current_settings["contrast"] / 100.0
        beta = self.current_settings["brightness"]
        frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        
        # 2. Chỉnh Sharpness (Độ nét)
        val = self.current_settings["sharpness"]
        if val > 0:
            # Kernel làm nét cơ bản
            kernel = np.array([[0, -1, 0],
                               [-1, 5 + val/5.0, -1], # Tăng số giữa để tăng độ nét
                               [0, -1, 0]])
            frame = cv2.filter2D(frame, -1, kernel)

        # 3. Chuyển Grayscale
        if self.current_settings["grayscale"] or self.current_settings["binary_mode"]:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 4. Binary Mode (Nhị phân hóa) -> Tốt cho nhận dạng viền
            if self.current_settings["binary_mode"]:
                thresh_val = self.current_settings["threshold"]
                _, frame = cv2.threshold(frame, thresh_val, 255, cv2.THRESH_BINARY)
        
        # Gửi ảnh đã xử lý sang Viewer
        self.viewer.set_frame(frame)

    def toggle_camera(self):
        if self.thread_cam is None:
            idx = int(self.combo_CameraIndex.currentText())
            self.thread_cam = CameraThread(index=idx)
            
            # --- QUAN TRỌNG: Nối signal vào hàm xử lý, thay vì nối thẳng vào viewer ---
            self.thread_cam.change_pixmap_signal.connect(self.process_frame)
            
            self.thread_cam.start()
            self.btn_StartCamera.setText("Stop Cam")
        else:
            self.thread_cam.stop()
            self.thread_cam = None
            self.btn_StartCamera.setText("Start Cam")
            self.viewer.current_frame = None
            self.viewer.update()

    # (Các hàm khác giữ nguyên)
    def toggle_pin_window(self):
        if self.btn_Pin.isChecked():
            self.setWindowFlags(self.windowFlags() | Qt.WindowType.WindowStaysOnTopHint)
            self.btn_Pin.setText("Bỏ Ghim")
            self.show()
        else:
            self.setWindowFlags(self.windowFlags() & ~Qt.WindowType.WindowStaysOnTopHint)
            self.btn_Pin.setText("Ghim")
            self.show()
    def action_zoom(self):
        self.viewer.toggle_zoom()
        zoom_lv = self.viewer.zoom_levels[self.viewer.zoom_idx]
        self.btn_Zoom.setText(f"Zoom: {zoom_lv}x")
    def action_goto(self):
        self.viewer.set_goto_mode(self.btn_GoTo.isChecked())
    def handle_pos_from_viewer(self, x, y):
        self.btn_GoTo.setChecked(False)
        self.send_coordinates.emit(x, y)
    def closeEvent(self, event):
        if self.thread_cam: self.thread_cam.stop()
        if self.settings_dialog: self.settings_dialog.close()
        event.accept()


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

# --- Camera Thread ---
# class CameraThread(QThread):
#     change_pixmap_signal = pyqtSignal(QImage)

#     def __init__(self, index=0):
#         super().__init__()
#         self.index = index
#         self._run_flag = True

#     def run(self):
#         # Mở camera (0 là webcam mặc định, 1 là cam phụ...)
#         cap = cv2.VideoCapture(self.index)
        
#         while self._run_flag:
#             ret, cv_img = cap.read()
#             if ret:
#                 # OpenCV dùng hệ màu BGR, PyQt dùng RGB -> Phải convert
#                 rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
#                 h, w, ch = rgb_image.shape
#                 bytes_per_line = ch * w
#                 convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                
#                 # Resize ảnh cho nhẹ bớt nếu cần (tùy chọn)
#                 p = convert_to_Qt_format.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
                
#                 self.change_pixmap_signal.emit(p)
#             else:
#                 # Nếu mất kết nối camera thì dừng
#                 break
                
#         # Giải phóng camera khi tắt thread
#         cap.release()

#     def stop(self):
#         """Sets run flag to False and waits for thread to finish"""
#         self._run_flag = False
#         self.wait()

# --- Main Window ---
class MainWindow(QMainWindow):
    # Khai báo Signal mới
    preset_data_received = pyqtSignal(str, float, float, float) # char, x, y, z
    def __init__(self):
        super().__init__()
        loadUi("Main_vTest.ui", self)  # Đảm bảo file .ui nằm cùng thư mục
        self.setWindowTitle("STM32 G-Code Controller System")
        self.camera_thread = None
        # try:
        #     self.start_camera()
        # except Exception as e:
        #     print(f"Camera start error: {e}")
        ##########################################################################
        ##########################################################################
        ##########################################################################
        ##########################################################################
        ##########################################################################
        # Khởi tạo cửa sổ camera (nhưng chưa show)
        self.camera_window = ControlCameraWindow()
        
        # Kết nối QAction
        self.actionControl_Camera.triggered.connect(self.show_camera_window)
        
        # Nhận tọa độ trả về
        self.camera_window.send_coordinates.connect(self.on_camera_goto_target)
        
        ##########################################################################
        ##########################################################################
        ##########################################################################
        ##########################################################################
        ##########################################################################
        ##########################################################################
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
    
    ###############################################################################
    ###############################################################################
    ###############################################################################
    ###############################################################################
    def show_camera_window(self):
        # Show dạng modeless (không dùng exec()) để chạy song song
        self.camera_window.show()
        # Đưa lên trên cùng nếu bị che
        self.camera_window.raise_()
        self.camera_window.activateWindow()

    def on_camera_goto_target(self, x, y):
        print(f"Main received Target: X={x}, Y={y}")
    # Hàm hiển thị thông báo "bất tử" (đè lên mọi cửa sổ)
    def show_message(self, title, message, icon_type="info"):
        msg = QMessageBox(self)
        msg.setWindowTitle(title)
        msg.setText(message)
        
        # Chọn icon
        if icon_type == "warning":
            msg.setIcon(QMessageBox.Icon.Warning)
        elif icon_type == "error":
            msg.setIcon(QMessageBox.Icon.Critical)
        else:
            msg.setIcon(QMessageBox.Icon.Information)

        # --- QUAN TRỌNG NHẤT: Set cờ Always On Top cho Popup ---
        # Lệnh này bắt buộc popup phải hiện trên cùng, cao hơn cả Camera Window
        msg.setWindowFlags(msg.windowFlags() | Qt.WindowType.WindowStaysOnTopHint)
        
        msg.exec()
    ###############################################################################
    ###############################################################################
    ###############################################################################
    ###############################################################################
    ###############################################################################
    
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
                self.show_message("Lỗi", "Cổng COM này đang được dùng cho Kim!", "warning")
                # QMessageBox.warning(self, "Lỗi", "Cổng COM này đang được dùng cho Kim!")
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
                self.show_message("Lỗi", "Cổng COM này đang được dùng cho Bàn!", "warning")
                # QMessageBox.warning(self, "Lỗi", "Cổng COM này đang được dùng cho Bàn!")
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
            self.show_message("Lỗi", "Chưa kết nối COM Bàn", "warning")
            # QMessageBox.warning(self, "Lỗi", "Chưa kết nối COM Bàn")

    def send_command_kim(self, cmd):
        if self.thread_kim:
            self.thread_kim.send_data(cmd)
            self.log_to_screen(f"Kim -> {cmd.strip()}")
        else:
            self.show_message("Lỗi", "Chưa kết nối COM Kim", "warning")
            # QMessageBox.warning(self, "Lỗi", "Chưa kết nối COM Kim")

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

    # --- CAMERA HANDLING ---
    def start_camera(self):
        # Index 0 là camera mặc định. Nếu cắm cam ngoài có thể là 1 hoặc 2
        self.camera_thread = CameraThread(index=0)
        self.camera_thread.change_pixmap_signal.connect(self.update_image)
        self.camera_thread.start()
        # Đổi text nút bấm nếu có

    def stop_camera(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread = None
            # Xóa hình cũ trên label
            if hasattr(self, 'Label_Camera'): self.Label_Camera.clear()

    @pyqtSlot(QImage)
    def update_image(self, qt_img):
        """Hàm này nhận ảnh từ Thread và hiển thị lên UI"""
        if hasattr(self, 'Label_Camera'):
            # Scale ảnh cho vừa khít với kích thước của Label trên UI
            scaled_img = qt_img.scaled(self.Label_Camera.width(), self.Label_Camera.height(), Qt.AspectRatioMode.KeepAspectRatio)
            self.Label_Camera.setPixmap(QPixmap.fromImage(scaled_img))
# --- Entry Point ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())