# app.py
import os
import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox,QFileDialog, QDialog, QVBoxLayout, QTextEdit, QPushButton
from PyQt6.QtCore import QThread, pyqtSignal, QObject,QTimer
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

# CLASS: MainWindow
# The main application window and logic.
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # Load the UI file created with Qt Designer
        loadUi("Main_v4.ui", self) 

        self.serial_port = None
        self.serial_thread = None
        self.serial_worker = None
        #for macro running
        self.command_queue = []     # This will hold our sequence of commands
        self.previous_state = "Unknown" # To detect state changes

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
        self.jogXPlusButton.clicked.connect(lambda: self.send_command(f"G0 X{ float(self.xPosLabel.text()) + float(self.txt_input_intervalDistance.text()) } \n"))
        self.jogXMinusButton.clicked.connect(lambda: self.send_command(f"G0 X{float( self.xPosLabel.text()) - float(self.txt_input_intervalDistance.text()) } \n"))
        self.jogYPlusButton.clicked.connect(lambda: self.send_command(f"G0 Y{float( self.yPosLabel.text()) + float( self.txt_input_intervalDistance.text()) } \n"))
        self.jogYMinusButton.clicked.connect(lambda: self.send_command(f"G0 Y{float( self.yPosLabel.text()) -  float(self.txt_input_intervalDistance.text()) } \n"))
        self.jogZPlusButton.clicked.connect(lambda: self.send_command(f"G0 Z{float(self.zPosLabel.text()) +   float(self.txt_input_intervalDistance.text()) } \n"))
        self.jogZMinusButton.clicked.connect(lambda: self.send_command(f"G0 Z{float(self.zPosLabel.text()) - float(self.txt_input_intervalDistance.text()) } \n"))
        
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
        
        # -- Import GCode ---
        self.importButton.clicked.connect(self.import_gcode_file)
        # --- Run GCode ---
        self.RunGCodeButton.clicked.connect(self.run_imported_gcode_file)
        # --- Macros ---
        self.runMacroButton.clicked.connect(self.start_macro)
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
        if command == "G0 ":
            QMessageBox.warning(self, "Invalid Input", "Please enter at least one valid coordinate.")
            return
        else:
            command += "\n"
            # command = f"G0 X{x} Y{y} Z{z}\n"
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
            "G-code Files (*.gcode *.nc *.txt);;All Files (*)"
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

        # --- Start running line by line ---
        self.run_next_macro_command()
    def start_macro(self):
        """
        Defines and starts a sequence of G-code commands.
        This function is called when the "Run Macro" button is clicked.
        """
        if self.command_queue:
            QMessageBox.warning(self, "Macro Running", "A macro is already in progress.")
            return
            
        # --- DEFINE YOUR COMMAND SEQUENCE HERE ---
        self.command_queue = [
            "G0 X1 Y3 Z5",   # Move 1
            "G1 Z-1",     # Move 2
            "G1 X2 Y4 ", # Move 3
            "G0 Z5",           # Move 4
            "G0 X0 Y0"         # Return to home
        ]
        # -----------------------------------------
        
        self.logTextEdit.append("--- Starting Macro ---")
        self.run_next_macro_command()
    def _check_auto_continue(self):
        # """
        # If machine is still Idle but queue still has commands, continue automatically.
        # """
        # if self.previous_state == "Idle" and self.command_queue:
        #     self.run_next_macro_command()
        """
        If machine is still Idle but queue still has commands, continue automatically.
        Otherwise, if queue is empty, finish the macro.
        """
        if not self.command_queue:
            # Không còn lệnh nào
            self.logTextEdit.append("--- Send GCode Finished ---")
            return

        if self.previous_state == "Idle":
            self.run_next_macro_command()
    def run_next_macro_command(self):
        """
        Sends the next command from the queue if the port is open and the queue is not empty.
        """
        if self.serial_port and self.serial_port.is_open and self.command_queue:
            command = self.command_queue.pop(0) # Get and remove the first command
            self.send_command(command)
            # Nếu vẫn Idle (không thay đổi trạng thái), tự tiếp tục sau 0.5 giây
            QTimer.singleShot(500, self._check_auto_continue)
        else:
            self.logTextEdit.append("--- Macro Finished ---")
            self.command_queue.clear()
            return
    def update_ui(self, data):
        
        # Parse GRBL-style status reports: <State|WPos:X,Y,Z>
        if data.startswith('<') and data.endswith('>'):
            parts = data[1:-1].split('|')
            #Identify state change for macro continuation
            state_str = "Unknown"
            if len(parts) >= 1:
                state_str = parts[0]
                self.stateLabel.setText(parts[0]) # Update state (Idle, Run, etc.)

                # If the machine was busy before and is now Idle, it means a move has just finished.
                if self.previous_state != "Idle" and state_str == "Idle":
                    # If there are still commands left in our queue, run the next one.
                    if self.command_queue:
                        self.run_next_macro_command()
                # Update the UI and store the current state for the next check
                self.previous_state = state_str
                self.stateLabel.setText(state_str)
            if len(parts) >= 2 and parts[1].startswith("WPos:"):
                coords_str = parts[1][5:] # Get "X,Y,Z"
                coords = coords_str.split(',')
                if len(coords) == 3:
                    self.xPosLabel.setText(coords[0])
                    self.yPosLabel.setText(coords[1])
                    self.zPosLabel.setText(coords[2])
        else:
            # Regular log message
            self.logTextEdit.append(f"<-- {data}")
    def closeEvent(self, event):
        """Ensure disconnection on window close."""
        self.disconnect_serial()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())