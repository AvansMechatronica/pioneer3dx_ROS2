#!/usr/bin/env python3
"""
Pioneer 3DX Status Monitor GUI.

This module provides a graphical interface for monitoring the status of a Pioneer 3DX robot.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                              QHBoxLayout, QLabel, QGroupBox, QGridLayout,
                              QPushButton, QTextEdit)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont

# ROS2 message types
from p3dx_interfaces.msg import Status
from std_msgs.msg import Bool


class ROS2Monitor(Node, QObject):
    """ROS2 node for monitoring robot status."""

    # Qt signal for thread-safe GUI updates
    status_updated = pyqtSignal(dict)
    status_message = pyqtSignal(str)

    def __init__(self):
        """Initialize the ROS2 monitor node."""
        Node.__init__(self, 'p3dx_status_monitor')
        QObject.__init__(self)

        # QoS profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Latest data storage
        self.status_data = {}

        # Create subscription to status topic
        self.status_sub = self.create_subscription(
            Status,
            'p3dx/status',
            self.status_callback,
            qos_profile
        )

        # Create publisher for reset topic
        self.reset_pub = self.create_publisher(Bool, 'p3dx/reset', 10)

        self.get_logger().info('P3DX Status Monitor initialized')
        self.status_message.emit('Status Monitor initialized - listening on p3dx/status topic')

    def status_callback(self, msg):
        """Process status messages."""
        self.status_data = {
            'motor_enable': msg.motor_enable,
            'error': msg.error,
            'battery_voltage': msg.battery_voltage,
            'wifi_rssi': msg.wifi_rssi,
            'bumper_front': msg.bumpers.front,
            'bumper_rear': msg.bumpers.rear
        }
        self.status_updated.emit(self.status_data)

    def publish_reset(self):
        """Publish reset command."""
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        self.get_logger().info('Reset command published')
        self.status_message.emit('Reset command sent')


class StatusMonitorGUI(QMainWindow):
    """Main GUI window for P3DX status monitoring."""

    def __init__(self, ros_monitor):
        """Initialize the GUI."""
        super().__init__()
        self.ros_monitor = ros_monitor
        self.init_ui()
        self.connect_signals()

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle('Pioneer 3DX Status Monitor')
        self.setGeometry(100, 100, 600, 500)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Title
        title = QLabel('Pioneer 3DX Robot Status Monitor')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Create status display
        self.create_status_display(main_layout)

        # Create control buttons
        self.create_control_buttons(main_layout)

        # Create log section
        self.create_log_section(main_layout)

        # Status bar
        self.statusBar().showMessage('Ready')

    def create_status_display(self, parent_layout):
        """Create the main status display."""
        # System Status group
        system_group = QGroupBox('System Status')
        system_layout = QGridLayout()

        self.motor_enable_label = self.create_value_label()
        self.error_label = self.create_value_label()

        system_layout.addWidget(QLabel('Motor Enable:'), 0, 0)
        system_layout.addWidget(self.motor_enable_label, 0, 1)
        system_layout.addWidget(QLabel('Error:'), 1, 0)
        system_layout.addWidget(self.error_label, 1, 1)

        system_group.setLayout(system_layout)
        parent_layout.addWidget(system_group)

        # Power group
        power_group = QGroupBox('Power')
        power_layout = QGridLayout()

        self.battery_voltage_label = self.create_value_label()
        power_layout.addWidget(QLabel('Battery Voltage:'), 0, 0)
        power_layout.addWidget(self.battery_voltage_label, 0, 1)
        power_layout.addWidget(QLabel('V'), 0, 2)

        power_group.setLayout(power_layout)
        parent_layout.addWidget(power_group)

        # WiFi group
        wifi_group = QGroupBox('WiFi')
        wifi_layout = QGridLayout()

        self.wifi_rssi_label = self.create_value_label()
        wifi_layout.addWidget(QLabel('Signal Strength (RSSI):'), 0, 0)
        wifi_layout.addWidget(self.wifi_rssi_label, 0, 1)
        wifi_layout.addWidget(QLabel('dBm'), 0, 2)

        wifi_group.setLayout(wifi_layout)
        parent_layout.addWidget(wifi_group)

        # Bumpers group
        bumpers_group = QGroupBox('Bumpers')
        bumpers_layout = QGridLayout()

        self.bumper_front_label = self.create_value_label()
        self.bumper_rear_label = self.create_value_label()

        bumpers_layout.addWidget(QLabel('Front Bumper:'), 0, 0)
        bumpers_layout.addWidget(self.bumper_front_label, 0, 1)
        bumpers_layout.addWidget(QLabel('Rear Bumper:'), 1, 0)
        bumpers_layout.addWidget(self.bumper_rear_label, 1, 1)

        bumpers_group.setLayout(bumpers_layout)
        parent_layout.addWidget(bumpers_group)

    def create_control_buttons(self, parent_layout):
        """Create control buttons section."""
        control_group = QGroupBox('Controls')
        control_layout = QHBoxLayout()

        # Reset button
        self.reset_btn = QPushButton('Reset Robot')
        self.reset_btn.setStyleSheet('QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }')
        self.reset_btn.clicked.connect(self.on_reset_clicked)
        control_layout.addWidget(self.reset_btn)

        control_group.setLayout(control_layout)
        parent_layout.addWidget(control_group)

    def create_log_section(self, parent_layout):
        """Create the log section."""
        log_group = QGroupBox('Log')
        log_layout = QVBoxLayout()

        # Log text area
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)

        # Clear button
        clear_btn = QPushButton('Clear Log')
        clear_btn.clicked.connect(self.log_text.clear)
        log_layout.addWidget(clear_btn)

        log_group.setLayout(log_layout)
        parent_layout.addWidget(log_group)

    def create_value_label(self):
        """Create a label for displaying values."""
        label = QLabel('--')
        label.setStyleSheet('QLabel { background-color: #f0f0f0; padding: 5px; border: 1px solid #ccc; }')
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        label.setMinimumWidth(100)
        return label

    def connect_signals(self):
        """Connect ROS2 signals to GUI update slots."""
        self.ros_monitor.status_updated.connect(self.update_status)
        self.ros_monitor.status_message.connect(self.log_message)

    def update_status(self, data):
        """Update status display."""
        # Motor enable
        motor_enable = data.get('motor_enable', False)
        self.motor_enable_label.setText('Enabled' if motor_enable else 'Disabled')
        if motor_enable:
            self.motor_enable_label.setStyleSheet(
                'QLabel { background-color: #90EE90; padding: 5px; border: 1px solid #ccc; font-weight: bold; }'
            )
        else:
            self.motor_enable_label.setStyleSheet(
                'QLabel { background-color: #FFD700; padding: 5px; border: 1px solid #ccc; font-weight: bold; }'
            )

        # Error status
        error = data.get('error', False)
        self.error_label.setText('ERROR' if error else 'OK')
        if error:
            self.error_label.setStyleSheet(
                'QLabel { background-color: #FF6B6B; color: white; padding: 5px; border: 1px solid #ccc; font-weight: bold; }'
            )
        else:
            self.error_label.setStyleSheet(
                'QLabel { background-color: #90EE90; padding: 5px; border: 1px solid #ccc; font-weight: bold; }'
            )

        # Battery voltage
        voltage = data.get('battery_voltage', 0.0)
        self.battery_voltage_label.setText(f"{voltage:.2f}")
        # Color code based on voltage (assuming 12V system, typical range 11-14V)
        if voltage > 13.0:
            self.battery_voltage_label.setStyleSheet(
                'QLabel { background-color: #90EE90; padding: 5px; border: 1px solid #ccc; }'
            )
        elif voltage > 11.5:
            self.battery_voltage_label.setStyleSheet(
                'QLabel { background-color: #FFD700; padding: 5px; border: 1px solid #ccc; }'
            )
        else:
            self.battery_voltage_label.setStyleSheet(
                'QLabel { background-color: #FF6B6B; padding: 5px; border: 1px solid #ccc; }'
            )

        # WiFi RSSI
        rssi = data.get('wifi_rssi', 0)
        self.wifi_rssi_label.setText(f"{rssi}")
        # Color code WiFi signal strength
        if rssi > -60:
            self.wifi_rssi_label.setStyleSheet(
                'QLabel { background-color: #90EE90; padding: 5px; border: 1px solid #ccc; }'
            )
        elif rssi > -80:
            self.wifi_rssi_label.setStyleSheet(
                'QLabel { background-color: #FFD700; padding: 5px; border: 1px solid #ccc; }'
            )
        else:
            self.wifi_rssi_label.setStyleSheet(
                'QLabel { background-color: #FF6B6B; padding: 5px; border: 1px solid #ccc; }'
            )

        # Bumpers
        front = data.get('bumper_front', False)
        rear = data.get('bumper_rear', False)
        
        self.bumper_front_label.setText('PRESSED' if front else 'Released')
        if front:
            self.bumper_front_label.setStyleSheet(
                'QLabel { background-color: #FF6B6B; color: white; padding: 5px; border: 1px solid #ccc; font-weight: bold; }'
            )
        else:
            self.bumper_front_label.setStyleSheet(
                'QLabel { background-color: #f0f0f0; padding: 5px; border: 1px solid #ccc; }'
            )

        self.bumper_rear_label.setText('PRESSED' if rear else 'Released')
        if rear:
            self.bumper_rear_label.setStyleSheet(
                'QLabel { background-color: #FF6B6B; color: white; padding: 5px; border: 1px solid #ccc; font-weight: bold; }'
            )
        else:
            self.bumper_rear_label.setStyleSheet(
                'QLabel { background-color: #f0f0f0; padding: 5px; border: 1px solid #ccc; }'
            )

    def on_reset_clicked(self):
        """Handle reset button click."""
        self.ros_monitor.publish_reset()

    def log_message(self, message):
        """Add a message to the log."""
        from datetime import datetime
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f"[{timestamp}] {message}")
        self.statusBar().showMessage(message)


def main(args=None):
    """Main function to run the status monitor."""
    # Initialize ROS2
    rclpy.init(args=args)

    # Create Qt application
    app = QApplication(sys.argv)

    # Create ROS2 monitor node
    ros_monitor = ROS2Monitor()

    # Create GUI
    gui = StatusMonitorGUI(ros_monitor)
    gui.show()

    # Set up timer to spin ROS2
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_monitor, timeout_sec=0.01))
    timer.start(50)  # 20 Hz update rate

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    ros_monitor.destroy_node()
    rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
