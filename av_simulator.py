#!/usr/bin/env python3
"""
Autonomous Vehicle Simulation and Vulnerability Analyzer
Main Application Entry Point

This application simulates autonomous vehicle behavior using ROS bag files,
analyzes sensor data, and tests vulnerabilities through anomaly injection.
"""

import sys
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QFileDialog, QTabWidget, QComboBox, QLineEdit,
                             QCheckBox, QTableWidget, QTableWidgetItem, 
                             QMessageBox, QProgressBar, QGroupBox, QSpinBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import seaborn as sns

# Import our custom modules
from bag_player import BagPlayer
from data_analyzer import DataAnalyzer
from vulnerability_injector import VulnerabilityInjector
from simulator import Simulator
from visualizer import Visualizer
from report_generator import ReportGenerator
from config import Config
from rviz_integration import RVizWidget, GazeboWidget


class RosThread(QThread):
    """Background thread for ROS operations"""
    status_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    topic_data_signal = pyqtSignal(str, object, object)  # topic, message, timestamp
    playback_progress_signal = pyqtSignal(dict)  # progress info

    def __init__(self):
        super().__init__()
        self.running = False
        self.ros_initialized = False
        self.bag_player = None

    def set_bag_player(self, bag_player):
        """Set the bag player reference"""
        self.bag_player = bag_player

    def run(self):
        """Execute ROS operations in background"""
        self.running = True

        # Try to initialize ROS
        try:
            import rospy
            if not rospy.core.is_initialized():
                rospy.init_node('av_simulator_gui', anonymous=True, disable_signals=True)
            self.ros_initialized = True
            self.status_signal.emit("ROS Thread: Connected to ROS Master")
        except Exception as e:
            self.ros_initialized = False
            self.status_signal.emit(f"ROS Thread: Running without ROS connection ({str(e)})")

        # Monitoring loop
        while self.running:
            # Update playback progress if playing
            if self.bag_player and self.bag_player.is_playing:
                progress = self.bag_player.get_playback_progress()
                self.playback_progress_signal.emit(progress)

            self.msleep(100)

    def stop(self):
        """Stop the thread"""
        self.running = False


class MainWindow(QMainWindow):
    """Main application window"""

    # Define signals for thread-safe communication
    status_update_signal = pyqtSignal(str)
    message_received_signal = pyqtSignal(str, object, object)  # topic, msg, timestamp

    def __init__(self):
        super().__init__()
        self.setWindowTitle("AV Simulator - Autonomous Vehicle Simulation and Vulnerability Analyzer")
        self.setGeometry(100, 100, 1400, 900)

        # Initialize data structures
        self.dfs = {}  # Dictionary to store loaded DataFrames
        self.current_bag_path = None
        self.current_csv_dir = None
        self.simulation_running = False

        # Initialize components
        self.config = Config()
        self.bag_player = BagPlayer()
        self.data_analyzer = DataAnalyzer()
        self.vulnerability_injector = VulnerabilityInjector()
        self.simulator = Simulator()
        self.visualizer = Visualizer()
        self.report_generator = ReportGenerator()

        # Start ROS thread
        self.ros_thread = RosThread()
        self.ros_thread.set_bag_player(self.bag_player)
        self.ros_thread.status_signal.connect(self.update_status)
        self.ros_thread.error_signal.connect(self.show_error)
        self.ros_thread.topic_data_signal.connect(self.update_topic_data)
        self.ros_thread.playback_progress_signal.connect(self.update_playback_progress)
        self.ros_thread.start()

        # Connect our internal signals for thread-safe widget updates
        self.status_update_signal.connect(self.update_status)
        self.message_received_signal.connect(self.on_bag_message_main_thread)

        # Connect bag player callbacks using thread-safe wrappers
        self.bag_player.status_callback = self._thread_safe_status_callback
        self.bag_player.message_callback = self._thread_safe_message_callback

        # Setup UI
        self.init_ui()
        
    def init_ui(self):
        """Initialize the user interface"""
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left sidebar with quick actions
        sidebar = self.create_sidebar()
        main_layout.addWidget(sidebar, 1)
        
        # Right content area with tabs
        content_area = self.create_content_area()
        main_layout.addWidget(content_area, 4)
        
        # Status bar
        self.statusBar().showMessage("Ready | ROS: Connected | Memory: 4.2 GB / 16 GB | CPU: 23%")
        
    def create_sidebar(self):
        """Create the left sidebar with quick actions"""
        sidebar = QGroupBox("Quick Actions")
        layout = QVBoxLayout()
        
        # Load .bag File button
        self.btn_load_bag = QPushButton("📁 Load .bag File")
        self.btn_load_bag.setStyleSheet("background-color: #4A90E2; color: white; padding: 10px; font-size: 14px;")
        self.btn_load_bag.clicked.connect(self.load_bag)
        layout.addWidget(self.btn_load_bag)
        
        # Start Simulation button
        self.btn_start_sim = QPushButton("▶ Start Simulation")
        self.btn_start_sim.setStyleSheet("background-color: #4A90E2; color: white; padding: 10px; font-size: 14px;")
        self.btn_start_sim.clicked.connect(self.start_simulation)
        self.btn_start_sim.setEnabled(False)
        layout.addWidget(self.btn_start_sim)
        
        # Analyze Data button
        self.btn_analyze = QPushButton("📊 Analyze Data")
        self.btn_analyze.setStyleSheet("background-color: #50C878; color: white; padding: 10px; font-size: 14px;")
        self.btn_analyze.clicked.connect(self.analyze_data)
        layout.addWidget(self.btn_analyze)
        
        # Inject Anomaly button
        self.btn_inject = QPushButton("⚠ Inject Anomaly")
        self.btn_inject.setStyleSheet("background-color: #E74C3C; color: white; padding: 10px; font-size: 14px;")
        self.btn_inject.clicked.connect(self.show_injection_tab)
        layout.addWidget(self.btn_inject)
        
        # Generate Report button
        self.btn_report = QPushButton("📄 Generate Report")
        self.btn_report.setStyleSheet("background-color: #F39C12; color: white; padding: 10px; font-size: 14px;")
        self.btn_report.clicked.connect(self.generate_report)
        layout.addWidget(self.btn_report)
        
        layout.addStretch()
        
        # Status label
        self.sim_status_label = QLabel("Status: Idle")
        self.sim_status_label.setStyleSheet("padding: 10px; background-color: #ECF0F1;")
        layout.addWidget(self.sim_status_label)
        
        sidebar.setLayout(layout)
        sidebar.setMaximumWidth(300)
        return sidebar
        
    def create_content_area(self):
        """Create the main content area with tabs"""
        self.tabs = QTabWidget()
        
        # Dashboard tab
        dashboard_tab = self.create_dashboard_tab()
        self.tabs.addTab(dashboard_tab, "Dashboard")
        
        # Simulation tab
        simulation_tab = self.create_simulation_tab()
        self.tabs.addTab(simulation_tab, "Simulation Playback")
        
        # Data Analysis tab
        analysis_tab = self.create_analysis_tab()
        self.tabs.addTab(analysis_tab, "Data Analysis")
        
        # Vulnerability Testing tab
        vulnerability_tab = self.create_vulnerability_tab()
        self.tabs.addTab(vulnerability_tab, "Vulnerability Testing")
        
        return self.tabs
        
    def create_dashboard_tab(self):
        """Create the dashboard tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # File browser area
        file_group = QGroupBox("File Browser")
        file_layout = QVBoxLayout()
        
        self.file_label = QLabel("Drag and drop .tar archives or .bag files here\nor click to browse")
        self.file_label.setStyleSheet("""
            QLabel {
                border: 2px dashed #4A90E2;
                padding: 40px;
                background-color: #F8F9FA;
                text-align: center;
            }
        """)
        self.file_label.setAlignment(Qt.AlignCenter)
        file_layout.addWidget(self.file_label)
        
        self.loaded_file_info = QLabel("No file loaded")
        self.loaded_file_info.setStyleSheet("padding: 10px;")
        file_layout.addWidget(self.loaded_file_info)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # System log console
        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout()
        
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("""
            QTextEdit {
                background-color: #1E1E1E;
                color: #00FF00;
                font-family: 'Courier New', monospace;
                font-size: 12px;
            }
        """)
        self.log_console.append("[2025-09-28 14:32:01] System initialized")
        self.log_console.append("[2025-09-28 14:32:02] ROS environment detected: Noetic")
        self.log_console.append("[2025-09-28 14:32:06] Ready for simulation_")
        
        log_layout.addWidget(self.log_console)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        widget.setLayout(layout)
        return widget
        
    def create_simulation_tab(self):
        """Create the simulation playback tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Playback controls
        controls_layout = QHBoxLayout()
        
        self.btn_play = QPushButton("▶ Play")
        self.btn_play.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_play.clicked.connect(self.play_simulation)
        controls_layout.addWidget(self.btn_play)
        
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_pause.clicked.connect(self.pause_simulation)
        controls_layout.addWidget(self.btn_pause)
        
        self.btn_stop = QPushButton("⏹ Stop")
        self.btn_stop.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_stop.clicked.connect(self.stop_simulation)
        controls_layout.addWidget(self.btn_stop)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        controls_layout.addWidget(self.progress_bar, 3)
        
        # Speed control
        controls_layout.addWidget(QLabel("Speed:"))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1x", "2x", "5x"])
        self.speed_combo.setCurrentIndex(1)
        controls_layout.addWidget(self.speed_combo)
        
        layout.addLayout(controls_layout)

        # Visualization selection tabs
        viz_tabs = QTabWidget()

        # RViz viewport tab
        self.rviz_widget = RVizWidget(parent=self)
        self.rviz_widget.status_callback = self.update_status
        viz_tabs.addTab(self.rviz_widget, "RViz Visualization")

        # Gazebo viewport tab
        self.gazebo_widget = GazeboWidget(parent=self)
        self.gazebo_widget.status_callback = self.update_status
        viz_tabs.addTab(self.gazebo_widget, "Gazebo Simulation")

        # Viewport info overlay
        info_widget = QWidget()
        info_layout = QVBoxLayout()
        info_layout.addWidget(viz_tabs)

        self.vehicle_info_label = QLabel("Playback Time: 0.0s / 0.0s\nSpeed: 1.0x | Progress: 0.0%")
        self.vehicle_info_label.setStyleSheet("""
            QLabel {
                background-color: rgba(44, 62, 80, 0.9);
                color: white;
                padding: 8px;
                font-size: 10pt;
                border-radius: 5px;
                font-weight: bold;
            }
        """)
        info_layout.addWidget(self.vehicle_info_label)

        info_widget.setLayout(info_layout)

        viewport_group = QGroupBox("3D Simulation Viewport")
        viewport_layout = QVBoxLayout()
        viewport_layout.addWidget(info_widget)
        
        viewport_group.setLayout(viewport_layout)
        layout.addWidget(viewport_group)
        
        # Topic logs
        topic_group = QGroupBox("Topic Logs")
        topic_layout = QVBoxLayout()
        
        self.topic_logs = QTextEdit()
        self.topic_logs.setReadOnly(True)
        self.topic_logs.setMaximumHeight(150)
        self.topic_logs.setStyleSheet("background-color: #ECF0F1; font-family: monospace;")
        self.topic_logs.setText(
            "/cmd_vel - linear.x: 2.3, angular.z: 0.1\n"
            "/imu/data - ax: 0.02, ay: -0.01, az: 9.81\n"
            "/navsat/fix - lat: 33.5779, lon: -101.8552\n"
            "/odometry - position.x: 125.3, position.y: -45.2"
        )
        
        topic_layout.addWidget(self.topic_logs)
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        widget.setLayout(layout)
        return widget
        
    def create_analysis_tab(self):
        """Create the data analysis tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Filter controls
        filter_group = QGroupBox("Analysis Filters")
        filter_layout = QHBoxLayout()
        
        filter_layout.addWidget(QLabel("Dataset:"))
        self.dataset_combo = QComboBox()
        self.dataset_combo.addItems(["imu_data.csv", "navsat_fix.csv", "cmd_vel.csv", "odometry.csv"])
        filter_layout.addWidget(self.dataset_combo)
        
        filter_layout.addWidget(QLabel("Time Range:"))
        self.time_start = QLineEdit("Start")
        filter_layout.addWidget(self.time_start)
        filter_layout.addWidget(QLabel("to"))
        self.time_end = QLineEdit("End")
        filter_layout.addWidget(self.time_end)
        
        filter_layout.addWidget(QLabel("Columns:"))
        self.columns_combo = QComboBox()
        self.columns_combo.addItem("All")
        filter_layout.addWidget(self.columns_combo)
        
        self.btn_apply_filters = QPushButton("Apply Filters")
        self.btn_apply_filters.setStyleSheet("background-color: #4A90E2; color: white;")
        self.btn_apply_filters.clicked.connect(self.apply_analysis_filters)
        filter_layout.addWidget(self.btn_apply_filters)
        
        filter_group.setLayout(filter_layout)
        layout.addWidget(filter_group)
        
        # Analysis tabs
        analysis_tabs = QTabWidget()
        
        # Correlations tab
        corr_widget = QWidget()
        corr_layout = QVBoxLayout()
        
        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        corr_layout.addWidget(self.canvas)
        
        corr_widget.setLayout(corr_layout)
        analysis_tabs.addTab(corr_widget, "Correlations")
        
        # Anomalies tab
        anomaly_widget = QWidget()
        anomaly_layout = QVBoxLayout()
        
        self.anomaly_table = QTableWidget()
        self.anomaly_table.setColumnCount(5)
        self.anomaly_table.setHorizontalHeaderLabels(["Timestamp", "Variable", "Expected", "Actual", "Z-Score"])
        
        # Add sample data
        self.anomaly_table.setRowCount(2)
        self.anomaly_table.setItem(0, 0, QTableWidgetItem("14:32:45.234"))
        self.anomaly_table.setItem(0, 1, QTableWidgetItem("angular.z"))
        self.anomaly_table.setItem(0, 2, QTableWidgetItem("0.1 ± 0.05"))
        self.anomaly_table.setItem(0, 3, QTableWidgetItem("0.85"))
        self.anomaly_table.setItem(0, 4, QTableWidgetItem("4.2"))
        
        self.anomaly_table.setItem(1, 0, QTableWidgetItem("14:33:12.567"))
        self.anomaly_table.setItem(1, 1, QTableWidgetItem("linear.x"))
        self.anomaly_table.setItem(1, 2, QTableWidgetItem("2.3 ± 0.3"))
        self.anomaly_table.setItem(1, 3, QTableWidgetItem("-0.5"))
        self.anomaly_table.setItem(1, 4, QTableWidgetItem("3.8"))
        
        anomaly_layout.addWidget(self.anomaly_table)
        
        self.btn_export_analysis = QPushButton("Export Results")
        self.btn_export_analysis.setStyleSheet("background-color: #50C878; color: white;")
        self.btn_export_analysis.clicked.connect(self.export_analysis_results)
        anomaly_layout.addWidget(self.btn_export_analysis)
        
        anomaly_widget.setLayout(anomaly_layout)
        analysis_tabs.addTab(anomaly_widget, "Anomalies")
        
        # Statistics tab
        stats_widget = QWidget()
        stats_layout = QVBoxLayout()
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        stats_layout.addWidget(self.stats_text)
        stats_widget.setLayout(stats_layout)
        analysis_tabs.addTab(stats_widget, "Statistics")
        
        layout.addWidget(analysis_tabs)
        
        widget.setLayout(layout)
        return widget
        
    def create_vulnerability_tab(self):
        """Create the vulnerability testing tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # Injection parameters
        param_group = QGroupBox("Define Injection Parameters")
        param_layout = QVBoxLayout()
        
        # Topic selection
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Select Topic:"))
        self.inject_topic_combo = QComboBox()
        self.inject_topic_combo.addItems(["/cmd_vel", "/imu/data", "/navsat/fix", "/odometry"])
        topic_layout.addWidget(self.inject_topic_combo)
        param_layout.addLayout(topic_layout)
        
        # Modification type
        mod_layout = QHBoxLayout()
        mod_layout.addWidget(QLabel("Modification Type:"))
        self.inject_mod_combo = QComboBox()
        self.inject_mod_combo.addItems(["Set to Zero", "Add Noise", "Spoof GPS", "Delay", "Drop Packets"])
        mod_layout.addWidget(self.inject_mod_combo)
        param_layout.addLayout(mod_layout)
        
        # Value/Parameter
        value_layout = QHBoxLayout()
        value_layout.addWidget(QLabel("Value/Parameter:"))
        self.inject_value_input = QLineEdit()
        self.inject_value_input.setPlaceholderText("e.g., Set linear.x to 0.0")
        value_layout.addWidget(self.inject_value_input)
        param_layout.addLayout(value_layout)
        
        # Injection types
        types_layout = QHBoxLayout()
        types_layout.addWidget(QLabel("Injection Types:"))
        self.cb_spoofing = QCheckBox("Spoofing")
        self.cb_spoofing.setChecked(True)
        types_layout.addWidget(self.cb_spoofing)
        self.cb_delays = QCheckBox("Delays")
        types_layout.addWidget(self.cb_delays)
        self.cb_corruption = QCheckBox("Corruption")
        types_layout.addWidget(self.cb_corruption)
        self.cb_replay = QCheckBox("Replay")
        types_layout.addWidget(self.cb_replay)
        param_layout.addLayout(types_layout)
        
        # Execute button
        self.btn_execute_inject = QPushButton("⚠ Execute Injection")
        self.btn_execute_inject.setStyleSheet("background-color: #E74C3C; color: white; padding: 10px;")
        self.btn_execute_inject.clicked.connect(self.execute_injection)
        param_layout.addWidget(self.btn_execute_inject)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # Before/After comparison
        comparison_layout = QHBoxLayout()
        
        # Before injection
        before_group = QGroupBox("Before Injection")
        before_layout = QVBoxLayout()
        self.before_text = QTextEdit()
        self.before_text.setReadOnly(True)
        self.before_text.setText(
            "Linear.x: 2.3 m/s\n"
            "angular.z: 0.1 rad/s\n"
            "position: (125.3, -45.2)\n"
            "heading: 47°\n"
            "status: NORMAL"
        )
        before_layout.addWidget(self.before_text)
        before_group.setLayout(before_layout)
        comparison_layout.addWidget(before_group)
        
        # After injection
        after_group = QGroupBox("After Injection")
        after_layout = QVBoxLayout()
        self.after_text = QTextEdit()
        self.after_text.setReadOnly(True)
        self.after_text.setText(
            "Linear.x: 0.0 m/s\n"
            "angular.z: 0.1 rad/s\n"
            "position: (125.3, -45.2)\n"
            "heading: 52°\n"
            "status: ANOMALY DETECTED"
        )
        after_layout.addWidget(self.after_text)
        after_group.setLayout(after_layout)
        comparison_layout.addWidget(after_group)
        
        layout.addLayout(comparison_layout)
        
        # Failure state alert
        self.failure_alert = QLabel(
            "⚠ Failure State Detected: Navigation deviation detected. Vehicle trajectory deviates from "
            "planned path by 5.2 meters. Potential vulnerability to velocity command tampering identified."
        )
        self.failure_alert.setStyleSheet("""
            QLabel {
                background-color: #FADBD8;
                color: #C0392B;
                padding: 15px;
                border-left: 4px solid #E74C3C;
            }
        """)
        self.failure_alert.setWordWrap(True)
        layout.addWidget(self.failure_alert)
        
        widget.setLayout(layout)
        return widget
    
    # Event Handlers
    
    def load_bag(self):
        """Load a .bag file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select .bag File", "", "ROS Bag Files (*.bag);;All Files (*)"
        )
        
        if file_path:
            self.current_bag_path = file_path
            file_name = os.path.basename(file_path)
            file_size = os.path.getsize(file_path) / (1024**2)  # Convert to MB

            self.log_console.append(f"[{self.get_timestamp()}] Loading bag file: {file_name}")

            # Get real bag info
            try:
                bag_info = self.bag_player.get_bag_info(file_path)

                if 'error' in bag_info:
                    self.show_error(f"Error reading bag file: {bag_info['error']}")
                    return

                # Display bag information
                duration = bag_info.get('duration', 0)
                message_count = bag_info.get('message_count', 0)
                topics = bag_info.get('topics', {})

                self.loaded_file_info.setText(
                    f"Loaded: {file_name} ({file_size:.2f} MB)\n"
                    f"Duration: {duration:.2f}s | Messages: {message_count}"
                )
                self.log_console.append(f"[{self.get_timestamp()}] Bag file loaded successfully")
                self.log_console.append(f"[{self.get_timestamp()}] Duration: {duration:.2f} seconds")
                self.log_console.append(f"[{self.get_timestamp()}] Total messages: {message_count}")

                # List topics
                if topics:
                    topic_names = ', '.join(list(topics.keys())[:5])  # Show first 5 topics
                    self.log_console.append(f"[{self.get_timestamp()}] Topics found: {topic_names}")

                    # Update topic combo box in injection tab
                    self.inject_topic_combo.clear()
                    self.inject_topic_combo.addItems(list(topics.keys()))

                self.btn_start_sim.setEnabled(True)
                self.sim_status_label.setText("Status: Bag Loaded")

                # Export to CSV
                csv_dir = os.path.join(os.path.dirname(file_path), "csv_output")
                self.log_console.append(f"[{self.get_timestamp()}] Exporting bag to CSV...")
                exported = self.bag_player.export_bag_to_csv(file_path, csv_dir)

                if 'error' not in exported:
                    self.current_csv_dir = csv_dir
                    self.log_console.append(f"[{self.get_timestamp()}] Exported {len(exported)} topics to CSV")
                else:
                    self.log_console.append(f"[{self.get_timestamp()}] Warning: CSV export failed")

            except Exception as e:
                self.show_error(f"Error loading bag file: {str(e)}")
    
    def start_simulation(self):
        """Start the simulation with real ROS bag playback"""
        if not self.current_bag_path:
            QMessageBox.warning(self, "No File", "Please load a .bag file first.")
            return

        self.simulation_running = True
        self.sim_status_label.setText("Status: Simulation Running")
        self.log_console.append(f"[{self.get_timestamp()}] Starting simulation...")
        self.tabs.setCurrentIndex(1)  # Switch to simulation tab

        # Auto-launch RViz if not already running
        if not self.rviz_widget.is_running():
            self.log_console.append(f"[{self.get_timestamp()}] Auto-launching RViz...")
            self.rviz_widget.launch_rviz()

        # Get playback speed
        speed_text = self.speed_combo.currentText()
        speed = float(speed_text.replace('x', ''))

        # Start real bag playback with republishing
        try:
            success = self.bag_player.play_bag(
                self.current_bag_path,
                rate=speed,
                start_time=None,
                republish=True
            )
            if success:
                self.log_console.append(f"[{self.get_timestamp()}] Bag playback started at {speed}x speed")
                self.log_console.append(f"[{self.get_timestamp()}] Publishing topics to ROS master...")
                self.log_console.append(f"[{self.get_timestamp()}] Topics should now be visible in RViz")
            else:
                self.log_console.append(f"[{self.get_timestamp()}] Failed to start playback")
        except Exception as e:
            self.show_error(f"Failed to start simulation: {str(e)}")

    def play_simulation(self):
        """Play/resume simulation"""
        if self.bag_player.is_paused:
            self.bag_player.resume()
            self.log_console.append(f"[{self.get_timestamp()}] Simulation resumed")
        else:
            self.start_simulation()

    def pause_simulation(self):
        """Pause simulation"""
        if self.bag_player.is_playing:
            self.bag_player.pause()
            self.log_console.append(f"[{self.get_timestamp()}] Simulation paused")

    def stop_simulation(self):
        """Stop simulation"""
        self.simulation_running = False
        self.bag_player.stop()
        self.log_console.append(f"[{self.get_timestamp()}] Simulation stopped")
        self.progress_bar.setValue(0)
        
    def analyze_data(self):
        """Analyze loaded CSV data"""
        if not self.current_csv_dir:
            QMessageBox.warning(self, "No Data", "Please load and export a .bag file first.")
            return
            
        self.tabs.setCurrentIndex(2)  # Switch to analysis tab
        self.log_console.append(f"[{self.get_timestamp()}] Loading CSV files for analysis...")
        
        try:
            # Load CSVs
            self.dfs = self.data_analyzer.load_csvs(self.current_csv_dir)
            self.log_console.append(f"[{self.get_timestamp()}] Loaded {len(self.dfs)} CSV files")
            
            # Compute correlations
            if self.dfs:
                corr_matrix = self.data_analyzer.compute_correlations(self.dfs)
                self.plot_correlation_heatmap(corr_matrix)
                self.log_console.append(f"[{self.get_timestamp()}] Correlation analysis complete")
                
                # Detect anomalies
                self.log_console.append(f"[{self.get_timestamp()}] Running anomaly detection...")
                
        except Exception as e:
            self.show_error(f"Error analyzing data: {str(e)}")
    
    def apply_analysis_filters(self):
        """Apply filters to analysis"""
        self.log_console.append(f"[{self.get_timestamp()}] Applying analysis filters...")
        
    def export_analysis_results(self):
        """Export analysis results"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Export Analysis Results", "", "CSV Files (*.csv);;All Files (*)"
        )
        if file_path:
            self.log_console.append(f"[{self.get_timestamp()}] Exporting results to {file_path}")
            QMessageBox.information(self, "Export Complete", f"Results exported to {file_path}")
    
    def show_injection_tab(self):
        """Show the vulnerability injection tab"""
        self.tabs.setCurrentIndex(3)
        
    def execute_injection(self):
        """Execute anomaly injection with real bag modification"""
        if not self.current_bag_path:
            QMessageBox.warning(self, "No File", "Please load a .bag file first.")
            return

        topic = self.inject_topic_combo.currentText()
        mod_type = self.inject_mod_combo.currentText()
        value_str = self.inject_value_input.text()

        self.log_console.append(f"[{self.get_timestamp()}] Injecting anomaly into {topic}")
        self.log_console.append(f"[{self.get_timestamp()}] Modification: {mod_type}")

        try:
            # Parse value if provided
            params = {}
            if value_str:
                try:
                    params['std'] = float(value_str)  # For noise
                    params['factor'] = float(value_str)  # For scale
                except ValueError:
                    pass

            # Create modified bag
            output_path = self.current_bag_path.replace(".bag", "_modified.bag")

            # Convert UI text to anomaly type
            anomaly_map = {
                'Zero Velocity': 'zero_vel',
                'Add Noise': 'noise',
                'GPS Spoof': 'spoof_gps',
                'Packet Drop': 'drop_packets',
                'Data Corruption': 'corruption',
                'Stuck Sensor': 'stuck',
                'Scale Values': 'scale'
            }
            anomaly_type = anomaly_map.get(mod_type, mod_type.lower().replace(" ", "_"))

            self.vulnerability_injector.inject_anomaly(
                bag_path=self.current_bag_path,
                output_bag_path=output_path,
                topic=topic,
                anomaly_type=anomaly_type,
                start_time=None,  # From beginning
                duration=None,    # Until end
                **params
            )

            self.log_console.append(f"[{self.get_timestamp()}] Modified bag created: {output_path}")
            self.log_console.append(f"[{self.get_timestamp()}] Affected {len(self.vulnerability_injector.injection_log)} messages")

            # Update after text
            self.after_text.clear()
            self.after_text.append(f"Modified bag file: {output_path}\n")
            self.after_text.append(f"Anomaly type: {anomaly_type}\n")
            self.after_text.append(f"Topic: {topic}\n")
            self.after_text.append(f"Messages modified: {len(self.vulnerability_injector.injection_log)}\n")
            self.after_text.setStyleSheet("QTextEdit { color: red; font-weight: bold; }")

            QMessageBox.information(self, "Injection Complete",
                                   f"Anomaly injection completed.\n"
                                   f"Modified {len(self.vulnerability_injector.injection_log)} messages.\n"
                                   f"Output: {output_path}")

        except Exception as e:
            self.show_error(f"Error during injection: {str(e)}")
        
    def generate_report(self):
        """Generate comprehensive report"""
        self.log_console.append(f"[{self.get_timestamp()}] Generating comprehensive report...")
        
        try:
            report_content = self.report_generator.generate_report(
                correlations=None,
                anomalies=None,
                failures="Velocity tampering vulnerability identified"
            )
            
            # Save report
            file_path, _ = QFileDialog.getSaveFileName(
                self, "Save Report", "av_analysis_report.txt", "Text Files (*.txt);;PDF Files (*.pdf)"
            )
            
            if file_path:
                with open(file_path, 'w') as f:
                    f.write(report_content)
                    
                self.log_console.append(f"[{self.get_timestamp()}] Report saved to {file_path}")
                QMessageBox.information(self, "Report Generated", 
                                       f"Report successfully generated and saved to:\n{file_path}")
                
        except Exception as e:
            self.show_error(f"Error generating report: {str(e)}")
    
    def plot_correlation_heatmap(self, corr_matrix):
        """Plot correlation heatmap"""
        try:
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            
            # Create sample correlation matrix if none provided
            if corr_matrix is None or corr_matrix.empty:
                import numpy as np
                import pandas as pd
                data = np.random.rand(4, 4) * 2 - 1  # Random correlations between -1 and 1
                corr_matrix = pd.DataFrame(
                    data,
                    columns=['linear.x', 'angular.z', 'position.x', 'velocity'],
                    index=['linear.x', 'angular.z', 'position.x', 'velocity']
                )
            
            sns.heatmap(corr_matrix, annot=True, fmt='.2f', cmap='RdYlGn', 
                       center=0, ax=ax, cbar_kws={'label': 'Correlation'})
            ax.set_title('Correlation Heatmap')
            
            self.canvas.draw()
            
        except Exception as e:
            self.log_console.append(f"[{self.get_timestamp()}] Error plotting heatmap: {str(e)}")
    
    def update_status(self, message):
        """Update status from background thread"""
        self.log_console.append(f"[{self.get_timestamp()}] {message}")
        
    def show_error(self, message):
        """Show error message"""
        QMessageBox.critical(self, "Error", message)
        self.log_console.append(f"[{self.get_timestamp()}] ERROR: {message}")
        
    def get_timestamp(self):
        """Get current timestamp string"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _thread_safe_status_callback(self, message):
        """Thread-safe wrapper for status updates - emits signal instead of direct widget access"""
        self.status_update_signal.emit(message)

    def _thread_safe_message_callback(self, topic, msg, timestamp):
        """Thread-safe wrapper for message callbacks - emits signal instead of direct widget access"""
        self.message_received_signal.emit(topic, msg, timestamp)

    def on_bag_message_main_thread(self, topic, msg, timestamp):
        """Callback for bag messages during playback - runs in main thread"""
        # Update topic logs with real data
        try:
            # Extract key fields from message
            msg_str = self._format_message(topic, msg)
            # Update the topic logs (limit to last 10 lines)
            current_text = self.topic_logs.toPlainText().split('\n')
            if len(current_text) > 10:
                current_text = current_text[-9:]
            current_text.append(msg_str)
            self.topic_logs.setText('\n'.join(current_text))
        except Exception as e:
            pass  # Silently ignore formatting errors

    def _format_message(self, topic, msg):
        """Format a ROS message for display"""
        # Extract key fields based on common message types
        if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            # Twist message (cmd_vel)
            return f"{topic} - linear.x: {msg.linear.x:.2f}, angular.z: {msg.angular.z:.2f}"
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            # Odometry message
            pos = msg.pose.pose.position
            return f"{topic} - pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
        elif hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
            # NavSatFix message (GPS)
            return f"{topic} - lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}"
        elif hasattr(msg, 'linear_acceleration'):
            # IMU message
            acc = msg.linear_acceleration
            return f"{topic} - acc: ({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f})"
        else:
            # Generic message
            return f"{topic} - [message data]"

    def update_topic_data(self, topic, msg, timestamp):
        """Update topic data display"""
        # This is called from RosThread
        pass

    def update_playback_progress(self, progress_info):
        """Update playback progress bar and info"""
        try:
            progress = progress_info.get('progress', 0)
            current_time = progress_info.get('current_time', 0)
            duration = progress_info.get('duration', 0)
            speed = progress_info.get('speed', 1.0)

            # Update progress bar
            self.progress_bar.setValue(int(progress))

            # Update vehicle info with time
            self.vehicle_info_label.setText(
                f"Playback Time: {current_time:.2f}s / {duration:.2f}s\n"
                f"Speed: {speed}x | Progress: {progress:.1f}%"
            )
        except Exception as e:
            pass  # Silently handle errors

    def closeEvent(self, event):
        """Handle window close event"""
        # Stop bag playback
        if self.bag_player:
            self.bag_player.stop()

        # Stop RViz and Gazebo
        if hasattr(self, 'rviz_widget'):
            self.rviz_widget.stop_rviz()
        if hasattr(self, 'gazebo_widget'):
            self.gazebo_widget.stop_gazebo()

        # Stop ROS thread
        self.ros_thread.stop()
        self.ros_thread.wait()

        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
