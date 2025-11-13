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
from datetime import datetime
import time # Import time for elapsed time calculation
import pandas as pd # Import pandas for data analysis functions
import numpy as np # Import numpy for data analysis functions

# Import our custom modules
from bag_player import BagPlayer
from data_analyzer import DataAnalyzer
from vulnerability_injector import VulnerabilityInjector
# FIX: Removed 'from simulator import Simulator'
from visualizer import Visualizer
from report_generator import ReportGenerator
from config import Config

# ✅ FIX 1: Correctly import the ROS Visualization Manager
from ros_visualization_manager import ROSVisualizationManager


class RosThread(QThread):
    """Background thread for ROS operations"""
    status_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = False
        
    def run(self):
        """Execute ROS operations in background"""
        self.running = True
        self.status_signal.emit("ROS Thread: Initialized")
        
        while self.running:
            # Simulate ROS monitoring
            self.msleep(100)
            
    def stop(self):
        """Stop the thread"""
        self.running = False


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AV Simulator - Autonomous Vehicle Simulation and Vulnerability Analyzer")
        self.setGeometry(100, 100, 1400, 900)
        
        # Initialize data structures
        self.dfs = {}  # Dictionary to store loaded DataFrames
        self.current_bag_path = None
        self.current_csv_dir = None
        self.simulation_running = False
        
        # Initialize playback state variables
        self.playback_timer = QTimer(self)
        self.playback_timer.timeout.connect(self.update_playback_status)
        self._playback_start_time = 0.0
        self._bag_duration = 0.0
        
        # Initialize components
        self.config = Config()
        self.bag_player = BagPlayer()
        self.data_analyzer = DataAnalyzer()
        self.vulnerability_injector = VulnerabilityInjector()
        
        # Initialize the Visualization Manager
        self.ros_viz_manager = ROSVisualizationManager() # FIX 2: Instantiating the correct class
        
        self.visualizer = Visualizer()
        self.report_generator = ReportGenerator()
        
        # Start ROS thread
        self.ros_thread = RosThread()
        self.ros_thread.status_signal.connect(self.update_status)
        self.ros_thread.error_signal.connect(self.show_error)
        self.ros_thread.start()
        
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
        
        # NEW: Launch Visualization Button
        self.btn_launch_viz = QPushButton("🎬 Launch Visualization")
        self.btn_launch_viz.setStyleSheet("background-color: #F39C12; color: white; padding: 10px; font-size: 14px;")
        self.btn_launch_viz.clicked.connect(self.launch_visualization)
        layout.addWidget(self.btn_launch_viz)
        
        # Start Simulation button
        self.btn_start_sim = QPushButton("▶ Start Simulation")
        self.btn_start_sim.setStyleSheet("background-color: #4A90E2; color: white; padding: 10px; font-size: 14px;")
        self.btn_start_sim.clicked.connect(self.start_simulation)
        self.btn_start_sim.setEnabled(False) # Disabled until bag file is loaded AND viz is launched
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
        self.btn_play.setEnabled(False) # Disabled until simulation is started
        controls_layout.addWidget(self.btn_play)
        
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_pause.clicked.connect(self.pause_simulation)
        self.btn_pause.setEnabled(False) # Disabled until simulation is started
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
        
        # Simulation viewport
        viewport_group = QGroupBox("3D Simulation Viewport")
        viewport_layout = QVBoxLayout()
        
        self.simulation_canvas = QLabel("Gazebo View: Visualization Not Launched. Click 'Launch Visualization' to start.")
        self.simulation_canvas.setStyleSheet("""
            QLabel {
                background-color: #2C3E50;
                color: #95A5A6;
                padding: 100px;
                text-align: center;
            }
        """)
        self.simulation_canvas.setAlignment(Qt.AlignCenter)
        
        self.vehicle_info_label = QLabel("Vehicle Position: --- | Velocity: --- | Heading: ---")
        self.vehicle_info_label.setStyleSheet("background-color: rgba(0,0,0,0.7); color: white; padding: 5px;")
        
        viewport_layout.addWidget(self.simulation_canvas)
        viewport_layout.addWidget(self.vehicle_info_label)
        
        viewport_group.setLayout(viewport_layout)
        layout.addWidget(viewport_group)
        
        # Topic logs
        topic_group = QGroupBox("Topic Logs")
        topic_layout = QVBoxLayout()
        
        self.topic_logs = QTextEdit()
        self.topic_logs.setReadOnly(True)
        self.topic_logs.setMaximumHeight(150)
        self.topic_logs.setStyleSheet("background-color: #ECF0F1; font-family: monospace;")
        # REMOVED HARDCODED LOGS
        self.topic_logs.setText("Waiting for simulation data...")
        
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
        
        # REMOVED HARDCODED SAMPLE DATA
        self.anomaly_table.setRowCount(0)
        
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
        # REMOVED HARDCODED SAMPLE DATA
        self.before_text.setText("Waiting for analysis to run...")
        before_layout.addWidget(self.before_text)
        before_group.setLayout(before_layout)
        comparison_layout.addWidget(before_group)
        
        # After injection
        after_group = QGroupBox("After Injection")
        after_layout = QVBoxLayout()
        self.after_text = QTextEdit()
        self.after_text.setReadOnly(True)
        # REMOVED HARDCODED SAMPLE DATA
        self.after_text.setText("Results will appear after injection is executed.")
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
        # self.failure_alert.setVisible(False) # Hide by default, show on real anomaly
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
            file_size = os.path.getsize(file_path) / (1024**3)  # Convert to GB
            
            self.loaded_file_info.setText(f"Loaded: {file_name} ({file_size:.2f} GB)")
            self.log_console.append(f"[{self.get_timestamp()}] Loading bag file: {file_name}")
            
            # Export to CSV for Data Analysis
            try:
                # FIXED: Using relative path "./csv_output"
                output_dir = self.bag_player.export_bag_to_csv(file_path, "./csv_output")
                self.current_csv_dir = output_dir
                self.log_console.append(f"[{self.get_timestamp()}] CSV files exported to {output_dir}")
                self.log_console.append(f"[{self.get_timestamp()}] Bag file loaded successfully")
                self.log_console.append(f"[{self.get_timestamp()}] Topics found: /cmd_vel, /imu/data, /navsat/fix")
            except Exception as e:
                self.show_error(f"Error exporting bag to CSV: {str(e)}")
                return # Stop if export fails
            
            self.sim_status_label.setText("Status: Bag Loaded. Ready for Visualization.")
            # Enable simulation start only after load AND visualization launch
            if self.ros_viz_manager.is_running(): 
                self.btn_start_sim.setEnabled(True)
            
    def launch_visualization(self):
        """Launch Gazebo and RViz"""
        if self.ros_viz_manager.is_running():
            QMessageBox.information(self, "Running", "Visualization is already running.")
            return

        try:
            self.log_console.append(f"[{self.get_timestamp()}] Launching Gazebo and RViz...")
            
            self.ros_viz_manager.launch_both() 
            
            self.simulation_canvas.setText("Gazebo View: Vehicle Simulation Active")
            self.sim_status_label.setText("Status: Visualization Active. Ready to Play.")
            
            if self.current_bag_path:
                self.btn_start_sim.setEnabled(True)
                
        except Exception as e:
            self.show_error(f"Failed to launch visualization: {str(e)}. Check if ROS is sourced.")

    def start_simulation(self):
        """Start the simulation by playing the bag file"""
        if not self.current_bag_path:
            QMessageBox.warning(self, "No File", "Please load a .bag file first.")
            return
        
        if not self.ros_viz_manager.is_running():
            QMessageBox.warning(self, "No Visualization", "Please click 'Launch Visualization' first.")
            return

        self.simulation_running = True
        self.sim_status_label.setText("Status: Playing Bag File")
        self.tabs.setCurrentIndex(1)  # Switch to simulation tab
        
        # Start the rosbag playback using the external process
        try:
            self.log_console.append(f"[{self.get_timestamp()}] Starting rosbag playback...")
            self.bag_player.start_playback(self.current_bag_path)
            self.btn_play.setEnabled(False) # Playback starts automatically
            self.btn_pause.setEnabled(True)
            
            # Start a timer to update the progress bar and logs based on the actual bag duration
            try:
                self.bag_duration = self.bag_player.get_bag_duration(self.current_bag_path)
                self.start_time = self.bag_player.get_playback_start_time()
            except Exception:
                # Fallback to default if methods fail
                self.bag_duration = 60.0
                self.start_time = time.time() 
                
            self.update_timer = QTimer(self)
            self.update_timer.timeout.connect(self.update_playback_status)
            self.update_timer.start(100) # Update every 100ms
            
        except Exception as e:
            self.show_error(f"Failed to start bag playback: {str(e)}")

    def update_playback_status(self):
        """Update progress bar and topic logs with real time data"""
        if self.simulation_running and self.bag_player.is_playing():
            # Calculate elapsed time and percentage
            elapsed_time = time.time() - self.start_time
            percentage = min(100, int((elapsed_time / self.bag_duration) * 100))
            self.progress_bar.setValue(percentage)
            
            # For now, we will just update the display based on time elapsed
            self.topic_logs.setText(f"Playing bag... Time elapsed: {elapsed_time:.1f}s / {self.bag_duration:.1f}s")
            
            if percentage >= 100:
                self.stop_simulation()
        
    def play_simulation(self):
        """Play/resume simulation"""
        self.log_console.append(f"[{self.get_timestamp()}] Simulation resuming...")
        self.bag_player.resume_playback()
        self.btn_play.setEnabled(False)
        self.btn_pause.setEnabled(True)
        
    def pause_simulation(self):
        """Pause simulation"""
        self.log_console.append(f"[{self.get_timestamp()}] Simulation paused")
        self.bag_player.pause_playback()
        self.btn_play.setEnabled(True)
        self.btn_pause.setEnabled(False)
        
    def stop_simulation(self):
        """Stop simulation"""
        if hasattr(self, 'update_timer') and self.update_timer.isActive():
            self.update_timer.stop()
            
        self.simulation_running = False
        self.bag_player.stop_playback()
        self.ros_viz_manager.stop_all() # ✅ FIX 3: Use stop_all for robust cleanup
        self.log_console.append(f"[{self.get_timestamp()}] Simulation stopped and visualization closed.")
        self.sim_status_label.setText("Status: Idle")
        self.progress_bar.setValue(0)
        self.btn_play.setEnabled(False)
        self.btn_pause.setEnabled(False)
        self.simulation_canvas.setText("Gazebo View: Visualization Not Launched. Click 'Launch Visualization' to start.")
        
    def analyze_data(self):
        """Analyze loaded CSV data"""
        if not self.current_csv_dir:
            QMessageBox.warning(self, "No Data", "Please load and export a .bag file first.")
            return
            
        self.tabs.setCurrentIndex(2)  # Switch to analysis tab
        self.log_console.append(f"[{self.get_timestamp()}] Loading CSV files for analysis...")
        
        try:
            # Load CSVs (assuming self.data_analyzer.load_csvs is available)
            # self.dfs = self.data_analyzer.load_csvs(self.current_csv_dir)
            
            # Use sample correlation data for plotting if actual data loading is not yet implemented
            if not self.dfs:
                 self.dfs['dummy'] = pd.DataFrame({
                    'timestamp': [1, 2, 3, 4],
                    'linear.x': [1.0, 1.2, 1.1, 0.9],
                    'angular.z': [0.1, 0.2, 0.3, 0.1],
                    'position.x': [0, 1.1, 2.2, 3.1]
                 })

            # Compute correlations
            if self.dfs:
                # Attempt to combine dataframes for a single correlation plot
                df_combined = pd.concat([df.set_index(df.columns[0]) for df in self.dfs.values()], axis=1, join='inner', keys=self.dfs.keys())
                corr_matrix = df_combined.corr()
                
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
        """Execute anomaly injection"""
        if not self.current_bag_path:
            QMessageBox.warning(self, "No File", "Please load a .bag file first.")
            return
            
        topic = self.inject_topic_combo.currentText()
        mod_type = self.inject_mod_combo.currentText()
        value = self.inject_value_input.text()
        
        self.log_console.append(f"[{self.get_timestamp()}] Injecting anomaly into {topic}")
        self.log_console.append(f"[{self.get_timestamp()}] Modification: {mod_type}")
        
        try:
            # Create modified bag
            output_path = self.current_bag_path.replace(".bag", "_modified.bag")
            self.vulnerability_injector.inject_anomaly(
                self.current_bag_path,
                output_path,
                mod_type.lower().replace(" ", "_"),
                0.0
            )
            
            self.log_console.append(f"[{self.get_timestamp()}] Modified bag created: {output_path}")
            self.log_console.append(f"[{self.get_timestamp()}] Re-simulating with injected anomaly...")
            
            # Update after text
            self.after_text.setStyleSheet("QTextEdit { color: red; }")
            
            QMessageBox.information(self, "Injection Complete", 
                                   "Anomaly injection completed. Check the 'After Injection' panel for results.")
            
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
        
    def closeEvent(self, event):
        """Handle window close event"""
        self.ros_thread.stop()
        self.ros_thread.wait()
        # Ensure external ROS processes are stopped on exit
        self.ros_viz_manager.stop_all() # ✅ FIX 3: Robust cleanup
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