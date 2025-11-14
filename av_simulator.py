#!/usr/bin/env python3
"""
Autonomous Vehicle Simulation and Vulnerability Analyzer
Main Application Entry Point - WITH ASYNC BAG LOADING

CHANGES FROM ORIGINAL:
1. Added BagExportProgressDialog class for showing progress
2. Updated load_bag() to use async export with progress updates
3. GUI stays responsive during 10GB+ bag file loading
"""

import sys
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QTextEdit, 
                             QFileDialog, QTabWidget, QComboBox, QLineEdit,
                             QCheckBox, QTableWidget, QTableWidgetItem, 
                             QMessageBox, QProgressBar, QGroupBox, QSpinBox,
                             QDialog)  # Added QDialog
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, pyqtSlot  # Added pyqtSlot
from PyQt5.QtGui import QFont, QColor
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import time
import pandas as pd
import numpy as np

# Import our custom modules
from bag_player import BagPlayer
from data_analyzer import DataAnalyzer
from vulnerability_injector import VulnerabilityInjector
from visualizer import Visualizer
from report_generator import ReportGenerator
from config import Config
from ros_visualization_manager import ROSVisualizationManager


# ============================================================================
# NEW: Progress Dialog for Bag Export
# ============================================================================

class BagExportProgressDialog(QDialog):
    """
    Progress dialog for bag file export
    Shows real-time progress and allows cancellation
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Loading Bag File")
        self.setModal(True)
        self.setMinimumWidth(500)
        
        # Create layout
        layout = QVBoxLayout()
        
        # Status label
        self.status_label = QLabel("Preparing to load bag file...")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        layout.addWidget(self.progress_bar)
        
        # Details label
        self.details_label = QLabel("")
        self.details_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(self.details_label)
        
        # Current topic label
        self.topic_label = QLabel("")
        self.topic_label.setStyleSheet("font-style: italic;")
        layout.addWidget(self.topic_label)
        
        # Cancel button
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.on_cancel)
        layout.addWidget(self.cancel_btn)
        
        self.setLayout(layout)
        
        self.export_thread = None
        self.was_cancelled = False
        
    def set_export_thread(self, thread):
        """Set the export thread to monitor"""
        self.export_thread = thread
        
    def on_cancel(self):
        """Handle cancel button click"""
        if self.export_thread:
            self.status_label.setText("Cancelling export...")
            self.cancel_btn.setEnabled(False)
            self.export_thread.cancel()
            self.was_cancelled = True
            
    @pyqtSlot(int, int, str)
    def update_progress(self, current, total, message):
        """Update progress display"""
        if total > 0:
            percentage = int((current / total) * 100)
            self.progress_bar.setValue(percentage)
            self.details_label.setText(f"{current:,} / {total:,} messages")
        
        self.status_label.setText(message)
        
    @pyqtSlot(str, int)
    def update_topic(self, topic_name, msg_count):
        """Update current topic being processed"""
        self.topic_label.setText(f"Processing topic: {topic_name} ({msg_count:,} messages)")
        
    @pyqtSlot(str, bool, str)
    def export_finished(self, output_dir, success, error_msg):
        """Handle export completion"""
        if self.was_cancelled:
            self.reject()
        elif success:
            self.status_label.setText("✓ Export complete!")
            self.progress_bar.setValue(100)
            self.accept()
        else:
            QMessageBox.critical(self, "Export Failed", f"Error: {error_msg}")
            self.reject()


# ============================================================================
# Rest of the original classes...
# ============================================================================

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
        self.dfs = {}
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
        self.ros_viz_manager = ROSVisualizationManager()
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
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        sidebar = self.create_sidebar()
        main_layout.addWidget(sidebar, 1)
        
        content_area = self.create_content_area()
        main_layout.addWidget(content_area, 4)
        
        self.statusBar().showMessage("Ready | ROS: Connected | Memory: 4.2 GB / 16 GB | CPU: 23%")
        
    def create_sidebar(self):
        """Create the left sidebar with quick actions"""
        sidebar = QGroupBox("Quick Actions")
        layout = QVBoxLayout()
        
        self.btn_load_bag = QPushButton("📁 Load .bag File")
        self.btn_load_bag.setStyleSheet("background-color: #4A90E2; color: white; padding: 10px; font-size: 14px;")
        self.btn_load_bag.clicked.connect(self.load_bag)
        layout.addWidget(self.btn_load_bag)
        
        self.btn_launch_viz = QPushButton("🎬 Launch RViz")
        self.btn_launch_viz.setStyleSheet("background-color: #F39C12; color: white; padding: 10px; font-size: 14px;")
        self.btn_launch_viz.clicked.connect(self.launch_visualization)
        layout.addWidget(self.btn_launch_viz)
        
        self.btn_start_sim = QPushButton("▶ Start Simulation")
        self.btn_start_sim.setStyleSheet("background-color: #4A90E2; color: white; padding: 10px; font-size: 14px;")
        self.btn_start_sim.clicked.connect(self.start_simulation)
        self.btn_start_sim.setEnabled(False)
        layout.addWidget(self.btn_start_sim)
        
        self.btn_analyze = QPushButton("📊 Analyze Data")
        self.btn_analyze.setStyleSheet("background-color: #50C878; color: white; padding: 10px; font-size: 14px;")
        self.btn_analyze.clicked.connect(self.analyze_data)
        layout.addWidget(self.btn_analyze)
        
        self.btn_inject = QPushButton("⚠ Inject Anomaly")
        self.btn_inject.setStyleSheet("background-color: #E74C3C; color: white; padding: 10px; font-size: 14px;")
        self.btn_inject.clicked.connect(self.show_injection_tab)
        layout.addWidget(self.btn_inject)
        
        self.btn_report = QPushButton("📄 Generate Report")
        self.btn_report.setStyleSheet("background-color: #F39C12; color: white; padding: 10px; font-size: 14px;")
        self.btn_report.clicked.connect(self.generate_report)
        layout.addWidget(self.btn_report)
        
        layout.addStretch()
        
        self.sim_status_label = QLabel("Status: Idle")
        self.sim_status_label.setStyleSheet("padding: 10px; background-color: #ECF0F1;")
        layout.addWidget(self.sim_status_label)
        
        sidebar.setLayout(layout)
        sidebar.setMaximumWidth(300)
        return sidebar
        
    def create_content_area(self):
        """Create the main content area with tabs"""
        self.tabs = QTabWidget()
        
        dashboard_tab = self.create_dashboard_tab()
        self.tabs.addTab(dashboard_tab, "Dashboard")
        
        simulation_tab = self.create_simulation_tab()
        self.tabs.addTab(simulation_tab, "Simulation Playback")
        
        analysis_tab = self.create_analysis_tab()
        self.tabs.addTab(analysis_tab, "Data Analysis")
        
        vulnerability_tab = self.create_vulnerability_tab()
        self.tabs.addTab(vulnerability_tab, "Vulnerability Testing")
        
        return self.tabs
        
    def create_dashboard_tab(self):
        """Create the dashboard tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
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
        
        controls_layout = QHBoxLayout()
        
        self.btn_play = QPushButton("▶ Play")
        self.btn_play.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_play.clicked.connect(self.play_simulation)
        self.btn_play.setEnabled(False)
        controls_layout.addWidget(self.btn_play)
        
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_pause.clicked.connect(self.pause_simulation)
        self.btn_pause.setEnabled(False)
        controls_layout.addWidget(self.btn_pause)
        
        self.btn_stop = QPushButton("⏹ Stop")
        self.btn_stop.setStyleSheet("background-color: #4A90E2; color: white; padding: 5px 15px;")
        self.btn_stop.clicked.connect(self.stop_simulation)
        controls_layout.addWidget(self.btn_stop)
        
        self.progress_bar = QProgressBar()
        controls_layout.addWidget(self.progress_bar, 3)
        
        controls_layout.addWidget(QLabel("Speed:"))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1x", "2x", "5x"])
        self.speed_combo.setCurrentIndex(1)
        controls_layout.addWidget(self.speed_combo)
        
        layout.addLayout(controls_layout)
        
        viewport_group = QGroupBox("3D Simulation Viewport")
        viewport_layout = QVBoxLayout()
        
        self.simulation_canvas = QLabel("RViz View: Not Launched\n\nClick 'Launch RViz' to start visualization")
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
        
        topic_group = QGroupBox("Topic Logs")
        topic_layout = QVBoxLayout()
        
        self.topic_logs = QTextEdit()
        self.topic_logs.setReadOnly(True)
        self.topic_logs.setMaximumHeight(150)
        self.topic_logs.setStyleSheet("background-color: #ECF0F1; font-family: monospace;")
        self.topic_logs.setText("Waiting for simulation data...")
        
        topic_layout.addWidget(self.topic_logs)
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        widget.setLayout(layout)
        return widget
        
    def create_analysis_tab(self):
        """Create the data analysis tab with security monitoring"""
        widget = QWidget()
        layout = QVBoxLayout()

        # === Security Threshold Configuration ===
        threshold_group = QGroupBox("Security Thresholds (Configurable)")
        threshold_layout = QVBoxLayout()

        # Row 1: Velocity thresholds
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Max Velocity (m/s):"))
        self.threshold_velocity = QLineEdit("5.0")
        self.threshold_velocity.setMaximumWidth(80)
        row1.addWidget(self.threshold_velocity)

        row1.addWidget(QLabel("Max Angular Vel (rad/s):"))
        self.threshold_angular = QLineEdit("2.0")
        self.threshold_angular.setMaximumWidth(80)
        row1.addWidget(self.threshold_angular)

        row1.addWidget(QLabel("Max Accel (m/s²):"))
        self.threshold_accel = QLineEdit("3.0")
        self.threshold_accel.setMaximumWidth(80)
        row1.addWidget(self.threshold_accel)
        row1.addStretch()
        threshold_layout.addLayout(row1)

        # Row 2: Other thresholds
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Max Path Deviation (m):"))
        self.threshold_path_dev = QLineEdit("2.0")
        self.threshold_path_dev.setMaximumWidth(80)
        row2.addWidget(self.threshold_path_dev)

        row2.addWidget(QLabel("Sensor Dropout (s):"))
        self.threshold_dropout = QLineEdit("1.0")
        self.threshold_dropout.setMaximumWidth(80)
        row2.addWidget(self.threshold_dropout)

        row2.addWidget(QLabel("Z-Score Threshold:"))
        self.threshold_zscore = QLineEdit("3.0")
        self.threshold_zscore.setMaximumWidth(80)
        row2.addWidget(self.threshold_zscore)
        row2.addStretch()
        threshold_layout.addLayout(row2)

        # Apply button
        apply_threshold_btn = QPushButton("Update Thresholds")
        apply_threshold_btn.setStyleSheet("background-color: #FF9800; color: white;")
        apply_threshold_btn.clicked.connect(self.update_security_thresholds)
        threshold_layout.addWidget(apply_threshold_btn)

        threshold_group.setLayout(threshold_layout)
        layout.addWidget(threshold_group)

        # === Analysis Control ===
        filter_group = QGroupBox("Analysis Controls")
        filter_layout = QHBoxLayout()

        self.btn_run_security_analysis = QPushButton("🔍 Run Security Analysis")
        self.btn_run_security_analysis.setStyleSheet("background-color: #E74C3C; color: white; font-weight: bold; padding: 10px;")
        self.btn_run_security_analysis.clicked.connect(self.run_security_analysis)
        filter_layout.addWidget(self.btn_run_security_analysis)

        self.btn_apply_filters = QPushButton("Apply Filters")
        self.btn_apply_filters.setStyleSheet("background-color: #4A90E2; color: white;")
        self.btn_apply_filters.clicked.connect(self.apply_analysis_filters)
        filter_layout.addWidget(self.btn_apply_filters)

        filter_group.setLayout(filter_layout)
        layout.addWidget(filter_group)
        
        analysis_tabs = QTabWidget()

        # === TAB 1: Anomaly Detection ===
        anomaly_widget = QWidget()
        anomaly_layout = QVBoxLayout()

        self.anomaly_table = QTableWidget()
        self.anomaly_table.setColumnCount(5)
        self.anomaly_table.setHorizontalHeaderLabels(["Timestamp", "Type", "Severity", "Details", "Value"])
        self.anomaly_table.setRowCount(0)

        anomaly_layout.addWidget(self.anomaly_table)

        self.btn_export_analysis = QPushButton("Export Anomalies")
        self.btn_export_analysis.setStyleSheet("background-color: #50C878; color: white;")
        self.btn_export_analysis.clicked.connect(self.export_analysis_results)
        anomaly_layout.addWidget(self.btn_export_analysis)

        anomaly_widget.setLayout(anomaly_layout)
        analysis_tabs.addTab(anomaly_widget, "🚨 Anomalies")

        # === TAB 2: Trajectory Deviation ===
        trajectory_widget = QWidget()
        trajectory_layout = QVBoxLayout()

        self.trajectory_figure = Figure(figsize=(8, 6))
        self.trajectory_canvas = FigureCanvas(self.trajectory_figure)
        trajectory_layout.addWidget(self.trajectory_canvas)

        # Trajectory stats
        self.trajectory_stats = QTextEdit()
        self.trajectory_stats.setReadOnly(True)
        self.trajectory_stats.setMaximumHeight(100)
        trajectory_layout.addWidget(self.trajectory_stats)

        trajectory_widget.setLayout(trajectory_layout)
        analysis_tabs.addTab(trajectory_widget, "📍 Trajectory")

        # === TAB 3: Sensor Health ===
        sensor_widget = QWidget()
        sensor_layout = QVBoxLayout()

        self.sensor_health_table = QTableWidget()
        self.sensor_health_table.setColumnCount(5)
        self.sensor_health_table.setHorizontalHeaderLabels(["Sensor", "Status", "Messages", "Avg Rate", "Health Score"])
        self.sensor_health_table.setRowCount(0)
        sensor_layout.addWidget(self.sensor_health_table)

        # Dropout details
        dropout_label = QLabel("Sensor Dropouts:")
        sensor_layout.addWidget(dropout_label)
        self.dropout_text = QTextEdit()
        self.dropout_text.setReadOnly(True)
        self.dropout_text.setMaximumHeight(150)
        sensor_layout.addWidget(self.dropout_text)

        sensor_widget.setLayout(sensor_layout)
        analysis_tabs.addTab(sensor_widget, "🔧 Sensor Health")

        # === TAB 4: Event Timeline ===
        timeline_widget = QWidget()
        timeline_layout = QVBoxLayout()

        self.timeline_table = QTableWidget()
        self.timeline_table.setColumnCount(4)
        self.timeline_table.setHorizontalHeaderLabels(["Timestamp", "Source", "Level", "Message"])
        self.timeline_table.setRowCount(0)
        timeline_layout.addWidget(self.timeline_table)

        timeline_widget.setLayout(timeline_layout)
        analysis_tabs.addTab(timeline_widget, "📋 Event Timeline")

        # === TAB 5: Correlation Heatmap (Original) ===
        corr_widget = QWidget()
        corr_layout = QVBoxLayout()

        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        corr_layout.addWidget(self.canvas)

        corr_widget.setLayout(corr_layout)
        analysis_tabs.addTab(corr_widget, "📊 Correlations")

        # === TAB 6: Statistics ===
        stats_widget = QWidget()
        stats_layout = QVBoxLayout()
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        stats_layout.addWidget(self.stats_text)
        stats_widget.setLayout(stats_layout)
        analysis_tabs.addTab(stats_widget, "📈 Statistics")
        
        layout.addWidget(analysis_tabs)
        
        widget.setLayout(layout)
        return widget
        
    def create_vulnerability_tab(self):
        """Create the vulnerability testing tab"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        param_group = QGroupBox("Define Injection Parameters")
        param_layout = QVBoxLayout()
        
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Select Topic:"))
        self.inject_topic_combo = QComboBox()
        self.inject_topic_combo.addItems(["/cmd_vel", "/imu/data", "/navsat/fix", "/odometry"])
        topic_layout.addWidget(self.inject_topic_combo)
        param_layout.addLayout(topic_layout)

        mod_layout = QHBoxLayout()
        mod_layout.addWidget(QLabel("Modification Type:"))
        self.inject_mod_combo = QComboBox()
        self.inject_mod_combo.addItems(["Set to Zero", "Add Noise", "Spoof GPS", "Delay", "Drop Packets"])
        mod_layout.addWidget(self.inject_mod_combo)
        param_layout.addLayout(mod_layout)
        
        value_layout = QHBoxLayout()
        value_layout.addWidget(QLabel("Value/Parameter:"))
        self.inject_value_input = QLineEdit()
        self.inject_value_input.setPlaceholderText("e.g., Set linear.x to 0.0")
        value_layout.addWidget(self.inject_value_input)
        param_layout.addLayout(value_layout)
        
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
        
        self.btn_execute_inject = QPushButton("⚠ Execute Injection")
        self.btn_execute_inject.setStyleSheet("background-color: #E74C3C; color: white; padding: 10px;")
        self.btn_execute_inject.clicked.connect(self.execute_injection)
        param_layout.addWidget(self.btn_execute_inject)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        comparison_layout = QHBoxLayout()
        
        before_group = QGroupBox("Before Injection")
        before_layout = QVBoxLayout()
        self.before_text = QTextEdit()
        self.before_text.setReadOnly(True)
        self.before_text.setText("Waiting for analysis to run...")
        before_layout.addWidget(self.before_text)
        before_group.setLayout(before_layout)
        comparison_layout.addWidget(before_group)
        
        after_group = QGroupBox("After Injection")
        after_layout = QVBoxLayout()
        self.after_text = QTextEdit()
        self.after_text.setReadOnly(True)
        self.after_text.setText("Results will appear after injection is executed.")
        after_layout.addWidget(self.after_text)
        after_group.setLayout(after_layout)
        comparison_layout.addWidget(after_group)
        
        layout.addLayout(comparison_layout)
        
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
    
    # ========================================================================
    # Event Handlers
    # ========================================================================
    
    def load_bag(self):
        """Load a .bag file WITH ASYNC EXPORT AND PROGRESS DIALOG"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select .bag File", "", "ROS Bag Files (*.bag);;All Files (*)"
        )
        
        if not file_path:
            return
            
        self.current_bag_path = file_path
        file_name = os.path.basename(file_path)
        file_size = os.path.getsize(file_path) / (1024**3)  # Convert to GB
        
        self.loaded_file_info.setText(f"Loaded: {file_name} ({file_size:.2f} GB)")
        self.log_console.append(f"[{self.get_timestamp()}] Loading bag file: {file_name}")
        
        # Create progress dialog
        progress_dialog = BagExportProgressDialog(self)
        
        # Output directory
        output_dir = "./csv_output"
        
        # Start async export
        try:
            export_thread = self.bag_player.export_bag_to_csv_async(
                file_path,
                output_dir,
                progress_callback=progress_dialog.update_progress,
                finished_callback=progress_dialog.export_finished,
                topic_started_callback=progress_dialog.update_topic
            )
            
            progress_dialog.set_export_thread(export_thread)
            
            # Show dialog (blocks until export finishes or is cancelled)
            result = progress_dialog.exec_()
            
            if result == QDialog.Accepted:
                # Export completed successfully
                self.current_csv_dir = output_dir
                self.log_console.append(f"[{self.get_timestamp()}] ✓ CSV files exported to {output_dir}")
                self.log_console.append(f"[{self.get_timestamp()}] ✓ Bag file loaded successfully")
                
                # Get topic info
                topics = self.bag_player.get_topics_list(file_path)
                self.log_console.append(f"[{self.get_timestamp()}] Topics found: {len(topics)} topics")
                
                self.sim_status_label.setText("Status: Bag Loaded. Ready for Visualization.")
                
                # Enable simulation start if visualization is already running
                if self.ros_viz_manager.is_running(): 
                    self.btn_start_sim.setEnabled(True)
            else:
                # Export was cancelled or failed
                self.log_console.append(f"[{self.get_timestamp()}] Bag file loading cancelled or failed")
                
        except Exception as e:
            self.show_error(f"Error loading bag file: {str(e)}")
            
    def launch_visualization(self):
        """Launch RViz only (lightweight visualization)"""
        if self.ros_viz_manager.is_running():
            QMessageBox.information(self, "Running", "RViz is already running.")
            return

        try:
            self.log_console.append(f"[{self.get_timestamp()}] Launching RViz...")
            
            self.ros_viz_manager.launch_rviz()  # RViz only - no Gazebo
            
            self.simulation_canvas.setText("RViz View: Ready for Bag Playback\n\nConfigure RViz:\n1. Set Fixed Frame to 'odom'\n2. Add: TF, RobotModel, Camera, Path")
            self.sim_status_label.setText("Status: RViz Active. Ready to Play.")
            
            if self.current_bag_path:
                self.btn_start_sim.setEnabled(True)
                
        except Exception as e:
            self.show_error(f"Failed to launch RViz: {str(e)}. Check if ROS is sourced.")

    def start_simulation(self):
        """Start the simulation by playing the bag file"""
        if not self.current_bag_path:
            QMessageBox.warning(self, "No File", "Please load a .bag file first.")
            return
    
        if not self.ros_viz_manager.is_running():
            QMessageBox.warning(self, "No Visualization", "Please click 'Launch RViz' first.")
            return

        self.simulation_running = True
        self.sim_status_label.setText("Status: Playing Bag File")
        self.tabs.setCurrentIndex(1)
    
        try:
            self.log_console.append(f"[{self.get_timestamp()}] Starting rosbag playback...")
        
            # Define lightweight topics for smooth visualization
            lightweight_topics = [
                '/warty/odom',
                '/warty/cmd_vel', 
                '/warty/imu/data',
                '/warty/pose',
                '/warty/lidar_points',
                '/warty/lidar_points_front',
                '/tf',
                '/tf_static',
                '/warty/navigation_manager/global_plan',
                '/warty/navigation_manager/status'
            ]
        
            # Play at 0.25x speed with loop=True and lightweight topics only
            self.bag_player.play_bag(
                self.current_bag_path,
                rate=0.25,
                topics=lightweight_topics,
                loop=True
            )
        
            self.log_console.append(f"[{self.get_timestamp()}] Playing at 0.25x speed with lightweight topics (no cameras)")
            self.log_console.append(f"[{self.get_timestamp()}] Looping enabled - playback will repeat automatically")
        
            self.btn_play.setEnabled(False)
            self.btn_pause.setEnabled(True)
        
            try:
                self.bag_duration = self.bag_player.get_bag_duration(self.current_bag_path)
                self.start_time = self.bag_player.get_playback_start_time()
            except Exception:
                self.bag_duration = 60.0
                self.start_time = time.time() 
            
            self.update_timer = QTimer(self)
            self.update_timer.timeout.connect(self.update_playback_status)
            self.update_timer.start(100)
        
        except Exception as e:
            self.show_error(f"Failed to start bag playback: {str(e)}")

    def update_playback_status(self):
        """Update progress bar and topic logs"""
        if self.simulation_running and self.bag_player.is_playing():
            elapsed_time = time.time() - self.start_time
            percentage = min(100, int((elapsed_time / self.bag_duration) * 100))
            self.progress_bar.setValue(percentage)
            
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
        self.ros_viz_manager.stop_all()
        self.log_console.append(f"[{self.get_timestamp()}] Simulation stopped and visualization closed.")
        self.sim_status_label.setText("Status: Idle")
        self.progress_bar.setValue(0)
        self.btn_play.setEnabled(False)
        self.btn_pause.setEnabled(False)
        self.simulation_canvas.setText("RViz View: Not Launched\n\nClick 'Launch RViz' to start visualization")
        
    def analyze_data(self):
        """Analyze loaded CSV data"""
        if not self.current_csv_dir:
            QMessageBox.warning(self, "No Data", "Please load and export a .bag file first.")
            return
            
        self.tabs.setCurrentIndex(2)
        self.log_console.append(f"[{self.get_timestamp()}] Loading CSV files for analysis...")
        
        try:
            if not self.dfs:
                 self.dfs['dummy'] = pd.DataFrame({
                    'timestamp': [1, 2, 3, 4],
                    'linear.x': [1.0, 1.2, 1.1, 0.9],
                    'angular.z': [0.1, 0.2, 0.3, 0.1],
                    'position.x': [0, 1.1, 2.2, 3.1]
                 })

            if self.dfs:
                df_combined = pd.concat([df.set_index(df.columns[0]) for df in self.dfs.values()], axis=1, join='inner', keys=self.dfs.keys())
                corr_matrix = df_combined.corr()
                
                self.plot_correlation_heatmap(corr_matrix)
                self.log_console.append(f"[{self.get_timestamp()}] Correlation analysis complete")
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

    # ==================== NEW SECURITY ANALYSIS METHODS ====================

    def update_security_thresholds(self):
        """Update security thresholds in the data analyzer"""
        try:
            new_thresholds = {
                'velocity_max': float(self.threshold_velocity.text()),
                'angular_velocity_max': float(self.threshold_angular.text()),
                'acceleration_max': float(self.threshold_accel.text()),
                'path_deviation_max': float(self.threshold_path_dev.text()),
                'sensor_dropout_max': float(self.threshold_dropout.text()),
                'zscore_threshold': float(self.threshold_zscore.text())
            }

            self.data_analyzer.update_thresholds(new_thresholds)
            self.log_console.append(f"[{self.get_timestamp()}] Security thresholds updated successfully")
            QMessageBox.information(self, "Thresholds Updated", "Security monitoring thresholds have been updated.")

        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numeric values for all thresholds.")
            self.log_console.append(f"[{self.get_timestamp()}] Error updating thresholds: Invalid input")

    def run_security_analysis(self):
        """Run comprehensive security analysis on loaded data"""
        if not self.current_csv_dir:
            QMessageBox.warning(self, "No Data", "Please load and export a .bag file first.")
            return

        self.log_console.append(f"[{self.get_timestamp()}] Starting security analysis...")

        try:
            # Load CSVs if not already loaded
            if not self.data_analyzer.dfs:
                self.log_console.append(f"[{self.get_timestamp()}] Loading CSV files from {self.current_csv_dir}")
                self.data_analyzer.load_csvs(self.current_csv_dir)

            # Update thresholds first
            self.update_security_thresholds()

            # 1. Anomaly Detection
            self.log_console.append(f"[{self.get_timestamp()}] Running anomaly detection...")
            anomalies_df = self.data_analyzer.detect_behavioral_anomalies()
            self.display_anomalies(anomalies_df)

            # 2. Trajectory Deviation
            self.log_console.append(f"[{self.get_timestamp()}] Computing trajectory deviation...")
            traj_result = self.data_analyzer.compute_trajectory_deviation()
            self.display_trajectory_deviation(traj_result)

            # 3. Sensor Health
            self.log_console.append(f"[{self.get_timestamp()}] Analyzing sensor health...")
            sensor_health = self.data_analyzer.analyze_sensor_health()
            self.display_sensor_health(sensor_health)

            # 4. Event Timeline
            self.log_console.append(f"[{self.get_timestamp()}] Parsing event timeline...")
            timeline_df = self.data_analyzer.parse_event_timeline()
            self.display_event_timeline(timeline_df)

            self.log_console.append(f"[{self.get_timestamp()}] Security analysis complete!")
            QMessageBox.information(self, "Analysis Complete",
                                   f"Security analysis complete!\n\n"
                                   f"Anomalies detected: {len(anomalies_df)}\n"
                                   f"Events logged: {len(timeline_df)}")

        except Exception as e:
            self.show_error(f"Error during security analysis: {str(e)}")
            import traceback
            self.log_console.append(f"[{self.get_timestamp()}] Error: {traceback.format_exc()}")

    def display_anomalies(self, anomalies_df):
        """Display anomaly detection results in table"""
        self.anomaly_table.setRowCount(0)

        if anomalies_df.empty:
            self.log_console.append(f"[{self.get_timestamp()}] No anomalies detected")
            return

        self.anomaly_table.setRowCount(len(anomalies_df))

        for i, (idx, row) in enumerate(anomalies_df.iterrows()):
            self.anomaly_table.setItem(i, 0, QTableWidgetItem(f"{row['timestamp']:.2f}"))
            self.anomaly_table.setItem(i, 1, QTableWidgetItem(str(row['type'])))

            # Color code by severity
            severity_item = QTableWidgetItem(str(row['severity']))
            if row['severity'] == 'Critical':
                severity_item.setBackground(QColor(231, 76, 60))  # Red
                severity_item.setForeground(QColor(255, 255, 255))
            elif row['severity'] == 'High':
                severity_item.setBackground(QColor(230, 126, 34))  # Orange
            elif row['severity'] == 'Medium':
                severity_item.setBackground(QColor(241, 196, 15))  # Yellow

            self.anomaly_table.setItem(i, 2, severity_item)
            self.anomaly_table.setItem(i, 3, QTableWidgetItem(str(row['details'])))
            self.anomaly_table.setItem(i, 4, QTableWidgetItem(str(row['value'])))

        self.anomaly_table.resizeColumnsToContents()
        self.log_console.append(f"[{self.get_timestamp()}] Displayed {len(anomalies_df)} anomalies")

    def display_trajectory_deviation(self, traj_result):
        """Display trajectory deviation visualization"""
        self.trajectory_figure.clear()

        if not traj_result['has_data']:
            ax = self.trajectory_figure.add_subplot(111)
            ax.text(0.5, 0.5, 'No trajectory data available',
                   ha='center', va='center', transform=ax.transAxes)
            self.trajectory_canvas.draw()
            return

        # Plot trajectory
        ax = self.trajectory_figure.add_subplot(111)

        if traj_result['trajectory_data'] is not None:
            traj_data = traj_result['trajectory_data']
            ax.plot(traj_data['pose_position_x'], traj_data['pose_position_y'],
                   'b-', linewidth=2, label='Actual Path')
            ax.scatter(traj_data['pose_position_x'].iloc[0],
                      traj_data['pose_position_y'].iloc[0],
                      c='green', s=100, marker='o', label='Start')
            ax.scatter(traj_data['pose_position_x'].iloc[-1],
                      traj_data['pose_position_y'].iloc[-1],
                      c='red', s=100, marker='X', label='End')

        ax.set_xlabel('Position X (m)')
        ax.set_ylabel('Position Y (m)')
        ax.set_title('Vehicle Trajectory')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        self.trajectory_figure.tight_layout()
        self.trajectory_canvas.draw()

        # Display stats
        stats_text = f"Mean Deviation: {traj_result['mean_deviation']:.3f} m\n"
        stats_text += f"Max Deviation: {traj_result['max_deviation']:.3f} m"
        self.trajectory_stats.setText(stats_text)

    def display_sensor_health(self, sensor_health):
        """Display sensor health monitoring results"""
        self.sensor_health_table.setRowCount(0)
        self.sensor_health_table.setRowCount(len(sensor_health))

        dropout_text = ""

        for i, (sensor_name, health) in enumerate(sensor_health.items()):
            self.sensor_health_table.setItem(i, 0, QTableWidgetItem(sensor_name))

            # Status with color coding
            status_item = QTableWidgetItem(health['status'])
            if health['status'] == 'Healthy':
                status_item.setBackground(QColor(46, 204, 113))  # Green
                status_item.setForeground(QColor(255, 255, 255))
            elif health['status'] == 'Degraded':
                status_item.setBackground(QColor(241, 196, 15))  # Yellow
            elif health['status'] in ['Critical', 'Missing']:
                status_item.setBackground(QColor(231, 76, 60))  # Red
                status_item.setForeground(QColor(255, 255, 255))

            self.sensor_health_table.setItem(i, 1, status_item)
            self.sensor_health_table.setItem(i, 2, QTableWidgetItem(str(health['message_count'])))
            self.sensor_health_table.setItem(i, 3, QTableWidgetItem(f"{health['avg_rate']:.1f} Hz"))
            self.sensor_health_table.setItem(i, 4, QTableWidgetItem(f"{health['health_score']}/100"))

            # Collect dropout info
            if health['dropouts']:
                dropout_text += f"\n{sensor_name}:\n"
                for dropout in health['dropouts'][:5]:  # Limit to 5
                    dropout_text += f"  - t={dropout['timestamp']:.2f}s, duration={dropout['duration']:.2f}s\n"

        self.sensor_health_table.resizeColumnsToContents()
        self.dropout_text.setText(dropout_text if dropout_text else "No sensor dropouts detected")

    def display_event_timeline(self, timeline_df):
        """Display event timeline"""
        self.timeline_table.setRowCount(0)

        if timeline_df.empty:
            self.log_console.append(f"[{self.get_timestamp()}] No timeline events")
            return

        # Limit display to most recent 100 events
        display_df = timeline_df.tail(100)
        self.timeline_table.setRowCount(len(display_df))

        for i, (idx, row) in enumerate(display_df.iterrows()):
            self.timeline_table.setItem(i, 0, QTableWidgetItem(f"{row['timestamp']:.2f}"))
            self.timeline_table.setItem(i, 1, QTableWidgetItem(str(row['source'])))

            # Color code by level
            level_item = QTableWidgetItem(str(row['level']))
            if row['level'] in ['Error', 'Critical']:
                level_item.setBackground(QColor(231, 76, 60))  # Red
                level_item.setForeground(QColor(255, 255, 255))
            elif row['level'] == 'Warning':
                level_item.setBackground(QColor(241, 196, 15))  # Yellow
            elif row['level'] in ['High', 'Medium']:
                level_item.setBackground(QColor(230, 126, 34))  # Orange

            self.timeline_table.setItem(i, 2, level_item)
            self.timeline_table.setItem(i, 3, QTableWidgetItem(str(row['message'])[:100]))

        self.timeline_table.resizeColumnsToContents()
        self.log_console.append(f"[{self.get_timestamp()}] Displayed {len(display_df)} timeline events")

    # ==================== END SECURITY ANALYSIS METHODS ====================

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
            output_path = self.current_bag_path.replace(".bag", "_modified.bag")
            self.vulnerability_injector.inject_anomaly(
                self.current_bag_path,
                output_path,
                mod_type.lower().replace(" ", "_"),
                0.0
            )
            
            self.log_console.append(f"[{self.get_timestamp()}] Modified bag created: {output_path}")
            self.log_console.append(f"[{self.get_timestamp()}] Re-simulating with injected anomaly...")
            
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
            
            if corr_matrix is None or corr_matrix.empty:
                data = np.random.rand(4, 4) * 2 - 1
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
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
    def closeEvent(self, event):
        """Handle window close event"""
        self.ros_thread.stop()
        self.ros_thread.wait()
        self.ros_viz_manager.stop_all()
        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()