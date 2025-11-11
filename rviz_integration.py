"""
RViz Integration Module
Handles automatic RViz launching, configuration, and optional embedding into PyQt5
"""

import os
import subprocess
import signal
import time
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import QProcess, QTimer, Qt, QProcessEnvironment
from PyQt5.QtGui import QWindow


class RVizWidget(QWidget):
    """Widget that manages RViz process and optionally embeds it"""

    def __init__(self, config_file=None, parent=None):
        super().__init__(parent)
        self.config_file = config_file or self._get_default_config_path()
        self.rviz_process = None
        self.rviz_window_id = None
        self.embedded = False
        self.status_callback = None

        self.init_ui()

    def init_ui(self):
        """Initialize the widget UI"""
        layout = QVBoxLayout()

        # Status area
        self.status_label = QLabel("RViz: Not Started")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #34495E;
                color: #ECF0F1;
                padding: 10px;
                font-size: 12pt;
                border-radius: 5px;
            }
        """)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # Control buttons
        button_layout = QHBoxLayout()

        self.btn_launch = QPushButton("Launch RViz")
        self.btn_launch.setStyleSheet("background-color: #27AE60; color: white; padding: 8px;")
        self.btn_launch.clicked.connect(self.launch_rviz)
        button_layout.addWidget(self.btn_launch)

        self.btn_stop = QPushButton("Stop RViz")
        self.btn_stop.setStyleSheet("background-color: #E74C3C; color: white; padding: 8px;")
        self.btn_stop.clicked.connect(self.stop_rviz)
        self.btn_stop.setEnabled(False)
        button_layout.addWidget(self.btn_stop)

        self.btn_reset = QPushButton("Reset View")
        self.btn_reset.setStyleSheet("background-color: #3498DB; color: white; padding: 8px;")
        self.btn_reset.clicked.connect(self.reset_rviz_view)
        self.btn_reset.setEnabled(False)
        button_layout.addWidget(self.btn_reset)

        layout.addLayout(button_layout)

        # Info text
        self.info_label = QLabel(
            "Click 'Launch RViz' to start visualization.\n"
            "RViz will open in a separate window showing live topic data.\n\n"
            "Recommended displays:\n"
            "• Add > By topic > Select your sensor topics\n"
            "• TF for coordinate frames\n"
            "• Grid for reference\n"
            "• Camera for image data\n"
            "• LaserScan/PointCloud2 for LIDAR"
        )
        self.info_label.setStyleSheet("""
            QLabel {
                background-color: #2C3E50;
                color: #BDC3C7;
                padding: 15px;
                border-radius: 5px;
                line-height: 1.5;
            }
        """)
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label)

        # Container for embedded RViz (if embedding works)
        self.rviz_container = QWidget()
        self.rviz_container.setMinimumSize(800, 600)
        self.rviz_container.setStyleSheet("background-color: #1E1E1E;")
        self.rviz_container_layout = QVBoxLayout()
        self.rviz_container.setLayout(self.rviz_container_layout)
        layout.addWidget(self.rviz_container)

        self.setLayout(layout)

    def _get_default_config_path(self):
        """Get path to default RViz config file"""
        return os.path.join(os.path.dirname(__file__), 'config', 'default.rviz')

    def launch_rviz(self, embed=False):
        """
        Launch RViz process

        Args:
            embed (bool): Attempt to embed RViz window (experimental)
        """
        if self.rviz_process and self.rviz_process.state() == QProcess.Running:
            self._update_status("RViz already running", "warning")
            return

        self._update_status("Launching RViz...", "info")

        # Create QProcess for RViz
        self.rviz_process = QProcess(self)
        self.rviz_process.setProcessChannelMode(QProcess.MergedChannels)

        # Connect signals
        self.rviz_process.started.connect(self._on_rviz_started)
        self.rviz_process.finished.connect(self._on_rviz_finished)
        self.rviz_process.errorOccurred.connect(self._on_rviz_error)
        self.rviz_process.readyReadStandardOutput.connect(self._on_rviz_output)

        # Setup environment
        env = QProcessEnvironment.systemEnvironment()

        # Try to source ROS environment
        ros_setup_paths = [
            '/opt/ros/noetic/setup.bash',
            '/opt/ros/melodic/setup.bash',
            '/opt/ros/kinetic/setup.bash'
        ]

        for setup_path in ros_setup_paths:
            if os.path.exists(setup_path):
                # Add ROS environment variables
                try:
                    import subprocess as sp
                    result = sp.run(
                        f'bash -c "source {setup_path} && env"',
                        shell=True,
                        capture_output=True,
                        text=True
                    )
                    if result.returncode == 0:
                        for line in result.stdout.split('\n'):
                            if '=' in line:
                                key, value = line.split('=', 1)
                                env.insert(key, value)
                except Exception as e:
                    print(f"Warning: Could not source ROS environment: {e}")
                break

        self.rviz_process.setProcessEnvironment(env)

        # Build RViz command
        rviz_cmd = "rviz"
        rviz_args = []

        # Add config file if it exists
        if self.config_file and os.path.exists(self.config_file):
            rviz_args.extend(["-d", self.config_file])

        # Start RViz
        self.rviz_process.start(rviz_cmd, rviz_args)

        # Try to embed after a delay (experimental)
        if embed:
            QTimer.singleShot(2000, self._try_embed_rviz)

    def _try_embed_rviz(self):
        """Attempt to embed RViz window into Qt widget (experimental)"""
        try:
            # Get RViz window ID
            import subprocess as sp
            result = sp.run(
                ['xdotool', 'search', '--name', 'RViz'],
                capture_output=True,
                text=True,
                timeout=2
            )

            if result.returncode == 0 and result.stdout.strip():
                window_id = int(result.stdout.strip().split('\n')[0])

                # Create QWindow from foreign window
                foreign_window = QWindow.fromWinId(window_id)

                if foreign_window:
                    # Create widget container for the window
                    container = QWidget.createWindowContainer(foreign_window, self.rviz_container)
                    self.rviz_container_layout.addWidget(container)

                    # Hide info label and show embedded view
                    self.info_label.hide()
                    self.embedded = True
                    self._update_status("RViz embedded successfully", "success")
                    return

        except Exception as e:
            print(f"Could not embed RViz window: {e}")

        # If embedding failed, just use external window
        self._update_status("RViz running in external window", "success")

    def stop_rviz(self):
        """Stop RViz process"""
        if self.rviz_process:
            self._update_status("Stopping RViz...", "info")

            # Try graceful shutdown first
            self.rviz_process.terminate()

            # Wait up to 3 seconds
            if not self.rviz_process.waitForFinished(3000):
                # Force kill if needed
                self.rviz_process.kill()

            self.rviz_process = None
            self.embedded = False

            # Show info label again
            self.info_label.show()

    def reset_rviz_view(self):
        """Reset RViz view to default"""
        if self.rviz_process and self.rviz_process.state() == QProcess.Running:
            # Stop and restart
            self.stop_rviz()
            QTimer.singleShot(500, self.launch_rviz)

    def _on_rviz_started(self):
        """Called when RViz process starts"""
        self._update_status("RViz running", "success")
        self.btn_launch.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_reset.setEnabled(True)

    def _on_rviz_finished(self, exit_code, exit_status):
        """Called when RViz process finishes"""
        self._update_status(f"RViz stopped (exit code: {exit_code})", "warning")
        self.btn_launch.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_reset.setEnabled(False)
        self.rviz_process = None

    def _on_rviz_error(self, error):
        """Called when RViz process has an error"""
        error_messages = {
            QProcess.FailedToStart: "Failed to start RViz. Make sure RViz is installed.",
            QProcess.Crashed: "RViz crashed",
            QProcess.Timedout: "RViz timed out",
            QProcess.WriteError: "Write error to RViz process",
            QProcess.ReadError: "Read error from RViz process",
            QProcess.UnknownError: "Unknown error with RViz"
        }

        msg = error_messages.get(error, "RViz error")
        self._update_status(msg, "error")

        if self.status_callback:
            self.status_callback(f"Error: {msg}")

    def _on_rviz_output(self):
        """Read RViz process output"""
        if self.rviz_process:
            output = bytes(self.rviz_process.readAllStandardOutput()).decode('utf-8')
            # Could log output if needed

    def _update_status(self, message, status_type="info"):
        """Update status label with color coding"""
        colors = {
            "info": "#3498DB",
            "success": "#27AE60",
            "warning": "#F39C12",
            "error": "#E74C3C"
        }

        color = colors.get(status_type, colors["info"])
        self.status_label.setText(f"RViz: {message}")
        self.status_label.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                color: white;
                padding: 10px;
                font-size: 12pt;
                border-radius: 5px;
                font-weight: bold;
            }}
        """)

        if self.status_callback:
            self.status_callback(message)

    def is_running(self):
        """Check if RViz is currently running"""
        return self.rviz_process and self.rviz_process.state() == QProcess.Running

    def closeEvent(self, event):
        """Clean up when widget is closed"""
        self.stop_rviz()
        event.accept()


class GazeboWidget(QWidget):
    """Widget that manages Gazebo simulation"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.gazebo_process = None
        self.gzclient_process = None
        self.status_callback = None

        self.init_ui()

    def init_ui(self):
        """Initialize the widget UI"""
        layout = QVBoxLayout()

        # Status area
        self.status_label = QLabel("Gazebo: Not Started")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #34495E;
                color: #ECF0F1;
                padding: 10px;
                font-size: 12pt;
                border-radius: 5px;
            }
        """)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # Control buttons
        button_layout = QHBoxLayout()

        self.btn_launch = QPushButton("Launch Gazebo")
        self.btn_launch.setStyleSheet("background-color: #27AE60; color: white; padding: 8px;")
        self.btn_launch.clicked.connect(self.launch_gazebo)
        button_layout.addWidget(self.btn_launch)

        self.btn_stop = QPushButton("Stop Gazebo")
        self.btn_stop.setStyleSheet("background-color: #E74C3C; color: white; padding: 8px;")
        self.btn_stop.clicked.connect(self.stop_gazebo)
        self.btn_stop.setEnabled(False)
        button_layout.addWidget(self.btn_stop)

        layout.addLayout(button_layout)

        # Info text
        info_label = QLabel(
            "Click 'Launch Gazebo' to start 3D simulation.\n"
            "Gazebo will open in a separate window.\n\n"
            "Note: Gazebo is primarily used for physics simulation.\n"
            "For bag playback visualization, RViz is recommended."
        )
        info_label.setStyleSheet("""
            QLabel {
                background-color: #2C3E50;
                color: #BDC3C7;
                padding: 15px;
                border-radius: 5px;
            }
        """)
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        self.setLayout(layout)

    def launch_gazebo(self):
        """Launch Gazebo simulation"""
        if self.gazebo_process and self.gazebo_process.state() == QProcess.Running:
            self._update_status("Gazebo already running", "warning")
            return

        self._update_status("Launching Gazebo...", "info")

        # Launch gzserver (physics server)
        self.gazebo_process = QProcess(self)
        self.gazebo_process.started.connect(lambda: self._on_gazebo_started("server"))
        self.gazebo_process.finished.connect(self._on_gazebo_finished)
        self.gazebo_process.errorOccurred.connect(self._on_gazebo_error)

        # Setup ROS environment
        env = QProcessEnvironment.systemEnvironment()
        ros_setup = '/opt/ros/noetic/setup.bash'
        if os.path.exists(ros_setup):
            try:
                import subprocess as sp
                result = sp.run(
                    f'bash -c "source {ros_setup} && env"',
                    shell=True,
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if '=' in line:
                            key, value = line.split('=', 1)
                            env.insert(key, value)
            except Exception as e:
                print(f"Warning: Could not source ROS environment: {e}")

        self.gazebo_process.setProcessEnvironment(env)
        self.gazebo_process.start("gzserver", [])

        # Launch gzclient (GUI) after short delay
        QTimer.singleShot(2000, self._launch_gzclient)

    def _launch_gzclient(self):
        """Launch Gazebo client GUI"""
        self.gzclient_process = QProcess(self)
        self.gzclient_process.started.connect(lambda: self._on_gazebo_started("client"))
        self.gzclient_process.start("gzclient", [])

    def stop_gazebo(self):
        """Stop Gazebo processes"""
        self._update_status("Stopping Gazebo...", "info")

        # Stop client first
        if self.gzclient_process:
            self.gzclient_process.terminate()
            self.gzclient_process.waitForFinished(2000)
            self.gzclient_process = None

        # Stop server
        if self.gazebo_process:
            self.gazebo_process.terminate()
            if not self.gazebo_process.waitForFinished(3000):
                self.gazebo_process.kill()
            self.gazebo_process = None

        self._update_status("Gazebo stopped", "info")
        self.btn_launch.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def _on_gazebo_started(self, component):
        """Called when Gazebo component starts"""
        self._update_status(f"Gazebo {component} running", "success")
        self.btn_launch.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def _on_gazebo_finished(self, exit_code, exit_status):
        """Called when Gazebo finishes"""
        self._update_status("Gazebo stopped", "warning")
        self.btn_launch.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def _on_gazebo_error(self, error):
        """Called on Gazebo error"""
        self._update_status("Gazebo error - make sure Gazebo is installed", "error")

    def _update_status(self, message, status_type="info"):
        """Update status label"""
        colors = {
            "info": "#3498DB",
            "success": "#27AE60",
            "warning": "#F39C12",
            "error": "#E74C3C"
        }

        color = colors.get(status_type, colors["info"])
        self.status_label.setText(f"Gazebo: {message}")
        self.status_label.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                color: white;
                padding: 10px;
                font-size: 12pt;
                border-radius: 5px;
                font-weight: bold;
            }}
        """)

        if self.status_callback:
            self.status_callback(message)

    def closeEvent(self, event):
        """Clean up when widget is closed"""
        self.stop_gazebo()
        event.accept()
