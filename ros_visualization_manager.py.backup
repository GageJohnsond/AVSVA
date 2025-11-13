"""
ROS Visualization Manager
Handles integration of RViz and Gazebo with the PyQt application
"""

import subprocess
import os
import time
import signal


class ROSVisualizationManager:
    """Manages ROS visualization tools (RViz and Gazebo)"""
    
    def __init__(self):
        self.rviz_process = None
        self.gazebo_process = None
        self.roscore_process = None
        self.is_ros_available = self._check_ros()
        
    def _check_ros(self):
        """Check if ROS is available"""
        try:
            result = subprocess.run(
                ['which', 'roscore'],
                capture_output=True,
                text=True
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def start_roscore(self):
        """Start roscore if not already running"""
        if not self.is_ros_available:
            print("ROS not available. Cannot start roscore.")
            return False
            
        # Check if roscore is already running
        try:
            result = subprocess.run(
                ['rostopic', 'list'],
                capture_output=True,
                timeout=2,
                env=os.environ.copy() # Stability fix
            )
            if result.returncode == 0:
                print("roscore already running")
                return True
        except (subprocess.TimeoutExpired, Exception):
            pass
        
        # Start roscore
        try:
            self.roscore_process = subprocess.Popen(
                ['roscore'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=os.environ.copy() # Stability fix
            )
            time.sleep(3)  # Give roscore time to start
            print("roscore started successfully")
            return True
        except Exception as e:
            print(f"Error starting roscore: {e}")
            return False
    
    def launch_rviz(self, config_file=None):
        """
        Launch RViz for visualization
        
        Args:
            config_file (str): Path to RViz config file
            
        Returns:
            bool: Success status
        """
        if not self.is_ros_available:
            print("ROS not available. Cannot launch RViz.")
            return False
        
        # Ensure roscore is running
        self.start_roscore()
        
        cmd = ['rviz']
        if config_file and os.path.exists(config_file):
            cmd.extend(['-d', config_file])
        
        try:
            self.rviz_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=os.environ.copy() # Stability fix
            )
            print("RViz launched successfully")
            return True
        except Exception as e:
            print(f"Error launching RViz: {e}")
            return False
    
    def launch_gazebo(self, world_file=None, robot_model=None):
        """
        Launch Gazebo simulator
        
        Args:
            world_file (str): Path to Gazebo world file
            robot_model (str): Robot model to spawn (e.g., 'husky')
            
        Returns:
            bool: Success status
        """
        if not self.is_ros_available:
            print("ROS not available. Cannot launch Gazebo.")
            return False
        
        # Ensure roscore is running
        self.start_roscore()
        
        # Launch Gazebo
        cmd = ['roslaunch', 'gazebo_ros', 'empty_world.launch']
        
        if world_file and os.path.exists(world_file):
            cmd.extend(['world_name:=' + world_file])
        
        try:
            self.gazebo_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=os.environ.copy() # Stability fix
            )
            time.sleep(5)  # Give Gazebo time to start
            
            # Spawn robot if specified
            if robot_model:
                self._spawn_robot(robot_model)
            
            print("Gazebo launched successfully")
            return True
        except Exception as e:
            print(f"Error launching Gazebo: {e}")
            return False
    
    def _spawn_robot(self, robot_model):
        """Spawn robot model in Gazebo"""
        if robot_model.lower() == 'husky':
            try:
                subprocess.Popen(
                    ['roslaunch', 'husky_gazebo', 'spawn_husky.launch'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=os.environ.copy() # Stability fix
                )
                print(f"Spawned {robot_model} in Gazebo")
            except Exception as e:
                print(f"Error spawning {robot_model}: {e}")
    
    def launch_both(self, rviz_config=None, world_file=None, robot_model='husky'):
        """
        Launch both RViz and Gazebo
        
        Args:
            rviz_config (str): RViz config file path
            world_file (str): Gazebo world file path
            robot_model (str): Robot model to spawn
            
        Returns:
            tuple: (rviz_success, gazebo_success)
        """
        gazebo_success = self.launch_gazebo(world_file, robot_model)
        time.sleep(2)  # Wait between launches
        rviz_success = self.launch_rviz(rviz_config)
        
        return (rviz_success, gazebo_success)
    
    def stop_rviz(self):
        """Stop RViz process"""
        if self.rviz_process:
            try:
                os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
                self.rviz_process.wait(timeout=5)
                print("RViz stopped")
            except Exception as e:
                print(f"Error stopping RViz: {e}")
            finally:
                self.rviz_process = None
    
    def stop_gazebo(self):
        """Stop Gazebo process"""
        if self.gazebo_process:
            try:
                os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
                self.gazebo_process.wait(timeout=5)
                print("Gazebo stopped")
            except Exception as e:
                print(f"Error stopping Gazebo: {e}")
            finally:
                self.gazebo_process = None
    
    def stop_roscore(self):
        """Stop roscore process"""
        if self.roscore_process:
            try:
                os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
                self.roscore_process.wait(timeout=5)
                print("roscore stopped")
            except Exception as e:
                print(f"Error stopping roscore: {e}")
            finally:
                self.roscore_process = None
    
    def stop_all(self):
        """Stop all visualization processes"""
        self.stop_rviz()
        self.stop_gazebo()
        time.sleep(1)
        self.stop_roscore()
    
    def stop_visualization(self):
        """Alias for stop_all - used by av_simulator.py"""
        self.stop_all()
    
    def is_running(self):
        """Check if visualization tools are running"""
        # Note: roscore process check is often unreliable due to the way it is started
        # and checked externally by rostopic list. We rely on the process object existing and not being finished.
        return self.rviz_process is not None and self.rviz_process.poll() is None
    
    def create_default_rviz_config(self, output_path):
        """
        Create a default RViz configuration file
        
        Args:
            output_path (str): Path to save config file
        """
        config = """
Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz/Grid
      Name: Grid
      Enabled: true
      
    - Class: rviz/TF
      Name: TF
      Enabled: true
      
    - Class: rviz/RobotModel
      Name: RobotModel
      Enabled: true
      
    - Class: rviz/Path
      Name: Path
      Enabled: true
      Topic: /husky_velocity_controller/odom
      
    - Class: rviz/LaserScan
      Name: LaserScan
      Enabled: true
      Topic: /scan
      
  Global Options:
    Fixed Frame: odom
    
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 10
      Focal Point:
        X: 0
        Y: 0
        Z: 0
"""
        
        try:
            with open(output_path, 'w') as f:
                f.write(config)
            print(f"Created RViz config: {output_path}")
            return True
        except Exception as e:
            print(f"Error creating RViz config: {e}")
            return False