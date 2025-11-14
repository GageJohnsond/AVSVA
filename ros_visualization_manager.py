"""
ROS Visualization Manager - RViz Only Version
Lightweight visualization without Gazebo to prevent system freezing
"""

import subprocess
import os
import time


class ROSVisualizationManager:
    """
    Manages RViz visualization for ROS bag playback
    RViz-only version - much lighter on system resources
    """
    
    def __init__(self):
        self.rviz_process = None
        self._is_running = False
        
    def _check_ros_sourced(self):
        """Check if ROS environment is properly sourced"""
        ros_distro = os.environ.get('ROS_DISTRO')
        if not ros_distro:
            print("⚠ WARNING: ROS environment not sourced")
            print("Run: source /opt/ros/noetic/setup.bash")
            return False
        print(f"✓ ROS {ros_distro} environment detected")
        return True
        
    def launch_rviz(self):
        """
        Launch RViz only (lightweight visualization)
        Perfect for demonstrating anomaly effects without heavy simulation
        """
        if self._is_running and self.rviz_process:
            print("⚠ RViz is already running")
            return
            
        if not self._check_ros_sourced():
            raise Exception("ROS environment not sourced. Run: source /opt/ros/noetic/setup.bash")
        
        try:
            print("Launching RViz...")
            
            # Launch RViz
            self.rviz_process = subprocess.Popen(
                ['rviz'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()
            )
            
            # Give it a moment to start
            time.sleep(2)
            
            # Check if it started successfully
            if self.rviz_process.poll() is None:
                self._is_running = True
                print("✓ RViz launched successfully")
                print("\nRViz Configuration Tips:")
                print("1. Set Fixed Frame to 'odom' or 'base_link'")
                print("2. Add displays: TF, RobotModel, Camera, PointCloud2, Path")
                print("3. Configure topics to match your bag file")
            else:
                raise Exception("RViz failed to start")
                
        except FileNotFoundError:
            raise Exception("RViz not found. Make sure ROS is installed.")
        except Exception as e:
            self._is_running = False
            raise Exception(f"Failed to launch RViz: {str(e)}")
    
    def launch_rviz_with_config(self, config_file):
        """
        Launch RViz with a pre-configured .rviz file
        This is better for demonstrations - loads your saved configuration
        
        Args:
            config_file (str): Path to .rviz configuration file
        """
        if self._is_running and self.rviz_process:
            print("⚠ RViz is already running")
            return
            
        if not self._check_ros_sourced():
            raise Exception("ROS environment not sourced")
        
        if not os.path.exists(config_file):
            print(f"⚠ Config file not found: {config_file}")
            print("Launching RViz with default config...")
            return self.launch_rviz()
        
        try:
            print(f"Launching RViz with config: {config_file}")
            
            self.rviz_process = subprocess.Popen(
                ['rviz', '-d', config_file],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()
            )
            
            time.sleep(2)
            
            if self.rviz_process.poll() is None:
                self._is_running = True
                print("✓ RViz launched with custom config")
            else:
                raise Exception("RViz failed to start")
                
        except Exception as e:
            self._is_running = False
            raise Exception(f"Failed to launch RViz: {str(e)}")
    
    def stop_rviz(self):
        """Stop RViz process"""
        if self.rviz_process:
            try:
                print("Stopping RViz...")
                self.rviz_process.terminate()
                self.rviz_process.wait(timeout=5)
                print("✓ RViz stopped")
            except subprocess.TimeoutExpired:
                print("Force killing RViz...")
                self.rviz_process.kill()
            except Exception as e:
                print(f"Error stopping RViz: {e}")
            finally:
                self.rviz_process = None
                self._is_running = False
    
    def stop_all(self):
        """Stop all visualization processes (alias for stop_rviz)"""
        self.stop_rviz()
    
    def is_running(self):
        """Check if RViz is running"""
        if self._is_running and self.rviz_process:
            # Check if process is still alive
            if self.rviz_process.poll() is None:
                return True
            else:
                # Process died
                self._is_running = False
                return False
        return False
    
    def restart_rviz(self):
        """Restart RViz"""
        print("Restarting RViz...")
        self.stop_rviz()
        time.sleep(1)
        self.launch_rviz()
    
    # Legacy methods for backward compatibility with old code
    def launch_both(self):
        """
        Legacy method - now just launches RViz
        Kept for backward compatibility with existing code
        """
        print("Note: Gazebo disabled for performance. Launching RViz only.")
        self.launch_rviz()
    
    def launch_gazebo(self):
        """
        Legacy method - disabled for performance
        """
        print("⚠ Gazebo launch disabled (too resource-intensive)")
        print("Using RViz-only mode for better performance")


# Convenience function for quick testing
def test_rviz_launch():
    """Test function to verify RViz can launch"""
    manager = ROSVisualizationManager()
    try:
        manager.launch_rviz()
        print("\nRViz is running. Close RViz window to exit.")
        
        # Wait for RViz to close
        while manager.is_running():
            time.sleep(1)
            
        print("RViz closed.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        manager.stop_all()


if __name__ == "__main__":
    print("=== RViz Visualization Manager Test ===")
    test_rviz_launch()