"""
Simulator Module
Performs offline trajectory simulation and failure detection
"""

import numpy as np
import pandas as pd


class HuskyKinematicModel:
    """Encapsulates Clearpath Husky kinematic model"""
    
    def __init__(self, wheelbase=0.57):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheelbase = wheelbase
        
    def update_pose(self, v, w, dt):
        """
        Update pose from velocity inputs
        
        Args:
            v (float): Linear velocity (m/s)
            w (float): Angular velocity (rad/s)
            dt (float): Time step (s)
        """
        self._validate_inputs(v, w)
        
        # Unicycle model integration
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
    def get_pose(self):
        """
        Get current pose
        
        Returns:
            tuple: (x, y, theta)
        """
        return (self.x, self.y, self.theta)
        
    def _validate_inputs(self, v, w):
        """
        Validate velocity inputs against Husky limits
        
        Args:
            v (float): Linear velocity
            w (float): Angular velocity
        """
        # Husky max speed is ~1.0 m/s
        MAX_LINEAR_VEL = 1.0
        MAX_ANGULAR_VEL = 2.0
        
        if abs(v) > MAX_LINEAR_VEL:
            print(f"Warning: Linear velocity {v} exceeds max {MAX_LINEAR_VEL}")
        if abs(w) > MAX_ANGULAR_VEL:
            print(f"Warning: Angular velocity {w} exceeds max {MAX_ANGULAR_VEL}")


class Simulator:
    """Performs trajectory simulation and failure detection"""
    
    def __init__(self):
        self.model = HuskyKinematicModel()
        
    def simulate_trajectory(self, odom_df, obstacles_df=None):
        """
        Simulate vehicle trajectory from odometry data
        
        Args:
            odom_df (pd.DataFrame): Odometry data with velocity commands
            obstacles_df (pd.DataFrame): Optional obstacle data
            
        Returns:
            tuple: (trajectory list, collision_detected bool)
        """
        trajectory = []
        collision_detected = False
        
        # Reset model
        self.model = HuskyKinematicModel()
        
        # Simulate if we have data
        if odom_df is not None and not odom_df.empty:
            # Assume columns: timestamp, linear_x, angular_z
            for idx, row in odom_df.iterrows():
                v = row.get('linear_x', 0.0) if 'linear_x' in row else 0.0
                w = row.get('angular_z', 0.0) if 'angular_z' in row else 0.0
                dt = 0.1  # Assume 10Hz
                
                self.model.update_pose(v, w, dt)
                pose = self.model.get_pose()
                trajectory.append(pose)
                
            # Check for collisions if obstacles provided
            if obstacles_df is not None and not obstacles_df.empty:
                collision_detected = self.detect_collisions(
                    trajectory, 
                    obstacles_df, 
                    threshold=0.5
                )
        else:
            # Demo trajectory
            for i in range(100):
                v = 1.0
                w = 0.1
                dt = 0.1
                self.model.update_pose(v, w, dt)
                trajectory.append(self.model.get_pose())
                
        return trajectory, collision_detected
        
    def detect_collisions(self, trajectory, obstacles, threshold=0.5):
        """
        Detect collisions between trajectory and obstacles
        
        Args:
            trajectory (list): List of (x, y, theta) tuples
            obstacles (list or pd.DataFrame): Obstacle positions
            threshold (float): Collision distance threshold (m)
            
        Returns:
            bool: True if collision detected
        """
        if isinstance(obstacles, pd.DataFrame):
            # Convert DataFrame to list of positions
            obstacle_list = []
            if 'x' in obstacles.columns and 'y' in obstacles.columns:
                for _, row in obstacles.iterrows():
                    obstacle_list.append((row['x'], row['y']))
        else:
            obstacle_list = obstacles
            
        # Check each trajectory point against obstacles
        for x, y, _ in trajectory:
            for obs_x, obs_y in obstacle_list:
                distance = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                if distance < threshold:
                    return True
                    
        return False
        
    def _integrate_pose(self, v, w, dt):
        """
        Integrate pose using unicycle model
        
        Args:
            v (float): Linear velocity
            w (float): Angular velocity
            dt (float): Time step
            
        Returns:
            tuple: (dx, dy, dtheta)
        """
        # Simple integration
        dx = v * dt
        dy = 0.0
        dtheta = w * dt
        
        return (dx, dy, dtheta)
        
    def analyze_trajectory(self, trajectory):
        """
        Analyze trajectory for anomalies
        
        Args:
            trajectory (list): List of poses
            
        Returns:
            dict: Trajectory analysis results
        """
        if not trajectory:
            return {}
            
        # Calculate total distance
        total_distance = 0.0
        for i in range(1, len(trajectory)):
            x1, y1, _ = trajectory[i-1]
            x2, y2, _ = trajectory[i]
            distance = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            total_distance += distance
            
        # Get final position
        final_x, final_y, final_theta = trajectory[-1]
        
        return {
            'total_distance': total_distance,
            'final_position': (final_x, final_y),
            'final_heading': final_theta,
            'num_points': len(trajectory)
        }
