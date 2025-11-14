"""
DataAnalyzer Module
Processes and characterizes CSV data for correlations and anomalies
Security-focused analysis for AV behavior monitoring
"""

import os
import pandas as pd
import numpy as np
from scipy import stats
from sklearn.ensemble import IsolationForest
from datetime import datetime


class DataAnalyzer:
    """Analyzes CSV data from simulations with security monitoring"""

    def __init__(self):
        self.dfs = {}
        # Default security thresholds (configurable)
        self.thresholds = {
            'velocity_max': 5.0,  # m/s
            'angular_velocity_max': 2.0,  # rad/s
            'acceleration_max': 3.0,  # m/s^2
            'path_deviation_max': 2.0,  # meters
            'sensor_dropout_max': 1.0,  # seconds
            'zscore_threshold': 3.0,
            'isolation_contamination': 0.1  # 10% anomalies expected
        }
        
    def load_csvs(self, csv_dir):
        """
        Load all CSV files from a directory
        
        Args:
            csv_dir (str): Directory containing CSV files
            
        Returns:
            dict: Dictionary of loaded DataFrames
        """
        self.dfs = {}
        
        if not os.path.exists(csv_dir):
            return self.dfs
            
        for filename in os.listdir(csv_dir):
            if filename.endswith('.csv'):
                file_path = os.path.join(csv_dir, filename)
                try:
                    df = self._parse_csv(file_path)
                    self.dfs[filename] = df
                except Exception as e:
                    print(f"Error loading {filename}: {e}")
                    
        return self.dfs
        
    def compute_correlations(self, dfs):
        """
        Compute correlation matrix from DataFrames
        
        Args:
            dfs (dict): Dictionary of DataFrames
            
        Returns:
            pd.DataFrame: Correlation matrix
        """
        if not dfs:
            return pd.DataFrame()
            
        # For demo: create sample correlation matrix
        # In real implementation, would merge DFs and compute correlations
        sample_data = np.random.rand(100, 4) * 10
        df = pd.DataFrame(
            sample_data,
            columns=['linear.x', 'angular.z', 'position.x', 'velocity']
        )
        
        # Add some correlations
        df['linear.x'] = df['velocity'] * 0.9 + np.random.randn(100) * 0.5
        df['angular.z'] = np.random.randn(100) * 0.2
        
        corr_matrix = df.corr()
        return corr_matrix
        
    def detect_anomalies(self, df, column, threshold=3.0):
        """
        Detect outliers using z-scores
        
        Args:
            df (pd.DataFrame): Input DataFrame
            column (str): Column to analyze
            threshold (float): Z-score threshold
            
        Returns:
            pd.DataFrame: Rows with anomalies
        """
        if df is None or df.empty or column not in df.columns:
            return pd.DataFrame()
            
        z_scores = self._compute_zscore(df[column])
        anomalies = df[np.abs(z_scores) > threshold]
        
        return anomalies
        
    def _parse_csv(self, file_path):
        """
        Parse a CSV file
        
        Args:
            file_path (str): Path to CSV file
            
        Returns:
            pd.DataFrame: Parsed DataFrame
        """
        try:
            df = pd.read_csv(file_path)
            return df
        except Exception as e:
            print(f"Error parsing CSV: {e}")
            return pd.DataFrame()
            
    def _compute_zscore(self, series):
        """
        Compute z-scores for a series
        
        Args:
            series (pd.Series): Input series
            
        Returns:
            np.array: Z-scores
        """
        return stats.zscore(series, nan_policy='omit')
        
    def get_statistics(self, df, column):
        """
        Get statistical summary for a column

        Args:
            df (pd.DataFrame): Input DataFrame
            column (str): Column to analyze

        Returns:
            dict: Statistical summary
        """
        if df is None or df.empty or column not in df.columns:
            return {}

        return {
            'mean': df[column].mean(),
            'std': df[column].std(),
            'min': df[column].min(),
            'max': df[column].max(),
            'median': df[column].median(),
            'count': len(df[column])
        }

    def update_thresholds(self, new_thresholds):
        """
        Update security monitoring thresholds

        Args:
            new_thresholds (dict): Dictionary of threshold values to update
        """
        self.thresholds.update(new_thresholds)

    # ==================== ANOMALY DETECTION ====================

    def detect_behavioral_anomalies(self):
        """
        Comprehensive anomaly detection across all loaded data
        Returns detailed anomaly report with timestamps and severity

        Returns:
            pd.DataFrame: Anomaly events with timestamp, type, severity, details
        """
        anomalies = []

        # 1. Velocity anomalies
        if 'warty_cmd_vel.csv' in self.dfs:
            vel_anomalies = self._detect_velocity_anomalies(self.dfs['warty_cmd_vel.csv'])
            anomalies.extend(vel_anomalies)

        # 2. Control anomalies (command vs actual)
        if 'warty_cmd_vel.csv' in self.dfs and 'warty_odom.csv' in self.dfs:
            control_anomalies = self._detect_control_anomalies(
                self.dfs['warty_cmd_vel.csv'],
                self.dfs['warty_odom.csv']
            )
            anomalies.extend(control_anomalies)

        # 3. Trajectory anomalies
        if 'warty_pose.csv' in self.dfs:
            traj_anomalies = self._detect_trajectory_anomalies(self.dfs['warty_pose.csv'])
            anomalies.extend(traj_anomalies)

        # 4. Multivariate anomalies using Isolation Forest
        if 'warty_odom.csv' in self.dfs:
            ml_anomalies = self._detect_ml_anomalies(self.dfs['warty_odom.csv'])
            anomalies.extend(ml_anomalies)

        # Convert to DataFrame and sort by timestamp
        if anomalies:
            df_anomalies = pd.DataFrame(anomalies)
            df_anomalies = df_anomalies.sort_values('timestamp')
            return df_anomalies
        else:
            return pd.DataFrame(columns=['timestamp', 'type', 'severity', 'details', 'value'])

    def _detect_velocity_anomalies(self, cmd_vel_df):
        """Detect excessive velocities"""
        anomalies = []

        if 'linear_x' in cmd_vel_df.columns:
            # Linear velocity violations
            excessive = cmd_vel_df[abs(cmd_vel_df['linear_x']) > self.thresholds['velocity_max']]
            for idx, row in excessive.iterrows():
                anomalies.append({
                    'timestamp': row['timestamp'],
                    'type': 'Velocity Limit',
                    'severity': 'High',
                    'details': f"Linear velocity exceeds threshold",
                    'value': f"{row['linear_x']:.2f} m/s"
                })

        if 'angular_z' in cmd_vel_df.columns:
            # Angular velocity violations
            excessive_angular = cmd_vel_df[abs(cmd_vel_df['angular_z']) > self.thresholds['angular_velocity_max']]
            for idx, row in excessive_angular.iterrows():
                anomalies.append({
                    'timestamp': row['timestamp'],
                    'type': 'Angular Velocity',
                    'severity': 'High',
                    'details': f"Angular velocity exceeds threshold",
                    'value': f"{row['angular_z']:.2f} rad/s"
                })

        # Detect sudden acceleration (velocity changes)
        if 'linear_x' in cmd_vel_df.columns and len(cmd_vel_df) > 1:
            cmd_vel_df = cmd_vel_df.copy()
            cmd_vel_df['accel'] = cmd_vel_df['linear_x'].diff() / cmd_vel_df['timestamp'].diff()
            excessive_accel = cmd_vel_df[abs(cmd_vel_df['accel']) > self.thresholds['acceleration_max']]
            for idx, row in excessive_accel.iterrows():
                if pd.notna(row['accel']):
                    anomalies.append({
                        'timestamp': row['timestamp'],
                        'type': 'Acceleration',
                        'severity': 'Medium',
                        'details': f"Sudden acceleration detected",
                        'value': f"{row['accel']:.2f} m/s²"
                    })

        return anomalies

    def _detect_control_anomalies(self, cmd_vel_df, odom_df):
        """Detect command injection / control mismatch"""
        anomalies = []

        # Merge on nearest timestamp
        if 'timestamp' in cmd_vel_df.columns and 'timestamp' in odom_df.columns:
            # Sample check: compare commanded vs actual velocity
            merged = pd.merge_asof(
                cmd_vel_df.sort_values('timestamp'),
                odom_df.sort_values('timestamp'),
                on='timestamp',
                direction='nearest',
                tolerance=0.5
            )

            if 'linear_x' in merged.columns and 'twist_twist_linear_x' in merged.columns:
                merged = merged.copy()
                merged['vel_diff'] = abs(merged['linear_x'] - merged['twist_twist_linear_x'])

                # Flag large discrepancies (potential command injection)
                threshold = 0.5  # 0.5 m/s difference
                suspicious = merged[merged['vel_diff'] > threshold]

                for idx, row in suspicious.iterrows():
                    anomalies.append({
                        'timestamp': row['timestamp'],
                        'type': 'Command Mismatch',
                        'severity': 'Critical',
                        'details': f"Commanded vs actual velocity mismatch",
                        'value': f"Δ={row['vel_diff']:.2f} m/s"
                    })

        return anomalies

    def _detect_trajectory_anomalies(self, pose_df):
        """Detect erratic trajectory patterns"""
        anomalies = []

        if len(pose_df) > 2 and 'pose_position_x' in pose_df.columns:
            pose_df = pose_df.copy()
            # Calculate position changes
            pose_df['dx'] = pose_df['pose_position_x'].diff()
            pose_df['dy'] = pose_df['pose_position_y'].diff() if 'pose_position_y' in pose_df.columns else 0
            pose_df['displacement'] = np.sqrt(pose_df['dx']**2 + pose_df['dy']**2)
            pose_df['dt'] = pose_df['timestamp'].diff()

            # Detect position jumps (teleportation / GPS spoofing)
            jump_threshold = 5.0  # 5 meters in one timestep
            jumps = pose_df[pose_df['displacement'] > jump_threshold]

            for idx, row in jumps.iterrows():
                if pd.notna(row['displacement']):
                    anomalies.append({
                        'timestamp': row['timestamp'],
                        'type': 'Position Jump',
                        'severity': 'Critical',
                        'details': f"Sudden position change detected",
                        'value': f"{row['displacement']:.2f} m"
                    })

        return anomalies

    def _detect_ml_anomalies(self, odom_df):
        """Use Isolation Forest for multivariate anomaly detection"""
        anomalies = []

        # Select relevant features for anomaly detection
        feature_cols = []
        for col in ['twist_twist_linear_x', 'twist_twist_angular_z',
                    'pose_pose_position_x', 'pose_pose_position_y']:
            if col in odom_df.columns:
                feature_cols.append(col)

        if len(feature_cols) >= 2 and len(odom_df) > 10:
            features = odom_df[feature_cols].dropna()

            if len(features) > 10:
                # Isolation Forest
                clf = IsolationForest(
                    contamination=self.thresholds['isolation_contamination'],
                    random_state=42
                )
                predictions = clf.fit_predict(features)

                # Mark anomalies (-1 from Isolation Forest)
                anomaly_indices = features.index[predictions == -1]

                for idx in anomaly_indices[:50]:  # Limit to 50 entries
                    row = odom_df.loc[idx]
                    anomalies.append({
                        'timestamp': row['timestamp'] if 'timestamp' in row else idx,
                        'type': 'Behavior Pattern',
                        'severity': 'Medium',
                        'details': f"Unusual multivariate behavior",
                        'value': 'ML Detection'
                    })

        return anomalies

    # ==================== TRAJECTORY DEVIATION ====================

    def compute_trajectory_deviation(self):
        """
        Compute deviation between planned path and actual trajectory

        Returns:
            dict: Contains deviation statistics and data for plotting
        """
        result = {
            'has_data': False,
            'mean_deviation': 0,
            'max_deviation': 0,
            'deviation_over_time': pd.DataFrame(),
            'trajectory_data': None
        }

        if 'warty_navigation_manager_global_plan.csv' not in self.dfs or 'warty_pose.csv' not in self.dfs:
            return result

        plan_df = self.dfs['warty_navigation_manager_global_plan.csv']
        pose_df = self.dfs['warty_pose.csv']

        # Sample implementation: compute distance from path
        # In real scenario, would need to parse path properly
        if 'pose_position_x' in pose_df.columns and 'pose_position_y' in pose_df.columns:
            pose_df = pose_df.copy()

            # Simple deviation: distance from origin (placeholder)
            # TODO: Parse actual planned path and compute nearest point distance
            pose_df['deviation'] = np.sqrt(
                pose_df['pose_position_x']**2 + pose_df['pose_position_y']**2
            )

            result['has_data'] = True
            result['mean_deviation'] = pose_df['deviation'].mean()
            result['max_deviation'] = pose_df['deviation'].max()
            result['deviation_over_time'] = pose_df[['timestamp', 'deviation']]
            result['trajectory_data'] = pose_df[['timestamp', 'pose_position_x', 'pose_position_y']]

        return result

    # ==================== SENSOR HEALTH MONITORING ====================

    def analyze_sensor_health(self):
        """
        Monitor sensor data integrity and health

        Returns:
            dict: Sensor health status for each sensor type
        """
        health_report = {}

        # Check each sensor data source
        sensor_files = {
            'IMU': 'warty_imu_data.csv',
            'LiDAR': 'warty_lidar_points.csv',
            'LiDAR Front': 'warty_lidar_points_front.csv',
            'Odometry': 'warty_odom.csv',
            'CMD Velocity': 'warty_cmd_vel.csv'
        }

        for sensor_name, filename in sensor_files.items():
            if filename in self.dfs:
                health = self._analyze_single_sensor(self.dfs[filename], sensor_name)
                health_report[sensor_name] = health
            else:
                health_report[sensor_name] = {
                    'status': 'Missing',
                    'message_count': 0,
                    'dropouts': [],
                    'avg_rate': 0,
                    'health_score': 0
                }

        return health_report

    def _analyze_single_sensor(self, df, sensor_name):
        """Analyze health of a single sensor"""
        health = {
            'status': 'Unknown',
            'message_count': len(df),
            'dropouts': [],
            'avg_rate': 0,
            'health_score': 100
        }

        if df.empty:
            health['status'] = 'No Data'
            health['health_score'] = 0
            return health

        if 'timestamp' in df.columns:
            # Calculate message rate
            time_span = df['timestamp'].max() - df['timestamp'].min()
            if time_span > 0:
                health['avg_rate'] = len(df) / time_span

            # Detect dropouts (gaps in data)
            df = df.sort_values('timestamp')
            time_diffs = df['timestamp'].diff()

            # Find gaps larger than threshold
            dropout_threshold = self.thresholds['sensor_dropout_max']
            dropouts = time_diffs[time_diffs > dropout_threshold]

            health['dropouts'] = [
                {
                    'timestamp': df.iloc[idx]['timestamp'],
                    'duration': time_diffs.iloc[idx]
                }
                for idx in dropouts.index
            ]

            # Calculate health score
            dropout_penalty = min(len(health['dropouts']) * 10, 50)
            health['health_score'] = max(0, 100 - dropout_penalty)

            if health['health_score'] > 80:
                health['status'] = 'Healthy'
            elif health['health_score'] > 50:
                health['status'] = 'Degraded'
            else:
                health['status'] = 'Critical'
        else:
            health['status'] = 'No Timestamp'
            health['health_score'] = 50

        return health

    # ==================== EVENT TIMELINE / DIAGNOSTICS ====================

    def parse_event_timeline(self):
        """
        Parse and organize diagnostic events and system logs

        Returns:
            pd.DataFrame: Timeline of events with timestamp, source, level, message
        """
        events = []

        # Parse diagnostics
        if 'diagnostics.csv' in self.dfs:
            diag_events = self._parse_diagnostics(self.dfs['diagnostics.csv'])
            events.extend(diag_events)

        # Parse rosout logs
        if 'rosout_agg.csv' in self.dfs:
            log_events = self._parse_rosout(self.dfs['rosout_agg.csv'])
            events.extend(log_events)

        # Add anomaly events
        anomaly_df = self.detect_behavioral_anomalies()
        if not anomaly_df.empty:
            for idx, row in anomaly_df.iterrows():
                events.append({
                    'timestamp': row['timestamp'],
                    'source': 'Anomaly Detection',
                    'level': row['severity'],
                    'message': f"{row['type']}: {row['details']} ({row['value']})"
                })

        # Convert to DataFrame and sort
        if events:
            timeline = pd.DataFrame(events)
            timeline = timeline.sort_values('timestamp')
            return timeline
        else:
            return pd.DataFrame(columns=['timestamp', 'source', 'level', 'message'])

    def _parse_diagnostics(self, diag_df):
        """Parse diagnostic messages"""
        events = []

        for idx, row in diag_df.iterrows():
            if 'timestamp' in row:
                # Parse diagnostic status (simplified)
                status_str = str(row.get('status', ''))

                # Extract level and message from status string
                level = 'Info'
                if 'level: 1' in status_str or 'WARN' in status_str:
                    level = 'Warning'
                elif 'level: 2' in status_str or 'ERROR' in status_str:
                    level = 'Error'

                events.append({
                    'timestamp': row['timestamp'],
                    'source': 'Diagnostics',
                    'level': level,
                    'message': status_str[:100]  # Truncate long messages
                })

        return events

    def _parse_rosout(self, rosout_df):
        """Parse ROS log messages"""
        events = []

        # Limit to prevent overwhelming output
        sample_df = rosout_df.head(100) if len(rosout_df) > 100 else rosout_df

        for idx, row in sample_df.iterrows():
            if 'timestamp' in row:
                level = 'Info'
                # Try to extract log level if available
                msg = str(row.to_dict())

                events.append({
                    'timestamp': row['timestamp'],
                    'source': 'ROS Log',
                    'level': level,
                    'message': msg[:100]
                })

        return events
