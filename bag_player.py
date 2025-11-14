"""
BagPlayer Module
Handles ROS bag file operations including export to CSV and playback
WITH BACKGROUND THREADING TO PREVENT GUI FREEZING
"""

import os
import subprocess
import csv
from collections import defaultdict
import signal
import time
from PyQt5.QtCore import QThread, pyqtSignal


class BagExportThread(QThread):
    """Background thread for exporting bag files to CSV"""
    
    # Signals for progress updates
    progress = pyqtSignal(int, int, str)  # (current, total, status_message)
    finished = pyqtSignal(str, bool, str)  # (output_dir, success, error_message)
    topic_started = pyqtSignal(str, int)  # (topic_name, message_count)
    
    def __init__(self, bag_path, output_dir, has_rosbag):
        super().__init__()
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.has_rosbag = has_rosbag
        self._is_cancelled = False
        
    def cancel(self):
        """Cancel the export operation"""
        self._is_cancelled = True
        
    def run(self):
        """Run the export in background thread"""
        try:
            if self.has_rosbag:
                output_dir = self._export_with_rosbag_api()
                if output_dir:
                    self.finished.emit(output_dir, True, "")
                else:
                    self.finished.emit("", False, "Export cancelled or failed")
            else:
                output_dir = self._create_sample_csvs()
                self.finished.emit(output_dir, True, "Sample data only - install python3-rosbag for real data")
        except Exception as e:
            self.finished.emit("", False, str(e))
            
    def _export_with_rosbag_api(self):
        """Export using rosbag Python API in background"""
        try:
            import rosbag
            
            os.makedirs(self.output_dir, exist_ok=True)
            
            self.progress.emit(0, 100, f"Opening bag file...")
            bag = rosbag.Bag(self.bag_path, 'r')
            
            if self._is_cancelled:
                bag.close()
                return None
            
            # Get bag info
            info = bag.get_type_and_topic_info()
            topics = list(info.topics.keys())
            total_messages = sum(info.topics[t].message_count for t in topics)
            
            self.progress.emit(0, 100, f"Found {len(topics)} topics, {total_messages:,} messages")
            
            # Dictionary to store CSV writers and files for each topic
            csv_writers = {}
            csv_files = {}
            topic_headers = {}
            
            # Process messages with progress updates
            message_count = 0
            last_progress_update = 0
            
            for topic, msg, t in bag.read_messages():
                if self._is_cancelled:
                    break
                    
                message_count += 1
                
                # Update progress every 500 messages or 5%
                progress_pct = int((message_count / total_messages) * 100)
                if message_count % 500 == 0 or progress_pct > last_progress_update + 5:
                    self.progress.emit(
                        message_count, 
                        total_messages,
                        f"Processing: {message_count:,}/{total_messages:,} messages ({progress_pct}%)"
                    )
                    last_progress_update = progress_pct
                
                # Sanitize topic name for filename
                topic_name = topic.replace('/', '_').strip('_')
                csv_path = os.path.join(self.output_dir, f'{topic_name}.csv')
                
                # Create CSV writer if not exists
                if topic not in csv_writers:
                    self.topic_started.emit(topic, info.topics[topic].message_count)
                    csv_files[topic] = open(csv_path, 'w', newline='')
                    csv_writers[topic] = csv.writer(csv_files[topic])
                    
                    # Write header
                    header = self._get_message_header(msg, t)
                    topic_headers[topic] = header
                    csv_writers[topic].writerow(header)
                
                # Write message data
                row = self._message_to_row(msg, t, topic_headers[topic])
                csv_writers[topic].writerow(row)
            
            # Close all CSV files
            for f in csv_files.values():
                f.close()
                
            bag.close()
            
            if self._is_cancelled:
                self.progress.emit(0, 100, "Export cancelled")
                return None
            
            self.progress.emit(
                total_messages, 
                total_messages,
                f"✓ Exported {message_count:,} messages to {len(csv_writers)} CSV files"
            )
            
            return self.output_dir
            
        except Exception as e:
            raise Exception(f"Error exporting bag: {e}")
            
    def _get_message_header(self, msg, timestamp):
        """Generate CSV header from message structure"""
        header = ['timestamp']
        
        def extract_fields(obj, prefix=''):
            fields = []
            if hasattr(obj, '__slots__'):
                for slot in obj.__slots__:
                    field = getattr(obj, slot)
                    field_name = f'{prefix}{slot}' if prefix else slot
                    
                    if hasattr(field, '__slots__'):
                        fields.extend(extract_fields(field, f'{field_name}_'))
                    else:
                        fields.append(field_name)
            return fields
        
        header.extend(extract_fields(msg))
        return header
        
    def _message_to_row(self, msg, timestamp, header):
        """Convert ROS message to CSV row"""
        row = [timestamp.to_sec()]
        
        def extract_values(obj):
            values = []
            if hasattr(obj, '__slots__'):
                for slot in obj.__slots__:
                    field = getattr(obj, slot)
                    
                    if hasattr(field, '__slots__'):
                        values.extend(extract_values(field))
                    elif isinstance(field, (list, tuple)):
                        values.extend(field)
                    else:
                        values.append(field)
            return values
        
        row.extend(extract_values(msg))
        return row
        
    def _create_sample_csvs(self):
        """Create sample CSV files as fallback"""
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.progress.emit(0, 100, "Creating sample data (python3-rosbag not installed)")
        
        topics = {
            'cmd_vel': ['timestamp', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'],
            'imu_data': ['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 
                        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                        'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'],
            'navsat_fix': ['timestamp', 'latitude', 'longitude', 'altitude', 'status', 'service'],
            'odometry': ['timestamp', 'position_x', 'position_y', 'position_z', 
                        'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                        'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z']
        }
        
        for i, (topic, headers) in enumerate(topics.items()):
            self.progress.emit(i + 1, len(topics), f"Creating sample CSV: {topic}")
            csv_path = os.path.join(self.output_dir, f'{topic}.csv')
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                import random
                base_time = time.time()
                for j in range(100):
                    row = [base_time + j * 0.1] + [random.uniform(-1, 1) for _ in range(len(headers) - 1)]
                    writer.writerow(row)
        
        return self.output_dir


class BagPlayer:
    """Handles .bag file operations with real ROS integration"""
    
    def __init__(self):
        self.bag_path = None
        self.output_dir = None
        self.play_process = None
        self.bag_info_cache = {}
        self.has_rosbag = self._check_rosbag_available()
        self.export_thread = None
        
        # Playback state
        self._playback_start_time = None
        self._is_playing = False
        
    def _check_rosbag_available(self):
        """Check if rosbag Python API is available"""
        try:
            import rosbag
            print("✓ rosbag Python API available")
            return True
        except ImportError:
            print("⚠ rosbag Python API not available. Install with:")
            print("  sudo apt-get install python3-rosbag")
            return False
        
    def export_bag_to_csv_async(self, bag_path, output_dir, progress_callback=None, 
                                 finished_callback=None, topic_started_callback=None):
        """
        Export bag topics to CSV files in background thread (NON-BLOCKING)
        
        Args:
            bag_path (str): Path to the .bag file
            output_dir (str): Directory for output CSVs
            progress_callback: Function(current, total, message) called for progress updates
            finished_callback: Function(output_dir, success, error_msg) called when done
            topic_started_callback: Function(topic_name, msg_count) called when new topic starts
            
        Returns:
            BagExportThread: The background thread (can call .cancel() to stop)
        """
        self.bag_path = bag_path
        self.output_dir = output_dir
        
        # Create and configure the export thread
        self.export_thread = BagExportThread(bag_path, output_dir, self.has_rosbag)
        
        # Connect callbacks if provided
        if progress_callback:
            self.export_thread.progress.connect(progress_callback)
        if finished_callback:
            self.export_thread.finished.connect(finished_callback)
        if topic_started_callback:
            self.export_thread.topic_started.connect(topic_started_callback)
            
        # Start the thread
        self.export_thread.start()
        
        return self.export_thread
    
    def export_bag_to_csv(self, bag_path, output_dir):
        """
        LEGACY SYNCHRONOUS METHOD - NOT RECOMMENDED FOR LARGE FILES
        Use export_bag_to_csv_async() instead for GUI applications
        
        This method is kept for backwards compatibility but will block
        """
        print("⚠ WARNING: Using synchronous export (will freeze GUI)")
        print("⚠ Consider using export_bag_to_csv_async() instead")
        
        self.bag_path = bag_path
        self.output_dir = output_dir
        
        os.makedirs(output_dir, exist_ok=True)
        
        if self.has_rosbag:
            return self._export_with_rosbag_api_sync(bag_path, output_dir)
        else:
            return self._create_sample_csvs_sync(output_dir)
    
    def _export_with_rosbag_api_sync(self, bag_path, output_dir):
        """Synchronous export (legacy)"""
        try:
            import rosbag
            
            print(f"Opening bag file: {bag_path}")
            bag = rosbag.Bag(bag_path, 'r')
            
            info = bag.get_type_and_topic_info()
            topics = list(info.topics.keys())
            
            print(f"Found {len(topics)} topics in bag:")
            for topic in topics:
                msg_count = info.topics[topic].message_count
                msg_type = info.topics[topic].msg_type
                print(f"  - {topic} ({msg_type}): {msg_count} messages")
            
            csv_writers = {}
            csv_files = {}
            topic_headers = {}
            
            message_count = 0
            for topic, msg, t in bag.read_messages():
                message_count += 1
                
                if message_count % 1000 == 0:
                    print(f"  Processed {message_count} messages...")
                
                topic_name = topic.replace('/', '_').strip('_')
                csv_path = os.path.join(output_dir, f'{topic_name}.csv')
                
                if topic not in csv_writers:
                    print(f"Creating CSV for topic: {topic}")
                    csv_files[topic] = open(csv_path, 'w', newline='')
                    csv_writers[topic] = csv.writer(csv_files[topic])
                    
                    header = self._get_message_header(msg, t)
                    topic_headers[topic] = header
                    csv_writers[topic].writerow(header)
                
                row = self._message_to_row(msg, t, topic_headers[topic])
                csv_writers[topic].writerow(row)
            
            for f in csv_files.values():
                f.close()
                
            bag.close()
            
            print(f"✓ Successfully exported {message_count} messages to {len(csv_writers)} CSV files")
            
            return output_dir
            
        except Exception as e:
            print(f"ERROR exporting bag to CSV: {e}")
            return self._create_sample_csvs_sync(output_dir)
    
    def _get_message_header(self, msg, timestamp):
        """Generate CSV header from message structure"""
        header = ['timestamp']
        
        def extract_fields(obj, prefix=''):
            fields = []
            if hasattr(obj, '__slots__'):
                for slot in obj.__slots__:
                    field = getattr(obj, slot)
                    field_name = f'{prefix}{slot}' if prefix else slot
                    
                    if hasattr(field, '__slots__'):
                        fields.extend(extract_fields(field, f'{field_name}_'))
                    else:
                        fields.append(field_name)
            return fields
        
        header.extend(extract_fields(msg))
        return header
        
    def _message_to_row(self, msg, timestamp, header):
        """Convert ROS message to CSV row"""
        row = [timestamp.to_sec()]
        
        def extract_values(obj):
            values = []
            if hasattr(obj, '__slots__'):
                for slot in obj.__slots__:
                    field = getattr(obj, slot)
                    
                    if hasattr(field, '__slots__'):
                        values.extend(extract_values(field))
                    elif isinstance(field, (list, tuple)):
                        values.extend(field)
                    else:
                        values.append(field)
            return values
        
        row.extend(extract_values(msg))
        return row
        
    def _create_sample_csvs_sync(self, output_dir):
        """Create sample CSV files (synchronous)"""
        os.makedirs(output_dir, exist_ok=True)
        
        topics = {
            'cmd_vel': ['timestamp', 'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'],
            'imu_data': ['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 
                        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                        'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'],
            'navsat_fix': ['timestamp', 'latitude', 'longitude', 'altitude', 'status', 'service'],
            'odometry': ['timestamp', 'position_x', 'position_y', 'position_z', 
                        'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                        'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z']
        }
        
        for topic, headers in topics.items():
            csv_path = os.path.join(output_dir, f'{topic}.csv')
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                import random
                base_time = time.time()
                for i in range(100):
                    row = [base_time + i * 0.1] + [random.uniform(-1, 1) for _ in range(len(headers) - 1)]
                    writer.writerow(row)
        
        return output_dir
        
    def play_bag(self, bag_path, rate=1.0, topics=None, start_time=None, duration=None, loop=True):
        """Play back bag messages using rosbag play command"""
        self.bag_path = bag_path
        self._is_playing = True
        self._playback_start_time = time.time()
        
        cmd = ['rosbag', 'play']
        
        # Add loop flag by default
        if loop:
            cmd.append('-l')
        
        if rate != 1.0:
            cmd.extend(['-r', str(rate)])
            
        if start_time is not None:
            cmd.extend(['-s', str(start_time)])
            
        if duration is not None:
            cmd.extend(['-u', str(duration)])
            
        if topics:
            cmd.extend(['--topics'] + topics)
            
        cmd.append(bag_path)
        
        try:
            print(f"Starting rosbag play: {' '.join(cmd)}")
            self.play_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=os.environ.copy()
            )
            print(f"✓ rosbag play started (PID: {self.play_process.pid})")
            return self.play_process
            
        except Exception as e:
            print(f"ERROR starting rosbag play: {e}")
            self._is_playing = False
            return None
            
    def stop_playback(self):
        """Stop the current bag playback"""
        self._is_playing = False
        if self.play_process:
            try:
                os.killpg(os.getpgid(self.play_process.pid), signal.SIGTERM)
                self.play_process.wait(timeout=5)
                print("✓ rosbag play stopped")
            except Exception as e:
                print(f"Error stopping playback: {e}")
            finally:
                self.play_process = None
                
    def pause_playback(self):
        """Pause the current bag playback"""
        self._is_playing = False
        if self.play_process:
            try:
                os.killpg(os.getpgid(self.play_process.pid), signal.SIGSTOP)
            except Exception as e:
                print(f"Error pausing playback: {e}")
                
    def resume_playback(self):
        """Resume the paused bag playback"""
        self._is_playing = True
        if self.play_process:
            try:
                os.killpg(os.getpgid(self.play_process.pid), signal.SIGCONT)
            except Exception as e:
                print(f"Error resuming playback: {e}")
                
    def get_bag_info(self, bag_path):
        """Get information about a bag file"""
        if bag_path in self.bag_info_cache:
            return self.bag_info_cache[bag_path]
            
        if not self.has_rosbag:
            return self._get_fallback_info(bag_path)
            
        try:
            import rosbag
            
            bag = rosbag.Bag(bag_path, 'r')
            type_info = bag.get_type_and_topic_info()
            topics = list(type_info.topics.keys())
            total_messages = sum(type_info.topics[t].message_count for t in topics)
            
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            duration = end_time - start_time if start_time and end_time else 0
            
            size = os.path.getsize(bag_path)
            
            bag.close()
            
            info_dict = {
                'duration': duration,
                'size': f'{size / (1024**3):.2f} GB',
                'topics': topics,
                'message_count': total_messages,
                'start_time': start_time,
                'end_time': end_time
            }
            
            self.bag_info_cache[bag_path] = info_dict
            return info_dict
            
        except Exception as e:
            print(f"Error getting bag info: {e}")
            return self._get_fallback_info(bag_path)
    
    def _get_fallback_info(self, bag_path):
        """Get fallback info when rosbag API not available"""
        size = os.path.getsize(bag_path) if os.path.exists(bag_path) else 0
        return {
            'duration': 'Unknown',
            'size': f'{size / (1024**3):.2f} GB',
            'topics': ['Unknown - install python3-rosbag to read'],
            'message_count': 0
        }
            
    def get_topics_list(self, bag_path):
        """Get list of topics in bag file"""
        info = self.get_bag_info(bag_path)
        return info.get('topics', [])
        
    def start_playback(self, bag_path):
        """Start playback for the GUI"""
        return self.play_bag(bag_path, rate=1.0)
        
    def is_playing(self):
        """Check if rosbag play process is active"""
        return self._is_playing and (self.play_process and self.play_process.poll() is None)

    def get_bag_duration(self, bag_path):
        """Get actual duration from bag file info"""
        info = self.get_bag_info(bag_path)
        duration = info.get('duration', 60.0)
        
        if isinstance(duration, str) and 'Unknown' not in duration:
            try:
                duration = float(duration.split()[0])
            except (ValueError, IndexError):
                duration = 60.0
        elif isinstance(duration, (int, float)):
            duration = float(duration)
        else:
            duration = 60.0
            
        return duration

    def get_playback_start_time(self):
        """Returns the system time when playback started"""
        return self._playback_start_time if self._playback_start_time else time.time()

    def get_elapsed_time(self, start_time):
        """Calculates elapsed time since start_time"""
        return time.time() - start_time