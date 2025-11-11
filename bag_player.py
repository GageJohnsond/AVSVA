"""
BagPlayer Module
Handles ROS bag file operations including export to CSV, playback, and live republishing
"""

import os
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime

import pandas as pd
import numpy as np

# ROS imports - handle gracefully if not installed
try:
    import rosbag
    import rospy
    import genpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: rosbag/rospy not available. Falling back to bagpy if available.")

# Try bagpy as fallback
try:
    from bagpy import bagreader
    BAGPY_AVAILABLE = True
except ImportError:
    BAGPY_AVAILABLE = False


class BagPlayer:
    """Handles .bag file operations with full ROS integration"""

    def __init__(self):
        self.bag_path = None
        self.output_dir = None
        self.bag = None
        self.topics = {}
        self.publishers = {}

        # Playback control
        self.is_playing = False
        self.is_paused = False
        self.playback_thread = None
        self.playback_speed = 1.0
        self.current_time = None
        self.start_time = None
        self.end_time = None
        self.seek_time = None

        # Callbacks
        self.message_callback = None
        self.status_callback = None

    def get_bag_info(self, bag_path):
        """
        Get detailed information about a bag file

        Args:
            bag_path (str): Path to the .bag file

        Returns:
            dict: Bag file information including topics, duration, size, message counts
        """
        if not os.path.exists(bag_path):
            return {'error': f'Bag file not found: {bag_path}'}

        info = {
            'path': bag_path,
            'size': os.path.getsize(bag_path),
            'topics': {},
            'duration': 0,
            'message_count': 0,
            'start_time': None,
            'end_time': None
        }

        if ROS_AVAILABLE:
            try:
                with rosbag.Bag(bag_path, 'r') as bag:
                    # Get bag info
                    bag_info = bag.get_type_and_topic_info()

                    # Extract topic information
                    for topic, topic_info in bag_info.topics.items():
                        info['topics'][topic] = {
                            'type': topic_info.msg_type,
                            'message_count': topic_info.message_count,
                            'frequency': topic_info.frequency if hasattr(topic_info, 'frequency') else 0
                        }
                        info['message_count'] += topic_info.message_count

                    # Get time range
                    start_time = bag.get_start_time()
                    end_time = bag.get_end_time()
                    info['start_time'] = start_time
                    info['end_time'] = end_time
                    info['duration'] = end_time - start_time

                return info

            except Exception as e:
                return {'error': f'Failed to read bag file: {str(e)}'}

        elif BAGPY_AVAILABLE:
            try:
                b = bagreader(bag_path)
                info['topics'] = {topic: {'type': 'unknown', 'message_count': 0}
                                 for topic in b.topics}
                return info
            except Exception as e:
                return {'error': f'Failed to read bag file with bagpy: {str(e)}'}

        return {'error': 'No ROS bag reader available. Please install rosbag or bagpy.'}

    def export_bag_to_csv(self, bag_path, output_dir):
        """
        Export bag topics to CSV files with automatic field extraction

        Args:
            bag_path (str): Path to the .bag file
            output_dir (str): Directory for output CSVs

        Returns:
            dict: Dictionary mapping topic names to CSV file paths
        """
        self.bag_path = bag_path
        self.output_dir = output_dir

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        exported_files = {}

        if ROS_AVAILABLE:
            try:
                with rosbag.Bag(bag_path, 'r') as bag:
                    # Get all topics
                    bag_info = bag.get_type_and_topic_info()
                    topics = list(bag_info.topics.keys())

                    # Create data containers for each topic
                    topic_data = defaultdict(list)

                    # Read all messages
                    for topic, msg, t in bag.read_messages(topics=topics):
                        # Extract message data
                        msg_dict = self._message_to_dict(msg)
                        msg_dict['timestamp'] = t.to_sec()
                        msg_dict['_topic'] = topic
                        topic_data[topic].append(msg_dict)

                    # Convert to DataFrames and save as CSV
                    for topic, messages in topic_data.items():
                        if messages:
                            df = pd.DataFrame(messages)

                            # Create safe filename
                            safe_topic = topic.replace('/', '_').strip('_')
                            csv_path = os.path.join(output_dir, f'{safe_topic}.csv')

                            # Save to CSV
                            df.to_csv(csv_path, index=False)
                            exported_files[topic] = csv_path

                            if self.status_callback:
                                self.status_callback(f'Exported {topic} ({len(messages)} messages) to {csv_path}')

                return exported_files

            except Exception as e:
                if self.status_callback:
                    self.status_callback(f'Error exporting bag: {str(e)}')
                return {'error': str(e)}

        elif BAGPY_AVAILABLE:
            try:
                b = bagreader(bag_path)
                csv_files = b.message_by_topic(topics=b.topics)
                return {topic: csv_file for topic, csv_file in zip(b.topics, csv_files)}
            except Exception as e:
                return {'error': f'Bagpy export failed: {str(e)}'}

        return {'error': 'No ROS bag reader available'}

    def _message_to_dict(self, msg):
        """
        Convert a ROS message to a dictionary with flattened fields

        Args:
            msg: ROS message object

        Returns:
            dict: Flattened message data
        """
        result = {}

        # Handle different message types
        if hasattr(msg, '__slots__'):
            for slot in msg.__slots__:
                attr = getattr(msg, slot)

                # Handle nested messages
                if hasattr(attr, '__slots__'):
                    nested = self._message_to_dict(attr)
                    for key, value in nested.items():
                        result[f'{slot}.{key}'] = value
                # Handle arrays
                elif isinstance(attr, (list, tuple)):
                    if len(attr) > 0 and hasattr(attr[0], '__slots__'):
                        # Array of messages - take first element
                        nested = self._message_to_dict(attr[0])
                        for key, value in nested.items():
                            result[f'{slot}[0].{key}'] = value
                    else:
                        # Simple array - convert to string or take first few elements
                        if len(attr) <= 10:
                            for i, val in enumerate(attr):
                                result[f'{slot}[{i}]'] = val
                        else:
                            result[f'{slot}_mean'] = np.mean(attr) if all(isinstance(x, (int, float)) for x in attr) else None
                # Handle simple types
                elif isinstance(attr, (int, float, str, bool)):
                    result[slot] = attr
                # Handle genpy.Time and genpy.Duration
                elif isinstance(attr, (genpy.Time, genpy.Duration)):
                    result[slot] = attr.to_sec()
                else:
                    result[slot] = str(attr)

        return result

    def play_bag(self, bag_path, rate=1.0, start_time=None, republish=True):
        """
        Play back bag messages with full control

        Args:
            bag_path (str): Path to the .bag file
            rate (float): Playback speed multiplier
            start_time (float): Optional start time in seconds
            republish (bool): Whether to republish messages to ROS topics

        Returns:
            bool: Success status
        """
        if not ROS_AVAILABLE:
            if self.status_callback:
                self.status_callback('Error: rosbag not available for playback')
            return False

        self.bag_path = bag_path
        self.playback_speed = rate

        # Stop any existing playback
        self.stop()

        # Start playback thread
        self.is_playing = True
        self.is_paused = False
        self.seek_time = start_time

        self.playback_thread = threading.Thread(
            target=self._playback_worker,
            args=(bag_path, rate, start_time, republish)
        )
        self.playback_thread.daemon = True
        self.playback_thread.start()

        return True

    def _playback_worker(self, bag_path, rate, start_time, republish):
        """
        Worker thread for bag playback
        """
        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                # Get time bounds
                self.start_time = bag.get_start_time()
                self.end_time = bag.get_end_time()

                # Initialize ROS node if republishing
                if republish and not rospy.core.is_initialized():
                    rospy.init_node('bag_player', anonymous=True, disable_signals=True)

                # Get all topics for publisher creation
                bag_info = bag.get_type_and_topic_info()

                # Create publishers for each topic
                if republish:
                    for topic, topic_info in bag_info.topics.items():
                        try:
                            msg_class = self._get_message_class(topic_info.msg_type)
                            if msg_class:
                                self.publishers[topic] = rospy.Publisher(
                                    topic, msg_class, queue_size=10
                                )
                        except Exception as e:
                            if self.status_callback:
                                self.status_callback(f'Failed to create publisher for {topic}: {str(e)}')

                # Wait for publishers to be ready
                if republish:
                    time.sleep(0.5)

                # Determine start point
                playback_start = start_time if start_time else self.start_time

                # Read and publish messages
                last_time = None
                real_start = time.time()

                for topic, msg, t in bag.read_messages():
                    # Check for stop signal
                    if not self.is_playing:
                        break

                    # Handle pause
                    while self.is_paused and self.is_playing:
                        time.sleep(0.1)

                    # Handle seek
                    if self.seek_time is not None:
                        if t.to_sec() < self.seek_time:
                            continue
                        else:
                            self.seek_time = None
                            last_time = None
                            real_start = time.time()

                    # Skip messages before start time
                    if t.to_sec() < playback_start:
                        continue

                    # Update current time
                    self.current_time = t.to_sec()

                    # Sleep to maintain playback rate
                    if last_time is not None:
                        msg_dt = t.to_sec() - last_time
                        real_dt = time.time() - real_start
                        sleep_time = (msg_dt / rate) - real_dt

                        if sleep_time > 0:
                            time.sleep(sleep_time)

                    # Publish message
                    if republish and topic in self.publishers:
                        try:
                            self.publishers[topic].publish(msg)
                        except Exception as e:
                            if self.status_callback:
                                self.status_callback(f'Publish error on {topic}: {str(e)}')

                    # Call message callback
                    if self.message_callback:
                        self.message_callback(topic, msg, t)

                    last_time = t.to_sec()
                    real_start = time.time()

                # Playback finished
                self.is_playing = False
                if self.status_callback:
                    self.status_callback('Playback completed')

        except Exception as e:
            self.is_playing = False
            if self.status_callback:
                self.status_callback(f'Playback error: {str(e)}')

    def _get_message_class(self, msg_type):
        """
        Dynamically import message class from type string

        Args:
            msg_type (str): Message type like 'geometry_msgs/Twist'

        Returns:
            class: Message class or None
        """
        try:
            parts = msg_type.split('/')
            if len(parts) == 2:
                package, message = parts
                module = __import__(f'{package}.msg', fromlist=[message])
                return getattr(module, message)
        except Exception:
            pass
        return None

    def pause(self):
        """Pause playback"""
        self.is_paused = True
        if self.status_callback:
            self.status_callback('Playback paused')

    def resume(self):
        """Resume playback"""
        self.is_paused = False
        if self.status_callback:
            self.status_callback('Playback resumed')

    def stop(self):
        """Stop playback"""
        self.is_playing = False
        self.is_paused = False

        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=2.0)

        # Clean up publishers
        for pub in self.publishers.values():
            try:
                pub.unregister()
            except Exception:
                pass
        self.publishers.clear()

        if self.status_callback:
            self.status_callback('Playback stopped')

    def seek(self, time_sec):
        """
        Seek to specific time in bag

        Args:
            time_sec (float): Time in seconds from bag start
        """
        if self.start_time:
            self.seek_time = self.start_time + time_sec
            if self.status_callback:
                self.status_callback(f'Seeking to {time_sec:.2f}s')

    def set_speed(self, speed):
        """
        Set playback speed

        Args:
            speed (float): Speed multiplier (0.5 = half speed, 2.0 = double speed)
        """
        self.playback_speed = speed
        if self.status_callback:
            self.status_callback(f'Playback speed set to {speed}x')

    def get_playback_progress(self):
        """
        Get current playback progress

        Returns:
            dict: Progress information
        """
        if not self.start_time or not self.end_time:
            return {'progress': 0, 'current_time': 0, 'duration': 0}

        duration = self.end_time - self.start_time
        current = (self.current_time - self.start_time) if self.current_time else 0
        progress = (current / duration * 100) if duration > 0 else 0

        return {
            'progress': progress,
            'current_time': current,
            'duration': duration,
            'is_playing': self.is_playing,
            'is_paused': self.is_paused,
            'speed': self.playback_speed
        }
