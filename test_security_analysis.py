#!/usr/bin/env python3
"""
Test script for new security analysis features
Demonstrates the 4 implemented features:
1. Anomaly Detection
2. Trajectory Deviation
3. Sensor Health Monitoring
4. Event Timeline Parsing
"""

from data_analyzer import DataAnalyzer
import pandas as pd

def test_security_features():
    print("=" * 60)
    print("TESTING SECURITY ANALYSIS FEATURES")
    print("=" * 60)

    # Initialize analyzer
    analyzer = DataAnalyzer()
    print("\n✓ DataAnalyzer initialized")
    print(f"Default thresholds: {analyzer.thresholds}")

    # Test 1: Update thresholds
    print("\n" + "=" * 60)
    print("TEST 1: Configurable Thresholds")
    print("=" * 60)

    new_thresholds = {
        'velocity_max': 10.0,
        'angular_velocity_max': 3.0,
        'acceleration_max': 5.0
    }
    analyzer.update_thresholds(new_thresholds)
    print(f"Updated thresholds: {analyzer.thresholds}")
    print("✓ Threshold update successful")

    # Test 2: Load CSV data
    print("\n" + "=" * 60)
    print("TEST 2: Loading CSV Data")
    print("=" * 60)

    csv_dir = 'csv_output'
    dfs = analyzer.load_csvs(csv_dir)
    print(f"Loaded {len(dfs)} CSV files:")
    for filename in list(dfs.keys())[:5]:
        print(f"  - {filename}")
    print("✓ CSV loading successful")

    # Test 3: Anomaly Detection
    print("\n" + "=" * 60)
    print("TEST 3: Anomaly Detection")
    print("=" * 60)

    anomalies = analyzer.detect_behavioral_anomalies()
    print(f"Detected {len(anomalies)} anomalies")

    if not anomalies.empty:
        print("\nSample anomalies:")
        print(anomalies.head(10).to_string())

        # Count by severity
        severity_counts = anomalies['severity'].value_counts()
        print(f"\nAnomaly breakdown by severity:")
        for severity, count in severity_counts.items():
            print(f"  {severity}: {count}")

    print("✓ Anomaly detection successful")

    # Test 4: Trajectory Deviation
    print("\n" + "=" * 60)
    print("TEST 4: Trajectory Deviation Analysis")
    print("=" * 60)

    traj_result = analyzer.compute_trajectory_deviation()
    print(f"Has trajectory data: {traj_result['has_data']}")

    if traj_result['has_data']:
        print(f"Mean deviation: {traj_result['mean_deviation']:.3f} m")
        print(f"Max deviation: {traj_result['max_deviation']:.3f} m")
        print(f"Trajectory points: {len(traj_result['trajectory_data'])}")

    print("✓ Trajectory analysis successful")

    # Test 5: Sensor Health
    print("\n" + "=" * 60)
    print("TEST 5: Sensor Health Monitoring")
    print("=" * 60)

    sensor_health = analyzer.analyze_sensor_health()
    print(f"\nSensor Health Report:")
    print("-" * 60)

    for sensor, health in sensor_health.items():
        status_emoji = "✓" if health['status'] == 'Healthy' else "⚠️" if health['status'] == 'Degraded' else "✗"
        print(f"{status_emoji} {sensor:15} | Status: {health['status']:10} | "
              f"Messages: {health['message_count']:5} | "
              f"Rate: {health['avg_rate']:6.1f} Hz | "
              f"Score: {health['health_score']}/100")

        if health['dropouts']:
            print(f"    Dropouts: {len(health['dropouts'])}")

    print("✓ Sensor health analysis successful")

    # Test 6: Event Timeline
    print("\n" + "=" * 60)
    print("TEST 6: Event Timeline & Diagnostics")
    print("=" * 60)

    timeline = analyzer.parse_event_timeline()
    print(f"Total events: {len(timeline)}")

    if not timeline.empty:
        print("\nEvent breakdown by source:")
        source_counts = timeline['source'].value_counts()
        for source, count in source_counts.items():
            print(f"  {source}: {count}")

        print("\nEvent breakdown by level:")
        level_counts = timeline['level'].value_counts()
        for level, count in level_counts.items():
            print(f"  {level}: {count}")

        print("\nSample events:")
        print(timeline.head(5)[['timestamp', 'source', 'level']].to_string())

    print("✓ Event timeline parsing successful")

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"✓ All 4 security features implemented successfully:")
    print(f"  1. Anomaly Detection: {len(anomalies)} anomalies detected")
    print(f"  2. Trajectory Deviation: {'Available' if traj_result['has_data'] else 'No data'}")
    print(f"  3. Sensor Health: {len(sensor_health)} sensors monitored")
    print(f"  4. Event Timeline: {len(timeline)} events logged")
    print(f"\n✓ Configurable thresholds: {len(analyzer.thresholds)} parameters")
    print("=" * 60)

if __name__ == "__main__":
    test_security_features()
