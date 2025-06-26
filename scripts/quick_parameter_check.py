#!/usr/bin/env python3
"""
Quick automated parameter check script.
Runs a brief test sequence to validate robot parameters without user interaction.
"""

import time
import numpy as np
import yaml
import sys
from pathlib import Path

# Add furuta to path
sys.path.append(str(Path(__file__).parent.parent))

from furuta.polimi_robot import PolimiRobot

def load_config(config_path="scripts/configs/env/polimi.yaml"):
    """Load the polimi configuration"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def quick_test_sequence(robot, config):
    """Run a quick automated test sequence"""
    print("🧪 Running automated test sequence...")
    
    # Test commands to try
    test_commands = [0.0, 0.2, -0.2, 0.4, -0.4, 0.0]
    test_duration = 2.0  # seconds per command
    control_freq = config.get('control_freq', 20)
    
    data = {
        'motor_angles': [],
        'pendulum_angles': [],
        'motor_speeds': [],
        'pendulum_speeds': [],
        'timestamps': [],
        'commands': []
    }
    
    for i, command in enumerate(test_commands):
        print(f"  Step {i+1}/{len(test_commands)}: Motor command = {command}")
        
        start_time = time.time()
        prev_motor_angle = None
        prev_pendulum_angle = None
        prev_timestamp = None
        
        while time.time() - start_time < test_duration:
            motor_angle, pendulum_angle, timestamp = robot.step(command)
            
            # Calculate speeds
            motor_speed = 0.0
            pendulum_speed = 0.0
            
            if prev_timestamp is not None:
                dt = timestamp - prev_timestamp
                if dt > 0:
                    motor_speed = (motor_angle - prev_motor_angle) / dt
                    pendulum_speed = (pendulum_angle - prev_pendulum_angle) / dt
            
            # Store data
            data['motor_angles'].append(motor_angle)
            data['pendulum_angles'].append(pendulum_angle)
            data['motor_speeds'].append(motor_speed)
            data['pendulum_speeds'].append(pendulum_speed)
            data['timestamps'].append(timestamp)
            data['commands'].append(command)
            
            prev_motor_angle = motor_angle
            prev_pendulum_angle = pendulum_angle
            prev_timestamp = timestamp
            
            time.sleep(1.0 / control_freq)
    
    # Final stop
    robot.step(0.0)
    print("✓ Test sequence completed")
    
    return data

def analyze_data(data, config):
    """Analyze collected data and compare with configuration"""
    print("\n📊 ANALYSIS RESULTS:")
    print("=" * 50)
    
    # Convert to numpy arrays for analysis
    motor_angles = np.array(data['motor_angles'])
    pendulum_angles = np.array(data['pendulum_angles'])
    motor_speeds = np.array(data['motor_speeds'])
    pendulum_speeds = np.array(data['pendulum_speeds'])
    
    # Basic statistics
    print(f"\n📈 MEASURED RANGES:")
    print(f"  Motor angle:     {np.degrees(np.min(motor_angles)):8.1f}° to {np.degrees(np.max(motor_angles)):8.1f}°")
    print(f"  Pendulum angle:  {np.degrees(np.min(pendulum_angles)):8.1f}° to {np.degrees(np.max(pendulum_angles)):8.1f}°")
    print(f"  Motor speed:     {np.min(motor_speeds)/(2*np.pi):8.1f} to {np.max(motor_speeds)/(2*np.pi):8.1f} rev/s")
    print(f"  Pendulum speed:  {np.min(pendulum_speeds)/(2*np.pi):8.1f} to {np.max(pendulum_speeds)/(2*np.pi):8.1f} rev/s")
    
    # Configuration comparison
    print(f"\n⚙️  CONFIGURATION LIMITS:")
    angle_limits = config.get('angle_limits', [None, None])
    speed_limits = config.get('speed_limits', [None, None])
    print(f"  Angle limits:    {angle_limits} (pendulum)")
    print(f"  Speed limits:    {speed_limits} (motor, rev/s)")
    
    # Validation
    issues = []
    warnings = []
    
    # Check angle limits
    max_pendulum_angle_deg = np.degrees(np.max(np.abs(pendulum_angles)))
    if angle_limits[1] is not None:
        if max_pendulum_angle_deg > angle_limits[1]:
            issues.append(f"Pendulum exceeded angle limit: {max_pendulum_angle_deg:.1f}° > {angle_limits[1]}°")
        elif max_pendulum_angle_deg < angle_limits[1] * 0.3:
            warnings.append(f"Pendulum angle limit may be too high: max observed {max_pendulum_angle_deg:.1f}° vs limit {angle_limits[1]}°")
    
    # Check speed limits
    max_motor_speed_rps = np.max(np.abs(motor_speeds)) / (2 * np.pi)
    if speed_limits[0] is not None:
        if max_motor_speed_rps > speed_limits[0]:
            issues.append(f"Motor exceeded speed limit: {max_motor_speed_rps:.1f} > {speed_limits[0]} rev/s")
        elif max_motor_speed_rps < speed_limits[0] * 0.3:
            warnings.append(f"Motor speed limit may be too high: max observed {max_motor_speed_rps:.1f} vs limit {speed_limits[0]} rev/s")
    
    # Control frequency check
    if len(data['timestamps']) > 1:
        actual_freq = 1.0 / np.mean(np.diff(data['timestamps']))
        config_freq = config.get('control_freq', 20)
        freq_error = abs(actual_freq - config_freq) / config_freq
        
        print(f"\n⏱️  CONTROL FREQUENCY:")
        print(f"  Configured: {config_freq} Hz")
        print(f"  Actual:     {actual_freq:.1f} Hz")
        print(f"  Error:      {freq_error*100:.1f}%")
        
        if freq_error > 0.1:
            warnings.append(f"Control frequency error is high: {freq_error*100:.1f}%")
    
    # Print results
    if issues:
        print(f"\n❌ ISSUES FOUND:")
        for issue in issues:
            print(f"  • {issue}")
    
    if warnings:
        print(f"\n⚠️  WARNINGS:")
        for warning in warnings:
            print(f"  • {warning}")
    
    if not issues and not warnings:
        print(f"\n✅ All parameters look good!")
    
    # Recommendations
    print(f"\n💡 RECOMMENDATIONS:")
    
    if angle_limits[1] is None:
        print(f"  • Set pendulum angle limit to ~{max_pendulum_angle_deg * 1.5:.0f}°")
    
    if speed_limits[0] is None:
        print(f"  • Set motor speed limit to ~{max_motor_speed_rps * 1.5:.1f} rev/s")
    
    # Encoder validation
    motor_range = np.max(motor_angles) - np.min(motor_angles)
    print(f"  • Motor encoder sensitivity: {np.degrees(motor_range):.1f}° range detected")
    
    pendulum_range = np.max(pendulum_angles) - np.min(pendulum_angles)
    print(f"  • Pendulum encoder sensitivity: {np.degrees(pendulum_range):.1f}° range detected")
    
    if motor_range < np.radians(1):
        print(f"  ⚠️  Motor encoder range is very small - check encoder connections")
    
    if pendulum_range < np.radians(1):
        print(f"  ⚠️  Pendulum encoder range is very small - try moving pendulum manually")

def main():
    config_path = "scripts/configs/env/polimi.yaml"
    
    # Load configuration
    try:
        config = load_config(config_path)
        print(f"✓ Loaded configuration from {config_path}")
    except Exception as e:
        print(f"❌ Failed to load config: {e}")
        return
    
    # Initialize robot
    try:
        print("\n🤖 Initializing robot...")
        robot = PolimiRobot()
        robot.reset_encoders()
        print("✓ Robot initialized and encoders reset")
    except Exception as e:
        print(f"❌ Failed to initialize robot: {e}")
        return
    
    try:
        # Run test sequence
        data = quick_test_sequence(robot, config)
        
        # Analyze results
        analyze_data(data, config)
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"❌ Error during test: {e}")
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        robot.step(0.0)  # Stop motor
        robot.close()
        print("✓ Robot stopped and closed")

if __name__ == "__main__":
    main() 