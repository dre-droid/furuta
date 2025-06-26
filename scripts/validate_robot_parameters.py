#!/usr/bin/env python3
"""
Script to validate robot parameters against real hardware behavior.
This helps ensure training parameters match actual robot capabilities.
"""

import time
import numpy as np
import yaml
import os
import sys
from pathlib import Path

# Add furuta to path
sys.path.append(str(Path(__file__).parent.parent))

from furuta.polimi_robot import PolimiRobot

class RobotParameterValidator:
    def __init__(self, config_path="scripts/configs/env/polimi.yaml"):
        self.config_path = config_path
        self.robot = None
        self.config = None
        self.load_config()
        
        # Data storage for analysis
        self.motor_angles = []
        self.pendulum_angles = []
        self.motor_speeds = []
        self.pendulum_speeds = []
        self.timestamps = []
        
    def load_config(self):
        """Load the polimi configuration"""
        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"✓ Loaded config from {self.config_path}")
            print(f"  Control frequency: {self.config.get('control_freq', 'Not set')} Hz")
            print(f"  Angle limits: {self.config.get('angle_limits', 'Not set')}")
            print(f"  Speed limits: {self.config.get('speed_limits', 'Not set')}")
            print(f"  Reward function: {self.config.get('reward', 'Not set')}")
        except Exception as e:
            print(f"❌ Failed to load config: {e}")
            sys.exit(1)
            
    def initialize_robot(self):
        """Initialize the robot hardware"""
        try:
            print("\n🤖 Initializing robot...")
            self.robot = PolimiRobot()
            print("✓ Robot initialized successfully")
            
            # Reset encoders to start from zero
            self.robot.reset_encoders()
            print("✓ Encoders reset to zero")
            
        except Exception as e:
            print(f"❌ Failed to initialize robot: {e}")
            sys.exit(1)
            
    def collect_data_point(self, motor_command=0.0):
        """Collect a single data point from the robot"""
        try:
            motor_angle, pendulum_angle, timestamp = self.robot.step(motor_command)
            
            # Calculate speeds if we have previous data
            motor_speed = 0.0
            pendulum_speed = 0.0
            
            if len(self.timestamps) > 0:
                dt = timestamp - self.timestamps[-1]
                if dt > 0:
                    motor_speed = (motor_angle - self.motor_angles[-1]) / dt
                    pendulum_speed = (pendulum_angle - self.pendulum_angles[-1]) / dt
            
            # Store data
            self.motor_angles.append(motor_angle)
            self.pendulum_angles.append(pendulum_angle)
            self.motor_speeds.append(motor_speed)
            self.pendulum_speeds.append(pendulum_speed)
            self.timestamps.append(timestamp)
            
            return motor_angle, pendulum_angle, motor_speed, pendulum_speed
            
        except Exception as e:
            print(f"❌ Error collecting data: {e}")
            return None, None, None, None
            
    def check_limits(self, motor_angle, pendulum_angle, motor_speed, pendulum_speed):
        """Check if current values are within configured limits"""
        issues = []
        
        # Check angle limits
        angle_limits = self.config.get('angle_limits', [None, None])
        if angle_limits[1] is not None:  # Upper limit for pendulum angle
            pendulum_angle_deg = np.degrees(abs(pendulum_angle))
            if pendulum_angle_deg > angle_limits[1]:
                issues.append(f"⚠️  Pendulum angle ({pendulum_angle_deg:.1f}°) exceeds limit ({angle_limits[1]}°)")
        
        # Check speed limits  
        speed_limits = self.config.get('speed_limits', [None, None])
        if speed_limits[0] is not None:  # Lower limit for motor speed
            motor_speed_rps = abs(motor_speed) / (2 * np.pi)  # Convert to rev/s
            if motor_speed_rps > speed_limits[0]:
                issues.append(f"⚠️  Motor speed ({motor_speed_rps:.1f} rev/s) exceeds limit ({speed_limits[0]} rev/s)")
                
        return issues
        
    def print_status(self, motor_angle, pendulum_angle, motor_speed, pendulum_speed, motor_command):
        """Print current robot status"""
        # Clear screen and print header
        os.system('clear' if os.name == 'posix' else 'cls')
        print("🔍 ROBOT PARAMETER VALIDATION")
        print("=" * 50)
        
        # Current readings
        print(f"\n📊 CURRENT READINGS:")
        print(f"  Motor angle:     {np.degrees(motor_angle):8.2f}° ({motor_angle:8.4f} rad)")
        print(f"  Pendulum angle:  {np.degrees(pendulum_angle):8.2f}° ({pendulum_angle:8.4f} rad)")
        print(f"  Motor speed:     {motor_speed/(2*np.pi):8.2f} rev/s ({motor_speed:8.4f} rad/s)")
        print(f"  Pendulum speed:  {pendulum_speed/(2*np.pi):8.2f} rev/s ({pendulum_speed:8.4f} rad/s)")
        print(f"  Motor command:   {motor_command:8.2f}")
        
        # Configuration comparison
        print(f"\n⚙️  CONFIGURATION LIMITS:")
        angle_limits = self.config.get('angle_limits', [None, None])
        speed_limits = self.config.get('speed_limits', [None, None])
        
        print(f"  Angle limits:    {angle_limits} (pendulum)")
        print(f"  Speed limits:    {speed_limits} (motor, rev/s)")
        
        # Check for issues
        issues = self.check_limits(motor_angle, pendulum_angle, motor_speed, pendulum_speed)
        if issues:
            print(f"\n⚠️  WARNINGS:")
            for issue in issues:
                print(f"  {issue}")
        else:
            print(f"\n✓ All values within configured limits")
            
        # Statistics if we have enough data
        if len(self.motor_angles) > 10:
            print(f"\n📈 STATISTICS (last {len(self.motor_angles)} samples):")
            print(f"  Motor angle range:    {np.degrees(np.min(self.motor_angles)):6.1f}° to {np.degrees(np.max(self.motor_angles)):6.1f}°")
            print(f"  Pendulum angle range: {np.degrees(np.min(self.pendulum_angles)):6.1f}° to {np.degrees(np.max(self.pendulum_angles)):6.1f}°")
            print(f"  Motor speed range:    {np.min(self.motor_speeds)/(2*np.pi):6.1f} to {np.max(self.motor_speeds)/(2*np.pi):6.1f} rev/s")
            print(f"  Pendulum speed range: {np.min(self.pendulum_speeds)/(2*np.pi):6.1f} to {np.max(self.pendulum_speeds)/(2*np.pi):6.1f} rev/s")
        
        # Instructions
        print(f"\n🎮 CONTROLS:")
        print(f"  a/d: Apply motor torque left/right")
        print(f"  s: Stop motor")
        print(f"  r: Reset encoders")
        print(f"  q: Quit")
        print(f"\n💡 TIPS:")
        print(f"  - Manually move the pendulum to test angle limits")
        print(f"  - Use motor commands to test speed limits")
        print(f"  - Check that encoder readings match physical movement")
        
    def run_interactive_test(self):
        """Run interactive validation test"""
        print("\n🎮 Starting interactive test...")
        print("Use keyboard controls to test the robot")
        
        motor_command = 0.0
        
        try:
            import select
            import tty
            import termios
            
            # Save terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            tty.cbreak(sys.stdin.fileno())
            
            while True:
                # Collect data
                motor_angle, pendulum_angle, motor_speed, pendulum_speed = self.collect_data_point(motor_command)
                
                if motor_angle is None:
                    break
                    
                # Print status
                self.print_status(motor_angle, pendulum_angle, motor_speed, pendulum_speed, motor_command)
                
                # Check for keyboard input (non-blocking)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'q':
                        break
                    elif key == 'a':
                        motor_command = -0.3  # Turn left
                    elif key == 'd':
                        motor_command = 0.3   # Turn right
                    elif key == 's':
                        motor_command = 0.0   # Stop
                    elif key == 'r':
                        self.robot.reset_encoders()
                        self.motor_angles.clear()
                        self.pendulum_angles.clear()
                        self.motor_speeds.clear()
                        self.pendulum_speeds.clear()
                        self.timestamps.clear()
                        print("Encoders reset!")
                        time.sleep(0.5)
                        
                time.sleep(1.0 / self.config.get('control_freq', 20))
                
        except KeyboardInterrupt:
            pass
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # Stop motor
            if self.robot:
                self.robot.step(0.0)
                
    def generate_recommendations(self):
        """Generate recommendations based on collected data"""
        if len(self.motor_angles) < 10:
            print("\n⚠️  Not enough data collected for recommendations")
            return
            
        print("\n📋 RECOMMENDATIONS:")
        print("=" * 50)
        
        # Angle analysis
        max_pendulum_angle = np.degrees(np.max(np.abs(self.pendulum_angles)))
        current_angle_limit = self.config.get('angle_limits', [None, None])[1]
        
        if current_angle_limit is not None:
            if max_pendulum_angle > current_angle_limit * 0.8:
                print(f"📐 Consider increasing pendulum angle limit from {current_angle_limit}° to {max_pendulum_angle * 1.2:.0f}°")
            elif max_pendulum_angle < current_angle_limit * 0.5:
                print(f"📐 Consider decreasing pendulum angle limit from {current_angle_limit}° to {max_pendulum_angle * 1.5:.0f}°")
        else:
            print(f"📐 Consider setting pendulum angle limit to {max_pendulum_angle * 1.2:.0f}°")
            
        # Speed analysis  
        max_motor_speed = np.max(np.abs(self.motor_speeds)) / (2 * np.pi)
        current_speed_limit = self.config.get('speed_limits', [None, None])[0]
        
        if current_speed_limit is not None:
            if max_motor_speed > current_speed_limit * 0.8:
                print(f"🏃 Consider increasing motor speed limit from {current_speed_limit} to {max_motor_speed * 1.2:.1f} rev/s")
            elif max_motor_speed < current_speed_limit * 0.5:
                print(f"🏃 Consider decreasing motor speed limit from {current_speed_limit} to {max_motor_speed * 1.5:.1f} rev/s")
        else:
            print(f"🏃 Consider setting motor speed limit to {max_motor_speed * 1.2:.1f} rev/s")
            
        # Control frequency analysis
        if len(self.timestamps) > 1:
            actual_freq = 1.0 / np.mean(np.diff(self.timestamps))
            config_freq = self.config.get('control_freq', 20)
            print(f"⏱️  Actual control frequency: {actual_freq:.1f} Hz (configured: {config_freq} Hz)")
            
            if abs(actual_freq - config_freq) > config_freq * 0.1:
                print(f"⏱️  Consider adjusting control_freq to {actual_freq:.0f} Hz")
                
    def cleanup(self):
        """Clean up resources"""
        if self.robot:
            print("\n🧹 Cleaning up...")
            self.robot.close()
            print("✓ Robot closed")

def main():
    validator = RobotParameterValidator()
    
    try:
        validator.initialize_robot()
        validator.run_interactive_test()
        validator.generate_recommendations()
        
    except Exception as e:
        print(f"❌ Error during validation: {e}")
    finally:
        validator.cleanup()

if __name__ == "__main__":
    main() 