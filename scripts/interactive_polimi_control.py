#!/usr/bin/env python3
"""
Interactive Polimi Robot Control Script

This script allows you to control the Polimi robot interactively using keyboard inputs.
Use WASD keys to control the motor with different strengths.

Controls:
- 'a': Turn left (negative motor command)
- 'd': Turn right (positive motor command)  
- 's': Stop motor
- 'r': Reset encoders
- 'q': Quit
- '1-9': Set motor strength (1=10%, 9=90%)
- '0': Set motor strength to 100%

Press keys to control the robot in real-time!
"""

import time
import numpy as np
import sys
import os
from pathlib import Path

# Add furuta to path
sys.path.append(str(Path(__file__).parent.parent))

from furuta.polimi_robot import PolimiRobot
from furuta.rl.envs.furuta_base import REWARDS


class InteractivePolimiControl:
    def __init__(self):
        self.robot = None
        self.motor_command = 0.0
        self.motor_strength = 0.3  # Default strength (30%)
        self.running = True
        self.control_freq = 30  # Hz
        self.reward_function = "cos_alpha"  # Default reward function
        self.show_all_rewards = True  # Show all reward functions
        
    def initialize_robot(self):
        """Initialize the Polimi robot"""
        try:
            print("🤖 Initializing Polimi robot...")
            self.robot = PolimiRobot()
            print("✓ Robot initialized successfully")
            
            # Reset encoders to start from zero
            self.robot.reset_encoders()
            print("✓ Encoders reset to zero")
            
            return True
        except Exception as e:
            print(f"❌ Failed to initialize robot: {e}")
            return False
    
    def calculate_reward(self, motor_angle, pendulum_angle, motor_speed, pendulum_speed):
        """Calculate reward for current state"""
        # Create state vector [theta, alpha, theta_dot, alpha_dot]
        state = np.array([motor_angle, pendulum_angle, motor_speed, pendulum_speed])
        
        # Calculate reward using the same function as the environment
        reward_func = REWARDS[self.reward_function]
        reward = reward_func(state)
        
        return reward
    
    def print_controls(self):
        """Print the control instructions"""
        print("\n" + "="*60)
        print("🎮 INTERACTIVE POLIMI ROBOT CONTROL")
        print("="*60)
        print("Controls:")
        print("  'a'     : Turn left (negative motor command)")
        print("  'd'     : Turn right (positive motor command)")
        print("  's'     : Stop motor")
        print("  'r'     : Reset encoders")
        print("  '1-9'   : Set motor strength (1=10%, 9=90%)")
        print("  '0'     : Set motor strength to 100%")
        print("  't'     : Toggle show all rewards")
        print("  'i'     : Show reward info")
        print("  'q'     : Quit")
        print("  'h'     : Show this help")
        print("="*60)
        print(f"Current motor strength: {self.motor_strength*100:.0f}%")
        print(f"Primary reward function: {self.reward_function}")
        print(f"Show all rewards: {'ON' if self.show_all_rewards else 'OFF'}")
        print("="*60)
    
    def print_status(self, motor_angle, pendulum_angle, motor_speed=None, pendulum_speed=None):
        """Print current robot status"""
        # Clear screen (works on most terminals)
        os.system('clear' if os.name == 'posix' else 'cls')
        
        self.print_controls()
        
        print(f"\n📊 ROBOT STATUS:")
        print(f"  Motor angle:     {np.rad2deg(motor_angle):8.2f}° ({motor_angle:8.4f} rad)")
        print(f"  Pendulum angle:  {np.rad2deg(pendulum_angle):8.2f}° ({pendulum_angle:8.4f} rad)")
        
        if motor_speed is not None:
            print(f"  Motor speed:     {motor_speed/(2*np.pi):8.2f} rev/s ({motor_speed:8.4f} rad/s)")
        if pendulum_speed is not None:
            print(f"  Pendulum speed:  {pendulum_speed/(2*np.pi):8.2f} rev/s ({pendulum_speed:8.4f} rad/s)")
        
        print(f"  Motor command:   {self.motor_command:8.2f}")
        print(f"  Motor strength:  {self.motor_strength*100:8.0f}%")
        
        # Calculate and display all rewards
        if motor_speed is not None and pendulum_speed is not None:
            state = np.array([motor_angle, pendulum_angle, motor_speed, pendulum_speed])
            
            if self.show_all_rewards:
                print(f"\n🎯 ALL REWARD FUNCTIONS:")
                for reward_name, reward_func in REWARDS.items():
                    reward_value = reward_func(state)
                    # Color code based on reward value
                    if reward_value > 0.8:
                        color = "🟢"
                    elif reward_value > 0.6:
                        color = "🟡"
                    elif reward_value > 0.4:
                        color = "🟠"
                    else:
                        color = "🔴"
                    
                    # Highlight current primary reward
                    marker = "★" if reward_name == self.reward_function else " "
                    print(f"  {marker} {color} {reward_name:12}: {reward_value:8.4f}")
            else:
                # Show only primary reward
                reward = self.calculate_reward(motor_angle, pendulum_angle, motor_speed, pendulum_speed)
                print(f"  Reward ({self.reward_function}): {reward:8.4f}")
        
        # Show direction indicator
        if self.motor_command > 0:
            direction = "→ RIGHT"
        elif self.motor_command < 0:
            direction = "← LEFT"
        else:
            direction = "● STOP"
        print(f"  Direction:       {direction}")
        
        # Show reward interpretation for primary reward
        if motor_speed is not None and pendulum_speed is not None:
            reward = self.calculate_reward(motor_angle, pendulum_angle, motor_speed, pendulum_speed)
            if reward > 0.8:
                reward_status = "🟢 EXCELLENT"
            elif reward > 0.6:
                reward_status = "🟡 GOOD"
            elif reward > 0.4:
                reward_status = "🟠 FAIR"
            else:
                reward_status = "🔴 POOR"
            print(f"  Primary status:  {reward_status}")
    
    def print_reward_info(self):
        """Print information about all reward functions"""
        print("\n" + "="*60)
        print("📚 REWARD FUNCTION INFORMATION")
        print("="*60)
        print("cos_alpha:")
        print("  - Rewards pendulum pointing UP (α = 0°)")
        print("  - Uses cosine function: gentle penalty for deviation")
        print("  - Also considers motor angle (θ) position")
        print("  - Range: 0.0 (pendulum down) to 1.0 (pendulum up)")
        print()
        print("exp_alpha_2/3/4/6:")
        print("  - Same target: pendulum pointing UP (α = 0°)")
        print("  - Uses exponential penalty: harsher for deviations")
        print("  - Higher numbers = steeper penalty curve")
        print("  - exp_alpha_6 is very strict, exp_alpha_2 is moderate")
        print("  - Range: 0.0 (pendulum down) to 1.0 (pendulum up)")
        print()
        print("All rewards:")
        print("  - Multiply alpha reward × theta reward")
        print("  - Theta reward prefers motor at θ = 0°")
        print("  - Higher values = better performance")
        print("  - 🟢 > 0.8, 🟡 > 0.6, 🟠 > 0.4, 🔴 ≤ 0.4")
        print("="*60)
        input("Press Enter to continue...")
    
    def handle_key_input(self, key):
        """Handle keyboard input and update motor command"""
        key = key.lower()
        
        if key == 'q':
            self.running = False
            return "Quitting..."
            
        elif key == 'a':
            self.motor_command = -self.motor_strength
            return f"Turning LEFT at {self.motor_strength*100:.0f}% power"
            
        elif key == 'd':
            self.motor_command = self.motor_strength
            return f"Turning RIGHT at {self.motor_strength*100:.0f}% power"
            
        elif key == 's':
            self.motor_command = 0.0
            return "Stopping motor"
            
        elif key == 'r':
            self.robot.reset_encoders()
            return "Encoders reset!"
            
        elif key == 'h':
            return "Help displayed"
            
        elif key in '123456789':
            self.motor_strength = int(key) * 0.1
            return f"Motor strength set to {self.motor_strength*100:.0f}%"
            
        elif key == '0':
            self.motor_strength = 1.0
            return "Motor strength set to 100%"
            
        elif key == 't':
            self.show_all_rewards = not self.show_all_rewards
            return f"Show all rewards: {'ON' if self.show_all_rewards else 'OFF'}"
            
        elif key == 'i':
            self.print_reward_info()
            return "Reward info displayed"
            
        else:
            return f"Unknown key: '{key}'"
    
    def run_interactive_control(self):
        """Main interactive control loop"""
        print("🎮 Starting interactive control...")
        print("Press keys to control the robot (press 'h' for help)")
        
        # Store previous values for speed calculation
        prev_motor_angle = None
        prev_pendulum_angle = None
        prev_timestamp = None
        
        try:
            import select
            import termios
            
            # Set up non-blocking input
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            new_settings = termios.tcgetattr(fd)
            new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
            termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
            
            while self.running:
                # Get robot state
                motor_angle, pendulum_angle, timestamp = self.robot.step(self.motor_command)
                
                # Calculate speeds
                motor_speed = 0.0
                pendulum_speed = 0.0
                
                if prev_timestamp is not None:
                    dt = timestamp - prev_timestamp
                    if dt > 0:
                        motor_speed = (motor_angle - prev_motor_angle) / dt
                        pendulum_speed = (pendulum_angle - prev_pendulum_angle) / dt
                
                # Update previous values
                prev_motor_angle = motor_angle
                prev_pendulum_angle = pendulum_angle
                prev_timestamp = timestamp
                
                # Print status
                self.print_status(motor_angle, pendulum_angle, motor_speed, pendulum_speed)
                
                # Check for keyboard input (non-blocking)
                if select.select([sys.stdin], [], [], 1.0/self.control_freq)[0]:
                    key = sys.stdin.read(1)
                    message = self.handle_key_input(key)
                    print(f"\n💬 {message}")
                    time.sleep(0.5)  # Brief pause to show message
                
                # Control frequency
                time.sleep(1.0 / self.control_freq)
                
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error during control: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            # Cleanup
            print("\n🧹 Cleaning up...")
            if self.robot:
                self.robot.step(0.0)  # Stop motor
                self.robot.close()
            print("✓ Robot stopped and closed")
    
    def run(self):
        """Main run method"""
        if not self.initialize_robot():
            return
        
        try:
            self.run_interactive_control()
        except Exception as e:
            print(f"❌ Fatal error: {e}")
        finally:
            if self.robot:
                self.robot.close()


def main():
    """Main function"""
    print("🎮 Interactive Polimi Robot Control")
    print("This script allows you to control the Polimi robot with keyboard inputs.")
    print("Make sure the robot is properly connected and powered on.")
    
    # Safety warning
    print("\n⚠️  SAFETY WARNING:")
    print("- Keep hands away from moving parts")
    print("- Be ready to press 's' to stop the motor")
    print("- Start with low power (press '1' for 10% power)")
    
    response = input("\nPress Enter to continue or 'q' to quit: ")
    if response.lower() == 'q':
        print("Exiting...")
        return
    
    # Create and run the interactive controller
    controller = InteractivePolimiControl()
    controller.run()


if __name__ == "__main__":
    main() 