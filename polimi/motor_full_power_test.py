#!/usr/bin/env python3
"""
Fast Motor Control Test Script - Direct Digital Control

Real-time keyboard control of the motor with ZERO lag.
Uses direct digital control (no PWM) for instant response.

Controls:
- 'a': Turn left (full power)
- 'd': Turn right (full power)  
- 's': Stop motor
- 'q': Quit
- '1-9': Set power level (1=10%, 9=90%) - using rapid switching
- '0': Set power to 100%

Press keys to control the motor in real-time!
"""

import RPi.GPIO as GPIO
import time
import sys
import select
import termios
import tty
import threading

# Pin definitions (BCM numbering)
IN1_PIN = 25    # GPIO25 for direction control
IN2_PIN = 24    # GPIO24 for direction control
D2_PIN = 12     # GPIO12 for PWM control (now used for power control)
EN_PIN = 6      # GPIO6 for Enable
SF_PIN = 2      # GPIO2 for SF

class FastMotorControl:
    def __init__(self):
        self.motor_command = 0.0
        self.motor_power = 1.0  # Default 100% power
        self.running = True
        self.control_freq = 1000  # Hz - very fast updates for power control
        
        # Power control variables
        self.power_on_time = 0.001  # 1ms on time for power control
        self.power_off_time = 0.001  # 1ms off time for power control
        self.power_thread_running = False
        
        # Setup GPIO
        self._setup_gpio()
        
    def _setup_gpio(self):
        """Setup GPIO with proper error handling"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Initialize pins
            GPIO.setup(IN1_PIN, GPIO.OUT)
            GPIO.setup(IN2_PIN, GPIO.OUT)
            GPIO.setup(D2_PIN, GPIO.OUT)
            GPIO.setup(EN_PIN, GPIO.OUT)
            GPIO.setup(SF_PIN, GPIO.IN)
            
            # Set all outputs to LOW initially
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.LOW)
            GPIO.output(D2_PIN, GPIO.LOW)
            
            # Enable motor driver
            GPIO.output(EN_PIN, GPIO.HIGH)
            
            print("✓ GPIO initialized successfully (Direct Digital Control)")
            
        except Exception as e:
            print(f"❌ Failed to setup GPIO: {e}")
            raise
    
    def check_fault(self):
        """Check for motor driver faults"""
        if GPIO.input(SF_PIN) == GPIO.LOW:
            print("⚠️  Motor driver fault detected!")
            return True
        return False
    
    def set_motor_direct(self, direction, power):
        """Set motor direction and power using direct digital control"""
        # Set direction immediately
        if direction > 0:  # Forward
            GPIO.output(IN1_PIN, GPIO.HIGH)
            GPIO.output(IN2_PIN, GPIO.LOW)
        elif direction < 0:  # Backward
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.HIGH)
        else:  # Stop
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.LOW)
            GPIO.output(D2_PIN, GPIO.LOW)
            return
        
        # For power control, use rapid switching on D2 pin
        if power >= 1.0:
            # Full power - keep D2 HIGH
            GPIO.output(D2_PIN, GPIO.HIGH)
        elif power <= 0.0:
            # No power - keep D2 LOW
            GPIO.output(D2_PIN, GPIO.LOW)
        else:
            # Variable power - start power control thread
            self.start_power_control(power)
    
    def start_power_control(self, power):
        """Start power control thread for variable power levels"""
        if not self.power_thread_running:
            self.power_thread_running = True
            self.target_power = power
            threading.Thread(target=self._power_control_loop, daemon=True).start()
    
    def _power_control_loop(self):
        """Power control loop using rapid switching"""
        while self.power_thread_running and self.running:
            # Calculate on/off times based on power level
            cycle_time = self.power_on_time + self.power_off_time
            on_ratio = self.target_power
            
            # Turn on for power ratio of the cycle
            GPIO.output(D2_PIN, GPIO.HIGH)
            time.sleep(self.power_on_time * on_ratio)
            
            # Turn off for the rest of the cycle
            GPIO.output(D2_PIN, GPIO.LOW)
            time.sleep(self.power_on_time * (1 - on_ratio))
    
    def stop_power_control(self):
        """Stop the power control thread"""
        self.power_thread_running = False
    
    def set_motor(self, direction, power):
        """Main motor control function"""
        # Stop any existing power control
        self.stop_power_control()
        
        # Set motor using direct control
        self.set_motor_direct(direction, power)
    
    def print_status(self):
        """Print current motor status"""
        # Clear screen (works on most terminals)
        import os
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("🚀 FAST MOTOR CONTROL - DIRECT DIGITAL")
        print("=" * 45)
        print("Controls:")
        print("  'a'     : Turn LEFT (full power)")
        print("  'd'     : Turn RIGHT (full power)")
        print("  's'     : STOP motor")
        print("  '1-9'   : Set power level (1=10%, 9=90%)")
        print("  '0'     : Set power to 100%")
        print("  'q'     : Quit")
        print("=" * 45)
        
        # Show current state
        if self.motor_command > 0:
            direction = "→ RIGHT"
        elif self.motor_command < 0:
            direction = "← LEFT"
        else:
            direction = "● STOP"
        
        print(f"Motor: {direction}")
        print(f"Power: {self.motor_power*100:.0f}%")
        print(f"Command: {self.motor_command:.2f}")
        print(f"Control: Direct Digital (no PWM)")
        
        # Check for faults
        if self.check_fault():
            print("⚠️  DRIVER FAULT!")
        else:
            print("✓ Driver OK")
        
        print("=" * 45)
        print("Press keys to control (instant response)...")
    
    def handle_key(self, key):
        """Handle keyboard input with immediate response"""
        key = key.lower()
        
        if key == 'q':
            self.running = False
            return "Quitting..."
            
        elif key == 'a':
            self.motor_command = -self.motor_power
            return f"LEFT at {self.motor_power*100:.0f}%"
            
        elif key == 'd':
            self.motor_command = self.motor_power
            return f"RIGHT at {self.motor_power*100:.0f}%"
            
        elif key == 's':
            self.motor_command = 0.0
            return "STOP"
            
        elif key in '123456789':
            self.motor_power = int(key) * 0.1
            return f"Power: {self.motor_power*100:.0f}%"
            
        elif key == '0':
            self.motor_power = 1.0
            return "Power: 100%"
            
        return None
    
    def run(self):
        """Main control loop with minimal latency"""
        print("🚀 Starting fast motor control (Direct Digital)...")
        print("Press keys to control the motor (instant response)")
        print("Press 'q' to quit")
        
        # Set up non-blocking input
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        new_settings = termios.tcgetattr(fd)
        new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
        
        try:
            while self.running:
                # Update motor immediately
                self.set_motor(self.motor_command, self.motor_power)
                
                # Print status
                self.print_status()
                
                # Check for keyboard input (non-blocking, very fast)
                if select.select([sys.stdin], [], [], 0.001)[0]:  # 1ms timeout
                    key = sys.stdin.read(1)
                    message = self.handle_key(key)
                    if message:
                        print(f"💬 {message}")
                        time.sleep(0.05)  # Brief pause to show message
                
                # Very fast control loop
                time.sleep(0.001)  # 1ms = 1000Hz update rate
                
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            # Cleanup
            print("\n🧹 Cleaning up...")
            self.stop_power_control()
            self.set_motor(0, 0)  # Stop motor
            GPIO.output(EN_PIN, GPIO.LOW)
            GPIO.cleanup()
            print("✓ Motor stopped and GPIO cleaned up")

def main():
    """Main function with safety warnings"""
    print("🚀 Fast Motor Control Test - Direct Digital")
    print("This script provides instant motor control with ZERO PWM lag.")
    print("Make sure the motor is properly connected and powered.")
    
    # Safety warning
    print("\n⚠️  SAFETY WARNING:")
    print("- Keep hands away from moving parts")
    print("- Be ready to press 's' to stop the motor")
    print("- Start with low power (press '1' for 10% power)")
    print("- Direct digital control provides instant response")
    
    response = input("\nPress Enter to continue or 'q' to quit: ")
    if response.lower() == 'q':
        print("Exiting...")
        return
    
    # Create and run the motor controller
    controller = FastMotorControl()
    controller.run()

if __name__ == "__main__":
    main()