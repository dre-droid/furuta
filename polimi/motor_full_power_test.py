#!/usr/bin/env python3
"""
Fast Motor Control Test Script

Real-time keyboard control of the motor with minimal lag.
Use WASD keys for immediate motor control.

Controls:
- 'a': Turn left (full power)
- 'd': Turn right (full power)  
- 's': Stop motor
- 'q': Quit
- '1-9': Set power level (1=10%, 9=90%)
- '0': Set power to 100%

Press keys to control the motor in real-time!
"""

import RPi.GPIO as GPIO
import time
import sys
import select
import termios
import tty

# Pin definitions (BCM numbering)
IN1_PIN = 25    # GPIO25 for direction control
IN2_PIN = 24    # GPIO24 for direction control
D2_PIN = 12     # GPIO12 for PWM control (active low)
EN_PIN = 6      # GPIO6 for Enable
SF_PIN = 2      # GPIO2 for SF

class FastMotorControl:
    def __init__(self):
        self.motor_command = 0.0
        self.motor_power = 1.0  # Default 100% power
        self.running = True
        self.control_freq = 100  # Hz - very fast updates
        
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
            
            # Initialize PWM at 1kHz for better low-end torque
            self.pwm = GPIO.PWM(D2_PIN, 1000)  # 1 kHz frequency
            self.pwm.start(0)
            
            # Enable motor driver
            GPIO.output(EN_PIN, GPIO.HIGH)
            
            print("✓ GPIO and PWM initialized successfully")
            
        except Exception as e:
            print(f"❌ Failed to setup GPIO: {e}")
            raise
    
    def check_fault(self):
        """Check for motor driver faults"""
        if GPIO.input(SF_PIN) == GPIO.LOW:
            print("⚠️  Motor driver fault detected!")
            return True
        return False
    
    def set_motor(self, direction, power):
        """Set motor direction and power with minimal latency"""
        # Convert power to duty cycle (0-100)
        duty_cycle = min(abs(power) * 100, 100)
        
        # Set direction
        if direction > 0:  # Forward
            GPIO.output(IN1_PIN, GPIO.HIGH)
            GPIO.output(IN2_PIN, GPIO.LOW)
        elif direction < 0:  # Backward
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.HIGH)
        else:  # Stop
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.LOW)
            duty_cycle = 0
        
        # Set PWM duty cycle
        self.pwm.ChangeDutyCycle(duty_cycle)
    
    def print_status(self):
        """Print current motor status"""
        # Clear screen (works on most terminals)
        import os
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("🚀 FAST MOTOR CONTROL TEST")
        print("=" * 40)
        print("Controls:")
        print("  'a'     : Turn LEFT (full power)")
        print("  'd'     : Turn RIGHT (full power)")
        print("  's'     : STOP motor")
        print("  '1-9'   : Set power level (1=10%, 9=90%)")
        print("  '0'     : Set power to 100%")
        print("  'q'     : Quit")
        print("=" * 40)
        
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
        
        # Check for faults
        if self.check_fault():
            print("⚠️  DRIVER FAULT!")
        else:
            print("✓ Driver OK")
        
        print("=" * 40)
        print("Press keys to control (no Enter needed)...")
    
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
        print("🚀 Starting fast motor control...")
        print("Press keys to control the motor (no Enter needed)")
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
                if select.select([sys.stdin], [], [], 0.01)[0]:  # 10ms timeout
                    key = sys.stdin.read(1)
                    message = self.handle_key(key)
                    if message:
                        print(f"💬 {message}")
                        time.sleep(0.1)  # Brief pause to show message
                
                # Very fast control loop
                time.sleep(0.01)  # 10ms = 100Hz update rate
                
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            # Cleanup
            print("\n🧹 Cleaning up...")
            self.set_motor(0, 0)  # Stop motor
            self.pwm.stop()
            GPIO.output(EN_PIN, GPIO.LOW)
            GPIO.cleanup()
            print("✓ Motor stopped and GPIO cleaned up")

def main():
    """Main function with safety warnings"""
    print("🚀 Fast Motor Control Test")
    print("This script provides real-time motor control with minimal lag.")
    print("Make sure the motor is properly connected and powered.")
    
    # Safety warning
    print("\n⚠️  SAFETY WARNING:")
    print("- Keep hands away from moving parts")
    print("- Be ready to press 's' to stop the motor")
    print("- Start with low power (press '1' for 10% power)")
    
    response = input("\nPress Enter to continue or 'q' to quit: ")
    if response.lower() == 'q':
        print("Exiting...")
        return
    
    # Create and run the motor controller
    controller = FastMotorControl()
    controller.run()

if __name__ == "__main__":
    main()