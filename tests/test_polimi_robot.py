# tests/test_polimi_robot.py
import time
import numpy as np
from furuta.polimi_robot import PolimiRobot

def test_robot_interface():
    print("Initializing PolimiRobot...")
    robot = PolimiRobot()
    
    try:
        # Test 1: Encoder reading
        print("\nTest 1: Reading encoders")
        print("Reading initial encoder values...")
        motor_angle, pendulum_angle, timestamp = robot.step(0.0)
        print(f"Motor angle: {np.rad2deg(motor_angle):.2f}°")
        print(f"Pendulum angle: {np.rad2deg(pendulum_angle):.2f}°")
        
        # Test 2: Encoder reset
        print("\nTest 2: Resetting encoders")
        robot.reset_encoders()
        motor_angle, pendulum_angle, timestamp = robot.step(0.0)
        print(f"Motor angle after reset: {np.rad2deg(motor_angle):.2f}°")
        print(f"Pendulum angle after reset: {np.rad2deg(pendulum_angle):.2f}°")
        
        # Test 3: Motor control - gentle movement
        print("\nTest 3: Testing motor control with gentle movement")
        print("Moving motor forward at 20% power...")
        for _ in range(5):  # Run for 5 steps
            motor_angle, pendulum_angle, timestamp = robot.step(0.2)
            print(f"Motor angle: {np.rad2deg(motor_angle):.2f}°")
            time.sleep(0.1)
            
        print("\nMoving motor backward at 20% power...")
        for _ in range(5):  # Run for 5 steps
            motor_angle, pendulum_angle, timestamp = robot.step(-0.2)
            print(f"Motor angle: {np.rad2deg(motor_angle):.2f}°")
            time.sleep(0.1)
            
        # Test 4: Motor control - full power (with safety)
        print("\nTest 4: Testing motor control with full power")
        print("WARNING: This will move the motor at full power!")
        input("Press Enter to continue (or Ctrl+C to abort)...")
        
        print("Moving motor forward at full power...")
        for _ in range(3):  # Run for 3 steps
            motor_angle, pendulum_angle, timestamp = robot.step(1.0)
            print(f"Motor angle: {np.rad2deg(motor_angle):.2f}°")
            time.sleep(0.1)
            
        print("\nMoving motor backward at full power...")
        for _ in range(3):  # Run for 3 steps
            motor_angle, pendulum_angle, timestamp = robot.step(-1.0)
            print(f"Motor angle: {np.rad2deg(motor_angle):.2f}°")
            time.sleep(0.1)
            
        # Test 5: Stop motor
        print("\nTest 5: Stopping motor")
        robot.step(0.0)
        print("Motor should be stopped")
        
        # Test 6: Continuous monitoring
        print("\nTest 6: Continuous monitoring (5 seconds)")
        print("Press Ctrl+C to stop...")
        start_time = time.time()
        while time.time() - start_time < 5:
            motor_angle, pendulum_angle, timestamp = robot.step(0.0)
            print(f"\rMotor: {np.rad2deg(motor_angle):.2f}° | Pendulum: {np.rad2deg(pendulum_angle):.2f}°", end="")
            time.sleep(0.1)
        print("\nMonitoring complete")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nError during test: {str(e)}")
    finally:
        print("\nCleaning up...")
        robot.close()
        print("Test complete")

if __name__ == "__main__":
    test_robot_interface()