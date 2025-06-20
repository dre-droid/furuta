import RPi.GPIO as GPIO
import spidev
import time
import numpy as np
from furuta.robot import Robot

class PolimiRobot(Robot):
    def __init__(self, 
                 motor_encoder_cpr=48,
                 pendulum_encoder_cpr=2048, #might be wrong
                 pwm_freq=15000):
        # Motor pins
        self.IN1_PIN = 25
        self.IN2_PIN = 24
        self.D2_PIN = 12
        self.EN_PIN = 6
        self.SF_PIN = 2
        
        # Initialize GPIO with error handling
        self._init_gpio()
        
        # Setup GPIO pins
        self._setup_gpio()
        
        # Setup encoders
        self._setup_encoders()
        
        # Initialize PWM
        self.pwm = GPIO.PWM(self.D2_PIN, pwm_freq)
        self.pwm.start(0)
        
        # Enable motor driver
        GPIO.output(self.EN_PIN, GPIO.HIGH)
        
        # Call parent constructor after GPIO setup
        super().__init__(None, motor_encoder_cpr, pendulum_encoder_cpr)  # No serial device needed
        
    def _init_gpio(self):
        """Initialize GPIO with proper error handling"""
        try:
            # Clean up any existing GPIO state
            GPIO.cleanup()
        except:
            pass  # Ignore cleanup errors
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize GPIO: {e}")
        
    def _setup_gpio(self):
        """Setup individual GPIO pins"""
        try:
            GPIO.setup(self.IN1_PIN, GPIO.OUT)
            GPIO.setup(self.IN2_PIN, GPIO.OUT)
            GPIO.setup(self.D2_PIN, GPIO.OUT)
            GPIO.setup(self.EN_PIN, GPIO.OUT)
            GPIO.setup(self.SF_PIN, GPIO.IN)
        except Exception as e:
            raise RuntimeError(f"Failed to setup GPIO pins: {e}")
        
    def _setup_encoders(self):
        # Setup motor encoder
        self.motor_spi = spidev.SpiDev()
        self.motor_spi.open(1, 0)
        self.motor_spi.max_speed_hz = 1000000
        self.motor_spi.mode = 0b00
        self._setup_ls7366r(self.motor_spi)
        
        # Setup pendulum encoder
        self.pendulum_spi = spidev.SpiDev()
        self.pendulum_spi.open(0, 0)
        self.pendulum_spi.max_speed_hz = 1000000
        self.pendulum_spi.mode = 0b00
        self._setup_ls7366r(self.pendulum_spi)
        
    def _setup_ls7366r(self, spi):
        # Use the same configuration as the working scripts
        # For motor encoder (bus 1): MDR0_CONF = 0b00000001 (4X quadrature)
        # For pendulum encoder (bus 0): MDR0_CONF = 0b00000011 (4X quadrature, free-running)
        if spi == self.motor_spi:
            # Motor encoder configuration
            spi.xfer2([0x88, 0b00000001])  # WR_MDR0: 4X quadrature
        else:
            # Pendulum encoder configuration  
            spi.xfer2([0x88, 0b00000011])  # WR_MDR0: 4X quadrature, free-running
        spi.xfer2([0x90, 0b00000000])  # WR_MDR1: 4-byte counter
        spi.xfer2([0x20])  # CLR_CNTR
        
    def _read_encoder(self, spi):
        resp = spi.xfer2([0x60, 0x00, 0x00, 0x00, 0x00])
        return (resp[1] << 24) | (resp[2] << 16) | (resp[3] << 8) | resp[4]
        
    def step(self, motor_command: float):
        # Check for faults
        if GPIO.input(self.SF_PIN) == GPIO.LOW:
            raise RuntimeError("Motor driver fault detected!")
            
        # Convert motor command (-1 to 1) to PWM and direction
        duty_cycle = min(abs(motor_command) * 100, 100)
        direction = motor_command < 0
        
        # Set direction
        GPIO.output(self.IN1_PIN, GPIO.HIGH if direction else GPIO.LOW)
        GPIO.output(self.IN2_PIN, GPIO.LOW if direction else GPIO.HIGH)
        
        # Set PWM
        self.pwm.ChangeDutyCycle(duty_cycle)
        
        # Read encoders
        motor_count = self._read_encoder(self.motor_spi)
        pendulum_count = self._read_encoder(self.pendulum_spi)
        
        # Convert counts to angles
        motor_angle = 2 * np.pi * motor_count / self.motor_encoder_cpr
        pendulum_angle = 2 * np.pi * pendulum_count / self.pendulum_encoder_cpr
        
        return motor_angle, pendulum_angle, time.time()
        
    def reset_encoders(self):
        self.motor_spi.xfer2([0x20])  # CLR_CNTR
        self.pendulum_spi.xfer2([0x20])  # CLR_CNTR
        
    def close(self):
        self.pwm.stop()
        GPIO.output(self.EN_PIN, GPIO.LOW)
        GPIO.cleanup()
        self.motor_spi.close()
        self.pendulum_spi.close()