import lgpio
import spidev
import time
import numpy as np
from furuta.robot import Robot 

class PolimiRobot(Robot):
    def __init__(self, 
                 motor_encoder_cpr=48,
                 pendulum_encoder_cpr=5120 * 4, #might be wrong
                 pwm_freq=10000):
        # Motor pins
        self.IN1_PIN = 25
        self.IN2_PIN = 24
        self.D2_PIN = 12
        self.EN_PIN = 6
        self.SF_PIN = 2
        
        # Store encoder parameters
        self.motor_encoder_cpr = motor_encoder_cpr
        self.pendulum_encoder_cpr = pendulum_encoder_cpr
        
        # Initialize GPIO and get pwm handle
        self._init_gpio()
        print("Self.h: ", self.h)
        
        # Setup GPIO pins
        self._setup_gpio()
        
        # Setup encoders
        self._setup_encoders()
        
        # Store initial encoder values for relative angle calculation
        self.motor_initial_count = self._read_encoder(self.motor_spi)
        self.pendulum_initial_count = self._read_encoder(self.pendulum_spi)
        
        # Initialize PWM
        self.pwm_freq = pwm_freq
        self.pwm_handle = lgpio.tx_pwm(self.h, self.D2_PIN, pwm_freq, 0)
        
        # Enable motor driver
        lgpio.gpio_write(self.h, self.EN_PIN, 1)
        
    def _init_gpio(self):
        """Initialize GPIO with proper error handling"""
        try:
            self.h = lgpio.gpiochip_open(0)
            if self.h < 0:
                raise RuntimeError("Failed to open GPIO chip")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize GPIO: {e}")
        
    def _setup_gpio(self):
        """Setup individual GPIO pins"""
        try:
            # Setup output pins
            lgpio.gpio_claim_output(self.h, self.IN1_PIN)
            lgpio.gpio_claim_output(self.h, self.IN2_PIN)
            lgpio.gpio_claim_output(self.h, self.D2_PIN)
            lgpio.gpio_claim_output(self.h, self.EN_PIN)
            
            # Setup input pin
            lgpio.gpio_claim_input(self.h, self.SF_PIN)
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
        raw = (resp[1] << 24) | (resp[2] << 16) | (resp[3] << 8) | resp[4]
        # Convert to signed 32-bit integer
        if raw & 0x80000000:
            raw = raw - 0x100000000
        return raw
        
    def step(self, motor_command: float):
        # Check for faults
        if lgpio.gpio_read(self.h, self.SF_PIN) == 0:
            raise RuntimeError("Motor driver fault detected!")
            
        # Convert motor command (-1 to 1) to PWM and direction
        duty_cycle = min(abs(motor_command) * 100, 100)
        direction = motor_command < 0
        
        # Set direction
        lgpio.gpio_write(self.h, self.IN1_PIN, 1 if direction else 0)
        lgpio.gpio_write(self.h, self.IN2_PIN, 0 if direction else 1)
        
        # Set PWM duty cycle
        lgpio.tx_pwm(self.h, self.D2_PIN, self.pwm_freq, duty_cycle)
        
        # Read encoders
        motor_count = self._read_encoder(self.motor_spi)
        pendulum_count = self._read_encoder(self.pendulum_spi)
        
        # Calculate relative counts
        rel_motor_count = motor_count - self.motor_initial_count
        rel_pendulum_count = pendulum_count - self.pendulum_initial_count
        
        # Convert counts to angles
        motor_angle = 2 * np.pi * rel_motor_count / self.motor_encoder_cpr
        pendulum_angle = 2 * np.pi * rel_pendulum_count / self.pendulum_encoder_cpr
        
        return motor_angle, pendulum_angle, time.time()
        
    def reset_encoders(self):
        self.motor_spi.xfer2([0x20])  # CLR_CNTR
        self.pendulum_spi.xfer2([0x20])  # CLR_CNTR
        # After clearing, update initial counts to new zeroed values
        self.motor_initial_count = self._read_encoder(self.motor_spi)
        self.pendulum_initial_count = self._read_encoder(self.pendulum_spi)
        
    def close(self):
        # Stop PWM
        lgpio.tx_pwm(self.h, self.D2_PIN, self.pwm_freq, 0)
        
        # Disable motor driver
        lgpio.gpio_write(self.h, self.EN_PIN, 0)
        
        # Close GPIO
        lgpio.gpiochip_close(self.h)
        
        # Close SPI connections
        self.motor_spi.close()
        self.pendulum_spi.close()