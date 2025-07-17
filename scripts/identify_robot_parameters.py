#!/usr/bin/env python3
"""
Robot Parameter Identification Script

This script runs various experiments to identify the physical parameters
of your Furuta pendulum robot for accurate simulation modeling.

Usage:
    python identify_robot_parameters.py --experiment [free_oscillation|step_response|friction|all]
"""

import argparse
import json
import time
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import logging
from dataclasses import dataclass
from scipy.optimize import curve_fit
from scipy.signal import find_peaks
import hydra
from omegaconf import DictConfig

# Import robot interfaces
from furuta.robot import Robot
from furuta.polimi_robot import PolimiRobot
from furuta.utils import VelocityFilter


@dataclass
class ExperimentData:
    """Container for experiment data"""
    timestamps: np.ndarray
    motor_angles: np.ndarray
    pendulum_angles: np.ndarray
    motor_commands: np.ndarray
    motor_velocities: Optional[np.ndarray] = None
    pendulum_velocities: Optional[np.ndarray] = None


class ParameterIdentifier:
    """Main class for robot parameter identification"""
    
    def __init__(self, robot, control_freq: float = 100.0):
        self.robot = robot
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        self.vel_filter = VelocityFilter(2, dt=self.dt)
        
        # Results storage
        self.results = {}
        self.experiment_data = {}
        
        # Safety limits
        self.max_motor_angle = np.deg2rad(180)  # 180 degrees
        self.max_pendulum_angle = np.deg2rad(90)  # 90 degrees
        self.max_motor_speed = 20.0  # rad/s
        self.max_command = 0.8  # 80% of max command
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
    def safety_check(self, motor_angle: float, pendulum_angle: float) -> bool:
        """Check if current state is within safety limits"""
        if abs(motor_angle) > self.max_motor_angle:
            self.logger.warning(f"Motor angle {np.rad2deg(motor_angle):.1f}° exceeds limit")
            return False
        if abs(pendulum_angle) > self.max_pendulum_angle:
            self.logger.warning(f"Pendulum angle {np.rad2deg(pendulum_angle):.1f}° exceeds limit")
            return False
        return True
    
    def collect_data(self, duration: float, command_func, description: str) -> ExperimentData:
        """Collect data for a given duration with a command function"""
        self.logger.info(f"Starting experiment: {description}")
        self.logger.info(f"Duration: {duration:.1f}s")
        
        # Initialize data collection
        timestamps = []
        motor_angles = []
        pendulum_angles = []
        motor_commands = []
        
        # Reset encoders and velocity filter
        self.robot.reset_encoders()
        self.vel_filter = VelocityFilter(2, dt=self.dt)
        
        start_time = time.time()
        step_count = 0
        
        try:
            while True:
                current_time = time.time()
                elapsed = current_time - start_time
                
                if elapsed >= duration:
                    break
                
                # Generate command
                command = command_func(elapsed, step_count)
                command = np.clip(command, -self.max_command, self.max_command)
                
                # Execute step
                motor_angle, pendulum_angle, timestamp = self.robot.step(command)
                
                # Safety check
                if not self.safety_check(motor_angle, pendulum_angle):
                    self.logger.error("Safety limit exceeded, stopping experiment")
                    break
                
                # Store data
                timestamps.append(elapsed)
                motor_angles.append(motor_angle)
                pendulum_angles.append(pendulum_angle)
                motor_commands.append(command)
                
                step_count += 1
                
                # Maintain control frequency
                time.sleep(max(0, self.dt - (time.time() - current_time)))
                
        except KeyboardInterrupt:
            self.logger.info("Experiment interrupted by user")
        finally:
            # Stop robot
            self.robot.step(0.0)
        
        # Convert to numpy arrays
        data = ExperimentData(
            timestamps=np.array(timestamps),
            motor_angles=np.array(motor_angles),
            pendulum_angles=np.array(pendulum_angles),
            motor_commands=np.array(motor_commands)
        )
        
        # Calculate velocities
        if len(data.timestamps) > 2:
            motor_positions = data.motor_angles.reshape(-1, 1)
            pendulum_positions = data.pendulum_angles.reshape(-1, 1)
            
            motor_vels = []
            pendulum_vels = []
            
            for i in range(len(data.timestamps)):
                if i < 2:
                    motor_vels.append(0.0)
                    pendulum_vels.append(0.0)
                else:
                    # Simple finite difference for velocity
                    dt = data.timestamps[i] - data.timestamps[i-1]
                    motor_vel = (data.motor_angles[i] - data.motor_angles[i-1]) / dt
                    pendulum_vel = (data.pendulum_angles[i] - data.pendulum_angles[i-1]) / dt
                    motor_vels.append(motor_vel)
                    pendulum_vels.append(pendulum_vel)
            
            data.motor_velocities = np.array(motor_vels)
            data.pendulum_velocities = np.array(pendulum_vels)
        
        self.logger.info(f"Collected {len(data.timestamps)} data points")
        return data
    
    def experiment_free_oscillation_pendulum(self) -> Dict:
        """Free oscillation experiment for pendulum damping identification"""
        self.logger.info("=== Pendulum Free Oscillation Experiment ===")
        
        def setup_command(t, step):
            # First, move motor to position and hold
            if t < 2.0:
                return 0.1 if step % 20 < 10 else -0.1  # Small oscillation to position motor
            else:
                return 0.0  # Hold motor still
        
        # Initial positioning
        self.logger.info("Positioning motor...")
        self.collect_data(3.0, setup_command, "Motor positioning")
        
        # Manual pendulum displacement
        input("\nPlease manually displace the pendulum to about 30-45 degrees and release it when ready.")
        input("Press Enter to start recording the free oscillation...")
        
        def hold_motor(t, step):
            return 0.0  # Keep motor stationary
        
        # Record free oscillation
        data = self.collect_data(15.0, hold_motor, "Pendulum free oscillation")
        self.experiment_data['pendulum_oscillation'] = data
        
        # Analyze oscillation
        results = self._analyze_pendulum_oscillation(data)
        self.results['pendulum_damping'] = results
        return results
    
    def experiment_motor_step_response(self) -> Dict:
        """Step response experiment for motor parameters"""
        self.logger.info("=== Motor Step Response Experiment ===")
        
        def step_command(t, step):
            if t < 1.0:
                return 0.0  # Initial rest
            elif t < 3.0:
                return 0.3  # Step command
            elif t < 4.0:
                return 0.0  # Return to zero
            elif t < 6.0:
                return -0.3  # Negative step
            else:
                return 0.0  # Final rest
        
        data = self.collect_data(8.0, step_command, "Motor step response")
        self.experiment_data['motor_step'] = data
        
        # Analyze step response
        results = self._analyze_motor_step_response(data)
        self.results['motor_parameters'] = results
        return results
    
    def experiment_friction_identification(self) -> Dict:
        """Friction identification using slow ramp commands"""
        self.logger.info("=== Friction Identification Experiment ===")
        
        def ramp_command(t, step):
            if t < 2.0:
                return 0.0
            elif t < 10.0:
                # Slow ramp up
                return 0.05 * (t - 2.0) / 8.0
            elif t < 12.0:
                return 0.0
            elif t < 20.0:
                # Slow ramp down (negative)
                return -0.05 * (t - 12.0) / 8.0
            else:
                return 0.0
        
        data = self.collect_data(22.0, ramp_command, "Friction identification")
        self.experiment_data['friction'] = data
        
        # Analyze friction
        results = self._analyze_friction(data)
        self.results['friction'] = results
        return results
    
    def experiment_pendulum_properties(self) -> Dict:
        """Estimate pendulum mass and length from natural frequency"""
        self.logger.info("=== Pendulum Properties Experiment ===")
        
        # Small amplitude oscillations to measure natural frequency
        def small_oscillation_command(t, step):
            if t < 1.0:
                return 0.0
            else:
                # Very gentle sinusoidal excitation
                return 0.02 * np.sin(2 * np.pi * 0.5 * t)
        
        input("\nPosition pendulum near vertical and press Enter to start measurement...")
        
        data = self.collect_data(20.0, small_oscillation_command, "Pendulum natural frequency")
        self.experiment_data['pendulum_properties'] = data
        
        # Analyze natural frequency
        results = self._analyze_pendulum_properties(data)
        self.results['pendulum_properties'] = results
        return results
    
    def _analyze_pendulum_oscillation(self, data: ExperimentData) -> Dict:
        """Analyze free oscillation to extract damping coefficient"""
        angles = data.pendulum_angles
        times = data.timestamps
        
        # Find peaks for amplitude decay analysis
        peaks, _ = find_peaks(np.abs(angles), height=np.deg2rad(5))
        
        if len(peaks) < 3:
            self.logger.warning("Not enough oscillation peaks found")
            return {"damping_ratio": None, "natural_frequency": None}
        
        # Calculate natural frequency
        peak_times = times[peaks[:5]]  # Use first 5 peaks
        if len(peak_times) > 1:
            periods = np.diff(peak_times) * 2  # Half periods to full periods
            natural_freq = 1.0 / np.mean(periods)
        else:
            natural_freq = None
        
        # Fit exponential decay to peak amplitudes
        peak_amplitudes = np.abs(angles[peaks])
        peak_times = times[peaks]
        
        def exp_decay(t, A, damping):
            return A * np.exp(-damping * t)
        
        try:
            popt, _ = curve_fit(exp_decay, peak_times, peak_amplitudes)
            damping_coeff = popt[1]
            
            # Calculate damping ratio
            if natural_freq:
                damping_ratio = damping_coeff / (2 * 2 * np.pi * natural_freq)
            else:
                damping_ratio = None
                
        except Exception as e:
            self.logger.warning(f"Could not fit damping: {e}")
            damping_coeff = None
            damping_ratio = None
        
        return {
            "natural_frequency": natural_freq,
            "damping_coefficient": damping_coeff,
            "damping_ratio": damping_ratio,
            "peak_count": len(peaks)
        }
    
    def _analyze_motor_step_response(self, data: ExperimentData) -> Dict:
        """Analyze motor step response for motor parameters"""
        # Find step transitions
        command_diff = np.diff(data.motor_commands)
        step_indices = np.where(np.abs(command_diff) > 0.1)[0]
        
        results = {}
        
        for i, step_idx in enumerate(step_indices[:2]):  # Analyze first two steps
            # Get data for this step response
            start_idx = step_idx
            end_idx = min(step_idx + int(2.0 / self.dt), len(data.timestamps))
            
            step_times = data.timestamps[start_idx:end_idx] - data.timestamps[start_idx]
            step_angles = data.motor_angles[start_idx:end_idx]
            step_command = data.motor_commands[start_idx]
            
            # Basic analysis
            if len(step_angles) > 10:
                final_angle = np.mean(step_angles[-10:])  # Average of last 10 points
                initial_angle = step_angles[0]
                
                # Calculate steady-state gain
                gain = (final_angle - initial_angle) / step_command if step_command != 0 else 0
                
                # Find settling time (within 5% of final value)
                target_range = 0.05 * abs(final_angle - initial_angle)
                settled_mask = np.abs(step_angles - final_angle) < target_range
                if np.any(settled_mask):
                    settling_time = step_times[np.where(settled_mask)[0][0]]
                else:
                    settling_time = None
                
                results[f'step_{i}'] = {
                    'gain': gain,
                    'settling_time': settling_time,
                    'final_angle': final_angle,
                    'command': step_command
                }
        
        return results
    
    def _analyze_friction(self, data: ExperimentData) -> Dict:
        """Analyze friction from slow ramp experiments"""
        # Find where motor starts moving vs command level
        if data.motor_velocities is None:
            return {"static_friction": None, "kinetic_friction": None}
        
        # Find breakaway points
        moving_threshold = 0.1  # rad/s
        is_moving = np.abs(data.motor_velocities) > moving_threshold
        
        # Find static friction (command level when motion starts)
        command_levels = []
        for direction in [1, -1]:
            dir_mask = np.sign(data.motor_commands) == direction
            dir_commands = data.motor_commands[dir_mask]
            dir_moving = is_moving[dir_mask]
            
            if len(dir_commands) > 0 and np.any(dir_moving):
                first_moving_idx = np.where(dir_moving)[0][0]
                static_friction_command = abs(dir_commands[first_moving_idx])
                command_levels.append(static_friction_command)
        
        static_friction = np.mean(command_levels) if command_levels else None
        
        # Estimate kinetic friction from steady-state velocity vs command
        steady_state_mask = np.abs(data.motor_velocities) > moving_threshold
        if np.any(steady_state_mask):
            steady_commands = data.motor_commands[steady_state_mask]
            steady_velocities = data.motor_velocities[steady_state_mask]
            
            # Simple linear fit: command = kinetic_friction + k * velocity
            if len(steady_commands) > 10:
                kinetic_friction = np.mean(np.abs(steady_commands) - 0.1 * np.abs(steady_velocities))
            else:
                kinetic_friction = None
        else:
            kinetic_friction = None
        
        return {
            "static_friction": static_friction,
            "kinetic_friction": kinetic_friction,
            "breakaway_points": len(command_levels)
        }
    
    def _analyze_pendulum_properties(self, data: ExperimentData) -> Dict:
        """Analyze pendulum properties from natural frequency"""
        # Use FFT to find dominant frequency
        if len(data.pendulum_angles) < 100:
            return {"natural_frequency": None}
        
        # Remove DC component
        angles = data.pendulum_angles - np.mean(data.pendulum_angles)
        
        # FFT
        fft = np.fft.fft(angles)
        freqs = np.fft.fftfreq(len(angles), d=self.dt)
        
        # Find peak frequency
        power = np.abs(fft)
        positive_freq_mask = freqs > 0
        pos_freqs = freqs[positive_freq_mask]
        pos_power = power[positive_freq_mask]
        
        # Find dominant frequency
        peak_idx = np.argmax(pos_power)
        natural_freq = pos_freqs[peak_idx]
        
        # Estimate pendulum length from natural frequency
        # For small angles: omega = sqrt(g/L_eff)
        g = 9.81
        if natural_freq > 0:
            omega = 2 * np.pi * natural_freq
            estimated_length = g / (omega ** 2)
        else:
            estimated_length = None
        
        return {
            "natural_frequency": natural_freq,
            "estimated_length": estimated_length,
            "frequency_peak_power": pos_power[peak_idx]
        }
    
    def plot_results(self, save_path: Optional[str] = None):
        """Plot all collected data and analysis results"""
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('Robot Parameter Identification Results', fontsize=16)
        
        # Plot pendulum oscillation
        if 'pendulum_oscillation' in self.experiment_data:
            data = self.experiment_data['pendulum_oscillation']
            ax = axes[0, 0]
            ax.plot(data.timestamps, np.rad2deg(data.pendulum_angles))
            ax.set_title('Pendulum Free Oscillation')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Pendulum Angle (deg)')
            ax.grid(True)
        
        # Plot motor step response
        if 'motor_step' in self.experiment_data:
            data = self.experiment_data['motor_step']
            ax = axes[0, 1]
            ax.plot(data.timestamps, np.rad2deg(data.motor_angles), label='Angle')
            ax2 = ax.twinx()
            ax2.plot(data.timestamps, data.motor_commands, 'r--', label='Command')
            ax.set_title('Motor Step Response')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Motor Angle (deg)')
            ax2.set_ylabel('Command')
            ax.legend()
            ax2.legend()
            ax.grid(True)
        
        # Plot friction test
        if 'friction' in self.experiment_data:
            data = self.experiment_data['friction']
            ax = axes[1, 0]
            if data.motor_velocities is not None:
                ax.scatter(data.motor_commands, data.motor_velocities, alpha=0.6)
                ax.set_title('Friction: Command vs Velocity')
                ax.set_xlabel('Motor Command')
                ax.set_ylabel('Motor Velocity (rad/s)')
                ax.grid(True)
        
        # Plot pendulum frequency analysis
        if 'pendulum_properties' in self.experiment_data:
            data = self.experiment_data['pendulum_properties']
            ax = axes[1, 1]
            
            # Time domain
            ax.plot(data.timestamps, np.rad2deg(data.pendulum_angles))
            ax.set_title('Pendulum Natural Frequency Test')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Pendulum Angle (deg)')
            ax.grid(True)
        
        # Summary results
        ax = axes[2, 0]
        ax.axis('off')
        
        # Create results summary text
        summary_text = "Parameter Identification Results:\n\n"
        
        if 'pendulum_damping' in self.results:
            r = self.results['pendulum_damping']
            summary_text += f"Pendulum Damping:\n"
            summary_text += f"  Natural freq: {r.get('natural_frequency', 'N/A'):.3f} Hz\n"
            summary_text += f"  Damping coeff: {r.get('damping_coefficient', 'N/A'):.6f}\n"
            summary_text += f"  Damping ratio: {r.get('damping_ratio', 'N/A'):.4f}\n\n"
        
        if 'friction' in self.results:
            r = self.results['friction']
            summary_text += f"Friction:\n"
            summary_text += f"  Static: {r.get('static_friction', 'N/A'):.3f}\n"
            summary_text += f"  Kinetic: {r.get('kinetic_friction', 'N/A'):.3f}\n\n"
        
        if 'pendulum_properties' in self.results:
            r = self.results['pendulum_properties']
            summary_text += f"Pendulum Properties:\n"
            summary_text += f"  Natural freq: {r.get('natural_frequency', 'N/A'):.3f} Hz\n"
            summary_text += f"  Est. length: {r.get('estimated_length', 'N/A'):.3f} m\n"
        
        ax.text(0.1, 0.9, summary_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace')
        
        # Recommended parameters
        ax = axes[2, 1]
        ax.axis('off')
        
        # Generate recommended parameter values
        recommendations = self.generate_parameter_recommendations()
        rec_text = "Recommended Parameters:\n\n"
        for param, value in recommendations.items():
            if value is not None:
                rec_text += f"{param}: {value:.6f}\n"
            else:
                rec_text += f"{param}: Manual measurement needed\n"
        
        ax.text(0.1, 0.9, rec_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            self.logger.info(f"Plot saved to {save_path}")
        
        plt.show()
    
    def generate_parameter_recommendations(self) -> Dict:
        """Generate recommended parameter values based on identification results"""
        recommendations = {}
        
        # Damping parameters
        if 'pendulum_damping' in self.results:
            r = self.results['pendulum_damping']
            if r.get('damping_coefficient'):
                # Convert to viscous damping coefficient Dp
                # This is a rough approximation - may need scaling
                recommendations['Dp'] = r['damping_coefficient'] * 1e-3
        
        if 'friction' in self.results:
            r = self.results['friction']
            if r.get('static_friction'):
                # Convert friction command to torque (approximate)
                recommendations['Dr'] = r['static_friction'] * 1e-5
        
        # Pendulum length
        if 'pendulum_properties' in self.results:
            r = self.results['pendulum_properties']
            if r.get('estimated_length'):
                # Remember: Lp in code is 2x center of mass distance
                recommendations['Lp'] = r['estimated_length'] * 2
        
        # Default values for parameters that need manual measurement
        defaults = {
            'Mr': None,  # Rotary arm mass - weigh it
            'Mp': None,  # Pendulum mass - weigh it  
            'Lr': None,  # Rotary arm length - measure it
            'Rm': None,  # Motor resistance - measure with multimeter
            'V': 12.0,   # Nominal voltage
            'reduction_ratio': None,  # Count gear teeth or measure
            'stall_torque': None,  # From motor datasheet
            'km': None,  # From motor datasheet or back-EMF test
        }
        
        # Merge with defaults
        for param, value in defaults.items():
            if param not in recommendations:
                recommendations[param] = value
        
        return recommendations
    
    def save_results(self, filepath: str):
        """Save all results to JSON file"""
        save_data = {
            'identification_results': self.results,
            'parameter_recommendations': self.generate_parameter_recommendations(),
            'experiment_metadata': {
                'control_frequency': self.control_freq,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'experiments_run': list(self.experiment_data.keys())
            }
        }
        
        with open(filepath, 'w') as f:
            json.dump(save_data, f, indent=2, default=str)
        
        self.logger.info(f"Results saved to {filepath}")
    
    def run_all_experiments(self):
        """Run all identification experiments"""
        self.logger.info("Starting comprehensive parameter identification...")
        
        try:
            # Run experiments in sequence
            self.experiment_motor_step_response()
            self.experiment_friction_identification() 
            self.experiment_pendulum_properties()
            self.experiment_free_oscillation_pendulum()
            
            self.logger.info("All experiments completed!")
            
        except Exception as e:
            self.logger.error(f"Experiment failed: {e}")
            raise
        finally:
            # Always stop the robot
            self.robot.step(0.0)


@hydra.main(version_base=None, config_path="configs", config_name="train")
def main(cfg: DictConfig) -> None:
    """Main function with Hydra configuration"""
    
    # Setup argument parser for experiment selection
    parser = argparse.ArgumentParser(description='Robot Parameter Identification')
    parser.add_argument('--experiment', 
                       choices=['motor_step', 'friction', 'pendulum_props', 'pendulum_osc', 'all'],
                       default='all',
                       help='Which experiment to run')
    parser.add_argument('--robot-type', 
                       choices=['regular', 'polimi'],
                       default='polimi',
                       help='Which robot interface to use')
    parser.add_argument('--save-results', 
                       default='parameter_identification_results.json',
                       help='File to save results')
    parser.add_argument('--save-plot', 
                       default='parameter_identification_plot.png',
                       help='File to save plot')
    
    args = parser.parse_args()
    
    # Initialize robot
    if args.robot_type == 'polimi':
        robot = PolimiRobot()
    else:
        robot = Robot()
    
    # Initialize identifier
    identifier = ParameterIdentifier(robot, control_freq=50.0)
    
    try:
        print("\n" + "="*60)
        print("ROBOT PARAMETER IDENTIFICATION")
        print("="*60)
        print(f"Experiment: {args.experiment}")
        print(f"Robot type: {args.robot_type}")
        print("="*60)
        
        # Safety warning
        print("\nSAFETY WARNING:")
        print("- Ensure robot is securely mounted")
        print("- Keep hands clear of moving parts")
        print("- Be ready to press Ctrl+C to stop")
        print("- Check that pendulum can move freely")
        
        input("\nPress Enter to continue or Ctrl+C to abort...")
        
        # Run selected experiment(s)
        if args.experiment == 'all':
            identifier.run_all_experiments()
        elif args.experiment == 'motor_step':
            identifier.experiment_motor_step_response()
        elif args.experiment == 'friction':
            identifier.experiment_friction_identification()
        elif args.experiment == 'pendulum_props':
            identifier.experiment_pendulum_properties()
        elif args.experiment == 'pendulum_osc':
            identifier.experiment_free_oscillation_pendulum()
        
        # Save and display results
        identifier.save_results(args.save_results)
        identifier.plot_results(args.save_plot)
        
        # Print recommendations
        print("\n" + "="*60)
        print("PARAMETER RECOMMENDATIONS")
        print("="*60)
        
        recommendations = identifier.generate_parameter_recommendations()
        for param, value in recommendations.items():
            if value is not None:
                print(f"{param:20s}: {value:.6f}")
            else:
                print(f"{param:20s}: Manual measurement needed")
        
        print("\n" + "="*60)
        print("NEXT STEPS:")
        print("1. Manually measure missing parameters (masses, lengths)")
        print("2. Update your robot configuration file")
        print("3. Run validation experiments to verify parameters")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\nExperiment interrupted by user")
    except Exception as e:
        print(f"\nError during identification: {e}")
    finally:
        robot.close()
        print("Robot connection closed")


if __name__ == "__main__":
    main() 