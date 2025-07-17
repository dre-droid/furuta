#!/usr/bin/env python3
"""
Manual Physical Parameter Measurement Guide

This script provides a guided workflow for manually measuring
the physical parameters of your Furuta pendulum robot.

Usage:
    python measure_physical_parameters.py
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, Optional
import argparse


class PhysicalParameterMeasurer:
    """Guide for measuring physical robot parameters"""
    
    def __init__(self):
        self.measurements = {}
        
    def measure_masses(self):
        """Guide for measuring component masses"""
        print("\n" + "="*60)
        print("MASS MEASUREMENTS")
        print("="*60)
        print("You'll need a precision scale (preferably 0.1g accuracy)")
        
        # Rotary arm mass
        print("\n1. ROTARY ARM MASS (Mr)")
        print("   - Remove the arm from the motor shaft")
        print("   - Include any mounting hardware that rotates with it")
        print("   - Weigh the complete rotating assembly")
        
        while True:
            try:
                mr_grams = float(input("   Enter rotary arm mass in grams: "))
                mr_kg = mr_grams / 1000.0
                self.measurements['Mr'] = mr_kg
                print(f"   Recorded: Mr = {mr_kg:.4f} kg")
                break
            except ValueError:
                print("   Please enter a valid number")
        
        # Pendulum mass  
        print("\n2. PENDULUM MASS (Mp)")
        print("   - Remove the pendulum from the arm")
        print("   - Include the pivot hardware")
        print("   - Weigh the complete pendulum assembly")
        
        while True:
            try:
                mp_grams = float(input("   Enter pendulum mass in grams: "))
                mp_kg = mp_grams / 1000.0
                self.measurements['Mp'] = mp_kg
                print(f"   Recorded: Mp = {mp_kg:.4f} kg")
                break
            except ValueError:
                print("   Please enter a valid number")
    
    def measure_lengths(self):
        """Guide for measuring component lengths"""
        print("\n" + "="*60)
        print("LENGTH MEASUREMENTS")
        print("="*60)
        print("You'll need a ruler or calipers")
        
        # Rotary arm length
        print("\n1. ROTARY ARM LENGTH (Lr)")
        print("   - Measure from the center of rotation (motor shaft)")
        print("   - To the center of the pendulum pivot point")
        print("   - This is the distance to where the pendulum attaches")
        
        while True:
            try:
                lr_mm = float(input("   Enter rotary arm length in millimeters: "))
                lr_m = lr_mm / 1000.0
                self.measurements['Lr'] = lr_m
                print(f"   Recorded: Lr = {lr_m:.4f} m")
                break
            except ValueError:
                print("   Please enter a valid number")
        
        # Pendulum length (center of mass)
        print("\n2. PENDULUM LENGTH (Lp)")
        print("   - Measure from the pivot point to the CENTER OF MASS")
        print("   - For uniform rod: measure to the middle")
        print("   - For weighted pendulum: estimate where the balance point is")
        print("   - NOTE: The code will multiply this by 2, so measure to center of mass only")
        
        while True:
            try:
                lp_mm = float(input("   Enter pendulum center-of-mass distance in millimeters: "))
                lp_m = lp_mm / 1000.0
                # The code expects Lp = 2 * center_of_mass_distance
                lp_code = lp_m * 2
                self.measurements['Lp'] = lp_code
                print(f"   Recorded: Lp = {lp_code:.4f} m (2x center of mass distance)")
                break
            except ValueError:
                print("   Please enter a valid number")
    
    def measure_motor_parameters(self):
        """Guide for measuring motor parameters"""
        print("\n" + "="*60)
        print("MOTOR PARAMETERS")
        print("="*60)
        
        # Motor resistance
        print("\n1. MOTOR RESISTANCE (Rm)")
        print("   - Disconnect motor from circuit")
        print("   - Use multimeter to measure resistance across motor terminals")
        print("   - Alternative: check motor datasheet for winding resistance")
        
        while True:
            try:
                rm_input = input("   Enter motor resistance in Ohms (or 'skip'): ")
                if rm_input.lower() == 'skip':
                    self.measurements['Rm'] = None
                    print("   Skipped - will need to measure later")
                    break
                rm = float(rm_input)
                self.measurements['Rm'] = rm
                print(f"   Recorded: Rm = {rm:.2f} Ω")
                break
            except ValueError:
                print("   Please enter a valid number or 'skip'")
        
        # Supply voltage
        print("\n2. SUPPLY VOLTAGE (V)")
        print("   - Measure actual voltage at motor terminals during operation")
        print("   - Or use nominal supply voltage")
        
        while True:
            try:
                v = float(input("   Enter supply voltage in Volts: "))
                self.measurements['V'] = v
                print(f"   Recorded: V = {v:.1f} V")
                break
            except ValueError:
                print("   Please enter a valid number")
        
        # Gear reduction ratio
        print("\n3. GEAR REDUCTION RATIO")
        print("   - Turn motor shaft by hand exactly N full rotations")
        print("   - Count how many rotations the output shaft makes")
        print("   - Ratio = motor_rotations / output_rotations")
        print("   - Example: if motor turns 10 times and output turns 1 time, ratio = 10")
        
        while True:
            try:
                ratio_input = input("   Enter gear reduction ratio (or 'skip'): ")
                if ratio_input.lower() == 'skip':
                    self.measurements['reduction_ratio'] = None
                    print("   Skipped - will need to measure later")
                    break
                ratio = float(ratio_input)
                self.measurements['reduction_ratio'] = ratio
                print(f"   Recorded: reduction_ratio = {ratio:.2f}")
                break
            except ValueError:
                print("   Please enter a valid number or 'skip'")
        
        # Motor constants (from datasheet)
        print("\n4. MOTOR CONSTANTS (from datasheet)")
        print("   - Look up motor part number in datasheet")
        
        # Stall torque
        while True:
            try:
                stall_input = input("   Enter stall torque in N⋅m (or 'skip'): ")
                if stall_input.lower() == 'skip':
                    self.measurements['stall_torque'] = None
                    print("   Skipped - will need from datasheet")
                    break
                stall = float(stall_input)
                self.measurements['stall_torque'] = stall
                print(f"   Recorded: stall_torque = {stall:.4f} N⋅m")
                break
            except ValueError:
                print("   Please enter a valid number or 'skip'")
        
        # Back-EMF constant
        while True:
            try:
                km_input = input("   Enter back-EMF constant in V⋅s/rad (or 'skip'): ")
                if km_input.lower() == 'skip':
                    self.measurements['km'] = None
                    print("   Skipped - will need from datasheet")
                    break
                km = float(km_input)
                self.measurements['km'] = km
                print(f"   Recorded: km = {km:.6f} V⋅s/rad")
                break
            except ValueError:
                print("   Please enter a valid number or 'skip'")
    
    def estimate_damping_parameters(self):
        """Provide guidance on damping parameters"""
        print("\n" + "="*60)
        print("DAMPING PARAMETERS")
        print("="*60)
        print("These are difficult to measure directly.")
        print("Recommendations:")
        print("")
        
        print("1. ROTARY ARM DAMPING (Dr):")
        print("   - Start with a small value: Dr = 5e-6")
        print("   - Adjust based on simulation vs real robot behavior")
        print("   - Increase if simulation arm spins too freely")
        
        self.measurements['Dr'] = 5e-6
        print(f"   Initial estimate: Dr = {self.measurements['Dr']:.2e} N⋅m⋅s/rad")
        
        print("\n2. PENDULUM DAMPING (Dp):")
        print("   - Start with a small value: Dp = 1e-6")
        print("   - Adjust based on pendulum oscillation behavior")
        print("   - Increase if simulation pendulum oscillates too long")
        
        self.measurements['Dp'] = 1e-6
        print(f"   Initial estimate: Dp = {self.measurements['Dp']:.2e} N⋅m⋅s/rad")
        
        print("\nNOTE: Use the parameter identification script to get better estimates!")
    
    def calculate_encoder_parameters(self):
        """Calculate encoder parameters"""
        print("\n" + "="*60)
        print("ENCODER PARAMETERS")
        print("="*60)
        
        # Motor encoder CPR
        print("1. MOTOR ENCODER CPR (Counts Per Revolution)")
        while True:
            try:
                motor_cpr = int(input("   Enter motor encoder CPR: "))
                self.measurements['motor_encoder_cpr'] = motor_cpr
                print(f"   Recorded: motor_encoder_cpr = {motor_cpr}")
                break
            except ValueError:
                print("   Please enter a valid integer")
        
        # Pendulum encoder CPR
        print("\n2. PENDULUM ENCODER CPR")
        while True:
            try:
                pendulum_cpr = int(input("   Enter pendulum encoder CPR: "))
                self.measurements['pendulum_encoder_cpr'] = pendulum_cpr
                print(f"   Recorded: pendulum_encoder_cpr = {pendulum_cpr}")
                break
            except ValueError:
                print("   Please enter a valid integer")
    
    def generate_config_file(self, template_file: str = "scripts/configs/env/dyn/tiny.yaml"):
        """Generate a configuration file with the measured parameters"""
        config_lines = [
            "_target_: furuta.robot.QubeDynamics",
            "# Robot parameters measured on " + str(np.datetime64('today')),
            "",
            "# Motor parameters",
        ]
        
        if self.measurements.get('Rm'):
            config_lines.append(f"Rm: {self.measurements['Rm']:.2f}  # Motor resistance (Ohms)")
        else:
            config_lines.append("Rm: 6.66  # Motor resistance (Ohms) - NEEDS MEASUREMENT")
        
        config_lines.append(f"V: {self.measurements.get('V', 12.0):.1f}  # Supply voltage (V)")
        
        if self.measurements.get('reduction_ratio'):
            config_lines.append(f"reduction_ratio: {self.measurements['reduction_ratio']:.2f}")
        else:
            config_lines.append("reduction_ratio: 1.0  # NEEDS MEASUREMENT")
        
        if self.measurements.get('stall_torque'):
            config_lines.append(f"stall_torque: {self.measurements['stall_torque']:.6f}  # N⋅m")
        else:
            config_lines.append("stall_torque: 0.16  # N⋅m - NEEDS DATASHEET VALUE")
        
        if self.measurements.get('km'):
            config_lines.append(f"km: {self.measurements['km']:.8f}  # V⋅s/rad")
        else:
            config_lines.append("km: 0.042  # V⋅s/rad - NEEDS DATASHEET VALUE")
        
        config_lines.extend([
            "",
            "# Rotary arm parameters",
            f"Mr: {self.measurements.get('Mr', 0.035):.4f}  # Rotary arm mass (kg)",
            f"Lr: {self.measurements.get('Lr', 0.057):.4f}  # Rotary arm length (m)",
            f"Dr: {self.measurements.get('Dr', 5e-6):.2e}  # Rotary arm damping (N⋅m⋅s/rad)",
            "",
            "# Pendulum parameters",
            f"Mp: {self.measurements.get('Mp', 0.016):.4f}  # Pendulum mass (kg)",
            f"Lp: {self.measurements.get('Lp', 0.16):.4f}  # Pendulum length (m) - 2x center of mass",
            f"Dp: {self.measurements.get('Dp', 1e-6):.2e}  # Pendulum damping (N⋅m⋅s/rad)",
        ])
        
        return "\n".join(config_lines)
    
    def save_measurements(self, filename: str = "measured_parameters.json"):
        """Save measurements to JSON file"""
        with open(filename, 'w') as f:
            json.dump(self.measurements, f, indent=2)
        print(f"\nMeasurements saved to {filename}")
    
    def run_measurement_workflow(self):
        """Run the complete measurement workflow"""
        print("FURUTA PENDULUM ROBOT")
        print("Physical Parameter Measurement Guide")
        print("="*60)
        print("This script will guide you through measuring all the physical")
        print("parameters needed for accurate robot simulation.")
        print("")
        print("Have ready:")
        print("- Precision scale (0.1g accuracy preferred)")
        print("- Ruler or calipers")
        print("- Multimeter")
        print("- Motor datasheet (if available)")
        print("- Disassembly tools")
        
        input("\nPress Enter to continue...")
        
        # Run measurement sections
        self.measure_masses()
        self.measure_lengths()
        self.measure_motor_parameters()
        self.estimate_damping_parameters()
        self.calculate_encoder_parameters()
        
        # Summary
        print("\n" + "="*60)
        print("MEASUREMENT SUMMARY")
        print("="*60)
        
        for param, value in self.measurements.items():
            if value is not None:
                if isinstance(value, float):
                    if value < 0.001:
                        print(f"{param:20s}: {value:.2e}")
                    else:
                        print(f"{param:20s}: {value:.6f}")
                else:
                    print(f"{param:20s}: {value}")
            else:
                print(f"{param:20s}: NEEDS MEASUREMENT")
        
        # Generate config file
        print("\n" + "="*60)
        print("CONFIGURATION FILE")
        print("="*60)
        
        config_content = self.generate_config_file()
        print("\nGenerated configuration:")
        print("-" * 40)
        print(config_content)
        print("-" * 40)
        
        # Save options
        save_json = input("\nSave measurements to JSON? (y/n): ").lower() == 'y'
        if save_json:
            self.save_measurements()
        
        save_config = input("Save configuration file? (y/n): ").lower() == 'y'
        if save_config:
            config_filename = input("Enter config filename (e.g., my_robot.yaml): ").strip()
            if not config_filename:
                config_filename = "my_robot_config.yaml"
            
            with open(config_filename, 'w') as f:
                f.write(config_content)
            print(f"Configuration saved to {config_filename}")
        
        print("\n" + "="*60)
        print("NEXT STEPS")
        print("="*60)
        print("1. Complete any missing measurements")
        print("2. Run parameter identification script for damping values:")
        print("   python identify_robot_parameters.py --experiment all")
        print("3. Update your robot configuration with measured values")
        print("4. Test simulation accuracy against real robot")
        print("="*60)


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Manual robot parameter measurement guide')
    parser.add_argument('--load', help='Load previous measurements from JSON file')
    args = parser.parse_args()
    
    measurer = PhysicalParameterMeasurer()
    
    if args.load:
        try:
            with open(args.load, 'r') as f:
                measurer.measurements = json.load(f)
            print(f"Loaded previous measurements from {args.load}")
        except FileNotFoundError:
            print(f"File {args.load} not found, starting fresh")
    
    measurer.run_measurement_workflow()


if __name__ == "__main__":
    main() 