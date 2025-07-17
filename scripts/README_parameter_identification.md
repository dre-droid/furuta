# Robot Parameter Identification Guide

This directory contains tools to help you identify the physical parameters of your Furuta pendulum robot for accurate simulation modeling.

## Quick Start

### 1. Manual Measurements (Start Here)
```bash
cd scripts
python measure_physical_parameters.py
```

This interactive script guides you through measuring:
- **Masses**: Rotary arm and pendulum (use precision scale)
- **Lengths**: Arm length and pendulum center of mass
- **Motor specs**: Resistance, voltage, gear ratio, torque, back-EMF
- **Encoders**: Counts per revolution

**Required tools:**
- Precision scale (0.1g accuracy preferred)
- Ruler or calipers  
- Multimeter
- Motor datasheet

### 2. Automated Identification (Advanced)
```bash
# Run all experiments
python identify_robot_parameters.py --experiment all --robot-type polimi

# Or run individual experiments
python identify_robot_parameters.py --experiment motor_step --robot-type polimi
python identify_robot_parameters.py --experiment friction --robot-type polimi
python identify_robot_parameters.py --experiment pendulum_osc --robot-type polimi
```

**⚠️ Safety Warning:** 
- Ensure robot is securely mounted
- Keep hands clear of moving parts
- Be ready to press Ctrl+C to emergency stop

## Parameter Overview

### Easy to Measure Manually
| Parameter | Method | Units |
|-----------|--------|-------|
| `Mr` | Weigh rotary arm | kg |
| `Mp` | Weigh pendulum | kg |
| `Lr` | Measure arm length | m |
| `Lp` | Measure to center of mass × 2 | m |
| `Rm` | Multimeter across motor terminals | Ω |
| `V` | Measure supply voltage | V |
| `reduction_ratio` | Count gear teeth or rotations | - |

### From Motor Datasheet
| Parameter | Description | Units |
|-----------|-------------|-------|
| `stall_torque` | Maximum torque | N⋅m |
| `km` | Back-EMF constant | V⋅s/rad |

### Requires Identification Experiments
| Parameter | Method | Units |
|-----------|--------|-------|
| `Dr` | Motor friction experiment | N⋅m⋅s/rad |
| `Dp` | Pendulum oscillation experiment | N⋅m⋅s/rad |

## Identification Experiments

### 1. Motor Step Response
- Tests motor dynamics and friction
- Applies step commands and measures response
- Identifies: steady-state gain, settling time, static friction

### 2. Pendulum Free Oscillation  
- Measures natural damping of pendulum
- You manually displace pendulum, then release
- Identifies: damping coefficient, natural frequency

### 3. Friction Identification
- Applies slow ramp commands
- Identifies: static and kinetic friction

### 4. Pendulum Properties
- Small sinusoidal excitation near vertical
- Estimates natural frequency and effective length

## Using Results

### Update Configuration Files
The scripts generate YAML config files like:
```yaml
_target_: furuta.robot.QubeDynamics
# Motor parameters  
Rm: 6.66
V: 12.0
reduction_ratio: 9.68
stall_torque: 0.16677
km: 0.01397458038
# Rotary arm
Mr: 0.035
Lr: 0.057  
Dr: 5e-6
# Pendulum
Mp: 0.016
Lp: 0.16
Dp: 1e-6
```

### Validation
1. Compare simulation vs real robot behavior
2. Adjust damping parameters if needed
3. Re-run identification experiments with better estimates

## Troubleshooting

### Common Issues
- **"Not enough oscillation peaks"**: Displace pendulum more initially
- **"Motor driver fault"**: Check wiring and power supply  
- **"Safety limit exceeded"**: Reduce experiment intensity or check limits
- **Noisy velocity estimates**: Increase velocity filter setting

### Parameter Sensitivity
- **Most important**: Masses (`Mr`, `Mp`) and lengths (`Lr`, `Lp`) 
- **Medium importance**: Motor constants (`Rm`, `km`, `stall_torque`)
- **Least critical**: Damping (`Dr`, `Dp`) - can be tuned iteratively

### Tips
- Start with manual measurements - they're most reliable
- Use identification experiments to refine damping values
- Compare multiple measurement runs for consistency
- Keep safety as top priority during automated experiments

## File Outputs

| File | Description |
|------|-------------|
| `measured_parameters.json` | Raw measurement data |
| `parameter_identification_results.json` | Identification experiment results |  
| `parameter_identification_plot.png` | Visualization of experiments |
| `my_robot_config.yaml` | Generated configuration file |

## Integration with Training

Once you have good parameters:

1. **Update your robot config:**
   ```bash
   cp my_robot_config.yaml scripts/configs/env/dyn/my_robot.yaml
   ```

2. **Create new environment config:**
   ```yaml
   # scripts/configs/env/my_env.yaml
   defaults:
     - _self_
     - dyn: my_robot  # Use your measured parameters
     - robot: polimi_robot
   
   _target_: furuta.rl.envs.furuta_real.FurutaReal
   control_freq: 50
   reward: "cos_alpha"
   angle_limits: [null, 6]
   speed_limits: [60, 100]
   ```

3. **Update experiment config:**
   ```yaml
   # scripts/configs/experiment/my_experiment.yaml
   defaults:
     - override /env: my_env  # Use your environment
     - override /wrappers: real_wrappers  
     - override /algo: sac_real
   
   total_timesteps: 100_000
   n_envs: 1
   model_artifact: "model:latest"
   ```

4. **Train with accurate parameters:**
   ```bash
   python train.py +experiment=my_experiment
   ```

This ensures your simulation uses the real physical parameters of your robot! 