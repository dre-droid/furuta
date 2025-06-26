# Furuta Pendulum - Comprehensive Documentation

## Table of Contents
1. [Overview](#overview)
2. [Furuta Package Architecture](#furuta-package-architecture)
3. [Training and Inference Scripts](#training-and-inference-scripts)
4. [Hydra Configuration System](#hydra-configuration-system)
5. [Simulation vs Real Robot Training](#simulation-vs-real-robot-training)
6. [Quick Start Guide](#quick-start-guide)

## Overview

The Furuta Pendulum project is a complete system for building, simulating, and training reinforcement learning agents to control a rotary inverted pendulum (Furuta pendulum). The codebase supports both simulation-based training and real robot deployment using a modular, configuration-driven approach powered by Hydra.

### Key Features
- **Dual Environment Support**: Seamless switching between simulation and real robot
- **Modular RL Framework**: Built on StableBaselines3 with custom wrappers and algorithms
- **Configuration Management**: Hydra-based configuration system for reproducible experiments
- **Comprehensive Logging**: MCAP logging for data analysis and experiment tracking
- **Hardware Integration**: Direct motor control and encoder feedback for real robot

## Furuta Package Architecture

### Core Package Structure

```
furuta/
├── __init__.py                 # Gymnasium environment registration
├── rl/                        # Reinforcement Learning components
│   ├── envs/                  # Environment implementations
│   ├── algos.py              # Custom RL algorithm wrappers
│   ├── utils.py              # RL utilities and helpers
│   └── wrappers.py           # Environment wrappers
├── controls/                  # Control system components
├── logging/                   # Data logging and protobuf definitions
├── robot.py                   # Robot dynamics and hardware interface
├── sim.py                     # Simulation utilities
├── state.py                   # State representations
├── utils.py                   # General utilities
└── viewer.py                  # Visualization components
```

### Environment Hierarchy

#### 1. FurutaBase (`furuta/rl/envs/furuta_base.py`)

The base class defining the common interface for both simulation and real robot environments.

**Key Components:**
- **State Space**: 4D state vector `[θ, α, θ̇, α̇]` where:
  - `θ` (theta): Motor/arm angle
  - `α` (alpha): Pendulum angle  
  - `θ̇`, `α̇`: Angular velocities
- **Action Space**: Continuous control `[-1, 1]` representing motor voltage
- **Observation Space**: Transformed state with trigonometric encoding:
  - `[cos(θ), sin(θ), cos(α), sin(α), θ̇, α̇]`
  - Optional angle limits added to observation if specified

**Reward Functions:**
Multiple reward functions available via configuration:
- `cos_alpha`: Basic cosine-based reward for pendulum balancing
- `exp_alpha_X`: Exponential rewards with different sharpness parameters

#### 2. FurutaSim (`furuta/rl/envs/furuta_sim.py`)

Simulation environment with physics-based dynamics and configurable noise models.

**Features:**
- **Physics Integration**: Runge-Kutta integration with configurable timesteps
- **Sensor Simulation**: Encoder quantization and velocity filtering
- **Domain Randomization**: Randomizable dynamics parameters
- **High-frequency Integration**: Sub-step integration for numerical stability

**Configuration Options:**
```yaml
control_freq: 50              # Control frequency (Hz)
reward: "cos_alpha"           # Reward function selection
angle_limits: [null, 6]       # Angle limits [theta, alpha] in radians
speed_limits: [50, null]      # Speed limits [theta_dot, alpha_dot] in rad/s
encoders_CPRs: null           # Encoder resolution simulation
velocity_filter: 2           # Velocity filter order
```

#### 3. FurutaReal (`furuta/rl/envs/furuta_real.py`)

Real robot environment with hardware interface and safety mechanisms.

**Features:**
- **Hardware Interface**: Serial communication with Arduino-based controller
- **Safety Systems**: Automatic reset sequences and speed limiting
- **Real-time Control**: Enforced timing constraints
- **Encoder Integration**: Direct encoder readings with velocity filtering

**Reset Behavior:**
1. **Motor Stopping**: Gradual motor deceleration 
2. **Pendulum Settling**: Wait for pendulum to hang down naturally
3. **Encoder Reset**: Zero both motor and pendulum encoders
4. **Velocity Filter Reset**: Clear filter history

### Environment Wrappers

Located in `furuta/rl/wrappers.py`, these provide additional functionality:

#### 1. GentlyTerminating
- **Purpose**: Sends zero command to robot when episode terminates
- **Use Case**: Prevents abrupt motor stops that could damage hardware

#### 2. DeadZone
- **Purpose**: Handles motor deadzone characteristics
- **Parameters**:
  - `deadzone`: Minimum effective action threshold
  - `center`: Action deadband around zero
  - `max_act`: Maximum action scaling

#### 3. HistoryWrapper
- **Purpose**: Maintains action-observation history for temporal learning
- **Features**:
  - Configurable history length
  - Continuity cost for smooth control
  - Zero-padding for initial steps

#### 4. MCAPLogger
- **Purpose**: Records environment data in MCAP format
- **Features**:
  - Configurable simulation vs real-time logging
  - Episodic or continuous logging modes
  - Protobuf-based data serialization

#### 5. ControlFrequency
- **Purpose**: Enforces real-time constraints
- **Features**:
  - Sleep timing to maintain control frequency
  - Performance monitoring and warnings

### Robot Dynamics and Hardware

#### QubeDynamics (`furuta/robot.py`)

Physics simulation based on the Quanser Qube design:

**Parameters:**
- **Motor**: Resistance, back-EMF constant, gear ratio
- **Rotary Arm**: Mass, length, damping coefficients  
- **Pendulum**: Mass, length, damping coefficients
- **Domain Randomization**: Gaussian noise on all parameters

**Equations of Motion:**
The system uses Lagrangian mechanics to derive the equations of motion for the coupled motor-pendulum system.

#### Hardware Interface

**Communication Protocol:**
- **Baud Rate**: 921600 bps
- **Command Format**: Binary packets with start sequence
- **Response Format**: Motor angle, pendulum angle, timestamp

**Encoder Specifications:**
- **Motor**: 400 CPR (counts per revolution)
- **Pendulum**: 5120×4 CPR for high precision

## Training and Inference Scripts

### Training Script (`scripts/train.py`)

Comprehensive training pipeline with experiment tracking and model management.

**Key Features:**

#### 1. Environment Setup
```python
# Hydra instantiation with recursive parameter resolution
env = hydra.utils.instantiate(cfg.env, _recursive_=True)

# Wrapper application in sequence
for wrapper in cfg.wrappers:
    env = hydra.utils.instantiate(wrapper, env=env)
```

#### 2. Algorithm Configuration
- **Multi-algorithm Support**: SAC, TQC, PPO through configuration
- **Hyperparameter Management**: All parameters configurable via Hydra
- **Model Checkpointing**: Automatic model and replay buffer saving

#### 3. Parallelization Strategy
```python
# Real robot: Single environment only
if isinstance(env.unwrapped, FurutaReal):
    assert cfg.n_envs == 1

# Simulation: Supports parallel environments
if cfg.n_envs > 1:
    vec_env = SubprocVecEnv([lambda: copy.deepcopy(env) for _ in range(cfg.n_envs)])
```

#### 4. Evaluation and Early Stopping
- **Periodic Evaluation**: Configurable frequency and episode count
- **Early Stopping**: Reward threshold-based termination
- **Deterministic Testing**: Separate evaluation environment

#### 5. Experiment Tracking
- **Weights & Biases Integration**: Automatic experiment logging
- **Video Recording**: Episodic video capture for analysis
- **Artifact Management**: Model and data artifact uploading

### Inference Script (`scripts/rl_inference.py`)

Model deployment and evaluation pipeline.

**Key Features:**

#### 1. Model Loading
```python
# Download model from W&B artifacts
artifact = run.use_artifact(cfg.model_artifact)
model = model.load(os.path.join(artifact_dir, "model.zip"))
```

#### 2. Configuration Inheritance
- **Producer Config**: Inherits training configuration from model artifact
- **Override Capability**: Allows environment and wrapper overrides
- **Deterministic Evaluation**: Consistent policy execution

#### 3. Data Collection
- **MCAP Logging**: Automatic data logging during inference
- **Performance Metrics**: Episode return statistics
- **Visualization**: Optional real-time rendering

## Hydra Configuration System

### Configuration Hierarchy

The configuration system uses Hydra's composition pattern:

```
scripts/configs/
├── train.yaml                # Main training configuration
├── rl_inference.yaml         # Inference configuration
├── env/                      # Environment configurations
├── algo/                     # Algorithm configurations
├── wrappers/                 # Wrapper configurations
├── experiment/               # Complete experiment configs
└── sweeps/                   # Hyperparameter sweep configs
```

### Configuration Composition

#### 1. Base Configuration (`train.yaml`)
```yaml
defaults:
  - _self_
  - env: ???                  # Must be specified
  - wrappers: ???             # Must be specified  
  - algo: sac.yaml            # Default algorithm

total_timesteps: 500_000
n_envs: 1
seed: 1
```

#### 2. Environment Configuration
**Simulation** (`env/sim.yaml`):
```yaml
defaults:
  - _self_
  - dyn: tiny                 # Dynamics parameters

_target_: furuta.rl.envs.furuta_sim.FurutaSim
control_freq: 50
reward: "cos_alpha"
angle_limits: [null, 6]
speed_limits: [50, null]
```

**Real Robot** (`env/real.yaml`):
```yaml
defaults:
  - _self_
  - robot: tiny_robot         # Robot hardware config

_target_: furuta.rl.envs.furuta_real.FurutaReal
control_freq: 50
reward: "cos_alpha"
angle_limits: [null, 12]
```

#### 3. Algorithm Configuration (`algo/sac.yaml`)
```yaml
_target_: furuta.rl.algos.SAC
policy: "MlpPolicy"
learning_rate: 0.0003
buffer_size: 1_000_000
use_sde: True                 # State-dependent exploration
train_freq: 1
gradient_steps: -1
```

#### 4. Wrapper Configuration (`wrappers/base_wrappers.yaml`)
```yaml
- _target_: gymnasium.wrappers.TimeLimit
  max_episode_steps: 400
- _target_: furuta.rl.wrappers.HistoryWrapper
  steps: 2
  use_continuity_cost: True
- _target_: stable_baselines3.common.monitor.Monitor
```

### Experiment Configurations

#### Complete Experiment Configs
Combine all components for reproducible experiments:

**Simulation Experiment** (`experiment/sim.yaml`):
```yaml
# @package _global_
defaults:
  - override /env: sim
  - override /wrappers: base_wrappers
  - override /algo: sac

total_timesteps: 200_000
evaluation:
  eval_freq: null             # Disable evaluation for speed
```

**Real Robot Experiment** (`experiment/real.yaml`):
```yaml
# @package _global_
defaults:
  - override /env: real
  - override /wrappers: real_wrappers
  - override /algo: sac_real

total_timesteps: 100_000
n_envs: 1                     # Required for real robot
debug: True
```

### Configuration Override System

#### 1. Command Line Overrides
```bash
# Override specific parameters
python scripts/train.py experiment=sim total_timesteps=1000000

# Override nested parameters
python scripts/train.py experiment=real env.control_freq=100

# Add new parameters
python scripts/train.py +wandb.tags=[test,debug]
```

#### 2. Hydra Composition
```bash
# Compose from multiple config groups
python scripts/train.py env=sim wrappers=base_wrappers algo=tqc

# Use experiment configs
python scripts/train.py experiment=sim_randomized
```

### Advanced Configuration Features

#### 1. Parameter Sweeps
Configuration for hyperparameter optimization:
```yaml
# sweeps/reward.yaml
defaults:
  - override /hydra/sweeper: optuna

env:
  reward: choice("cos_alpha", "exp_alpha_2", "exp_alpha_4")
algo:
  learning_rate: interval(0.0001, 0.001)
```

#### 2. Conditional Configuration
```yaml
# Real robot specific settings
defaults:
  - wrappers: base_wrappers
  - wrappers@_here_: real_wrappers  # Append real robot wrappers
```

## Simulation vs Real Robot Training

### Simulation Training

**Advantages:**
- **Speed**: Fast parallel training with multiple environments
- **Safety**: No hardware damage risk during exploration
- **Reproducibility**: Deterministic physics and seeded randomness
- **Parameter Exploration**: Easy dynamics randomization

**Configuration Example:**
```bash
# Fast parallel simulation training
python scripts/train.py experiment=sim n_envs=8 total_timesteps=1000000
```

**Domain Randomization:**
The simulation supports randomizing dynamics parameters to improve sim-to-real transfer:
```yaml
# env/sim_randomized.yaml
dyn:
  Mr_std: 0.01              # Arm mass variation
  Lp_std: 0.005             # Pendulum length variation
  Dr_std: 1e-6              # Damping variation
```

### Real Robot Training

**Advantages:**
- **Reality**: Direct policy learning on target hardware
- **No Sim-to-Real Gap**: Eliminates transfer learning issues
- **True Dynamics**: Captures unmodeled effects and nonlinearities

**Safety Considerations:**
- **Speed Limits**: Configurable maximum angular velocities
- **Gentle Termination**: Gradual motor stopping on episode end
- **Reset Sequences**: Automatic pendulum settling between episodes
- **Emergency Stops**: Manual intervention capabilities

**Configuration Example:**
```bash
# Real robot training with safety limits
python scripts/train.py experiment=real env.speed_limits=[30,200]
```

**Typical Training Progression:**
1. **Initial Training**: Simulation with domain randomization
2. **Fine-tuning**: Real robot training starting from simulation checkpoint
3. **Deployment**: Inference with trained policy

### Transfer Learning Workflow

#### 1. Simulation Pre-training
```bash
# Train base policy in simulation
python scripts/train.py experiment=sim_randomized total_timesteps=500000
```

#### 2. Real Robot Fine-tuning
```bash
# Fine-tune on real robot
python scripts/train.py experiment=real_finetune \
  model_artifact="wandb-run-id:model:latest" \
  total_timesteps=50000
```

#### 3. Inference and Evaluation
```bash
# Deploy trained policy
python scripts/rl_inference.py \
  model_artifact="wandb-run-id:model:latest" \
  nb_episodes=10
```

## Quick Start Guide

### 1. Installation
```bash
# Clone repository
git clone <repository-url>
cd furuta

# Install dependencies
poetry install
```

### 2. Simulation Training
```bash
# Basic simulation training
python scripts/train.py experiment=sim

# Advanced simulation with domain randomization
python scripts/train.py experiment=sim_randomized n_envs=4
```

### 3. Real Robot Setup
```bash
# Find robot port
sudo dmesg | grep tty

# Test robot connection
python tests/interactive_robot_selftest.py

# Train on real robot
python scripts/train.py experiment=real env.robot.device="/dev/ttyACM0"
```

### 4. Model Inference
```bash
# Inference with visualization
python scripts/rl_inference.py \
  model_artifact="your-wandb-run:model:latest" \
  render=True

# Batch evaluation
python scripts/rl_inference.py \
  model_artifact="your-wandb-run:model:latest" \
  nb_episodes=50
```

### 5. Configuration Customization
```bash
# Custom reward function
python scripts/train.py experiment=sim env.reward="exp_alpha_4"

# Modified training schedule
python scripts/train.py experiment=sim \
  total_timesteps=1000000 \
  evaluation.eval_freq=10000

# Different algorithm
python scripts/train.py experiment=sim algo=tqc
```

### 6. Monitoring and Logging
- **Weights & Biases**: Automatic experiment tracking
- **TensorBoard**: Local training monitoring
- **MCAP Logs**: Detailed episode data for analysis
- **Video Recording**: Visual policy evaluation

This documentation provides a comprehensive guide to the Furuta pendulum codebase. For specific implementation details, refer to the source code and inline comments. For questions or issues, consult the repository's issue tracker or contact the maintainers. 