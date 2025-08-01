# Furuta Pendulum with Reinforcement Learning

This project documents the construction and control of a rotary inverted pendulum (Furuta Pendulum) using a Raspberry Pi and Reinforcement Learning. It is an adaptation of [the original work](https://github.com/Armandpl/furuta) by Armand du Parc Locmaria and Pierre Fabre, of which this repository is a fork. 

The primary goal of this initiative, led by Professor Marcello Restelli at the Politecnico di Milano's AIRlab, was to create a compelling physical demonstration of reinforcement learning. The robot serves as an educational tool for university open days and other public events, showcasing advanced control concepts in an accessible way. [Here](https://youtu.be/lfd6InJiKxQ) is a video of the robot running the final trained model (around 8h of total training).

---

## Table of Contents

- [Project Overview](#project-overview)
- [Getting Started: How to Use This Repository](#getting-started-how-to-replicate-the-project)
- [Hardware Implementation](#hardware-implementation)
- [Software and Control](#software-and-control)
- [Training Process](#training-process)
- [Challenges and Solutions](#challenges-and-solutions)
- [Original Project Credits](#original-project-credits)
- [Author](#author)

---

## Project Overview

The project began with an in-depth analysis of the original hardware design to understand the function of each component and the core engineering principles. The initial goal was to replicate the pendulum, but with a significant architectural change: simplifying the control system. While the original project used a combination of an Arduino and a Jetson Nano, this implementation consolidates both signal processing and high-level control onto a single Raspberry Pi. This approach reduces complexity and cost without compromising performance.

---

## Getting Started: How to replicate the project

This section outlines the steps to build, program, and run your own Furuta Pendulum.

### 1. Hardware Assembly

- **Bill of Materials**: A complete list of required components (motor, encoders, Raspberry Pi, etc.) can be found in the `robot/hardware/` directory.
- **3D Printing**: Print all `.stl` files located in the `robot/hardware/v1` directory.
- **Assembly**: Follow the hardware assembly guide, paying close attention to the wiring diagrams and the process for installing heat-set inserts.

### 2. Software Setup 
To setup the software, clone this repository on your raspberry and navigate to it
```
git clone github.com/dre-droid/furuta && cd furuta
```

install requirements
```
python3.11 -m venv .venv &&
source .venv/bin/activate &&
pip install -r requirements.txt
```
The scripts in the `robot/` directory handle communication with sensors and the motor.

### 3. System and Hardware Check

Before training, verify hardware functionality by running `python scripts/interactive_polimi_control.py` to test the encoder readings and the motor control.


### 4. Training the Agent

To train the agent, use `python train.py +experiment=sim` for simulation or `python train.py +experiment=polimi` for real-world training (ensure the robot is in a safe, clear area and you have an emergency stop for the motor). The config files for the experiment, as the other configs, are under `scripts/config/experiment`.

### 5. Running inference on a model

To run a pre-trained model, run `python scripts/rl_inference.py`. As before, the model and the other parameters can be configured with the `.yaml` files under `scripts/`.

---

## Hardware Implementation

The physical construction involved several key stages:

- **3D Printing**: All structural components were 3D printed using the provided files.
- **Soldering and Assembly**: The motor driver and encoder boards were soldered and integrated into the main assembly. To ensure durability and avoid stripping the plastic parts, threaded inserts were heat-set into the 3D prints for all screw connections.
- **Raspberry Pi Setup**: The Raspberry Pi was configured to handle all I/O operations, including reading data from the encoders and sending commands to the motor driver.


## Software and Control

With the hardware assembled, the next phase focused on software development and control systems.

- **Sensor and Actuator Interfacing**: Custom Python scripts were developed to interface directly with the hardware. This involved reading the GPIO pins connected to the encoders to get precise angular position data and sending PWM signals to the motor driver to control the arm's velocity.
- **Reinforcement Learning Stack Adaptation**: The original RL code, built on Stable-Baselines3, was carefully analyzed and adapted. The core logic was preserved, but modifications were made to ensure compatibility with the Raspberry Pi's processing capabilities and the custom hardware interface scripts. This involved tuning the observation and action spaces to match the physical characteristics of the new setup (mainly angle limits, speed limits and control frequency).

---

## Training Process

Training the agent to balance the pendulum was an iterative and challenging process.

- **Simulation First**: Initial training and debugging were performed in a simulated environment. This allowed for rapid testing of different reward functions and hyperparameters without risking damage to the physical robot.
- **Real-World Training**: Once the simulated agent showed promise, training was moved to the actual hardware. This phase required careful monitoring. The robot was trained over many hours to learn how to swing the pendulum up from a resting state and maintain its balance at the top.
- **Final Model**: After extensive testing and refinement, a stable model was achieved. This final model can reliably perform the swing-up maneuver and keep the pendulum balanced indefinitely, successfully meeting the project's primary goal.

- The last fine-tuning run showed a mean reward of 300, where the reward function assigns 1 to each of the 400 steps if the pendulum is upright (alpha angle of 180 degrees). This results in very solid inference runs, where the robot can balance the it upright immediately and is even tolerant to disturbances like being touched or pushed.

  <img width="362" height="307" alt="image" src="https://github.com/user-attachments/assets/772bb424-46c4-4b6d-b897-fd16900d1514" />


---

## Challenges and Solutions

Several obstacles were encountered and overcome during development:

- **Issue**: Wires coming loose during operation, causing unpredictable behavior.  
  **Solution**: All critical connections were soldered permanently. Using a multimeter to constantly check for continuity became a standard debugging step.

- **Issue**: The motor lacked sufficient torque for the swing-up maneuver.  
  **Solution**: Investigation revealed that the power supply unit had a current limit that was too low. Turning up the C.C. current solved the torque problem.

- **Issue**: The robot broke repeatedly during unsupervised training sessions when a connection failed, causing it to spin out of control.  
  **Solution**: This highlighted the need for robust physical and software failsafes. Improved soldered connections and careful monitoring during training runs mitigated this risk.

- **Issue**: Understanding the precise specifications of the encoders (clicks per revolution) and the motor driver was crucial for accurate control.  
  **Solution**: Time was invested in studying the component datasheets and writing small test scripts to validate their behavior experimentally before integrating them into the main control loop.

---

### Original Project Credits

This project builds on the foundational work from the original creators—repository: [github.com/Armandpl/furuta](https://github.com/Armandpl/furuta)—and incorporates contributions from Quanser, Mack Tang, Angelos Lovatto, Stable-Baselines3, and Antonin Raffin. For full details, consult the original repository.

### Author

This version was adapted and built by [Andrea Mastroberti].



