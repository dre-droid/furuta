1. Plug raspberry usb-c power and power supply power
2. Check connections for encoders/motor power as in the picture
3. Connect motor driver red/black to power supply red/black
4. Turn on raspberry (if it didn't turn on automatically, should blink green)
5. ssh into raspberry (or connect keyboard/mouse and hdmi to a computer, root password is multiproject if needed)
6. open terminal
7. run $python scripts/interactive_polimi_control.py to check encoder signals and motor actions manually

To run inference: $python scripts/rl_inference.py
To run training: $python scripts/train.py
Config files for both are under scripts/config
scripts/config/env/polimi.yaml for environment parameters (like angle limits)
scripts/config/exp/polimi.yaml for experiment parameters like which model to load
scripts/config/wrappers/real_wrappers.yaml for wrapper settings (steps per episode parameter is here)