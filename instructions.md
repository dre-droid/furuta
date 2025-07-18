## Hardware Setup
1. Plug raspberry usb-c power and power supply power
2. Check connections for encoders/motor power as in the picture
3. Connect motor driver red/black to power supply red/black
4. Turn on raspberry (if it didn't turn on automatically, should blink green)
## Setup environment
5. ssh into raspberry (or connect keyboard/mouse and hdmi to a computer, root password is multiproject if needed)
6. open terminal
7. run $python scripts/interactive_polimi_control.py to check encoder signals and motor actions manually

## Run the software
To run inference: $python scripts/rl_inference.py
To run training: $python scripts/train.py
Config files for both are under scripts/config
scripts/config/env/polimi.yaml for environment parameters (like angle limits)
scripts/config/exp/polimi.yaml for experiment parameters like which model to load
scripts/config/wrappers/real_wrappers.yaml for wrapper settings (steps per episode parameter is here)

![IMG_1163](https://github.com/user-attachments/assets/73dbfb5a-a8fb-4320-ac8f-37f177ad8282)
![IMG_1164](https://github.com/user-attachments/assets/d2845c96-c395-4bd5-ab56-4dba0925a6c8)
![IMG_1165](https://github.com/user-attachments/assets/4f468cbf-a7d5-4ecc-8969-9f2f4771c405)
![IMG_1166](https://github.com/user-attachments/assets/92be1ab0-23f7-424f-b5b2-fc5cccbf7716)
