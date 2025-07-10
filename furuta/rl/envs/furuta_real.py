import logging
from time import sleep
from typing import Optional
import time

import numpy as np
from simple_pid import PID

from furuta.rl.envs.furuta_base import FurutaBase
from furuta.utils import ALPHA, ALPHA_DOT, THETA, THETA_DOT, VelocityFilter

MAX_RESET_TIME = 4  # seconds
MAX_MOTOR_RESET_TIME = 0.2  # seconds
RESET_TIME = 0.5
ALPHA_THRESH = np.cos(
    np.deg2rad(2)
)  # alpha should stay between -2 and 2 deg for 0.5 sec for us to consider the env reset

class SensorFailureException(Exception):
    pass

class FurutaReal(FurutaBase):
    def __init__(
        self,
        robot,  # Any robot that implements step(), reset_encoders(), close()
        control_freq=100,
        reward="exp_alpha_2",
        angle_limits=None,
        speed_limits=None,
        motor_stop_pid=[0.04, 0.0, 0.001],
        enable_episode_logging=True,
        log_frequency=10,  # Log every N steps
    ):
        super().__init__(control_freq, reward, angle_limits, speed_limits)
        self.motor_stop_pid = motor_stop_pid
        self.enable_episode_logging = enable_episode_logging
        self.log_frequency = log_frequency

        self.robot = robot
        self._state = None
        
        # Episode logging variables
        self.episode_step = 0
        self.episode_rewards = []
        self.episode_states = []
        self.episode_actions = []
        self.episode_start_time = None
        self.episode_number = 0

        # Sensor failure detection
        self.zero_sensor_episode_count = 0
        self.last_nonzero_time = time.time()
        self.failed = False
        self._last_episode_zero = False

    def _init_vel_filt(self):
        self.vel_filt = VelocityFilter(2, dt=self.timing.dt)

    def _update_state(self, action):
        # If failed, always send zero command
        if self.failed:
            action = 0.0
        motor_angle, pendulum_angle, _ = self.robot.step(action)

        # motor_angle: theta, pendulum angle: alpha
        pos = np.array([motor_angle, pendulum_angle], dtype=np.float32)
        vel = self.vel_filt(pos)
        state = np.concatenate([pos, vel])
        self._state = state
        self._last_motor_angle = motor_angle
        self._last_pendulum_angle = pendulum_angle

    def _check_sensor_failure(self):
        # Check for zero readings
        motor_zero = np.isclose(self._last_motor_angle, 0.0, atol=1e-6)
        pendulum_zero = np.isclose(self._last_pendulum_angle, 0.0, atol=1e-6)
        current_zero = motor_zero or pendulum_zero
        now = time.time()
        if current_zero:
            if self._last_episode_zero:
                self.zero_sensor_episode_count += 1
            else:
                self.zero_sensor_episode_count = 1
            if now - self.last_nonzero_time > 60:
                self.failed = True
                self.robot.step(0.0)  # Ensure motor is stopped
                raise SensorFailureException("Sensor stuck at zero for 60 seconds")
        else:
            self.zero_sensor_episode_count = 0
            self.last_nonzero_time = now
        self._last_episode_zero = current_zero
        if self.zero_sensor_episode_count >= 2:
            self.failed = True
            self.robot.step(0.0)
            raise SensorFailureException("Sensor stuck at zero for 2 episodes")

    def _log_episode_step(self, action, reward):
        """Log current step information for episode tracking"""
        if not self.enable_episode_logging:
            return
            
        # Store step data
        self.episode_rewards.append(reward)
        self.episode_states.append(self._state.copy())
        self.episode_actions.append(action)
        
        # Log every N steps
        if self.episode_step % self.log_frequency == 0:
            motor_angle_deg = np.rad2deg(self._state[THETA])
            pendulum_angle_deg = np.rad2deg(self._state[ALPHA])
            motor_speed_rps = self._state[THETA_DOT] / (2 * np.pi)
            pendulum_speed_rps = self._state[ALPHA_DOT] / (2 * np.pi)
            
            logging.info(
                f"Episode {self.episode_number}, Step {self.episode_step}: "
                f"Motor={motor_angle_deg:6.1f}°, Pendulum={pendulum_angle_deg:6.1f}°, "
                f"Motor_speed={motor_speed_rps:5.2f} rev/s, Pendulum_speed={pendulum_speed_rps:5.2f} rev/s, "
                f"Action={action[0]:6.3f}, Reward={reward:6.4f}"
            )
        
        self.episode_step += 1

    def _log_episode_summary(self, terminated, truncated):
        """Log episode summary statistics"""
        if not self.enable_episode_logging or len(self.episode_rewards) == 0:
            return
            
        episode_duration = time.time() - self.episode_start_time
        total_reward = sum(self.episode_rewards)
        mean_reward = np.mean(self.episode_rewards)
        max_reward = np.max(self.episode_rewards)
        min_reward = np.min(self.episode_rewards)
        
        # Calculate state statistics
        states_array = np.array(self.episode_states)
        motor_angles_deg = np.rad2deg(states_array[:, THETA])
        pendulum_angles_deg = np.rad2deg(states_array[:, ALPHA])
        motor_speeds_rps = states_array[:, THETA_DOT] / (2 * np.pi)
        pendulum_speeds_rps = states_array[:, ALPHA_DOT] / (2 * np.pi)
        
        # Check for state violations using state_max from parent class
        angle_limits_violated = False
        speed_limits_violated = False
        
        # Check if any angle limits were violated
        if not np.isinf(self.state_max[THETA]):
            if np.any(np.abs(motor_angles_deg) > np.rad2deg(self.state_max[THETA])):
                angle_limits_violated = True
        if not np.isinf(self.state_max[ALPHA]):
            if np.any(np.abs(pendulum_angles_deg) > np.rad2deg(self.state_max[ALPHA])):
                angle_limits_violated = True
                
        # Check if any speed limits were violated
        if not np.isinf(self.state_max[THETA_DOT]):
            if np.any(np.abs(motor_speeds_rps) > self.state_max[THETA_DOT] / (2 * np.pi)):
                speed_limits_violated = True
        if not np.isinf(self.state_max[ALPHA_DOT]):
            if np.any(np.abs(pendulum_speeds_rps) > self.state_max[ALPHA_DOT] / (2 * np.pi)):
                speed_limits_violated = True
        
        # Log episode summary
        logging.info("=" * 80)
        logging.info(f"EPISODE {self.episode_number} SUMMARY:")
        logging.info(f"  Duration: {episode_duration:.2f}s ({self.episode_step} steps)")
        logging.info(f"  Total Reward: {total_reward:.4f}")
        logging.info(f"  Mean Reward: {mean_reward:.4f}")
        logging.info(f"  Max Reward: {max_reward:.4f}")
        logging.info(f"  Min Reward: {min_reward:.4f}")
        logging.info(f"  Termination: {'State bounds' if terminated else 'Time limit' if truncated else 'Unknown'}")
        
        logging.info(f"  Motor Angle: {np.mean(motor_angles_deg):6.1f}° ± {np.std(motor_angles_deg):5.1f}° "
                    f"[{np.min(motor_angles_deg):6.1f}°, {np.max(motor_angles_deg):6.1f}°]")
        logging.info(f"  Pendulum Angle: {np.mean(pendulum_angles_deg):6.1f}° ± {np.std(pendulum_angles_deg):5.1f}° "
                    f"[{np.min(pendulum_angles_deg):6.1f}°, {np.max(pendulum_angles_deg):6.1f}°]")
        logging.info(f"  Motor Speed: {np.mean(motor_speeds_rps):5.2f} ± {np.std(motor_speeds_rps):5.2f} rev/s "
                    f"[{np.min(motor_speeds_rps):5.2f}, {np.max(motor_speeds_rps):5.2f}]")
        logging.info(f"  Pendulum Speed: {np.mean(pendulum_speeds_rps):5.2f} ± {np.std(pendulum_speeds_rps):5.2f} rev/s "
                    f"[{np.min(pendulum_speeds_rps):5.2f}, {np.max(pendulum_speeds_rps):5.2f}]")
        
        if angle_limits_violated:
            logging.warning("  ⚠️  ANGLE LIMITS VIOLATED!")
        if speed_limits_violated:
            logging.warning("  ⚠️  SPEED LIMITS VIOLATED!")
            
        logging.info("=" * 80)

    def step(self, action):
        """Override step method to add logging"""
        # If failed, always send zero command and return done
        if self.failed:
            self._update_state(0.0)
            obs = self.get_obs()
            reward = 0.0
            terminated = True
            truncated = False
            return obs, reward, terminated, truncated, {"failure": True}
        # Get reward and observation from parent
        reward = self._reward_func(self._state)
        obs = self.get_obs()
        
        # Update state
        self._update_state(action[0])
        
        # Check for sensor failure
        self._check_sensor_failure()

        # Check termination
        terminated = not self.state_space.contains(self._state)
        truncated = False
        
        # Log step information
        self._log_episode_step(action, reward)
        
        return obs, reward, terminated, truncated, {}

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        # Log previous episode summary if this isn't the first reset
        if self.episode_start_time is not None:
            self._log_episode_summary(terminated=False, truncated=False)
        
        # Start new episode logging
        self.episode_number += 1
        self.episode_step = 0
        self.episode_rewards = []
        self.episode_states = []
        self.episode_actions = []
        self.episode_start_time = time.time()
        
        logging.info(f"Starting Episode {self.episode_number}...")
        
        super().reset(seed=seed)

        if self._state is not None:  # if not first reset
            logging.debug("Stopping motor")
            # motor_pid = PID(
            #     self.motor_stop_pid[0],
            #     self.motor_stop_pid[1],
            #     self.motor_stop_pid[2],
            #     setpoint=0.0,
            #     output_limits=(-1, 1),
            # )

            reset_time = 0
            while abs(self._state[THETA_DOT]) > 0.5 and reset_time < MAX_MOTOR_RESET_TIME:
                # act = motor_pid(self._state[THETA_DOT])
                # self._update_state(act)
                reset_time += self.timing.dt
                sleep(self.timing.dt)

            logging.debug("Waiting for pendulum to fall back down")
            time_under_thresh = 0
            reset_time = 0
            while time_under_thresh < RESET_TIME and reset_time < MAX_RESET_TIME:
                if np.cos(self._state[ALPHA]) > ALPHA_THRESH:
                    time_under_thresh += self.timing.dt
                else:
                    time_under_thresh = 0
                self._update_state(0.0)
                reset_time += self.timing.dt
                sleep(self.timing.dt)

            if reset_time >= MAX_RESET_TIME:
                logging.info(f"Reset timeout, alpha: {np.rad2deg(self._state[ALPHA])}")

        # reset both encoder, motor back to pos=0
        self.robot.reset_encoders()

        logging.info("Reset done")
        # else the first computed velocity will take into account previous episode
        # and it'll be huge and wrong and will terminate the episode
        self._init_vel_filt()
        self._update_state(0.0)  # initial state
        # After reset, check for sensor failure
        self._check_sensor_failure()
        return self.get_obs(), {}

    # TODO: override parent render function
    # replace by taking webcam snapshot
    # and outputing rgb array?
    # but webcam can't record at 100hz
    # or could just use the same render function!!!
    # def render(self, mode='human'):
    #     raise NotImplementedError

    def close(self):
        # Log final episode summary
        if self.episode_start_time is not None:
            self._log_episode_summary(terminated=False, truncated=False)
        
        self.robot.close()
