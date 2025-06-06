# tests/test_polimi_env.py
import hydra
from omegaconf import DictConfig
from stable_baselines3.common.env_checker import check_env

@hydra.main(version_base="1.3", config_path="configs", config_name="experiment/polimi.yaml")
def test_env(cfg: DictConfig):
    # Create environment
    env = hydra.utils.instantiate(cfg.env, _recursive_=True)
    
    # Check if environment follows gym interface
    check_env(env)
    
    # Test basic environment functionality
    print("Testing environment...")
    
    # Reset
    obs, info = env.reset()
    print(f"Initial observation: {obs}")
    
    # Take some random actions
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"\nStep {i+1}:")
        print(f"Action: {action}")
        print(f"Observation: {obs}")
        print(f"Reward: {reward}")
        print(f"Terminated: {terminated}")
        print(f"Truncated: {truncated}")
        
        if terminated or truncated:
            print("Episode ended, resetting...")
            obs, info = env.reset()
    
    # Cleanup
    env.close()
    print("\nEnvironment test completed successfully!")

if __name__ == "__main__":
    test_env()