#!/usr/bin/env python3
"""
Data Collection Utilities for Training
Collect trajectories from simulation or real hardware.
"""

import numpy as np
import pickle
import gzip
from typing import Dict, List, Tuple, Optional, Callable
from pathlib import Path
import json
from dataclasses import dataclass, asdict
import time


@dataclass
class Trajectory:
    """Single trajectory/episodes."""
    observations: List[np.ndarray]
    actions: List[np.ndarray]
    rewards: List[float]
    dones: List[bool]
    infos: List[Dict]
    
    def __len__(self):
        return len(self.observations)
    
    def to_dict(self) -> Dict:
        return {
            'observations': np.array(self.observations),
            'actions': np.array(self.actions),
            'rewards': np.array(self.rewards),
            'dones': np.array(self.dones),
            'infos': self.infos
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Trajectory':
        return cls(
            observations=list(data['observations']),
            actions=list(data['actions']),
            rewards=list(data['rewards']),
            dones=list(data['dones']),
            infos=data.get('infos', [])
        )


class DataCollector:
    """Collect trajectories from environment."""
    
    def __init__(self, env_factory: Callable):
        self.env_factory = env_factory
        self.trajectories: List[Trajectory] = []
    
    def collect_episode(self, policy: Optional[Callable] = None, 
                       max_steps: int = 1000,
                       render: bool = False) -> Trajectory:
        """Collect single episode."""
        env = self.env_factory()
        obs = env.reset()
        
        observations = [obs]
        actions = []
        rewards = []
        dones = []
        infos = []
        
        done = False
        step = 0
        
        while not done and step < max_steps:
            # Select action
            if policy is None:
                action = env.action_space.sample()
            else:
                action = policy(obs)
            
            # Step environment
            next_obs, reward, done, info = env.step(action)
            
            if render:
                env.render()
            
            # Store transition
            observations.append(next_obs)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            infos.append(info)
            
            obs = next_obs
            step += 1
        
        trajectory = Trajectory(
            observations=observations[:-1],  # Exclude final obs
            actions=actions,
            rewards=rewards,
            dones=dones,
            infos=infos
        )
        
        self.trajectories.append(trajectory)
        return trajectory
    
    def collect_episodes(self, num_episodes: int, 
                        policy: Optional[Callable] = None,
                        max_steps: int = 1000) -> List[Trajectory]:
        """Collect multiple episodes."""
        print(f"Collecting {num_episodes} episodes...")
        
        for i in range(num_episodes):
            traj = self.collect_episode(policy, max_steps)
            print(f"  Episode {i+1}: {len(traj)} steps, reward={sum(traj.rewards):.2f}")
        
        return self.trajectories
    
    def save_trajectories(self, path: str, compress: bool = True):
        """Save collected trajectories."""
        data = [traj.to_dict() for traj in self.trajectories]
        
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        if compress:
            with gzip.open(str(path) + '.gz', 'wb') as f:
                pickle.dump(data, f)
        else:
            with open(path, 'wb') as f:
                pickle.dump(data, f)
        
        print(f"Saved {len(self.trajectories)} trajectories to {path}")
    
    @staticmethod
    def load_trajectories(path: str) -> List[Dict]:
        """Load trajectories from file."""
        path = Path(path)
        
        if str(path).endswith('.gz'):
            with gzip.open(path, 'rb') as f:
                data = pickle.load(f)
        else:
            with open(path, 'rb') as f:
                data = pickle.load(f)
        
        return data
    
    def get_statistics(self) -> Dict:
        """Get statistics about collected data."""
        if not self.trajectories:
            return {}
        
        lengths = [len(traj) for traj in self.trajectories]
        total_rewards = [sum(traj.rewards) for traj in self.trajectories]
        
        return {
            'num_episodes': len(self.trajectories),
            'total_steps': sum(lengths),
            'avg_episode_length': np.mean(lengths),
            'avg_reward': np.mean(total_rewards),
            'max_reward': np.max(total_rewards),
            'min_reward': np.min(total_rewards)
        }


class ROSDataCollector:
    """Collect data from ROS topics."""
    
    def __init__(self):
        self.buffer = []
        self.recording = False
    
    def start_recording(self):
        """Start recording data."""
        self.recording = True
        self.buffer = []
        print("Started recording...")
    
    def stop_recording(self) -> List[Dict]:
        """Stop recording and return data."""
        self.recording = False
        print(f"Stopped recording. Collected {len(self.buffer)} samples.")
        return self.buffer
    
    def add_sample(self, sample: Dict):
        """Add sample to buffer."""
        if self.recording:
            sample['timestamp'] = time.time()
            self.buffer.append(sample)
    
    def save_recording(self, path: str):
        """Save recorded data."""
        with open(path, 'w') as f:
            json.dump(self.buffer, f)
        print(f"Saved recording to {path}")


def create_synthetic_dataset(
    num_episodes: int = 100,
    episode_length: int = 100,
    observation_dim: int = 64,
    action_dim: int = 4,
    save_path: Optional[str] = None
) -> List[Dict]:
    """
    Create synthetic dataset for testing training pipeline.
    
    Args:
        num_episodes: Number of episodes to generate
        episode_length: Length of each episode
        observation_dim: Dimension of observations
        action_dim: Dimension of actions
        save_path: Optional path to save dataset
    
    Returns:
        List of trajectory dictionaries
    """
    trajectories = []
    
    for ep in range(num_episodes):
        observations = []
        actions = []
        rewards = []
        dones = []
        
        obs = np.random.randn(observation_dim)
        
        for t in range(episode_length):
            # Random walk observations
            obs = obs + 0.1 * np.random.randn(observation_dim)
            observations.append(obs.copy())
            
            # Random actions
            action = np.random.randn(action_dim)
            actions.append(action)
            
            # Reward based on some arbitrary function
            reward = -np.sum(obs**2) + np.random.randn() * 0.1
            rewards.append(reward)
            
            # Done on last step
            dones.append(t == episode_length - 1)
        
        trajectories.append({
            'observations': np.array(observations),
            'actions': np.array(actions),
            'rewards': np.array(rewards),
            'dones': np.array(dones)
        })
    
    print(f"Created synthetic dataset: {num_episodes} episodes, "
          f"{num_episodes * episode_length} total steps")
    
    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(save_path, 'wb') as f:
            pickle.dump(trajectories, f)
        print(f"Saved to {save_path}")
    
    return trajectories


if __name__ == '__main__':
    # Example: Create synthetic dataset
    trajectories = create_synthetic_dataset(
        num_episodes=100,
        save_path="training/data/synthetic_trajectories.pkl.gz"
    )
