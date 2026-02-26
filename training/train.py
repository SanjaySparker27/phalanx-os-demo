#!/usr/bin/env python3
"""
Main Training Script for Robotics
Unified interface for training world models, policies, and RL agents.
"""

import argparse
import yaml
import torch
from pathlib import Path
import sys
from typing import Dict, Optional

# Add paths
sys.path.insert(0, str(Path(__file__).parent))

from utils.data_collection import DataCollector, create_synthetic_dataset
from world_models.train_world_model import train_world_model
from policies.train_policy import train_policy
from reinforcement_learning.train_dreamer import train_dreamer
from supervised_learning.train_supervised import train_behavioral_cloning


def load_config(config_path: str) -> Dict:
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def collect_data(args, config: Dict):
    """Collect training data."""
    print("\n" + "="*60)
    print("DATA COLLECTION")
    print("="*60)
    
    # For now, create synthetic data
    # In production, this would interface with simulation or real hardware
    data_config = config['data']
    
    trajectories = create_synthetic_dataset(
        num_episodes=data_config['num_episodes'],
        episode_length=data_config['max_episode_length'],
        observation_dim=config['world_model']['observation_dim'],
        action_dim=config['world_model']['action_dim'],
        save_path="training/data/trajectories.pkl.gz"
    )
    
    return trajectories


def train_world_model_phase(trajectories, config: Dict, args):
    """Train world model."""
    print("\n" + "="*60)
    print("TRAINING WORLD MODEL")
    print("="*60)
    
    from world_models.train_world_model import WorldModelConfig
    
    wm_config = WorldModelConfig(**config['world_model'])
    
    observation_shape = (config['world_model']['observation_dim'],)
    
    trainer = train_world_model(
        trajectories=trajectories,
        observation_shape=observation_shape,
        config=wm_config,
        num_epochs=args.world_model_epochs,
        save_dir="training/world_models/checkpoints"
    )
    
    return trainer


def train_policy_phase(trajectories, config: Dict, args):
    """Train policy using RL."""
    print("\n" + "="*60)
    print(f"TRAINING POLICY ({args.algorithm.upper()})")
    print("="*60)
    
    # Create dummy environment factory
    def env_factory():
        # Would return actual environment
        class DummyEnv:
            def __init__(self):
                self.observation_space = type('obj', (object,), {
                    'shape': (config['world_model']['observation_dim'],)
                })()
                self.action_space = type('obj', (object,), {
                    'sample': lambda: torch.randn(config['world_model']['action_dim'])
                })()
            
            def reset(self):
                return torch.randn(config['world_model']['observation_dim'])
            
            def step(self, action):
                obs = torch.randn(config['world_model']['observation_dim'])
                reward = torch.randn(1).item()
                done = torch.rand(1).item() < 0.01
                return obs, reward, done, {}
        
        return DummyEnv()
    
    trainer = train_policy(
        env_factory=env_factory,
        algorithm=args.algorithm,
        num_iterations=config['training']['num_iterations'],
        steps_per_iter=config['training']['steps_per_iteration'],
        save_dir=f"training/policies/checkpoints/{args.algorithm}"
    )
    
    return trainer


def train_dreamer_phase(trajectories, config: Dict, args):
    """Train Dreamer agent."""
    print("\n" + "="*60)
    print("TRAINING DREAMER AGENT")
    print("="*60)
    
    from reinforcement_learning.train_dreamer import DreamerConfig
    
    dreamer_config = DreamerConfig(**config['dreamer'])
    
    observation_shape = (config['world_model']['observation_dim'],)
    
    trainer = train_dreamer(
        trajectories=trajectories,
        observation_shape=observation_shape,
        config=dreamer_config,
        num_steps=args.dreamer_steps,
        save_dir="training/reinforcement_learning/checkpoints"
    )
    
    return trainer


def train_supervised_phase(trajectories, config: Dict, args):
    """Train supervised policy."""
    print("\n" + "="*60)
    print("TRAINING SUPERVISED POLICY")
    print("="*60)
    
    from supervised_learning.train_supervised import SupervisedConfig
    
    supervised_config = SupervisedConfig(**config['supervised'])
    
    trainer = train_behavioral_cloning(
        demonstrations=trajectories,
        config=supervised_config,
        use_sequences=args.use_sequences,
        val_split=0.1,
        save_dir="training/supervised_learning/checkpoints"
    )
    
    return trainer


def main():
    parser = argparse.ArgumentParser(description='Train robotics models')
    
    # Config
    parser.add_argument('--config', '-c', type=str, 
                       default='training/configs/default_config.yaml',
                       help='Path to config file')
    
    # Training mode
    parser.add_argument('--mode', '-m', type=str,
                       choices=['world_model', 'policy', 'dreamer', 'supervised', 'all'],
                       default='all',
                       help='Training mode')
    
    # Algorithm selection
    parser.add_argument('--algorithm', '-a', type=str,
                       choices=['ppo', 'sac'],
                       default='ppo',
                       help='RL algorithm for policy training')
    
    # Data
    parser.add_argument('--data', '-d', type=str,
                       help='Path to pre-collected trajectories')
    
    # Training parameters
    parser.add_argument('--world-model-epochs', type=int, default=100,
                       help='Epochs for world model training')
    parser.add_argument('--dreamer-steps', type=int, default=1000,
                       help='Steps for Dreamer training')
    parser.add_argument('--use-sequences', action='store_true',
                       help='Use sequence model for supervised learning')
    
    # Device
    parser.add_argument('--device', type=str, default='auto',
                       choices=['auto', 'cuda', 'cpu'],
                       help='Device for training')
    
    args = parser.parse_args()
    
    # Load config
    config = load_config(args.config)
    
    # Set device
    if args.device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    else:
        device = args.device
    
    print(f"Using device: {device}")
    
    # Collect or load data
    if args.data:
        print(f"Loading data from {args.data}")
        # Load trajectories
        from utils.data_collection import DataCollector
        trajectories = DataCollector.load_trajectories(args.data)
    else:
        trajectories = collect_data(args, config)
    
    # Run training phases
    if args.mode == 'world_model' or args.mode == 'all':
        train_world_model_phase(trajectories, config, args)
    
    if args.mode == 'policy' or args.mode == 'all':
        train_policy_phase(trajectories, config, args)
    
    if args.mode == 'dreamer' or args.mode == 'all':
        train_dreamer_phase(trajectories, config, args)
    
    if args.mode == 'supervised' or args.mode == 'all':
        train_supervised_phase(trajectories, config, args)
    
    print("\n" + "="*60)
    print("TRAINING COMPLETE")
    print("="*60)


if __name__ == '__main__':
    main()
