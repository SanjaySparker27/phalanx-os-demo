# Training Framework for Robotics

Neural network training pipeline for world models, policies, and reinforcement learning.

## Directory Structure

```
training/
├── world_models/              # World model training (Dreamer-style)
│   └── train_world_model.py
├── policies/                  # Policy training (PPO/SAC)
│   └── train_policy.py
├── reinforcement_learning/    # RL agents (Dreamer)
│   └── train_dreamer.py
├── supervised_learning/       # Imitation learning
│   └── train_supervised.py
├── utils/                     # Training utilities
│   └── data_collection.py
├── configs/                   # Configuration files
│   └── default_config.yaml
└── train.py                   # Main training script
```

## Quick Start

### Train All Components
```bash
python training/train.py --mode all
```

### Train Specific Component
```bash
# World model only
python training/train.py --mode world_model

# RL policy only
python training/train.py --mode policy --algorithm ppo

# Dreamer agent
python training/train.py --mode dreamer

# Supervised learning
python training/train.py --mode supervised
```

### Use Custom Config
```bash
python training/train.py --config my_config.yaml
```

## World Models

Implements PlaNet-style Recurrent State-Space Model (RSSM):

### Features
- **Encoder**: CNN for images, MLP for vectors
- **RSSM**: Recurrent dynamics model with deterministic and stochastic states
- **Decoder**: Reconstructs observations from latent state
- **Reward Predictor**: Predicts future rewards

### Training
```python
from training.world_models.train_world_model import train_world_model

trainer = train_world_model(
    trajectories=trajectory_list,
    observation_shape=(64, 64, 3),
    num_epochs=100
)
```

## Policy Training

### PPO (Proximal Policy Optimization)
- On-policy actor-critic
- GAE for advantage estimation
- Clipped surrogate objective

### SAC (Soft Actor-Critic)
- Off-policy maximum entropy
- Twin Q-networks
- Automatic temperature tuning

### Training
```python
from training.policies.train_policy import train_policy

trainer = train_policy(
    env_factory=make_env,
    algorithm='ppo',  # or 'sac'
    num_iterations=1000
)
```

## Dreamer Agent

End-to-end model-based RL combining world model + policy:

### Algorithm
1. Train world model on real trajectories
2. Imagine future trajectories
3. Update policy on imagined data
4. Update value function

### Training
```python
from training.reinforcement_learning.train_dreamer import train_dreamer

trainer = train_dreamer(
    trajectories=trajectory_list,
    observation_shape=(64,),
    num_steps=1000
)
```

## Supervised Learning

### Behavioral Cloning
- Learn from expert demonstrations
- MSE loss on action predictions
- Optional LSTM for temporal modeling

### Training
```python
from training.supervised_learning.train_supervised import train_behavioral_cloning

trainer = train_behavioral_cloning(
    demonstrations=demo_list,
    use_sequences=True
)
```

## Data Collection

### From Simulation
```python
from training.utils.data_collection import DataCollector

collector = DataCollector(env_factory)
trajectories = collector.collect_episodes(
    num_episodes=100,
    policy=None  # Random policy
)
collector.save_trajectories("data/trajectories.pkl.gz")
```

### From ROS Topics
```python
from training.utils.data_collection import ROSDataCollector

collector = ROSDataCollector()
collector.start_recording()
# ... run robot ...
data = collector.stop_recording()
```

### Synthetic Data
```python
from training.utils.data_collection import create_synthetic_dataset

trajectories = create_synthetic_dataset(
    num_episodes=100,
    observation_dim=64,
    action_dim=4
)
```

## Configuration

### Default Config (`configs/default_config.yaml`)

```yaml
world_model:
  state_dim: 256
  action_dim: 4
  hidden_dim: 256
  learning_rate: 0.0001
  batch_size: 32
  sequence_length: 50

policy:
  hidden_dim: 256
  ppo_clip: 0.2
  gamma: 0.99
  learning_rate: 0.0003

dreamer:
  imagine_horizon: 15
  discount: 0.99
  kl_balance: 0.8

training:
  num_iterations: 1000
  steps_per_iteration: 2048
```

## Model Checkpoints

Checkpoints saved to:
- `training/world_models/checkpoints/`
- `training/policies/checkpoints/`
- `training/reinforcement_learning/checkpoints/`
- `training/supervised_learning/checkpoints/`

### Loading Checkpoints
```python
# World model
trainer.load_checkpoint("path/to/checkpoint.pt")

# Policy
trainer.load("path/to/policy.pt")

# Dreamer
trainer.agent.load("path/to/agent.pt")
```

## Integration with Testing

Use trained models in simulation tests:

```python
from training.policies.train_policy import PolicyTrainer
from tests.scenarios.test_waypoint import WaypointNavigator

# Load trained policy
trainer = PolicyTrainer('ppo')
trainer.load("policies/checkpoints/ppo/policy_ppo_final.pt")

# Use in test
navigator = WaypointNavigator('uav')
# Override action selection with trained policy
```

## Requirements

- PyTorch 2.0+
- NumPy
- ROS2 (for data collection)
- Gazebo (for simulation)

## Performance Tips

1. **GPU Acceleration**: Set `device: cuda` in config
2. **Batch Size**: Increase for larger GPUs
3. **Sequence Length**: 50 good for most tasks
4. **Pretraining**: Pretrain world model before RL
5. **Data Augmentation**: Add noise to observations
