#!/usr/bin/env python3
"""
World Model Training for Robotics Simulation
Implements PlaNet-style recurrent state-space model for environment dynamics.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import numpy as np
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass
from pathlib import Path
import json


@dataclass
class WorldModelConfig:
    """Configuration for world model."""
    state_dim: int = 256
    action_dim: int = 4  # [throttle, roll, pitch, yaw]
    observation_dim: int = 64  # Latent observation size
    hidden_dim: int = 256
    num_layers: int = 2
    learning_rate: float = 1e-4
    batch_size: int = 32
    sequence_length: int = 50
    kl_weight: float = 1.0
    reward_weight: float = 1.0


class ObservationEncoder(nn.Module):
    """Encode observations into latent space."""
    
    def __init__(self, observation_shape: Tuple[int, ...], latent_dim: int):
        super().__init__()
        
        self.observation_shape = observation_shape
        
        # Convolutional encoder for image observations
        if len(observation_shape) == 3:  # Image (C, H, W)
            self.encoder = nn.Sequential(
                nn.Conv2d(observation_shape[0], 32, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(32, 64, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(64, 128, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(128, 256, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.Flatten(),
            )
            
            # Calculate flattened size
            with torch.no_grad():
                dummy = torch.zeros(1, *observation_shape)
                flat_size = self.encoder(dummy).shape[1]
            
            self.fc = nn.Linear(flat_size, latent_dim * 2)  # mean and logvar
        else:
            # MLP encoder for vector observations
            self.encoder = nn.Sequential(
                nn.Linear(np.prod(observation_shape), 256),
                nn.ReLU(),
                nn.Linear(256, 256),
                nn.ReLU(),
                nn.Linear(256, latent_dim * 2)
            )
    
    def forward(self, obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Returns (mean, logvar) of latent distribution."""
        x = self.encoder(obs)
        params = self.fc(x) if hasattr(self, 'fc') else x
        mean, logvar = torch.chunk(params, 2, dim=-1)
        return mean, logvar


class ObservationDecoder(nn.Module):
    """Decode latent state back to observation."""
    
    def __init__(self, latent_dim: int, observation_shape: Tuple[int, ...]):
        super().__init__()
        
        self.observation_shape = observation_shape
        
        if len(observation_shape) == 3:
            # Calculate initial size
            h, w = observation_shape[1] // 16, observation_shape[2] // 16
            self.fc = nn.Linear(latent_dim, 256 * h * w)
            self.h, self.w = h, w
            
            self.decoder = nn.Sequential(
                nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),
                nn.ReLU(),
                nn.ConvTranspose2d(32, observation_shape[0], 4, stride=2, padding=1),
            )
        else:
            self.decoder = nn.Sequential(
                nn.Linear(latent_dim, 256),
                nn.ReLU(),
                nn.Linear(256, 256),
                nn.ReLU(),
                nn.Linear(256, np.prod(observation_shape))
            )
    
    def forward(self, latent: torch.Tensor) -> torch.Tensor:
        """Returns reconstructed observation."""
        if len(self.observation_shape) == 3:
            x = self.fc(latent)
            x = x.view(-1, 256, self.h, self.w)
            x = self.decoder(x)
        else:
            x = self.decoder(latent)
            x = x.view(-1, *self.observation_shape)
        return x


class RecurrentStateSpaceModel(nn.Module):
    """RSSM for world modeling."""
    
    def __init__(self, config: WorldModelConfig):
        super().__init__()
        self.config = config
        
        # State components
        self.state_dim = config.state_dim
        self.action_dim = config.action_dim
        
        # Recurrent model (GRU)
        self.gru = nn.GRUCell(
            config.observation_dim + config.action_dim,
            config.state_dim
        )
        
        # Prior p(s_t | s_{t-1}, a_{t-1})
        self.prior = nn.Sequential(
            nn.Linear(config.state_dim, config.hidden_dim),
            nn.ReLU(),
            nn.Linear(config.hidden_dim, config.observation_dim * 2)
        )
        
        # Posterior q(s_t | s_{t-1}, a_{t-1}, o_t)
        self.posterior = nn.Sequential(
            nn.Linear(config.state_dim + config.observation_dim, config.hidden_dim),
            nn.ReLU(),
            nn.Linear(config.hidden_dim, config.observation_dim * 2)
        )
        
        # Reward predictor
        self.reward_model = nn.Sequential(
            nn.Linear(config.state_dim, config.hidden_dim),
            nn.ReLU(),
            nn.Linear(config.hidden_dim, 1)
        )
        
        # Done predictor
        self.done_model = nn.Sequential(
            nn.Linear(config.state_dim, config.hidden_dim),
            nn.ReLU(),
            nn.Linear(config.hidden_dim, 1)
        )
    
    def init_state(self, batch_size: int, device: torch.device) -> torch.Tensor:
        """Initialize hidden state."""
        return torch.zeros(batch_size, self.state_dim, device=device)
    
    def forward(self, prev_state: torch.Tensor, action: torch.Tensor, 
                obs_embed: Optional[torch.Tensor] = None) -> Dict:
        """One step of RSSM."""
        # Recurrent update
        rnn_input = torch.cat([prev_state, action], dim=-1)
        h = self.gru(rnn_input, prev_state)
        
        # Prior
        prior_params = self.prior(h)
        prior_mean, prior_logvar = torch.chunk(prior_params, 2, dim=-1)
        prior_std = torch.exp(0.5 * prior_logvar)
        
        # Posterior (if observation available)
        if obs_embed is not None:
            post_input = torch.cat([h, obs_embed], dim=-1)
            post_params = self.posterior(post_input)
            post_mean, post_logvar = torch.chunk(post_params, 2, dim=-1)
            post_std = torch.exp(0.5 * post_logvar)
            
            # Sample state
            z = post_mean + post_std * torch.randn_like(post_std)
            
            state = h + z
            return {
                'state': state,
                'h': h,
                'z': z,
                'prior_mean': prior_mean,
                'prior_std': prior_std,
                'post_mean': post_mean,
                'post_std': post_std
            }
        else:
            # Use prior for imagination
            z = prior_mean + prior_std * torch.randn_like(prior_std)
            state = h + z
            return {
                'state': state,
                'h': h,
                'z': z,
                'prior_mean': prior_mean,
                'prior_std': prior_std
            }
    
    def predict_reward(self, state: torch.Tensor) -> torch.Tensor:
        """Predict reward from state."""
        return self.reward_model(state)
    
    def predict_done(self, state: torch.Tensor) -> torch.Tensor:
        """Predict episode termination."""
        return torch.sigmoid(self.done_model(state))


class WorldModel(nn.Module):
    """Complete world model with encoder, RSSM, and decoder."""
    
    def __init__(self, config: WorldModelConfig, observation_shape: Tuple[int, ...]):
        super().__init__()
        self.config = config
        self.observation_shape = observation_shape
        
        self.encoder = ObservationEncoder(observation_shape, config.observation_dim)
        self.decoder = ObservationDecoder(config.state_dim, observation_shape)
        self.rssm = RecurrentStateSpaceModel(config)
    
    def forward(self, observations: torch.Tensor, actions: torch.Tensor) -> Dict:
        """
        Forward pass through world model.
        observations: (batch, seq_len, *obs_shape)
        actions: (batch, seq_len, action_dim)
        """
        batch_size, seq_len = observations.shape[:2]
        device = observations.device
        
        # Initialize state
        state = self.rssm.init_state(batch_size, device)
        
        # Lists to store outputs
        states = []
        prior_means = []
        prior_stds = []
        post_means = []
        post_stds = []
        
        for t in range(seq_len):
            # Encode observation
            obs_t = observations[:, t]
            obs_mean, obs_logvar = self.encoder(obs_t)
            obs_embed = obs_mean + torch.exp(0.5 * obs_logvar) * torch.randn_like(obs_mean)
            
            # RSSM step
            result = self.rssm.forward(state, actions[:, t], obs_embed)
            state = result['state']
            
            states.append(state)
            prior_means.append(result['prior_mean'])
            prior_stds.append(result['prior_std'])
            post_means.append(result['post_mean'])
            post_stds.append(result['post_std'])
        
        states = torch.stack(states, dim=1)
        prior_means = torch.stack(prior_means, dim=1)
        prior_stds = torch.stack(prior_stds, dim=1)
        post_means = torch.stack(post_means, dim=1)
        post_stds = torch.stack(post_stds, dim=1)
        
        # Decode observations
        obs_recon = self.decoder(states.reshape(-1, self.config.state_dim))
        obs_recon = obs_recon.view(batch_size, seq_len, *self.observation_shape)
        
        # Predict rewards
        rewards = self.rssm.predict_reward(states.reshape(-1, self.config.state_dim))
        rewards = rewards.view(batch_size, seq_len)
        
        # Predict dones
        dones = self.rssm.predict_done(states.reshape(-1, self.config.state_dim))
        dones = dones.view(batch_size, seq_len)
        
        return {
            'obs_recon': obs_recon,
            'states': states,
            'rewards': rewards,
            'dones': dones,
            'prior_means': prior_means,
            'prior_stds': prior_stds,
            'post_means': post_means,
            'post_stds': post_stds
        }
    
    def imagine_trajectory(self, initial_state: torch.Tensor, 
                          policy: nn.Module, horizon: int) -> Dict:
        """Imagine future trajectory using learned policy."""
        batch_size = initial_state.shape[0]
        device = initial_state.device
        
        states = [initial_state]
        actions = []
        rewards = []
        
        state = initial_state
        
        for _ in range(horizon):
            # Sample action from policy
            with torch.no_grad():
                action = policy(state)
            
            # Imagine next state
            result = self.rssm.forward(state, action, obs_embed=None)
            state = result['state']
            
            # Predict reward
            reward = self.rssm.predict_reward(state)
            
            states.append(state)
            actions.append(action)
            rewards.append(reward)
        
        return {
            'states': torch.stack(states[1:], dim=1),
            'actions': torch.stack(actions, dim=1),
            'rewards': torch.stack(rewards, dim=1).squeeze(-1)
        }


class ExperienceDataset(Dataset):
    """Dataset for training world model."""
    
    def __init__(self, trajectories: List[Dict], sequence_length: int):
        self.trajectories = trajectories
        self.sequence_length = sequence_length
        
        # Create sequences
        self.sequences = []
        for traj in trajectories:
            length = len(traj['observations'])
            for start in range(0, length - sequence_length):
                self.sequences.append((traj, start))
    
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        traj, start = self.sequences[idx]
        end = start + self.sequence_length
        
        return {
            'observations': torch.FloatTensor(traj['observations'][start:end]),
            'actions': torch.FloatTensor(traj['actions'][start:end]),
            'rewards': torch.FloatTensor(traj['rewards'][start:end]),
            'dones': torch.FloatTensor(traj['dones'][start:end])
        }


class WorldModelTrainer:
    """Trainer for world models."""
    
    def __init__(self, config: WorldModelConfig, observation_shape: Tuple[int, ...]):
        self.config = config
        self.model = WorldModel(config, observation_shape)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=config.learning_rate)
        
    def compute_loss(self, predictions: Dict, targets: Dict) -> Dict:
        """Compute training losses."""
        # Reconstruction loss
        recon_loss = F.mse_loss(predictions['obs_recon'], targets['observations'])
        
        # KL divergence loss
        prior_mean = predictions['prior_means']
        prior_std = predictions['prior_stds']
        post_mean = predictions['post_means']
        post_std = predictions['post_stds']
        
        kl_loss = torch.mean(
            torch.log(prior_std / post_std) +
            (post_std**2 + (post_mean - prior_mean)**2) / (2 * prior_std**2) -
            0.5
        )
        
        # Reward prediction loss
        reward_loss = F.mse_loss(predictions['rewards'], targets['rewards'])
        
        # Total loss
        total_loss = recon_loss + self.config.kl_weight * kl_loss + self.config.reward_weight * reward_loss
        
        return {
            'total': total_loss,
            'reconstruction': recon_loss.item(),
            'kl': kl_loss.item(),
            'reward': reward_loss.item()
        }
    
    def train_step(self, batch: Dict) -> Dict:
        """Single training step."""
        self.optimizer.zero_grad()
        
        predictions = self.model(batch['observations'], batch['actions'])
        losses = self.compute_loss(predictions, batch)
        
        losses['total'].backward()
        torch.nn.utils.clip_grad_norm_(self.model.parameters(), 100.0)
        self.optimizer.step()
        
        return {k: v.item() if isinstance(v, torch.Tensor) else v for k, v in losses.items()}
    
    def train_epoch(self, dataloader: DataLoader) -> Dict:
        """Train for one epoch."""
        self.model.train()
        total_losses = {'total': 0, 'reconstruction': 0, 'kl': 0, 'reward': 0}
        
        for batch in dataloader:
            losses = self.train_step(batch)
            for key in total_losses:
                total_losses[key] += losses[key]
        
        n_batches = len(dataloader)
        return {k: v / n_batches for k, v in total_losses.items()}
    
    def save_checkpoint(self, path: str):
        """Save model checkpoint."""
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'config': self.config
        }, path)
    
    def load_checkpoint(self, path: str):
        """Load model checkpoint."""
        checkpoint = torch.load(path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])


def train_world_model(
    trajectories: List[Dict],
    observation_shape: Tuple[int, ...],
    config: Optional[WorldModelConfig] = None,
    num_epochs: int = 100,
    save_dir: str = "training/world_models/checkpoints"
) -> WorldModelTrainer:
    """
    Train world model from collected trajectories.
    
    Args:
        trajectories: List of trajectory dictionaries with keys:
            - observations: np.array (T, *obs_shape)
            - actions: np.array (T, action_dim)
            - rewards: np.array (T,)
            - dones: np.array (T,)
        observation_shape: Shape of observations
        config: World model configuration
        num_epochs: Number of training epochs
        save_dir: Directory to save checkpoints
    
    Returns:
        Trained WorldModelTrainer
    """
    if config is None:
        config = WorldModelConfig()
    
    # Create dataset
    dataset = ExperienceDataset(trajectories, config.sequence_length)
    dataloader = DataLoader(dataset, batch_size=config.batch_size, shuffle=True)
    
    # Create trainer
    trainer = WorldModelTrainer(config, observation_shape)
    
    # Training loop
    print("Training World Model...")
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    
    for epoch in range(num_epochs):
        losses = trainer.train_epoch(dataloader)
        
        print(f"Epoch {epoch+1}/{num_epochs}: "
              f"Total={losses['total']:.4f}, "
              f"Recon={losses['reconstruction']:.4f}, "
              f"KL={losses['kl']:.4f}, "
              f"Reward={losses['reward']:.4f}")
        
        # Save checkpoint
        if (epoch + 1) % 10 == 0:
            checkpoint_path = Path(save_dir) / f"world_model_epoch_{epoch+1}.pt"
            trainer.save_checkpoint(str(checkpoint_path))
    
    # Save final model
    trainer.save_checkpoint(str(Path(save_dir) / "world_model_final.pt"))
    print(f"Model saved to {save_dir}")
    
    return trainer


if __name__ == '__main__':
    # Example usage
    print("World Model Training Module")
    print("Import and use train_world_model() function with your data")
