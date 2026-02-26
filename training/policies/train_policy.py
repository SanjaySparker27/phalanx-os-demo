#!/usr/bin/env python3
"""
Policy Training for Robotics Control
Implements PPO and SAC algorithms for policy optimization.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
import numpy as np
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass
from pathlib import Path
import json


@dataclass
class PolicyConfig:
    """Configuration for policy training."""
    state_dim: int = 256
    action_dim: int = 4
    hidden_dim: int = 256
    num_layers: int = 3
    
    # PPO hyperparameters
    ppo_clip: float = 0.2
    ppo_epochs: int = 10
    value_coef: float = 0.5
    entropy_coef: float = 0.01
    
    # SAC hyperparameters
    tau: float = 0.005
    alpha: float = 0.2
    automatic_entropy_tuning: bool = True
    
    # Common
    gamma: float = 0.99
    gae_lambda: float = 0.95
    learning_rate: float = 3e-4
    batch_size: int = 64
    buffer_size: int = 1000000


class Actor(nn.Module):
    """Policy network."""
    
    def __init__(self, state_dim: int, action_dim: int, 
                 hidden_dim: int = 256, num_layers: int = 3):
        super().__init__()
        
        layers = []
        prev_dim = state_dim
        
        for _ in range(num_layers):
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU()
            ])
            prev_dim = hidden_dim
        
        self.trunk = nn.Sequential(*layers)
        
        # Mean and log std for continuous actions
        self.mean_layer = nn.Linear(hidden_dim, action_dim)
        self.log_std_layer = nn.Linear(hidden_dim, action_dim)
        
        # Action bounds
        self.action_scale = torch.tensor([1.0])
        self.action_bias = torch.tensor([0.0])
    
    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Returns (mean, log_std)."""
        x = self.trunk(state)
        mean = self.mean_layer(x)
        log_std = self.log_std_layer(x)
        log_std = torch.clamp(log_std, -20, 2)
        return mean, log_std
    
    def sample(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Sample action from policy. Returns (action, log_prob)."""
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        dist = Normal(mean, std)
        z = dist.rsample()
        action = torch.tanh(z) * self.action_scale + self.action_bias
        
        log_prob = dist.log_prob(z) - torch.log(
            self.action_scale * (1 - torch.tanh(z).pow(2)) + 1e-6
        )
        log_prob = log_prob.sum(-1, keepdim=True)
        
        return action, log_prob
    
    def get_log_prob(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Get log probability of action."""
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        dist = Normal(mean, std)
        z = torch.atanh((action - self.action_bias) / self.action_scale)
        log_prob = dist.log_prob(z).sum(-1, keepdim=True)
        
        return log_prob


class Critic(nn.Module):
    """Value network for PPO."""
    
    def __init__(self, state_dim: int, hidden_dim: int = 256, num_layers: int = 3):
        super().__init__()
        
        layers = []
        prev_dim = state_dim
        
        for _ in range(num_layers):
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU()
            ])
            prev_dim = hidden_dim
        
        layers.append(nn.Linear(hidden_dim, 1))
        self.network = nn.Sequential(*layers)
    
    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.network(state)


class QNetwork(nn.Module):
    """Q-network for SAC."""
    
    def __init__(self, state_dim: int, action_dim: int, 
                 hidden_dim: int = 256, num_layers: int = 3):
        super().__init__()
        
        layers = []
        prev_dim = state_dim + action_dim
        
        for _ in range(num_layers):
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU()
            ])
            prev_dim = hidden_dim
        
        layers.append(nn.Linear(hidden_dim, 1))
        self.network = nn.Sequential(*layers)
    
    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        x = torch.cat([state, action], dim=-1)
        return self.network(x)


class ReplayBuffer:
    """Experience replay buffer."""
    
    def __init__(self, state_dim: int, action_dim: int, 
                 capacity: int = 1000000):
        self.capacity = capacity
        self.ptr = 0
        self.size = 0
        
        self.states = np.zeros((capacity, state_dim), dtype=np.float32)
        self.actions = np.zeros((capacity, action_dim), dtype=np.float32)
        self.rewards = np.zeros((capacity, 1), dtype=np.float32)
        self.next_states = np.zeros((capacity, state_dim), dtype=np.float32)
        self.dones = np.zeros((capacity, 1), dtype=np.float32)
    
    def add(self, state, action, reward, next_state, done):
        self.states[self.ptr] = state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.next_states[self.ptr] = next_state
        self.dones[self.ptr] = done
        
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)
    
    def sample(self, batch_size: int) -> Dict:
        indices = np.random.randint(0, self.size, size=batch_size)
        
        return {
            'states': torch.FloatTensor(self.states[indices]),
            'actions': torch.FloatTensor(self.actions[indices]),
            'rewards': torch.FloatTensor(self.rewards[indices]),
            'next_states': torch.FloatTensor(self.next_states[indices]),
            'dones': torch.FloatTensor(self.dones[indices])
        }


class PPOTrainer:
    """PPO policy trainer."""
    
    def __init__(self, config: PolicyConfig):
        self.config = config
        
        self.actor = Actor(config.state_dim, config.action_dim, 
                          config.hidden_dim, config.num_layers)
        self.critic = Critic(config.state_dim, config.hidden_dim, config.num_layers)
        
        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=config.learning_rate
        )
        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(), lr=config.learning_rate
        )
        
        self.trajectory_buffer = []
    
    def compute_gae(self, rewards: List, values: List, dones: List) -> Tuple[List, List]:
        """Compute Generalized Advantage Estimation."""
        advantages = []
        gae = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            
            delta = rewards[t] + self.config.gamma * next_value * (1 - dones[t]) - values[t]
            gae = delta + self.config.gamma * self.config.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
        returns = [adv + val for adv, val in zip(advantages, values)]
        return advantages, returns
    
    def update(self, batch: Dict) -> Dict:
        """Update policy using PPO."""
        states = batch['states']
        actions = batch['actions']
        old_log_probs = batch['log_probs']
        advantages = batch['advantages']
        returns = batch['returns']
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        losses = {'actor': [], 'critic': [], 'entropy': []}
        
        for _ in range(self.config.ppo_epochs):
            # Compute new log probs and values
            new_log_probs = self.actor.get_log_prob(states, actions)
            values = self.critic(states)
            
            # PPO loss
            ratio = torch.exp(new_log_probs - old_log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.config.ppo_clip, 
                               1 + self.config.ppo_clip) * advantages
            actor_loss = -torch.min(surr1, surr2).mean()
            
            # Value loss
            critic_loss = F.mse_loss(values, returns)
            
            # Entropy bonus
            mean, log_std = self.actor.forward(states)
            std = log_std.exp()
            dist = Normal(mean, std)
            entropy = dist.entropy().mean()
            
            # Total loss
            total_loss = (
                actor_loss + 
                self.config.value_coef * critic_loss - 
                self.config.entropy_coef * entropy
            )
            
            # Update
            self.actor_optimizer.zero_grad()
            self.critic_optimizer.zero_grad()
            total_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 0.5)
            torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 0.5)
            self.actor_optimizer.step()
            self.critic_optimizer.step()
            
            losses['actor'].append(actor_loss.item())
            losses['critic'].append(critic_loss.item())
            losses['entropy'].append(entropy.item())
        
        return {k: np.mean(v) for k, v in losses.items()}


class SACTrainer:
    """Soft Actor-Critic trainer."""
    
    def __init__(self, config: PolicyConfig):
        self.config = config
        
        # Actor
        self.actor = Actor(config.state_dim, config.action_dim,
                          config.hidden_dim, config.num_layers)
        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=config.learning_rate
        )
        
        # Critics (twin Q-networks)
        self.critic1 = QNetwork(config.state_dim, config.action_dim,
                               config.hidden_dim, config.num_layers)
        self.critic2 = QNetwork(config.state_dim, config.action_dim,
                               config.hidden_dim, config.num_layers)
        
        # Target critics
        self.critic1_target = QNetwork(config.state_dim, config.action_dim,
                                      config.hidden_dim, config.num_layers)
        self.critic2_target = QNetwork(config.state_dim, config.action_dim,
                                      config.hidden_dim, config.num_layers)
        
        # Copy parameters
        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic2_target.load_state_dict(self.critic2.state_dict())
        
        self.critic1_optimizer = torch.optim.Adam(
            self.critic1.parameters(), lr=config.learning_rate
        )
        self.critic2_optimizer = torch.optim.Adam(
            self.critic2.parameters(), lr=config.learning_rate
        )
        
        # Temperature parameter
        if config.automatic_entropy_tuning:
            self.target_entropy = -config.action_dim
            self.log_alpha = torch.zeros(1, requires_grad=True)
            self.alpha_optimizer = torch.optim.Adam([self.log_alpha], lr=config.learning_rate)
            self.alpha = self.log_alpha.exp()
        else:
            self.alpha = config.alpha
    
    def update_critics(self, batch: Dict) -> float:
        """Update critic networks."""
        states = batch['states']
        actions = batch['actions']
        rewards = batch['rewards']
        next_states = batch['next_states']
        dones = batch['dones']
        
        with torch.no_grad():
            # Sample actions from current policy
            next_actions, next_log_probs = self.actor.sample(next_states)
            
            # Compute target Q-values
            q1_next = self.critic1_target(next_states, next_actions)
            q2_next = self.critic2_target(next_states, next_actions)
            q_next = torch.min(q1_next, q2_next) - self.alpha * next_log_probs
            
            q_target = rewards + self.config.gamma * (1 - dones) * q_next
        
        # Update Q-functions
        q1 = self.critic1(states, actions)
        q2 = self.critic2(states, actions)
        
        critic1_loss = F.mse_loss(q1, q_target)
        critic2_loss = F.mse_loss(q2, q_target)
        
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()
        
        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()
        
        return critic1_loss.item() + critic2_loss.item()
    
    def update_actor(self, batch: Dict) -> Tuple[float, float]:
        """Update actor network."""
        states = batch['states']
        
        actions, log_probs = self.actor.sample(states)
        
        q1 = self.critic1(states, actions)
        q2 = self.critic2(states, actions)
        q = torch.min(q1, q2)
        
        actor_loss = (self.alpha * log_probs - q).mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        # Update temperature
        if self.config.automatic_entropy_tuning:
            alpha_loss = -(self.log_alpha * (log_probs + self.target_entropy).detach()).mean()
            
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            
            self.alpha = self.log_alpha.exp()
            return actor_loss.item(), alpha_loss.item()
        
        return actor_loss.item(), 0.0
    
    def soft_update(self):
        """Soft update target networks."""
        for param, target_param in zip(
            self.critic1.parameters(), self.critic1_target.parameters()
        ):
            target_param.data.copy_(
                self.config.tau * param.data + (1 - self.config.tau) * target_param.data
            )
        
        for param, target_param in zip(
            self.critic2.parameters(), self.critic2_target.parameters()
        ):
            target_param.data.copy_(
                self.config.tau * param.data + (1 - self.config.tau) * target_param.data
            )


class PolicyTrainer:
    """Main policy training interface."""
    
    def __init__(self, algorithm: str = 'ppo', config: Optional[PolicyConfig] = None):
        self.algorithm = algorithm.lower()
        self.config = config or PolicyConfig()
        
        if self.algorithm == 'ppo':
            self.trainer = PPOTrainer(self.config)
        elif self.algorithm == 'sac':
            self.trainer = SACTrainer(self.config)
            self.replay_buffer = ReplayBuffer(
                self.config.state_dim, self.config.action_dim, self.config.buffer_size
            )
        else:
            raise ValueError(f"Unknown algorithm: {algorithm}")
    
    def select_action(self, state: np.ndarray, deterministic: bool = False) -> np.ndarray:
        """Select action from policy."""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        
        if deterministic:
            with torch.no_grad():
                mean, _ = self.trainer.actor.forward(state_tensor)
                action = torch.tanh(mean)
        else:
            with torch.no_grad():
                action, _ = self.trainer.actor.sample(state_tensor)
        
        return action.squeeze(0).cpu().numpy()
    
    def train_step(self, batch: Optional[Dict] = None) -> Dict:
        """Single training step."""
        if self.algorithm == 'ppo':
            return self.trainer.update(batch)
        elif self.algorithm == 'sac':
            if batch is None:
                return {}
            critic_loss = self.trainer.update_critics(batch)
            actor_loss, alpha_loss = self.trainer.update_actor(batch)
            self.trainer.soft_update()
            return {
                'critic_loss': critic_loss,
                'actor_loss': actor_loss,
                'alpha_loss': alpha_loss
            }
    
    def save(self, path: str):
        """Save trained policy."""
        save_dict = {
            'algorithm': self.algorithm,
            'config': self.config,
            'actor_state_dict': self.trainer.actor.state_dict()
        }
        
        if self.algorithm == 'ppo':
            save_dict['critic_state_dict'] = self.trainer.critic.state_dict()
        elif self.algorithm == 'sac':
            save_dict['critic1_state_dict'] = self.trainer.critic1.state_dict()
            save_dict['critic2_state_dict'] = self.trainer.critic2.state_dict()
        
        torch.save(save_dict, path)
    
    def load(self, path: str):
        """Load trained policy."""
        checkpoint = torch.load(path)
        
        self.trainer.actor.load_state_dict(checkpoint['actor_state_dict'])
        
        if self.algorithm == 'ppo' and 'critic_state_dict' in checkpoint:
            self.trainer.critic.load_state_dict(checkpoint['critic_state_dict'])
        elif self.algorithm == 'sac':
            self.trainer.critic1.load_state_dict(checkpoint['critic1_state_dict'])
            self.trainer.critic2.load_state_dict(checkpoint['critic2_state_dict'])


def train_policy(
    env_factory,
    algorithm: str = 'ppo',
    num_iterations: int = 1000,
    steps_per_iter: int = 2048,
    save_dir: str = "training/policies/checkpoints"
) -> PolicyTrainer:
    """
    Train policy using specified algorithm.
    
    Args:
        env_factory: Function that creates environment
        algorithm: 'ppo' or 'sac'
        num_iterations: Number of training iterations
        steps_per_iter: Steps per iteration
        save_dir: Directory to save checkpoints
    
    Returns:
        Trained PolicyTrainer
    """
    config = PolicyConfig()
    trainer = PolicyTrainer(algorithm, config)
    
    print(f"Training {algorithm.upper()} policy...")
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    
    for iteration in range(num_iterations):
        # Collect trajectory
        states, actions, rewards, log_probs, values, dones = [], [], [], [], [], []
        
        env = env_factory()
        state = env.reset()
        
        for step in range(steps_per_iter):
            # Select action
            state_tensor = torch.FloatTensor(state)
            with torch.no_grad():
                if algorithm == 'ppo':
                    action, log_prob = trainer.trainer.actor.sample(state_tensor.unsqueeze(0))
                    value = trainer.trainer.critic(state_tensor.unsqueeze(0))
                    action = action.squeeze(0).numpy()
                    log_prob = log_prob.item()
                    value = value.item()
                else:
                    action = trainer.select_action(state)
                    log_prob = 0
                    value = 0
            
            # Step environment
            next_state, reward, done, _ = env.step(action)
            
            # Store transition
            states.append(state)
            actions.append(action)
            rewards.append(reward)
            log_probs.append(log_prob)
            values.append(value)
            dones.append(done)
            
            if algorithm == 'sac':
                trainer.replay_buffer.add(state, action, reward, next_state, float(done))
            
            state = next_state
            
            if done:
                state = env.reset()
        
        # Update policy
        if algorithm == 'ppo':
            # Compute advantages
            advantages, returns = trainer.trainer.compute_gae(rewards, values, dones)
            
            # Create batch
            batch = {
                'states': torch.FloatTensor(states),
                'actions': torch.FloatTensor(actions),
                'log_probs': torch.FloatTensor(log_probs).unsqueeze(-1),
                'advantages': torch.FloatTensor(advantages).unsqueeze(-1),
                'returns': torch.FloatTensor(returns).unsqueeze(-1)
            }
            
            losses = trainer.train_step(batch)
        else:  # SAC
            if trainer.replay_buffer.size > trainer.config.batch_size:
                batch = trainer.replay_buffer.sample(trainer.config.batch_size)
                losses = trainer.train_step(batch)
            else:
                losses = {}
        
        # Logging
        avg_reward = np.mean(rewards)
        print(f"Iter {iteration+1}/{num_iterations}: Reward={avg_reward:.2f}, Losses={losses}")
        
        # Save checkpoint
        if (iteration + 1) % 100 == 0:
            checkpoint_path = Path(save_dir) / f"policy_{algorithm}_iter_{iteration+1}.pt"
            trainer.save(str(checkpoint_path))
    
    # Save final policy
    trainer.save(str(Path(save_dir) / f"policy_{algorithm}_final.pt"))
    print(f"Policy saved to {save_dir}")
    
    return trainer


if __name__ == '__main__':
    print("Policy Training Module")
    print("Import and use train_policy() function with your environment")
