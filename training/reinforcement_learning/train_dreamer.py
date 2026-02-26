#!/usr/bin/env python3
"""
Dreamer-style Reinforcement Learning
Combines world models with policy learning for sample-efficient RL.
"""

import torch
import torch.nn as nn
import numpy as np
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass
from pathlib import Path
import sys

# Import world model and policy modules
sys.path.insert(0, str(Path(__file__).parent.parent))
from world_models.train_world_model import (
    WorldModel, WorldModelConfig, WorldModelTrainer, ExperienceDataset
)
from policies.train_policy import (
    Actor, Critic, PolicyConfig, PolicyTrainer
)


@dataclass
class DreamerConfig:
    """Configuration for Dreamer training."""
    # World model
    world_model_config: WorldModelConfig = None
    
    # Policy
    policy_config: PolicyConfig = None
    
    # Training
    batch_size: int = 32
    sequence_length: int = 50
    imagine_horizon: int = 15
    
    # Discount
    discount: float = 0.99
    lambda_: float = 0.95
    
    # KL regularization
    kl_balance: float = 0.8
    
    # Training schedule
    pretrain_steps: int = 100
    train_steps: int = 1000
    
    def __post_init__(self):
        if self.world_model_config is None:
            self.world_model_config = WorldModelConfig()
        if self.policy_config is None:
            self.policy_config = PolicyConfig()


class DreamerAgent:
    """
    Dreamer agent that learns from imagined trajectories.
    """
    
    def __init__(self, config: DreamerConfig, observation_shape: Tuple[int, ...]):
        self.config = config
        self.observation_shape = observation_shape
        
        # World model
        self.world_model = WorldModel(config.world_model_config, observation_shape)
        
        # Policy (actor)
        self.actor = Actor(
            config.world_model_config.state_dim,
            config.policy_config.action_dim,
            config.policy_config.hidden_dim,
            config.policy_config.num_layers
        )
        
        # Value function (critic)
        self.critic = Critic(
            config.world_model_config.state_dim,
            config.policy_config.hidden_dim,
            config.policy_config.num_layers
        )
        
        # Optimizers
        self.world_model_optimizer = torch.optim.Adam(
            self.world_model.parameters(),
            lr=config.world_model_config.learning_rate
        )
        
        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(),
            lr=config.policy_config.learning_rate
        )
        
        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(),
            lr=config.policy_config.learning_rate
        )
    
    def imagine_trajectory(self, initial_state: torch.Tensor, 
                          horizon: int) -> Dict[str, torch.Tensor]:
        """
        Imagine trajectory using world model.
        
        Returns dict with keys: states, actions, rewards, values, discounts
        """
        states = [initial_state]
        actions = []
        rewards = []
        values = []
        
        state = initial_state
        batch_size = state.shape[0]
        
        for t in range(horizon):
            # Sample action from policy
            with torch.no_grad():
                action, _ = self.actor.sample(state)
            
            # Predict next state using world model
            with torch.no_grad():
                result = self.world_model.rssm.forward(state, action, obs_embed=None)
                next_state = result['state']
                reward = self.world_model.rssm.predict_reward(next_state)
                value = self.critic(next_state)
            
            states.append(next_state)
            actions.append(action)
            rewards.append(reward)
            values.append(value)
            
            state = next_state
        
        # Stack into tensors
        states = torch.stack(states[:-1], dim=1)  # Exclude final state
        actions = torch.stack(actions, dim=1)
        rewards = torch.stack(rewards, dim=1).squeeze(-1)
        values = torch.stack(values, dim=1).squeeze(-1)
        
        # Compute discounts
        discounts = torch.ones_like(rewards) * self.config.discount
        
        return {
            'states': states,
            'actions': actions,
            'rewards': rewards,
            'values': values,
            'discounts': discounts
        }
    
    def compute_target_values(self, rewards: torch.Tensor, 
                             values: torch.Tensor,
                             discounts: torch.Tensor) -> torch.Tensor:
        """Compute lambda-returns for value targets."""
        horizon = rewards.shape[1]
        targets = []
        
        for t in range(horizon):
            # Compute n-step return
            ret = 0
            discount = 1
            
            for n in range(t, horizon):
                ret += discount * rewards[:, n]
                discount *= self.config.discount
                
                if n < horizon - 1:
                    ret += discount * values[:, n + 1]
            
            targets.append(ret)
        
        return torch.stack(targets, dim=1)
    
    def update_world_model(self, batch: Dict) -> Dict[str, float]:
        """Update world model."""
        observations = batch['observations']
        actions = batch['actions']
        rewards = batch['rewards']
        dones = batch['dones']
        
        # Forward pass
        predictions = self.world_model(observations, actions)
        
        # Compute losses
        # Reconstruction loss
        recon_loss = nn.functional.mse_loss(
            predictions['obs_recon'], observations
        )
        
        # KL loss with balancing
        prior_mean = predictions['prior_means']
        prior_std = predictions['prior_stds']
        post_mean = predictions['post_means']
        post_std = predictions['post_stds']
        
        # KL(posterior || prior)
        kl_loss = torch.mean(
            torch.log(prior_std / post_std) +
            (post_std**2 + (post_mean - prior_mean)**2) / (2 * prior_std**2) -
            0.5
        )
        
        # Reward prediction loss
        reward_loss = nn.functional.mse_loss(predictions['rewards'], rewards)
        
        # Total loss
        total_loss = (
            recon_loss + 
            self.config.world_model_config.kl_weight * kl_loss +
            self.config.world_model_config.reward_weight * reward_loss
        )
        
        # Optimize
        self.world_model_optimizer.zero_grad()
        total_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.world_model.parameters(), 100.0)
        self.world_model_optimizer.step()
        
        return {
            'world_model_total': total_loss.item(),
            'reconstruction': recon_loss.item(),
            'kl': kl_loss.item(),
            'reward_pred': reward_loss.item()
        }
    
    def update_actor(self, imagined_trajectory: Dict) -> float:
        """Update policy using imagined trajectories."""
        states = imagined_trajectory['states']
        actions = imagined_trajectory['actions']
        rewards = imagined_trajectory['rewards']
        values = imagined_trajectory['values']
        discounts = imagined_trajectory['discounts']
        
        # Compute lambda-returns
        targets = self.compute_target_values(rewards, values, discounts)
        
        # Get log probabilities and new values
        batch_size, horizon = states.shape[:2]
        states_flat = states.reshape(-1, states.shape[-1])
        actions_flat = actions.reshape(-1, actions.shape[-1])
        
        # Resample actions for current policy
        new_actions, log_probs = self.actor.sample(states_flat)
        new_actions = new_actions.view(batch_size, horizon, -1)
        log_probs = log_probs.view(batch_size, horizon)
        
        # Get new state values
        with torch.no_grad():
            for t in range(horizon):
                # Re-imagine with new actions
                if t == 0:
                    result = self.world_model.rssm.forward(
                        states[:, 0], new_actions[:, 0], obs_embed=None
                    )
                    new_state = result['state']
                else:
                    result = self.world_model.rssm.forward(
                        new_state, new_actions[:, t], obs_embed=None
                    )
                    new_state = result['state']
                
                if t == 0:
                    new_states = [new_state]
                else:
                    new_states.append(new_state)
        
        new_states = torch.stack(new_states, dim=1)
        new_values = self.critic(new_states.reshape(-1, new_states.shape[-1]))
        new_values = new_values.view(batch_size, horizon)
        
        # Advantage
        advantages = (targets - new_values).detach()
        
        # Policy loss (PPO-style)
        actor_loss = -(log_probs * advantages).mean()
        
        # Entropy bonus
        mean, log_std = self.actor.forward(states_flat)
        std = log_std.exp()
        dist = torch.distributions.Normal(mean, std)
        entropy = dist.entropy().mean()
        
        actor_loss = actor_loss - 0.001 * entropy
        
        # Optimize
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        return actor_loss.item()
    
    def update_critic(self, imagined_trajectory: Dict) -> float:
        """Update value function using imagined trajectories."""
        states = imagined_trajectory['states']
        rewards = imagined_trajectory['rewards']
        values = imagined_trajectory['values']
        discounts = imagined_trajectory['discounts']
        
        # Compute lambda-returns as targets
        targets = self.compute_target_values(rewards, values, discounts)
        
        # Get current value estimates
        batch_size, horizon = states.shape[:2]
        states_flat = states.reshape(-1, states.shape[-1])
        predicted_values = self.critic(states_flat).view(batch_size, horizon)
        
        # Value loss
        value_loss = nn.functional.mse_loss(predicted_values, targets.detach())
        
        # Optimize
        self.critic_optimizer.zero_grad()
        value_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 100.0)
        self.critic_optimizer.step()
        
        return value_loss.item()
    
    def train_step(self, real_batch: Dict) -> Dict[str, float]:
        """Single training step."""
        # Update world model on real data
        world_model_losses = self.update_world_model(real_batch)
        
        # Get initial states from real batch
        with torch.no_grad():
            observations = real_batch['observations']
            batch_size = observations.shape[0]
            
            # Encode initial observations
            initial_states = []
            for b in range(batch_size):
                obs_t = observations[b, 0]
                obs_mean, _ = self.world_model.encoder(obs_t.unsqueeze(0))
                
                # Initialize RSSM state
                h = self.world_model.rssm.init_state(1, observations.device)
                
                # First step with observation
                result = self.world_model.rssm.forward(h, 
                    torch.zeros(1, self.config.policy_config.action_dim, device=observations.device),
                    obs_mean
                )
                initial_states.append(result['state'])
            
            initial_states = torch.cat(initial_states, dim=0)
        
        # Imagine trajectories
        imagined = self.imagine_trajectory(initial_states, self.config.imagine_horizon)
        
        # Update actor and critic on imagined data
        actor_loss = self.update_actor(imagined)
        critic_loss = self.update_critic(imagined)
        
        # Combine losses
        losses = world_model_losses
        losses['actor'] = actor_loss
        losses['critic'] = critic_loss
        
        return losses
    
    def save(self, path: str):
        """Save agent."""
        torch.save({
            'world_model': self.world_model.state_dict(),
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'config': self.config
        }, path)
    
    def load(self, path: str):
        """Load agent."""
        checkpoint = torch.load(path)
        self.world_model.load_state_dict(checkpoint['world_model'])
        self.actor.load_state_dict(checkpoint['actor'])
        self.critic.load_state_dict(checkpoint['critic'])


class DreamerTrainer:
    """Trainer for Dreamer agent."""
    
    def __init__(self, config: DreamerConfig, observation_shape: Tuple[int, ...]):
        self.config = config
        self.agent = DreamerAgent(config, observation_shape)
        
    def train(self, trajectories: List[Dict], num_steps: int = 1000) -> List[Dict]:
        """Train Dreamer agent on collected trajectories."""
        # Create dataset
        from torch.utils.data import DataLoader
        dataset = ExperienceDataset(trajectories, self.config.sequence_length)
        dataloader = DataLoader(
            dataset, 
            batch_size=self.config.batch_size,
            shuffle=True,
            drop_last=True
        )
        
        history = []
        
        print("Training Dreamer agent...")
        for step in range(num_steps):
            # Get batch
            try:
                batch = next(iter(dataloader))
            except StopIteration:
                dataloader = DataLoader(dataset, batch_size=self.config.batch_size, shuffle=True)
                batch = next(iter(dataloader))
            
            # Train step
            losses = self.agent.train_step(batch)
            history.append(losses)
            
            if (step + 1) % 100 == 0:
                print(f"Step {step+1}/{num_steps}: {losses}")
        
        return history
    
    def save(self, path: str):
        """Save trained agent."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.agent.save(path)


def train_dreamer(
    trajectories: List[Dict],
    observation_shape: Tuple[int, ...],
    config: Optional[DreamerConfig] = None,
    num_steps: int = 1000,
    save_dir: str = "training/reinforcement_learning/checkpoints"
) -> DreamerTrainer:
    """
    Train Dreamer agent from collected trajectories.
    
    Args:
        trajectories: List of trajectory dictionaries
        observation_shape: Shape of observations
        config: Dreamer configuration
        num_steps: Number of training steps
        save_dir: Directory to save checkpoints
    
    Returns:
        Trained DreamerTrainer
    """
    if config is None:
        config = DreamerConfig()
    
    trainer = DreamerTrainer(config, observation_shape)
    
    # Train
    history = trainer.train(trajectories, num_steps)
    
    # Save
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    trainer.save(str(Path(save_dir) / "dreamer_agent.pt"))
    
    print(f"Agent saved to {save_dir}")
    
    return trainer


if __name__ == '__main__':
    print("Dreamer RL Training Module")
    print("Import and use train_dreamer() function with your data")
