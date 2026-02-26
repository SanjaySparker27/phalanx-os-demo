#!/usr/bin/env python3
"""
Supervised Learning for Robotics
Train models for imitation learning and behavior cloning.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import numpy as np
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass
from pathlib import Path
import json


@dataclass
class SupervisedConfig:
    """Configuration for supervised training."""
    input_dim: int = 64
    output_dim: int = 4
    hidden_dim: int = 256
    num_layers: int = 3
    dropout: float = 0.2
    learning_rate: float = 1e-3
    batch_size: int = 32
    num_epochs: int = 100
    sequence_length: int = 10


class BehavioralCloningModel(nn.Module):
    """Behavioral cloning policy network."""
    
    def __init__(self, config: SupervisedConfig):
        super().__init__()
        
        layers = []
        prev_dim = config.input_dim
        
        for i in range(config.num_layers):
            layers.extend([
                nn.Linear(prev_dim, config.hidden_dim),
                nn.ReLU(),
                nn.Dropout(config.dropout) if config.dropout > 0 else nn.Identity()
            ])
            prev_dim = config.hidden_dim
        
        self.encoder = nn.Sequential(*layers)
        self.output_layer = nn.Linear(config.hidden_dim, config.output_dim)
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass."""
        features = self.encoder(x)
        output = self.output_layer(features)
        return torch.tanh(output)  # Normalize to [-1, 1]


class SequenceModel(nn.Module):
    """LSTM-based sequence model for temporal predictions."""
    
    def __init__(self, config: SupervisedConfig):
        super().__init__()
        
        self.lstm = nn.LSTM(
            input_size=config.input_dim,
            hidden_size=config.hidden_dim,
            num_layers=config.num_layers,
            dropout=config.dropout if config.num_layers > 1 else 0,
            batch_first=True
        )
        
        self.output_layer = nn.Linear(config.hidden_dim, config.output_dim)
    
    def forward(self, x: torch.Tensor, hidden=None) -> Tuple[torch.Tensor, Tuple]:
        """Forward pass through sequence."""
        lstm_out, hidden = self.lstm(x, hidden)
        output = self.output_layer(lstm_out)
        return torch.tanh(output), hidden


class ExpertDataset(Dataset):
    """Dataset of expert demonstrations."""
    
    def __init__(self, demonstrations: List[Dict], use_sequences: bool = False, 
                 seq_length: int = 10):
        self.demonstrations = demonstrations
        self.use_sequences = use_sequences
        self.seq_length = seq_length
        
        # Flatten demonstrations
        self.states = []
        self.actions = []
        
        for demo in demonstrations:
            states = demo['states']
            actions = demo['actions']
            
            if use_sequences and len(states) >= seq_length:
                for i in range(len(states) - seq_length + 1):
                    self.states.append(states[i:i+seq_length])
                    self.actions.append(actions[i:i+seq_length])
            else:
                self.states.extend(states)
                self.actions.extend(actions)
    
    def __len__(self):
        return len(self.states)
    
    def __getitem__(self, idx):
        return {
            'state': torch.FloatTensor(self.states[idx]),
            'action': torch.FloatTensor(self.actions[idx])
        }


class SupervisedTrainer:
    """Trainer for supervised learning."""
    
    def __init__(self, model: nn.Module, config: SupervisedConfig):
        self.model = model
        self.config = config
        self.optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
        self.criterion = nn.MSELoss()
    
    def train_epoch(self, dataloader: DataLoader) -> Dict[str, float]:
        """Train for one epoch."""
        self.model.train()
        total_loss = 0
        num_batches = 0
        
        for batch in dataloader:
            states = batch['state']
            actions = batch['action']
            
            # Forward pass
            if isinstance(self.model, SequenceModel):
                predicted_actions, _ = self.model(states)
            else:
                predicted_actions = self.model(states)
            
            # Compute loss
            loss = self.criterion(predicted_actions, actions)
            
            # Backward pass
            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), 1.0)
            self.optimizer.step()
            
            total_loss += loss.item()
            num_batches += 1
        
        return {
            'train_loss': total_loss / num_batches
        }
    
    def validate(self, dataloader: DataLoader) -> Dict[str, float]:
        """Validate model."""
        self.model.eval()
        total_loss = 0
        num_batches = 0
        
        with torch.no_grad():
            for batch in dataloader:
                states = batch['state']
                actions = batch['action']
                
                if isinstance(self.model, SequenceModel):
                    predicted_actions, _ = self.model(states)
                else:
                    predicted_actions = self.model(states)
                
                loss = self.criterion(predicted_actions, actions)
                
                total_loss += loss.item()
                num_batches += 1
        
        return {
            'val_loss': total_loss / num_batches
        }
    
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


def train_behavioral_cloning(
    demonstrations: List[Dict],
    config: Optional[SupervisedConfig] = None,
    use_sequences: bool = False,
    val_split: float = 0.1,
    save_dir: str = "training/supervised_learning/checkpoints"
) -> SupervisedTrainer:
    """
    Train behavioral cloning policy from expert demonstrations.
    
    Args:
        demonstrations: List of expert trajectories
        config: Training configuration
        use_sequences: Whether to use sequence model
        val_split: Validation split ratio
        save_dir: Directory to save checkpoints
    
    Returns:
        Trained SupervisedTrainer
    """
    if config is None:
        config = SupervisedConfig()
    
    # Split data
    n_val = int(len(demonstrations) * val_split)
    train_demos = demonstrations[n_val:]
    val_demos = demonstrations[:n_val]
    
    # Create datasets
    train_dataset = ExpertDataset(train_demos, use_sequences, config.sequence_length)
    val_dataset = ExpertDataset(val_demos, use_sequences, config.sequence_length)
    
    train_loader = DataLoader(train_dataset, batch_size=config.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=config.batch_size)
    
    # Create model
    if use_sequences:
        model = SequenceModel(config)
    else:
        model = BehavioralCloningModel(config)
    
    # Create trainer
    trainer = SupervisedTrainer(model, config)
    
    # Training loop
    print("Training Behavioral Cloning Policy...")
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    
    best_val_loss = float('inf')
    
    for epoch in range(config.num_epochs):
        train_metrics = trainer.train_epoch(train_loader)
        val_metrics = trainer.validate(val_loader)
        
        print(f"Epoch {epoch+1}/{config.num_epochs}: "
              f"Train Loss={train_metrics['train_loss']:.4f}, "
              f"Val Loss={val_metrics['val_loss']:.4f}")
        
        # Save best model
        if val_metrics['val_loss'] < best_val_loss:
            best_val_loss = val_metrics['val_loss']
            trainer.save_checkpoint(str(Path(save_dir) / "bc_policy_best.pt"))
    
    # Save final model
    trainer.save_checkpoint(str(Path(save_dir) / "bc_policy_final.pt"))
    print(f"Model saved to {save_dir}")
    
    return trainer


if __name__ == '__main__':
    print("Supervised Learning Module")
    print("Import and use train_behavioral_cloning() with expert demonstrations")
