"""
Mally Multi-Agent System for Autonomous Vehicles

A Python-based multi-agent system for UAV/USV/UGV with:
- Perception Agent: YOLO/ONNX-based object detection
- Navigation Agent: ORB-SLAM3 GPS-denied navigation
- Planning Agent: Model Predictive Control
- Communication Agent: MAVLink 2.0 and SATCOM
- Swarm Orchestrator: Distributed consensus with Raft

Example:
    from mally import MallySystem
    
    system = MallySystem()
    await system.initialize()
    await system.run()
"""

__version__ = "0.1.0"
__author__ = "Mally Team"

from mally.main import MallySystem
from mally.core.base_agent import BaseAgent, AgentMessage, MessageBus
from mally.utils.config import AgentConfig

__all__ = [
    "MallySystem",
    "BaseAgent",
    "AgentMessage",
    "MessageBus",
    "AgentConfig",
]
