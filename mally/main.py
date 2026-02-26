"""
Mally Multi-Agent System - Main Entry Point

This module initializes and orchestrates the multi-agent system for UAV/USV/UGV.
"""

import asyncio
import signal
import sys
from typing import Dict, List, Optional

from mally.core.base_agent import MessageBus, AgentRegistry
from mally.utils.config import AgentConfig, AsyncLogger, MetricsCollector

from mally.agents.perception.perception_agent import PerceptionAgent
from mally.agents.navigation.navigation_agent import NavigationAgent
from mally.agents.planning.planning_agent import PlanningAgent
from mally.agents.communication.communication_agent import CommunicationAgent
from mally.agents.swarm.swarm_orchestrator import SwarmOrchestrator


class MallySystem:
    """Main Mally multi-agent system orchestrator."""
    
    def __init__(self, config_path: Optional[str] = None):
        self.config = self._load_config(config_path)
        self.logger = AsyncLogger("mally_system", self.config.log_path, self.config.log_level)
        
        # Core components
        self.message_bus = MessageBus()
        self.registry = AgentRegistry()
        self.metrics = MetricsCollector(self.config.metrics_interval)
        
        # Agents
        self.agents: Dict[str, object] = {}
        self._running = False
        self._shutdown_event = asyncio.Event()
        
    def _load_config(self, config_path: Optional[str]) -> AgentConfig:
        """Load configuration from file or use defaults."""
        if config_path:
            if config_path.endswith('.yaml') or config_path.endswith('.yml'):
                return AgentConfig.from_yaml(config_path)
            elif config_path.endswith('.json'):
                return AgentConfig.from_json(config_path)
        return AgentConfig()
    
    async def initialize(self) -> bool:
        """Initialize the Mally system."""
        await self.logger.info("Initializing Mally Multi-Agent System...")
        
        try:
            # Start message bus
            await self.message_bus.start()
            await self.logger.info("Message bus started")
            
            # Start metrics collector
            await self.metrics.start()
            await self.logger.info("Metrics collector started")
            
            # Initialize agents
            await self._init_agents()
            
            # Setup signal handlers
            self._setup_signal_handlers()
            
            self._running = True
            await self.logger.info("Mally system initialized successfully")
            return True
            
        except Exception as e:
            await self.logger.error(f"Initialization failed: {e}")
            return False
    
    async def _init_agents(self):
        """Initialize all agents."""
        # Perception Agent
        perception = PerceptionAgent(
            "perception_001",
            self.config.perception,
            self.message_bus
        )
        if await perception.initialize():
            self.agents["perception"] = perception
            await self.registry.register(perception)
            await self.logger.info("Perception Agent initialized")
        
        # Navigation Agent
        navigation = NavigationAgent(
            "navigation_001",
            self.config.navigation,
            self.message_bus
        )
        if await navigation.initialize():
            self.agents["navigation"] = navigation
            await self.registry.register(navigation)
            await self.logger.info("Navigation Agent initialized")
        
        # Planning Agent
        planning = PlanningAgent(
            "planning_001",
            self.config.planning,
            self.message_bus
        )
        if await planning.initialize():
            self.agents["planning"] = planning
            await self.registry.register(planning)
            await self.logger.info("Planning Agent initialized")
        
        # Communication Agent
        communication = CommunicationAgent(
            "communication_001",
            self.config.communication,
            self.message_bus
        )
        if await communication.initialize():
            self.agents["communication"] = communication
            await self.registry.register(communication)
            await self.logger.info("Communication Agent initialized")
        
        # Swarm Orchestrator
        swarm = SwarmOrchestrator(
            "swarm_001",
            self.config.swarm,
            self.message_bus
        )
        if await swarm.initialize():
            self.agents["swarm"] = swarm
            await self.registry.register(swarm)
            await self.logger.info("Swarm Orchestrator initialized")
    
    def _setup_signal_handlers(self):
        """Setup graceful shutdown handlers."""
        def signal_handler(sig, frame):
            asyncio.create_task(self.shutdown())
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    
    async def run(self):
        """Main run loop."""
        await self.logger.info("Mally system running")
        
        try:
            # Wait for shutdown signal
            await self._shutdown_event.wait()
            
        except asyncio.CancelledError:
            await self.logger.info("Run loop cancelled")
        
        except Exception as e:
            await self.logger.error(f"Run error: {e}")
        
        finally:
            await self.shutdown()
    
    async def shutdown(self):
        """Gracefully shutdown the system."""
        if not self._running:
            return
        
        self._running = False
        await self.logger.info("Shutting down Mally system...")
        
        # Shutdown agents in reverse order
        for name, agent in reversed(list(self.agents.items())):
            try:
                await agent.shutdown()
                await self.logger.info(f"{name} agent shutdown")
            except Exception as e:
                await self.logger.error(f"Error shutting down {name}: {e}")
        
        # Stop metrics collector
        await self.metrics.stop()
        
        # Stop message bus
        await self.message_bus.stop()
        
        await self.logger.info("Mally system shutdown complete")
        self._shutdown_event.set()
    
    async def get_system_status(self) -> Dict:
        """Get complete system status."""
        return {
            "running": self._running,
            "agents": await self.registry.get_all_status(),
            "metrics": await self.metrics.get_metrics()
        }
    
    async def send_command(self, target: str, command: str, params: Dict) -> bool:
        """Send command to specific agent."""
        agent = await self.registry.get_agent(target)
        if agent:
            from mally.core.base_agent import AgentMessage
            message = AgentMessage(
                source="system",
                target=target,
                message_type=command,
                payload=params
            )
            await self.message_bus.publish(message)
            return True
        return False


async def main():
    """Main entry point."""
    # Parse arguments
    import argparse
    parser = argparse.ArgumentParser(description="Mally Multi-Agent System")
    parser.add_argument("--config", "-c", help="Configuration file path")
    parser.add_argument("--debug", "-d", action="store_true", help="Enable debug logging")
    args = parser.parse_args()
    
    # Create and initialize system
    system = MallySystem(args.config)
    
    if args.debug:
        system.config.log_level = "DEBUG"
    
    if await system.initialize():
        # Run system
        await system.run()
    else:
        print("Failed to initialize Mally system")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
