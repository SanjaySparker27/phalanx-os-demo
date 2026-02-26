"""
Base Agent Classes for Mally Multi-Agent System
Provides abstract base classes and message infrastructure for autonomous agents.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Callable, Set
import asyncio
import uuid
import logging


class AgentState(Enum):
    """Agent lifecycle states."""
    IDLE = auto()
    INITIALIZING = auto()
    RUNNING = auto()
    PAUSED = auto()
    ERROR = auto()
    SHUTTING_DOWN = auto()
    SHUTDOWN = auto()


class MessagePriority(Enum):
    """Message priority levels."""
    CRITICAL = 0
    HIGH = 1
    NORMAL = 2
    LOW = 3
    BACKGROUND = 4


@dataclass
class AgentMessage:
    """Inter-agent message structure."""
    message_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = field(default_factory=datetime.utcnow)
    source: str = ""
    target: str = ""  # Empty string = broadcast
    message_type: str = ""
    payload: Dict[str, Any] = field(default_factory=dict)
    priority: MessagePriority = MessagePriority.NORMAL
    ttl: float = 30.0  # Time to live in seconds
    
    def is_expired(self) -> bool:
        """Check if message has expired."""
        age = (datetime.utcnow() - self.timestamp).total_seconds()
        return age > self.ttl


class MessageBus:
    """Asynchronous message bus for inter-agent communication."""
    
    def __init__(self):
        self._subscribers: Dict[str, Set[Callable]] = {}
        self._global_subscribers: Set[Callable] = set()
        self._queue: asyncio.PriorityQueue = asyncio.PriorityQueue()
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()
        
    async def start(self):
        """Start the message bus."""
        self._running = True
        self._task = asyncio.create_task(self._process_loop())
        
    async def stop(self):
        """Stop the message bus."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
                
    async def subscribe(self, agent_id: str, callback: Callable[[AgentMessage], None]):
        """Subscribe an agent to receive messages."""
        async with self._lock:
            if agent_id not in self._subscribers:
                self._subscribers[agent_id] = set()
            self._subscribers[agent_id].add(callback)
            
    async def unsubscribe(self, agent_id: str, callback: Callable[[AgentMessage], None]):
        """Unsubscribe an agent."""
        async with self._lock:
            if agent_id in self._subscribers:
                self._subscribers[agent_id].discard(callback)
                
    async def subscribe_broadcast(self, callback: Callable[[AgentMessage], None]):
        """Subscribe to broadcast messages."""
        async with self._lock:
            self._global_subscribers.add(callback)
            
    async def publish(self, message: AgentMessage):
        """Publish a message to the bus."""
        priority = message.priority.value
        await self._queue.put((priority, message.timestamp, message))
        
    async def _process_loop(self):
        """Main processing loop."""
        while self._running:
            try:
                _, _, message = await asyncio.wait_for(
                    self._queue.get(), timeout=1.0
                )
                
                if message.is_expired():
                    continue
                    
                await self._route_message(message)
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logging.error(f"Message bus error: {e}")
                
    async def _route_message(self, message: AgentMessage):
        """Route message to appropriate subscribers."""
        callbacks = []
        
        async with self._lock:
            # Direct target routing
            if message.target and message.target in self._subscribers:
                callbacks.extend(self._subscribers[message.target])
            
            # Broadcast routing
            if not message.target:
                callbacks.extend(self._global_subscribers)
                for subs in self._subscribers.values():
                    callbacks.extend(subs)
                    
        # Execute callbacks outside lock
        for callback in callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    asyncio.create_task(callback(message))
                else:
                    callback(message)
            except Exception as e:
                logging.error(f"Callback error: {e}")


class BaseAgent(ABC):
    """Abstract base class for all Mally agents."""
    
    def __init__(self, agent_id: str, agent_type: str, message_bus: MessageBus):
        self.agent_id = agent_id
        self.agent_type = agent_type
        self._state = AgentState.IDLE
        self._message_bus = message_bus
        self._lock = asyncio.Lock()
        self._metrics: Dict[str, Any] = {}
        self._logger = logging.getLogger(f"{agent_type}.{agent_id}")
        
    @property
    def state(self) -> AgentState:
        """Current agent state."""
        return self._state
        
    async def set_state(self, state: AgentState):
        """Update agent state."""
        async with self._lock:
            old_state = self._state
            self._state = state
            self._logger.info(f"State transition: {old_state.name} -> {state.name}")
            
            # Notify state change
            await self.send_message("orchestrator", {
                "type": "state_change",
                "old_state": old_state.name,
                "new_state": state.name,
                "agent_id": self.agent_id
            })
    
    @abstractmethod
    async def initialize(self) -> bool:
        """Initialize the agent. Returns True if successful."""
        pass
    
    @abstractmethod
    async def process(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process an incoming message. Returns response if applicable."""
        pass
    
    @abstractmethod
    async def shutdown(self) -> bool:
        """Shutdown the agent gracefully. Returns True if successful."""
        pass
    
    async def send_message(self, target: str, payload: Dict[str, Any], 
                          message_type: str = "generic",
                          priority: MessagePriority = MessagePriority.NORMAL):
        """Send a message to another agent."""
        message = AgentMessage(
            source=self.agent_id,
            target=target,
            message_type=message_type,
            payload=payload,
            priority=priority
        )
        await self._message_bus.publish(message)
        
    async def broadcast(self, payload: Dict[str, Any], 
                       message_type: str = "broadcast",
                       priority: MessagePriority = MessagePriority.NORMAL):
        """Broadcast a message to all agents."""
        await self.send_message("", payload, message_type, priority)
        
    def get_status(self) -> Dict[str, Any]:
        """Get current agent status."""
        return {
            "agent_id": self.agent_id,
            "agent_type": self.agent_type,
            "state": self._state.name,
            "metrics": self._metrics.copy(),
            "timestamp": datetime.utcnow().isoformat()
        }
        
    async def update_metric(self, key: str, value: Any):
        """Update a metric value."""
        self._metrics[key] = value
        
    async def _message_handler(self, message: AgentMessage):
        """Internal message handler wrapper."""
        if message.source == self.agent_id:
            return  # Ignore self-messages
            
        try:
            response = await self.process(message)
            if response and message.source:
                response.target = message.source
                response.source = self.agent_id
                await self._message_bus.publish(response)
        except Exception as e:
            self._logger.error(f"Error processing message: {e}")
            await self.set_state(AgentState.ERROR)


class AgentRegistry:
    """Registry for managing agent instances."""
    
    def __init__(self):
        self._agents: Dict[str, BaseAgent] = {}
        self._lock = asyncio.Lock()
        
    async def register(self, agent: BaseAgent) -> bool:
        """Register an agent."""
        async with self._lock:
            if agent.agent_id in self._agents:
                return False
            self._agents[agent.agent_id] = agent
            return True
            
    async def unregister(self, agent_id: str) -> bool:
        """Unregister an agent."""
        async with self._lock:
            if agent_id not in self._agents:
                return False
            del self._agents[agent_id]
            return True
            
    async def get_agent(self, agent_id: str) -> Optional[BaseAgent]:
        """Get an agent by ID."""
        async with self._lock:
            return self._agents.get(agent_id)
            
    async def get_agents_by_type(self, agent_type: str) -> List[BaseAgent]:
        """Get all agents of a specific type."""
        async with self._lock:
            return [a for a in self._agents.values() if a.agent_type == agent_type]
            
    async def get_all_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all agents."""
        async with self._lock:
            return {aid: agent.get_status() for aid, agent in self._agents.items()}
