"""
Swarm Orchestrator - Multi-Vehicle Coordination with Consensus Algorithms
Manages distributed consensus, leader election, and formation control for UAV/USV/UGV swarms.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Set, Tuple, Callable
from enum import Enum, auto
import asyncio
import random
import time
import hashlib

from ...core.base_agent import BaseAgent, AgentMessage, MessagePriority, AgentState
from ...utils.config import SwarmConfig, AsyncLogger


class NodeRole(Enum):
    """Raft node roles."""
    FOLLOWER = auto()
    CANDIDATE = auto()
    LEADER = auto()


class ConsensusState(Enum):
    """Consensus state."""
    PENDING = auto()
    COMMITTED = auto()
    REJECTED = auto()


@dataclass
class ConsensusEntry:
    """Log entry for consensus."""
    term: int
    index: int
    command: Dict[str, Any]
    committed: bool = False
    timestamp: float = field(default_factory=time.time)
    
    def hash(self) -> str:
        """Generate hash of entry."""
        data = f"{self.term}:{self.index}:{self.command}"
        return hashlib.sha256(data.encode()).hexdigest()[:16]


@dataclass
class SwarmMember:
    """Swarm member information."""
    agent_id: str
    agent_type: str
    role: NodeRole = NodeRole.FOLLOWER
    last_heartbeat: float = field(default_factory=time.time)
    is_healthy: bool = True
    position: Optional[Tuple[float, float, float]] = None
    capabilities: List[str] = field(default_factory=list)


@dataclass
class SwarmState:
    """Global swarm state."""
    formation_type: str = "leader_follower"
    spacing: float = 5.0
    alignment: str = "grid"
    mission_phase: str = "idle"
    targets: List[Dict[str, Any]] = field(default_factory=list)
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


class RaftConsensus:
    """Raft consensus algorithm implementation."""
    
    def __init__(self, node_id: str, config: SwarmConfig):
        self.node_id = node_id
        self.config = config
        
        # Raft state
        self.current_term = 0
        self.voted_for = None
        self.log: List[ConsensusEntry] = []
        self.commit_index = 0
        self.last_applied = 0
        
        # Leader state
        self.role = NodeRole.FOLLOWER
        self.leader_id: Optional[str] = None
        self.next_index: Dict[str, int] = {}
        self.match_index: Dict[str, int] = {}
        
        # Election timing
        self.election_timeout = config.election_timeout
        self.heartbeat_interval = config.heartbeat_interval
        self._election_timer: Optional[asyncio.Task] = None
        self._heartbeat_timer: Optional[asyncio.Task] = None
        
        # Callbacks
        self.on_leader_elected: Optional[Callable] = None
        self.on_commit: Optional[Callable] = None
        
        # Votes received in current term
        self.votes_received: Set[str] = set()
        
    async def start(self):
        """Start Raft consensus."""
        self._reset_election_timer()
        
    async def stop(self):
        """Stop Raft consensus."""
        if self._election_timer:
            self._election_timer.cancel()
        if self._heartbeat_timer:
            self._heartbeat_timer.cancel()
    
    def _reset_election_timer(self):
        """Reset election timeout."""
        if self._election_timer:
            self._election_timer.cancel()
        
        # Random timeout to avoid split votes
        timeout = self.election_timeout * (1 + random.random())
        self._election_timer = asyncio.create_task(self._election_timeout(timeout))
    
    async def _election_timeout(self, timeout: float):
        """Handle election timeout."""
        await asyncio.sleep(timeout)
        
        if self.role != NodeRole.LEADER:
            await self._start_election()
    
    async def _start_election(self):
        """Start leader election."""
        self.role = NodeRole.CANDIDATE
        self.current_term += 1
        self.voted_for = self.node_id
        self.votes_received = {self.node_id}
        
        print(f"Node {self.node_id} starting election for term {self.current_term}")
        
        # Request votes from other nodes
        # This would send RequestVote RPCs
        
        # Reset timer
        self._reset_election_timer()
    
    def become_leader(self):
        """Transition to leader role."""
        self.role = NodeRole.LEADER
        self.leader_id = self.node_id
        
        # Initialize leader state
        # In practice, we'd track all nodes
        self.next_index = {}
        self.match_index = {}
        
        # Start heartbeat
        if self._heartbeat_timer:
            self._heartbeat_timer.cancel()
        self._heartbeat_timer = asyncio.create_task(self._send_heartbeats())
        
        if self.on_leader_elected:
            asyncio.create_task(self.on_leader_elected(self.node_id))
    
    async def _send_heartbeats(self):
        """Send periodic heartbeats."""
        while self.role == NodeRole.LEADER:
            # Send AppendEntries RPCs (heartbeats)
            await asyncio.sleep(self.heartbeat_interval)
    
    def handle_request_vote(self, term: int, candidate_id: str,
                          last_log_index: int, last_log_term: int) -> Tuple[bool, int]:
        """Handle RequestVote RPC."""
        if term < self.current_term:
            return False, self.current_term
        
        if term > self.current_term:
            self.current_term = term
            self.voted_for = None
            self.role = NodeRole.FOLLOWER
        
        if (self.voted_for is None or self.voted_for == candidate_id) and \
           self._is_log_up_to_date(last_log_index, last_log_term):
            self.voted_for = candidate_id
            self._reset_election_timer()
            return True, self.current_term
        
        return False, self.current_term
    
    def handle_append_entries(self, term: int, leader_id: str,
                             prev_log_index: int, prev_log_term: int,
                             entries: List[ConsensusEntry],
                             leader_commit: int) -> Tuple[bool, int]:
        """Handle AppendEntries RPC."""
        if term < self.current_term:
            return False, self.current_term
        
        if term > self.current_term:
            self.current_term = term
            self.voted_for = None
        
        self.role = NodeRole.FOLLOWER
        self.leader_id = leader_id
        self._reset_election_timer()
        
        # Log consistency check
        if prev_log_index > 0:
            if prev_log_index > len(self.log):
                return False, self.current_term
            if self.log[prev_log_index - 1].term != prev_log_term:
                return False, self.current_term
        
        # Append new entries
        for i, entry in enumerate(entries):
            index = prev_log_index + i + 1
            if index <= len(self.log):
                if self.log[index - 1].term != entry.term:
                    # Conflict found, truncate
                    self.log = self.log[:index - 1]
                    self.log.append(entry)
            else:
                self.log.append(entry)
        
        # Update commit index
        if leader_commit > self.commit_index:
            self.commit_index = min(leader_commit, len(self.log))
            self._apply_committed()
        
        return True, self.current_term
    
    def _is_log_up_to_date(self, last_log_index: int, last_log_term: int) -> bool:
        """Check if candidate's log is at least as up-to-date."""
        if len(self.log) == 0:
            return True
        
        last_term = self.log[-1].term
        if last_term != last_log_term:
            return last_log_term > last_term
        return last_log_index >= len(self.log)
    
    def _apply_committed(self):
        """Apply committed entries."""
        while self.last_applied < self.commit_index:
            self.last_applied += 1
            entry = self.log[self.last_applied - 1]
            entry.committed = True
            
            if self.on_commit:
                asyncio.create_task(self.on_commit(entry))
    
    async def propose(self, command: Dict[str, Any]) -> bool:
        """Propose a new command (leader only)."""
        if self.role != NodeRole.LEADER:
            return False
        
        entry = ConsensusEntry(
            term=self.current_term,
            index=len(self.log) + 1,
            command=command
        )
        self.log.append(entry)
        
        # Replicate to followers
        # In practice, send AppendEntries RPCs
        
        return True


class SwarmOrchestrator(BaseAgent):
    """Swarm Orchestrator for multi-vehicle coordination."""
    
    def __init__(self, agent_id: str, config: SwarmConfig, message_bus):
        super().__init__(agent_id, "swarm_orchestrator", message_bus)
        self.config = config
        self.logger = AsyncLogger(f"swarm_{agent_id}")
        
        # Consensus
        self.consensus = RaftConsensus(agent_id, config)
        self.consensus.on_leader_elected = self._on_leader_elected
        self.consensus.on_commit = self._on_commit
        
        # Swarm state
        self.members: Dict[str, SwarmMember] = {}
        self.swarm_state = SwarmState()
        self.swarm_id = config.swarm_id
        
        # Formation control
        self.formation_positions: Dict[str, Tuple[float, float, float]] = {}
        
        # Background tasks
        self._consensus_task: Optional[asyncio.Task] = None
        self._health_check_task: Optional[asyncio.Task] = None
        self._running = False
        
    async def initialize(self) -> bool:
        """Initialize swarm orchestrator."""
        await self.set_state(AgentState.INITIALIZING)
        await self.logger.info("Initializing Swarm Orchestrator...")
        
        try:
            # Start consensus
            await self.consensus.start()
            
            # Subscribe to message bus
            await self._message_bus.subscribe(self.agent_id, self._message_handler)
            
            # Add self to swarm
            self.members[self.agent_id] = SwarmMember(
                agent_id=self.agent_id,
                agent_type="orchestrator"
            )
            
            # Start background tasks
            self._running = True
            self._health_check_task = asyncio.create_task(self._health_check_loop())
            
            await self.set_state(AgentState.IDLE)
            await self.logger.info("Swarm Orchestrator initialized")
            return True
            
        except Exception as e:
            await self.logger.error(f"Initialization error: {e}")
            await self.set_state(AgentState.ERROR)
            return False
    
    async def process(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process incoming messages."""
        msg_type = message.message_type
        payload = message.payload
        
        if msg_type == "JOIN_REQUEST":
            agent_info = payload.get("agent_info", {})
            success = await self.add_agent(agent_info)
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="JOIN_RESPONSE",
                payload={
                    "success": success,
                    "swarm_id": self.swarm_id,
                    "leader_id": self.consensus.leader_id,
                    "members": list(self.members.keys())
                }
            )
        
        elif msg_type == "LEAVE_REQUEST":
            success = await self.remove_agent(message.source)
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="LEAVE_RESPONSE",
                payload={"success": success}
            )
        
        elif msg_type == "HEARTBEAT":
            # Update member heartbeat
            if message.source in self.members:
                self.members[message.source].last_heartbeat = time.time()
                if "position" in payload:
                    self.members[message.source].position = tuple(payload["position"])
        
        elif msg_type == "VOTE_REQUEST":
            # Handle Raft vote request
            term = payload.get("term", 0)
            candidate_id = payload.get("candidate_id")
            last_log_index = payload.get("last_log_index", 0)
            last_log_term = payload.get("last_log_term", 0)
            
            vote_granted, current_term = self.consensus.handle_request_vote(
                term, candidate_id, last_log_index, last_log_term
            )
            
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="VOTE_RESPONSE",
                payload={
                    "term": current_term,
                    "vote_granted": vote_granted
                }
            )
        
        elif msg_type == "APPEND_ENTRIES":
            # Handle Raft append entries
            term = payload.get("term", 0)
            leader_id = payload.get("leader_id")
            prev_log_index = payload.get("prev_log_index", 0)
            prev_log_term = payload.get("prev_log_term", 0)
            entries = [
                ConsensusEntry(**e) for e in payload.get("entries", [])
            ]
            leader_commit = payload.get("leader_commit", 0)
            
            success, current_term = self.consensus.handle_append_entries(
                term, leader_id, prev_log_index, prev_log_term,
                entries, leader_commit
            )
            
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="APPEND_ENTRIES_RESPONSE",
                payload={
                    "term": current_term,
                    "success": success
                }
            )
        
        elif msg_type == "SWARM_COMMAND":
            # Propose command via consensus
            if self.consensus.role == NodeRole.LEADER:
                await self.consensus.propose(payload)
            else:
                # Forward to leader
                if self.consensus.leader_id:
                    await self.send_message(
                        self.consensus.leader_id,
                        payload,
                        message_type="SWARM_COMMAND"
                    )
        
        elif msg_type == "FORMATION_REQUEST":
            formation_type = payload.get("formation_type", "leader_follower")
            await self.form_formation(formation_type)
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="FORMATION_RESPONSE",
                payload={"formation": formation_type, "members": list(self.members.keys())}
            )
        
        return None
    
    async def add_agent(self, agent_info: Dict[str, Any]) -> bool:
        """Add agent to swarm."""
        agent_id = agent_info.get("agent_id")
        if not agent_id or agent_id in self.members:
            return False
        
        self.members[agent_id] = SwarmMember(
            agent_id=agent_id,
            agent_type=agent_info.get("agent_type", "unknown"),
            capabilities=agent_info.get("capabilities", [])
        )
        
        await self.logger.info(f"Agent {agent_id} joined swarm")
        
        # Broadcast member update
        await self.broadcast(
            payload={
                "type": "member_joined",
                "agent_id": agent_id,
                "total_members": len(self.members)
            },
            message_type="SWARM_UPDATE"
        )
        
        return True
    
    async def remove_agent(self, agent_id: str) -> bool:
        """Remove agent from swarm."""
        if agent_id not in self.members:
            return False
        
        del self.members[agent_id]
        await self.logger.info(f"Agent {agent_id} left swarm")
        
        # Broadcast member update
        await self.broadcast(
            payload={
                "type": "member_left",
                "agent_id": agent_id,
                "total_members": len(self.members)
            },
            message_type="SWARM_UPDATE"
        )
        
        return True
    
    async def elect_leader(self) -> Optional[str]:
        """Trigger leader election."""
        await self.consensus._start_election()
        return self.consensus.leader_id
    
    async def broadcast_command(self, command: Dict[str, Any]):
        """Broadcast command to all swarm members."""
        if self.consensus.role == NodeRole.LEADER:
            await self.consensus.propose(command)
        else:
            # Forward to leader
            if self.consensus.leader_id:
                await self.send_message(
                    self.consensus.leader_id,
                    command,
                    message_type="SWARM_COMMAND"
                )
    
    async def form_formation(self, formation_type: str):
        """Execute formation control."""
        self.swarm_state.formation_type = formation_type
        
        member_ids = [m for m in self.members.keys() if m != self.agent_id]
        num_members = len(member_ids)
        
        if formation_type == "line":
            # Line formation along x-axis
            for i, member_id in enumerate(member_ids):
                self.formation_positions[member_id] = (
                    (i + 1) * self.swarm_state.spacing,
                    0.0,
                    0.0
                )
        
        elif formation_type == "grid":
            # Grid formation
            grid_size = int(num_members ** 0.5) + 1
            for i, member_id in enumerate(member_ids):
                row = i // grid_size
                col = i % grid_size
                self.formation_positions[member_id] = (
                    col * self.swarm_state.spacing,
                    row * self.swarm_state.spacing,
                    0.0
                )
        
        elif formation_type == "circle":
            # Circle formation
            import math
            radius = self.swarm_state.spacing * num_members / (2 * math.pi)
            for i, member_id in enumerate(member_ids):
                angle = 2 * math.pi * i / num_members
                self.formation_positions[member_id] = (
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    0.0
                )
        
        # Broadcast formation assignments
        for member_id, position in self.formation_positions.items():
            await self.send_message(
                member_id,
                {
                    "type": "formation_assignment",
                    "position": position,
                    "formation_type": formation_type
                },
                message_type="FORMATION_ASSIGNMENT"
            )
    
    async def reach_consensus(self, proposal: Dict[str, Any]) -> bool:
        """Propose and reach consensus on a value."""
        return await self.consensus.propose(proposal)
    
    async def _on_leader_elected(self, leader_id: str):
        """Callback when leader is elected."""
        await self.logger.info(f"Leader elected: {leader_id}")
        self.config.leader_id = leader_id
        
        # Update own role
        if leader_id == self.agent_id:
            self.members[self.agent_id].role = NodeRole.LEADER
        else:
            self.members[self.agent_id].role = NodeRole.FOLLOWER
        
        # Broadcast leadership change
        await self.broadcast(
            payload={
                "type": "leader_elected",
                "leader_id": leader_id
            },
            message_type="LEADERSHIP_CHANGE"
        )
    
    async def _on_commit(self, entry: ConsensusEntry):
        """Callback when entry is committed."""
        await self.logger.info(f"Entry committed: {entry.hash()}")
        
        # Execute command
        command = entry.command
        cmd_type = command.get("type")
        
        if cmd_type == "formation_change":
            await self.form_formation(command.get("formation_type"))
        elif cmd_type == "mission_update":
            self.swarm_state.mission_phase = command.get("phase", "idle")
        
        # Broadcast commit
        await self.broadcast(
            payload={
                "type": "consensus_commit",
                "entry_hash": entry.hash(),
                "command": command
            },
            message_type="CONSENSUS_COMMIT"
        )
    
    async def _health_check_loop(self):
        """Monitor swarm member health."""
        while self._running:
            try:
                current_time = time.time()
                timeout = self.config.heartbeat_interval * 3
                
                for member_id, member in list(self.members.items()):
                    if member_id == self.agent_id:
                        continue
                    
                    if current_time - member.last_heartbeat > timeout:
                        if member.is_healthy:
                            member.is_healthy = False
                            await self.logger.warning(f"Member {member_id} unhealthy")
                    else:
                        member.is_healthy = True
                
                await asyncio.sleep(self.config.heartbeat_interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                await self.logger.error(f"Health check error: {e}")
                await asyncio.sleep(1)
    
    async def shutdown(self) -> bool:
        """Shutdown swarm orchestrator."""
        await self.set_state(AgentState.SHUTTING_DOWN)
        await self.logger.info("Shutting down Swarm Orchestrator...")
        
        self._running = False
        
        await self.consensus.stop()
        
        if self._health_check_task:
            self._health_check_task.cancel()
            try:
                await self._health_check_task
            except asyncio.CancelledError:
                pass
        
        # Disband swarm
        await self.broadcast(
            payload={"type": "swarm_disbanding"},
            message_type="SWARM_DISBAND",
            priority=MessagePriority.CRITICAL
        )
        
        await self.set_state(AgentState.SHUTDOWN)
        await self.logger.info("Swarm Orchestrator shutdown complete")
        return True
