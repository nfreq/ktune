from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional
from pykos import KOS
@dataclass
class TestConfig:
    """Base configuration for test parameters"""
    ip: str
    actuator_id: int
    kp: float
    max_torque: float
    acceleration: float

class TestBench(ABC):
    """Abstract base class for actuator test benches"""
    
    def __init__(self, config: TestConfig):
        self.config = config
        self.kos = KOS(config.ip)

    @abstractmethod
    def get_parameters(self) -> dict:
        """Get bench-specific parameters"""
        pass

    @abstractmethod
    def get_safety_limits(self) -> dict:
        """Get safety limits for this test bench"""
        pass

    @abstractmethod
    async def read_state(self) -> Dict:
        """Read current state of the system"""
        pass

    @abstractmethod
    async def command_state(self, state: Dict):
        """Command system to desired state"""
        pass