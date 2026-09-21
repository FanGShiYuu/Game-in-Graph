"""Compact research preview of the Game in Graph workflow."""

from .config import DemoConfig, load_config
from .simulation import Simulation, SimulationResult

__all__ = ["DemoConfig", "Simulation", "SimulationResult", "load_config"]
__version__ = "0.1.0"
