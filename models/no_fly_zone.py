from typing import Tuple, List
from dataclasses import dataclass

@dataclass
class NoFlyZone:
    id: int
    coordinates: List[Tuple[float, float]]
    active_time: Tuple[int, int]