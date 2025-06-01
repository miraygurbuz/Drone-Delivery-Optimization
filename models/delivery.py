from typing import Tuple
from dataclasses import dataclass

@dataclass
class Delivery:
    id: int
    pos: Tuple[float, float]
    weight: float
    priority: int
    time_window: Tuple[int, int]
    delivered: bool = False