from typing import Tuple
from dataclasses import dataclass, field

@dataclass
class Drone:
    id: int
    max_weight: float
    battery: int
    speed: float
    start_pos: Tuple[float, float]
    current_pos: Tuple[float, float] = field(init=False)
    current_battery: int = field(init=False)
    current_weight: float = field(init=False)
    recharge_rate: float = 10.0

    def __post_init__(self):
        self.current_pos = self.start_pos
        self.current_battery = self.battery
        self.current_weight = 0.0