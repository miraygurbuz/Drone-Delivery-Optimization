from typing import Tuple, List
import numpy as np
import random
import time

def calculate_distance(pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def point_in_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def line_intersects_polygon(p1: Tuple[float, float], p2: Tuple[float, float],
                          polygon: List[Tuple[float, float]]) -> bool:
    """İki nokta arasındaki doğrunun poligonla kesişiyor mu kontrolü"""
    steps = 20
    for i in range(steps + 1):
        t = i / steps
        point = (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
        if point_in_polygon(point, polygon):
            return True
    return False

def calculate_energy_consumption(distance: float, weight: float) -> float:
    """Mesafe ve ağırlığa göre enerji tüketimini hesaplama"""
    base_consumption = 10  # mAh/metre
    weight_factor = 1 + (weight * 0.2)  # Her kg için %20 ek tüketim
    return distance * base_consumption * weight_factor

def analyze_time_complexity(n_deliveries: List[int], n_drones: int = 5):
    from src.main import DroneDeliverySimulation
    """Farklı teslimat sayıları için zaman karmaşıklığı analizi"""
    results = {'a_star': [], 'genetic': []}

    for n in n_deliveries:
        deliveries = []
        for i in range(n):
            deliveries.append({
                'id': i + 1,
                'pos': (random.uniform(0, 100), random.uniform(0, 100)),
                'weight': random.uniform(0.5, 4.0),
                'priority': random.randint(1, 5),
                'time_window': (0, 120)
            })

        drones = []
        for i in range(n_drones):
            drones.append({
                'id': i + 1,
                'max_weight': random.uniform(3.0, 6.0),
                'battery': random.randint(10000, 20000),
                'speed': random.uniform(5.0, 12.0),
                'start_pos': (random.uniform(0, 100), random.uniform(0, 100))
            })

        no_fly_zones = [
            {
                "id": 1,
                "coordinates": [(40, 30), (60, 30), (60, 50), (40, 50)],
                "active_time": (0, 120)
            }
        ]

        sim = DroneDeliverySimulation(drones, deliveries, no_fly_zones)

        start = time.time()
        sim.run_a_star_simulation()
        a_star_time = time.time() - start
        results['a_star'].append(a_star_time)

        start = time.time()
        sim.run_genetic_algorithm_simulation()
        ga_time = time.time() - start
        results['genetic'].append(ga_time)

    return results