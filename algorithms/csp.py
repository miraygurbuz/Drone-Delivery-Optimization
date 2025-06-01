from typing import List, Dict
from models.drone import Drone
from models.no_fly_zone import NoFlyZone
from models.delivery import Delivery
from algorithms.a_star import AStarPathfinder
from models.graph import DeliveryGraph
from utils.helpers import calculate_distance, calculate_energy_consumption

class CSPSolverWithAStar:

    def __init__(self, drones: List[Drone], deliveries: List[Delivery], no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.deliveries = deliveries
        self.no_fly_zones = no_fly_zones
        self.assignments = {}
        self.routes = {}
        self.violation_logs = []
        self.graph = DeliveryGraph(deliveries, no_fly_zones)

    def solve(self) -> Dict[int, List[int]]:
        unvisited = set(d.id for d in self.deliveries)

        for drone in self.drones:
            drone.current_pos = drone.start_pos
            drone.current_battery = drone.battery
            drone.current_weight = 0.0
            current_time = 0
            route = []

            while unvisited:
                pathfinder = AStarPathfinder(self.graph, drone)
                best_path = pathfinder.find_path(drone.current_pos, unvisited, current_time)

                if not best_path:
                    self.violation_logs.append(f"Drone {drone.id} için uygun yol bulunamadı.")
                    break

                delivery_id = best_path[0]
                delivery = next(d for d in self.deliveries if d.id == delivery_id)

                if delivery.weight > drone.max_weight:
                    self.violation_logs.append(f"Drone {drone.id} ağırlık sınırı aşıldı (teslimat {delivery.id}).")
                    unvisited.discard(delivery_id)
                    continue

                dist = calculate_distance(drone.current_pos, delivery.pos)
                energy_required = calculate_energy_consumption(dist, delivery.weight)

                if energy_required > drone.current_battery:
                    drone.current_battery = drone.battery
                    current_time += 5

                if energy_required > drone.current_battery:
                    self.violation_logs.append(f"Drone {drone.id} enerji yetersizliği (teslimat {delivery.id}).")
                    unvisited.discard(delivery_id)
                    continue

                drone.current_battery -= energy_required
                drone.current_pos = delivery.pos
                drone.current_weight = delivery.weight
                current_time += dist / drone.speed

                delivery.delivered = True
                self.assignments[delivery.id] = drone.id
                route.append(delivery.id)
                unvisited.discard(delivery.id)

            self.routes[drone.id] = route

        return self.routes