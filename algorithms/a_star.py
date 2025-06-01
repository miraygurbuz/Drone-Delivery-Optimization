import heapq
from typing import Tuple, List, Set, Dict
from models.drone import Drone
from models.graph import DeliveryGraph
from utils.helpers import calculate_distance, calculate_energy_consumption, point_in_polygon, line_intersects_polygon

class AStarPathfinder:
    def __init__(self, graph: DeliveryGraph, drone: Drone):
        self.graph = graph
        self.drone = drone

    def calculate_nfz_penalty(self, current_pos: Tuple[float, float], target_delivery_id: int,
                              current_time: float) -> float:
        """Heuristic için NFZ cezası hesaplama"""
        delivery = self.graph.deliveries[target_delivery_id]
        target_pos = delivery.pos

        penalty = 0

        # Tahmini varış zamanı
        distance = calculate_distance(current_pos, target_pos)
        travel_time = distance / self.drone.speed
        estimated_arrival = current_time + travel_time

        for nfz in self.graph.no_fly_zones:
            # NFZ aktifliği kontrolü
            if nfz.active_time[0] <= estimated_arrival <= nfz.active_time[1]:
                # Rota NFZ ile kesişiyor mu kontrolü
                if line_intersects_polygon(current_pos, target_pos, nfz.coordinates):
                    penalty += 50  # Kesişiyorsa ceza

                # Hedef nokta NFZ içinde mi kontrolü
                if point_in_polygon(target_pos, nfz.coordinates):
                    penalty += 100  # Ceza

        return penalty

    def heuristic(self, current_id: int, remaining_deliveries: Set[int], current_time: float) -> float:
        """
        heuristic = distance + nofly_zone_penalty
        """
        if not remaining_deliveries:
            return 0

        if current_id == -1:  # Başlangıç konumu
            current_pos = self.drone.start_pos
        else:
            current_pos = self.graph.deliveries[current_id].pos

        min_cost = float('inf')
        for delivery_id in remaining_deliveries:
            delivery_pos = self.graph.deliveries[delivery_id].pos
            distance = calculate_distance(current_pos, delivery_pos)

            nfz_penalty = self.calculate_nfz_penalty(current_pos, delivery_id, current_time)

            # heuristic = distance + no-fly zone penalty
            total_cost = distance + nfz_penalty
            min_cost = min(min_cost, total_cost)

        return min_cost if min_cost != float('inf') else 0

    def is_feasible_delivery(self, delivery_id: int, current_pos: Tuple[float, float],
                             current_time: float, current_battery: float) -> bool:
        """Kısıtlamalara göre teslimat için uygun mu kontrolü"""
        delivery = self.graph.deliveries[delivery_id]

        # Ağırlık
        if delivery.weight > self.drone.max_weight:
            return False

        # Gereksinimlerin hesaplanması
        distance = calculate_distance(current_pos, delivery.pos)
        travel_time = distance / self.drone.speed
        arrival_time = current_time + travel_time

        # Şarj
        energy_needed = calculate_energy_consumption(distance, delivery.weight)
        effective_battery = current_battery
        charge_time = 0

        if energy_needed > current_battery:
            # Şarj etme simülasyonu
            effective_battery = self.drone.battery
            charge_time = 5  # 5 dakika şarj süresi
            if energy_needed > effective_battery:
                return False

        # Şarj süresi hesabı
        effective_arrival = arrival_time + charge_time

        if effective_arrival > delivery.time_window[1]:
            return False

        # No-fly zone
        final_arrival = max(effective_arrival, delivery.time_window[0])
        for nfz in self.graph.no_fly_zones:
            if nfz.active_time[0] <= final_arrival <= nfz.active_time[1]:
                if (line_intersects_polygon(current_pos, delivery.pos, nfz.coordinates) or
                        point_in_polygon(delivery.pos, nfz.coordinates)):
                    return False

        return True

    def calculate_actual_cost(self, from_pos: Tuple[float, float], to_delivery_id: int,
                              current_time: float, current_battery: float) -> Tuple[float, float, float]:
        """G-score hesabı"""
        delivery = self.graph.deliveries[to_delivery_id]
        distance = calculate_distance(from_pos, delivery.pos)

        # Maliyetler
        time_cost = distance / self.drone.speed
        energy_cost = calculate_energy_consumption(distance, delivery.weight)

        # Şarj
        charge_time = 0
        if energy_cost > current_battery:
            charge_time = 5  # 5 dakika şarj

        # Erken gelme durumu
        arrival_time = current_time + time_cost + charge_time
        wait_time = 0
        if arrival_time < delivery.time_window[0]:
            wait_time = delivery.time_window[0] - arrival_time

        total_time_cost = time_cost + charge_time + wait_time

        # G-score
        priority_bonus = (delivery.priority - 1) * 5
        total_cost = distance - priority_bonus

        return total_cost, total_time_cost, energy_cost

    def find_path(self, start_pos: Tuple[float, float],
                  target_deliveries: Set[int],
                  current_time: float) -> List[int]:
        """
        State: (current_delivery_id, visited_deliveries_frozenset, time, battery)
        Hedef: Tüm teslimatlara gitmek
        """
        if not target_deliveries:
            return []

        # A* Veri Yapıları
        open_set = []  # Öncelik kuyruğu: (f_score, g_score, state, path)
        closed_set = set()
        g_score = {}  # Maliyet

        initial_state = (-1, frozenset(), current_time, self.drone.current_battery)
        g_score[initial_state] = 0

        h_initial = self.heuristic(-1, target_deliveries, current_time)
        f_initial = 0 + h_initial

        heapq.heappush(open_set, (f_initial, 0, initial_state, []))

        nodes_expanded = 0
        max_nodes = 100

        # A* Main Loop
        while open_set and nodes_expanded < max_nodes:
            # En düşük f-scorelu state'i al
            f_score, g_current, current_state, current_path = heapq.heappop(open_set)

            current_id, visited_set, time, battery = current_state

            # Closed set mi kontrolü
            if current_state in closed_set:
                continue

            # Closed set'e ekle
            closed_set.add(current_state)
            nodes_expanded += 1

            if visited_set == frozenset(target_deliveries):
                return current_path

            if current_id == -1:
                current_pos = start_pos
            else:
                current_pos = self.graph.deliveries[current_id].pos

            unvisited = target_deliveries - visited_set

            for next_delivery_id in unvisited:
                if not self.is_feasible_delivery(next_delivery_id, current_pos, time, battery):
                    continue

                edge_cost, time_cost, energy_cost = self.calculate_actual_cost(
                    current_pos, next_delivery_id, time, battery
                )

                new_visited = visited_set | frozenset([next_delivery_id])
                new_time = time + time_cost

                if energy_cost > battery:
                    new_battery = self.drone.battery - energy_cost
                else:
                    new_battery = battery - energy_cost

                new_state = (next_delivery_id, new_visited, new_time, new_battery)

                tentative_g = g_current + edge_cost

                if new_state in g_score and tentative_g >= g_score[new_state]:
                    continue

                g_score[new_state] = tentative_g

                remaining_deliveries = target_deliveries - new_visited
                h_score = self.heuristic(next_delivery_id, remaining_deliveries, new_time)

                f_score = tentative_g + h_score

                new_path = current_path + [next_delivery_id]
                heapq.heappush(open_set, (f_score, tentative_g, new_state, new_path))

        return self._greedy_fallback(start_pos, target_deliveries, current_time)

    def _greedy_fallback(self, start_pos: Tuple[float, float],
                         target_deliveries: Set[int], current_time: float) -> List[int]:
        """A* algoritmasının time out olmasına karşın greedy fallback"""
        path = []
        remaining = set(target_deliveries)
        current_pos = start_pos
        current_battery = self.drone.current_battery
        time = current_time

        while remaining:
            best_delivery = None
            best_distance = float('inf')

            for delivery_id in remaining:
                if self.is_feasible_delivery(delivery_id, current_pos, time, current_battery):
                    delivery = self.graph.deliveries[delivery_id]
                    distance = calculate_distance(current_pos, delivery.pos)

                    adjusted_distance = distance - delivery.priority * 2

                    if adjusted_distance < best_distance:
                        best_distance = adjusted_distance
                        best_delivery = delivery_id

            if best_delivery is None:
                break

            path.append(best_delivery)
            remaining.remove(best_delivery)

            delivery = self.graph.deliveries[best_delivery]
            distance = calculate_distance(current_pos, delivery.pos)
            energy = calculate_energy_consumption(distance, delivery.weight)

            current_pos = delivery.pos
            current_battery -= energy
            time += distance / self.drone.speed

        return path

    def get_path_statistics(self, path: List[int], start_time: float = 0) -> Dict:
        """Bulunan rota için istatistikleri hesaplama"""
        if not path:
            return {
                "total_distance": 0,
                "total_time": 0,
                "total_energy": 0,
                "path_length": 0,
                "estimated_optimal": False,
                "deliveries_completed": 0,
                "average_priority": 0,
                "energy_efficiency": 0
            }

        total_distance = 0
        total_energy = 0
        current_pos = self.drone.start_pos
        priorities = []

        for delivery_id in path:
            delivery = self.graph.deliveries[delivery_id]
            distance = calculate_distance(current_pos, delivery.pos)
            energy = calculate_energy_consumption(distance, delivery.weight)

            total_distance += distance
            total_energy += energy
            priorities.append(delivery.priority)

            current_pos = delivery.pos

        total_time = total_distance / self.drone.speed

        return {
            "total_distance": round(total_distance, 2),
            "total_time": round(total_time, 2),
            "total_energy": round(total_energy, 2),
            "path_length": len(path),
            "estimated_optimal": len(path) <= 3,
            "deliveries_completed": len(path),
            "average_priority": round(sum(priorities) / len(priorities), 2) if priorities else 0,
            "energy_efficiency": round((total_distance / total_energy) if total_energy > 0 else 0, 2)
        }