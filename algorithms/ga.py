import random
from typing import List, Dict
from models.drone import Drone
from models.delivery import Delivery
from models.graph import DeliveryGraph
from utils.helpers import calculate_distance, calculate_energy_consumption, line_intersects_polygon, point_in_polygon

class GeneticAlgorithm:
    def __init__(self, drones: List[Drone], deliveries: List[Delivery], graph: DeliveryGraph,
                 population_size: int = 30, generations: int = 75, start_time: float = 0):
        self.drones = drones
        self.deliveries = deliveries
        self.graph = graph
        self.population_size = population_size
        self.generations = generations
        self.start_time = start_time  # Simülasyon başlangıç zamanı

    def create_individual(self) -> Dict[int, List[int]]:
        """Rastgele bir birey (çözüm) oluşturma"""
        individual = {d.id: [] for d in self.drones}
        delivery_ids = [d.id for d in self.deliveries]
        random.shuffle(delivery_ids)

        for delivery_id in delivery_ids:
            delivery = self.graph.deliveries[delivery_id]

            # Ağırlık kapasitesi uygun olan dronelar
            candidates = [d for d in self.drones if delivery.weight <= d.max_weight]

            if candidates:
                # Zaman kontrolü
                valid_candidates = []
                for drone in candidates:
                    if self._can_deliver_in_time(drone, delivery, individual[drone.id]):
                        valid_candidates.append(drone)

                chosen_candidates = valid_candidates if valid_candidates else candidates
                chosen = random.choice(chosen_candidates)
                individual[chosen.id].append(delivery_id)

        return individual

    def _can_deliver_in_time(self, drone: Drone, delivery: Delivery, current_route: List[int]) -> bool:
        """Drone'un bu teslimatı zamanında yapıp yapamayacağını kontrol etme"""
        current_pos = drone.start_pos
        current_time = self.start_time
        current_battery = drone.battery

        for delivery_id in current_route:
            route_delivery = self.graph.deliveries[delivery_id]
            distance = calculate_distance(current_pos, route_delivery.pos)
            energy = calculate_energy_consumption(distance, route_delivery.weight)

            if energy > current_battery:
                # Şarj
                current_battery = drone.battery
                current_time += 5  # 5 dakika

            current_time += distance / drone.speed
            current_battery -= energy
            current_pos = route_delivery.pos

        distance = calculate_distance(current_pos, delivery.pos)
        energy = calculate_energy_consumption(distance, delivery.weight)

        if energy > current_battery:
            current_time += 5

        arrival_time = current_time + (distance / drone.speed)

        # Zaman kontrolü
        return delivery.time_window[0] <= arrival_time <= delivery.time_window[1]

    def fitness(self, individual: Dict[int, List[int]]) -> float:
        """fitness = (teslimat sayisi x 50) - (toplam enerji x 0.1) - (ihlal edilen kısıt x 100)"""
        delivery_count = 0
        total_energy = 0
        total_violations = 0
        delivery_map = {d.id: d for d in self.deliveries}

        for drone_id, route in individual.items():
            drone = next(d for d in self.drones if d.id == drone_id)
            current_pos = drone.start_pos
            current_battery = drone.battery
            current_time = self.start_time

            for delivery_id in route:
                delivery = delivery_map[delivery_id]
                distance = calculate_distance(current_pos, delivery.pos)
                energy_needed = calculate_energy_consumption(distance, delivery.weight)

                # Ağırlık kontrolü
                if delivery.weight > drone.max_weight:
                    total_violations += 1
                    continue

                # Enerji kontrolü
                if energy_needed > current_battery:
                    # Şarj
                    current_battery = drone.battery
                    current_time += 5  # 5 dakika

                    if energy_needed > current_battery:
                        total_violations += 1
                        continue

                # Seyahat ve varış zamanı hesaplama
                travel_time = distance / drone.speed
                arrival_time = current_time + travel_time

                # Zaman kontrolü
                if arrival_time < delivery.time_window[0]:
                    # Çok erken varış - bekleme süresi
                    current_time = delivery.time_window[0]
                elif arrival_time > delivery.time_window[1]:
                    # Geç varış - ihlal
                    total_violations += 1
                    current_time = arrival_time
                else:
                    # Zamanında varış
                    current_time = arrival_time

                # No-fly zone kontrolü
                nfz_violation = False

                # Rota no-fly zone kontrolü
                for nfz in self.graph.no_fly_zones:
                    if nfz.active_time[0] <= current_time <= nfz.active_time[1]:
                        if line_intersects_polygon(current_pos, delivery.pos, nfz.coordinates):
                            total_violations += 1
                            nfz_violation = True
                            break

                # Varış noktası no-fly zone kontrolü
                if not nfz_violation:
                    for nfz in self.graph.no_fly_zones:
                        if nfz.active_time[0] <= current_time <= nfz.active_time[1]:
                            if point_in_polygon(delivery.pos, nfz.coordinates):
                                total_violations += 1
                                nfz_violation = True
                                break

                if not nfz_violation and arrival_time <= delivery.time_window[1]:
                    delivery_count += 1

                current_battery -= energy_needed
                total_energy += energy_needed
                current_pos = delivery.pos

        final_score = (delivery_count * 50) - (total_energy * 0.1) - (total_violations * 100)

        return final_score

    def crossover(self, parent1: Dict[int, List[int]], parent2: Dict[int, List[int]]) -> Dict[int, List[int]]:
        child = {drone_id: [] for drone_id in parent1.keys()}
        all_deliveries = set(d.id for d in self.deliveries)
        assigned_deliveries = set()

        for drone_id in parent1.keys():
            route1 = parent1[drone_id]
            route2 = parent2[drone_id]

            if route1 and route2:
                cross_point1 = random.randint(0, len(route1))
                cross_point2 = random.randint(0, len(route2))

                child_route = route1[:cross_point1]
                child_route.extend([d for d in route2[cross_point2:] if d not in child_route])

            elif route1:
                child_route = route1[:len(route1) // 2]
            elif route2:
                child_route = route2[:len(route2) // 2]
            else:
                child_route = []

            child[drone_id] = child_route
            assigned_deliveries.update(child_route)

        missing_deliveries = all_deliveries - assigned_deliveries
        for delivery_id in missing_deliveries:
            delivery = self.graph.deliveries[delivery_id]

            suitable_drones = []
            for drone in self.drones:
                if (delivery.weight <= drone.max_weight and
                        self._can_deliver_in_time(drone, delivery, child[drone.id])):
                    suitable_drones.append(drone)

            if not suitable_drones:
                suitable_drones = [d for d in self.drones if delivery.weight <= d.max_weight]

            if suitable_drones:
                chosen_drone = random.choice(suitable_drones)
                child[chosen_drone.id].append(delivery_id)

        return child

    def mutate(self, individual: Dict[int, List[int]]) -> Dict[int, List[int]]:
        """Çoklu mutasyon türleri"""
        mutation_type = random.choice(['swap', 'move', 'reverse', 'shuffle'])

        if mutation_type == 'swap':
            # İki farklı drone arasında teslimat değiş tokuşu
            self._swap_between_drones(individual)

        elif mutation_type == 'move':
            # Teslimatı bir drone'dan diğerine taşı
            self._move_delivery(individual)

        elif mutation_type == 'reverse':
            # Bir drone'un rotasında sırayı ters çevir
            self._reverse_route_segment(individual)

        elif mutation_type == 'shuffle':
            # Bir drone'un rotasında rastgele karıştır
            self._shuffle_route_segment(individual)

        return individual

    def _swap_between_drones(self, individual: Dict[int, List[int]]):
        """İki drone arasında teslimat değiş tokuşu"""
        drone_ids = [did for did, route in individual.items() if route]
        if len(drone_ids) >= 2:
            d1, d2 = random.sample(drone_ids, 2)
            if individual[d1] and individual[d2]:
                i1 = random.randint(0, len(individual[d1]) - 1)
                i2 = random.randint(0, len(individual[d2]) - 1)

                # Kapasite kontrolü
                delivery1 = self.graph.deliveries[individual[d1][i1]]
                delivery2 = self.graph.deliveries[individual[d2][i2]]
                drone1 = next(d for d in self.drones if d.id == d1)
                drone2 = next(d for d in self.drones if d.id == d2)

                if (delivery1.weight <= drone2.max_weight and
                        delivery2.weight <= drone1.max_weight):
                    individual[d1][i1], individual[d2][i2] = individual[d2][i2], individual[d1][i1]

    def _move_delivery(self, individual: Dict[int, List[int]]):
        """Teslimatı bir drone'dan diğerine taşıma"""
        source_drones = [did for did, route in individual.items() if route]
        if source_drones:
            source_drone = random.choice(source_drones)
            delivery_idx = random.randint(0, len(individual[source_drone]) - 1)
            delivery_id = individual[source_drone].pop(delivery_idx)

            # Hedef drone
            delivery = self.graph.deliveries[delivery_id]
            target_candidates = [d for d in self.drones
                                 if d.id != source_drone and delivery.weight <= d.max_weight]

            if target_candidates:
                target_drone = random.choice(target_candidates)
                individual[target_drone.id].append(delivery_id)
            else:
                # Geri koy
                individual[source_drone].insert(delivery_idx, delivery_id)

    def _reverse_route_segment(self, individual: Dict[int, List[int]]):
        """Rota segmentini ters çevirme"""
        drone_ids = [did for did, route in individual.items() if len(route) > 1]
        if drone_ids:
            drone_id = random.choice(drone_ids)
            route = individual[drone_id]
            start = random.randint(0, len(route) - 2)
            end = random.randint(start + 1, len(route))
            individual[drone_id][start:end] = reversed(individual[drone_id][start:end])

    def _shuffle_route_segment(self, individual: Dict[int, List[int]]):
        """Rota segmentini karıştırma"""
        drone_ids = [did for did, route in individual.items() if len(route) > 2]
        if drone_ids:
            drone_id = random.choice(drone_ids)
            route = individual[drone_id]
            start = random.randint(0, len(route) - 3)
            end = random.randint(start + 2, len(route))
            segment = individual[drone_id][start:end]
            random.shuffle(segment)
            individual[drone_id][start:end] = segment

    def run(self) -> Dict[int, List[int]]:
        """Ana genetik algoritma döngüsü"""
        print(f"Genetik Algoritma başlatılıyor: {self.population_size} birey, {self.generations} jenerasyon")

        # İlk popülasyon
        population = [self.create_individual() for _ in range(self.population_size)]
        best_individual = None
        best_score = float('-inf')
        generation_scores = []

        for generation in range(self.generations):
            # Fitness
            evaluated_population = [(individual, self.fitness(individual)) for individual in population]
            evaluated_population.sort(key=lambda x: x[1], reverse=True)

            # En iyi bireyi güncelle
            current_best_score = evaluated_population[0][1]
            if current_best_score > best_score:
                best_score = current_best_score
                best_individual = evaluated_population[0][0].copy()

            generation_scores.append(current_best_score)

            # Seçilim
            elite_size = max(2, self.population_size // 10)
            survivors = [ind for ind, _ in evaluated_population[:elite_size]]

            # Yeni popülasyon
            new_population = survivors.copy()

            # Turnuva seçimi ile ebeveyn seçimi
            tournament_size = 3
            while len(new_population) < self.population_size:
                # Ebeveyn seçimi
                parent1 = self._tournament_selection(evaluated_population, tournament_size)
                parent2 = self._tournament_selection(evaluated_population, tournament_size)

                # Crossover
                child = self.crossover(parent1, parent2)

                # Mutasyon
                if random.random() < 0.15:
                    child = self.mutate(child)

                new_population.append(child)

            population = new_population

        return best_individual

    def _tournament_selection(self, evaluated_population: List, tournament_size: int):
        """Turnuva seçimi"""
        tournament = random.sample(evaluated_population, min(tournament_size, len(evaluated_population)))
        return max(tournament, key=lambda x: x[1])[0]

    def get_algorithm_statistics(self, solution: Dict[int, List[int]]) -> Dict:
        """Algoritma istatistiklerini hesaplama"""
        total_deliveries = sum(len(route) for route in solution.values())
        active_drones = len([route for route in solution.values() if route])

        return {
            'total_deliveries': total_deliveries,
            'active_drones': active_drones,
            'drone_utilization': active_drones / len(self.drones) * 100,
            'deliveries_per_drone': total_deliveries / active_drones if active_drones > 0 else 0,
            'final_fitness': self.fitness(solution)
        }