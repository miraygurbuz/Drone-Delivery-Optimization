import matplotlib.pyplot as plt
import matplotlib.patches as patches
from algorithms.a_star import AStarPathfinder
from algorithms.ga import GeneticAlgorithm
from algorithms.csp import CSPSolverWithAStar
from models.drone import Drone
from models.delivery import Delivery
from models.no_fly_zone import NoFlyZone
from models.graph import DeliveryGraph
from matplotlib.lines import Line2D
from utils.helpers import *
from utils.random_data_generator import *

class DroneDeliverySimulation:
    """Drone teslimat simülasyonunu yöneten ana sınıf"""

    def __init__(self, drones_data: List[Dict], deliveries_data: List[Dict],
                 no_fly_zones_data: List[Dict]):
        self.drones = [Drone(**d) for d in drones_data]
        self.deliveries = [Delivery(**d) for d in deliveries_data]
        self.no_fly_zones = [NoFlyZone(**nfz) for nfz in no_fly_zones_data]

        self.graph = DeliveryGraph(self.deliveries, self.no_fly_zones)

        self.csp_solver = CSPSolverWithAStar(self.drones, self.deliveries, self.no_fly_zones)
        self.genetic_algorithm = GeneticAlgorithm(self.drones, self.deliveries, self.graph)

        self.results = {
            'a_star': {'routes': {}, 'metrics': {}},
            'genetic': {'routes': {}, 'metrics': {}},
            'csp_violations': []
        }

    def run_a_star_simulation(self) -> Dict:
        """A* algoritması ile simülasyonu çalıştırır"""
        start_time = time.time()

        routes = {d.id: [] for d in self.drones}
        delivered = set()
        total_energy = 0
        total_distance = 0
        current_time = 0

        for drone in self.drones:
            pathfinder = AStarPathfinder(self.graph, drone)
            drone_route = []
            drone_energy = 0
            drone_distance = 0

            drone.current_pos = drone.start_pos
            drone.current_battery = drone.battery
            drone.current_weight = 0
            current_pos = drone.start_pos

            max_deliveries_per_drone = 10
            deliveries_made = 0
            consecutive_failures = 0
            max_consecutive_failures = 3

            while deliveries_made < max_deliveries_per_drone and consecutive_failures < max_consecutive_failures:
                unvisited = set(d.id for d in self.deliveries if d.id not in delivered)

                if not unvisited:
                    print(f"Drone {drone.id}: Tüm teslimatlar tamamlandı")
                    break

                if drone.current_battery <= 0:
                    print(f"Drone {drone.id}: Batarya bitti")
                    break

                path = pathfinder.find_path(current_pos, unvisited, current_time)

                if not path:
                    print(f"Drone {drone.id}: Rota bulunamadı")
                    break

                delivery_made_this_iteration = False

                for delivery_id in path:
                    delivery = self.graph.deliveries[delivery_id]

                    if delivery_id in delivered:
                        continue

                    distance = calculate_distance(current_pos, delivery.pos)
                    energy = calculate_energy_consumption(distance, delivery.weight)

                    if delivery.weight > drone.max_weight:
                        print(
                            f"Teslimat {delivery_id} ({delivery.weight} kg) > Drone {drone.id} kapasitesi ({drone.max_weight} kg)")
                        continue

                    if energy > drone.current_battery:
                        print(
                            f"Drone {drone.id}: Yetersiz batarya (Gerekli: {energy}, Mevcut: {drone.current_battery})")
                        break

                    # Zaman kontrolü
                    travel_time = distance / drone.speed
                    arrival_time = current_time + travel_time

                    if arrival_time > delivery.time_window[1]:
                        print(
                            f"Teslimat {delivery_id} zaman penceresini kaçırdı (Varış: {arrival_time:.2f}, Deadline: {delivery.time_window[1]})")
                        continue

                    drone_route.append({
                        'delivery_id': delivery_id,
                        'position': delivery.pos,
                        'time': arrival_time,
                        'weight': delivery.weight,
                        'priority': delivery.priority
                    })

                    # Drone durumunu güncelle
                    drone.current_battery -= energy
                    drone_energy += energy
                    drone_distance += distance
                    current_pos = delivery.pos

                    # Teslimatı delivered olarak işaretle
                    delivered.add(delivery_id)
                    delivery.delivered = True

                    current_time = arrival_time
                    deliveries_made += 1
                    delivery_made_this_iteration = True  # ← Bu iterasyonda teslimat yapıldı

                    print(f"Drone {drone.id}: Teslimat {delivery_id} tamamlandı")

                    # Maksimum teslimat sayısına ulaşıldı mı kontrolü
                    if deliveries_made >= max_deliveries_per_drone:
                        break

                if delivery_made_this_iteration:
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    print(f"Drone {drone.id}: Bu iterasyonda teslimat yapılamadı ({consecutive_failures}/{max_consecutive_failures})")

            routes[drone.id] = drone_route
            total_energy += drone_energy
            total_distance += drone_distance

            print(f"Drone {drone.id} özet: {len(drone_route)} teslimat, {drone_energy:.2f} mAh, {consecutive_failures} ardışık başarısızlık")

        end_time = time.time()

        metrics = {
            'completed_deliveries': len(delivered),
            'completion_rate': len(delivered) / len(self.deliveries) * 100 if self.deliveries else 0,
            'total_energy': total_energy,
            'total_distance': total_distance,
            'avg_energy_per_delivery': total_energy / len(delivered) if delivered else 0,
            'execution_time': end_time - start_time
        }

        self.results['a_star']['routes'] = routes
        self.results['a_star']['metrics'] = metrics

        print(f"A* Algoritması Tamamlandı:")
        print(f"Toplam teslimat: {len(delivered)}/{len(self.deliveries)} (%{metrics['completion_rate']:.1f})")
        print(f"Çalışma süresi: {metrics['execution_time']:.2f} saniye")

        return routes

    def run_genetic_algorithm_simulation(self) -> Dict:
        """Genetik algoritma ile simülasyonu çalıştırır"""
        start_time = time.time()

        best_solution = self.genetic_algorithm.run()

        routes = {d.id: [] for d in self.drones}
        total_energy = 0
        total_distance = 0
        delivered = set()
        simulation_time = 0

        for drone_id, delivery_ids in best_solution.items():
            if not delivery_ids:
                continue

            drone = next(d for d in self.drones if d.id == drone_id)
            current_pos = drone.start_pos
            current_time = simulation_time
            current_battery = drone.battery

            for delivery_id in delivery_ids:
                delivery = next(d for d in self.deliveries if d.id == delivery_id)

                distance = calculate_distance(current_pos, delivery.pos)
                energy = calculate_energy_consumption(distance, delivery.weight)
                travel_time = distance / drone.speed

                if energy > current_battery:
                    current_battery = drone.battery
                    current_time += 5

                arrival_time = current_time + travel_time

                routes[drone_id].append({
                    'delivery_id': delivery_id,
                    'position': delivery.pos,
                    'time': arrival_time,
                    'weight': delivery.weight,
                    'priority': delivery.priority
                })

                total_energy += energy
                total_distance += distance
                delivered.add(delivery_id)

                current_pos = delivery.pos
                current_time = arrival_time
                current_battery -= energy

        end_time = time.time()

        metrics = {
            'completed_deliveries': len(delivered),
            'completion_rate': len(delivered) / len(self.deliveries) * 100,
            'total_energy': total_energy,
            'total_distance': total_distance,
            'avg_energy_per_delivery': total_energy / len(delivered) if delivered else 0,
            'execution_time': end_time - start_time,
            'final_fitness': self.genetic_algorithm.fitness(best_solution)
        }

        self.results['genetic']['routes'] = routes
        self.results['genetic']['metrics'] = metrics

        return routes

    def visualize_routes(self, algorithm: str = 'a_star', current_simulation_time: float = None):
        """Rotaları görselleştirir"""
        fig, ax = plt.subplots(figsize=(14, 10))

        if current_simulation_time is None:
            current_simulation_time = 0

        active_nfz_count = 0
        inactive_nfz_count = 0

        for nfz in self.no_fly_zones:
            is_active = nfz.active_time[0] <= current_simulation_time <= nfz.active_time[1]

            if is_active:
                polygon = patches.Polygon(nfz.coordinates, alpha=0.6, facecolor='red',
                                          edgecolor='darkred', linewidth=3, hatch='///')
                active_nfz_count += 1
                status_text = "Aktif"
                text_color = 'darkred'
                text_weight = 'bold'
            else:
                polygon = patches.Polygon(nfz.coordinates, alpha=0.2, facecolor='gray',
                                          edgecolor='darkgray', linewidth=1, linestyle='--')
                inactive_nfz_count += 1
                status_text = "Pasif"
                text_color = 'gray'
                text_weight = 'normal'

            ax.add_patch(polygon)

            center_x = np.mean([p[0] for p in nfz.coordinates])
            center_y = np.mean([p[1] for p in nfz.coordinates])

            ax.text(center_x, center_y + 1, f'NFZ {nfz.id}', ha='center', va='center',
                    fontsize=11, fontweight=text_weight, color=text_color,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))

            ax.text(center_x, center_y - 2, status_text, ha='center', va='center',
                    fontsize=9, fontweight=text_weight, color=text_color)

            time_info = f"({nfz.active_time[0]:.0f}-{nfz.active_time[1]:.0f})"
            ax.text(center_x, center_y - 4, time_info, ha='center', va='center',
                    fontsize=8, color=text_color, style='italic')

        for delivery in self.deliveries:
            if delivery.priority >= 4:
                color = 'green'
                priority_label = 'Yüksek'
            elif delivery.priority >= 2:
                color = 'orange'
                priority_label = 'Orta'
            else:
                color = 'yellow'
                priority_label = 'Düşük'

            ax.scatter(delivery.pos[0], delivery.pos[1], c=color, s=120,
                       edgecolors='black', marker='o', zorder=5)

            ax.text(delivery.pos[0], delivery.pos[1] - 3, f'D{delivery.id}',
                    ha='center', fontsize=9)

            ax.text(delivery.pos[0], delivery.pos[1] - 6, f'{delivery.weight}kg\nÖ:{delivery.priority}',
                    ha='center', fontsize=7, alpha=0.8)

        for drone in self.drones:
            ax.scatter(drone.start_pos[0], drone.start_pos[1], c='blue', s=250,
                       marker='^', edgecolors='black', linewidth=2, zorder=6)
            ax.text(drone.start_pos[0], drone.start_pos[1] + 4, f'Drone {drone.id}',
                    ha='center', fontsize=10)
            ax.text(drone.start_pos[0], drone.start_pos[1] - 4, f'Kapasite: {drone.max_weight}kg',
                    ha='center', fontsize=8, alpha=0.7)

        routes = self.results[algorithm]['routes']
        colors = plt.cm.Set1(np.linspace(0, 1, len(self.drones)))

        total_routes_drawn = 0

        for drone_idx, (drone_id, route) in enumerate(routes.items()):
            if not route:
                continue

            drone = next(d for d in self.drones if d.id == drone_id)
            color = colors[drone_idx % len(colors)]

            current_pos = drone.start_pos
            route_time = current_simulation_time

            for i, delivery_info in enumerate(route):
                delivery_pos = delivery_info['position']

                ax.plot([current_pos[0], delivery_pos[0]],
                        [current_pos[1], delivery_pos[1]],
                        color=color, linewidth=2.5, alpha=0.8, zorder=3)

                mid_x = (current_pos[0] + delivery_pos[0]) / 2
                mid_y = (current_pos[1] + delivery_pos[1]) / 2
                dx = delivery_pos[0] - current_pos[0]
                dy = delivery_pos[1] - current_pos[1]

                if dx != 0 or dy != 0:
                    length = np.sqrt(dx ** 2 + dy ** 2)
                    if length > 0:
                        dx_norm = dx / length * 3
                        dy_norm = dy / length * 3
                        ax.arrow(mid_x, mid_y, dx_norm, dy_norm,
                                 head_width=2, head_length=1.5,
                                 fc=color, ec=color, alpha=0.8, zorder=4)

                ax.text(delivery_pos[0] + 1, delivery_pos[1] + 1, f'{i + 1}',
                        ha='center', va='center', fontsize=8,
                        bbox=dict(boxstyle="circle,pad=0.2", facecolor=color, alpha=0.7))

                current_pos = delivery_pos
                total_routes_drawn += 1

        ax.set_xlim(-15, 115)
        ax.set_ylim(-15, 115)
        ax.set_xlabel('X Koordinatı (metre)', fontsize=12)
        ax.set_ylabel('Y Koordinatı (metre)', fontsize=12)

        title = f'Drone Teslimat Rotaları - {algorithm.upper()} Algoritması\n'
        ax.set_title(title, fontsize=14, pad=20)

        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='green',
                   markersize=12, label='Yüksek Öncelik (4-5)', markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='orange',
                   markersize=12, label='Orta Öncelik (2-3)', markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow',
                   markersize=12, label='Düşük Öncelik (1)', markeredgecolor='black'),

            Line2D([0], [0], marker='^', color='w', markerfacecolor='blue',
                   markersize=15, label='Drone Başlangıç', markeredgecolor='black'),

            patches.Patch(facecolor='red', alpha=0.6, hatch='///',
                          edgecolor='darkred', linewidth=2, label='Aktif No-Fly Zone'),
            patches.Patch(facecolor='gray', alpha=0.2, linestyle='--',
                          edgecolor='darkgray', label='Pasif No-Fly Zone'),

            Line2D([0], [0], color='black', linewidth=2.5, label='Drone Rotası')
        ]

        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.5),
                  fontsize=10, title='Açıklama', title_fontsize=12)

        plt.tight_layout()
        return fig

    def generate_report(self) -> str:
        report = []

        report.append("A* Algoritması")
        report.append("-" * 40)
        a_star_metrics = self.results['a_star']['metrics']
        report.append(f"Tamamlanan Teslimat Sayısı: {a_star_metrics['completed_deliveries']}/{len(self.deliveries)}")
        report.append(f"Tamamlanma Oranı: {a_star_metrics['completion_rate']:.2f}%")
        report.append(f"Toplam Enerji Tüketimi: {a_star_metrics['total_energy']:.2f} mAh")
        report.append(f"Toplam Mesafe: {a_star_metrics['total_distance']:.2f} metre")
        report.append(f"Teslimat Başına Ortalama Enerji: {a_star_metrics['avg_energy_per_delivery']:.2f} mAh")
        report.append(f"Algoritma Çalışma Süresi: {a_star_metrics['execution_time']:.4f} saniye")
        report.append("")

        report.append("Genetik Algoritma")
        report.append("-" * 40)
        ga_metrics = self.results['genetic']['metrics']
        report.append(f"Tamamlanan Teslimat Sayısı: {ga_metrics['completed_deliveries']}/{len(self.deliveries)}")
        report.append(f"Tamamlanma Oranı: {ga_metrics['completion_rate']:.2f}%")
        report.append(f"Toplam Enerji Tüketimi: {ga_metrics['total_energy']:.2f} mAh")
        report.append(f"Toplam Mesafe: {ga_metrics['total_distance']:.2f} metre")
        report.append(f"Teslimat Başına Ortalama Enerji: {ga_metrics['avg_energy_per_delivery']:.2f} mAh")
        report.append(f"Final Fitness Skoru: {ga_metrics['final_fitness']:.2f}")
        report.append(f"Algoritma Çalışma Süresi: {ga_metrics['execution_time']:.4f} saniye")
        report.append("")

        report.append("Zaman Karmaşıklığı")
        report.append("-" * 40)
        report.append("A* Algoritması:")
        report.append(f"  - Teorik: O(b^d) - b: branching factor, d: derinlik")
        report.append(f"  - Pratik: O(V * E * log V) - V: düğüm sayısı, E: kenar sayısı")
        report.append(f"  - Bu durumda: O({len(self.deliveries)} * {len(self.deliveries)**2} * log {len(self.deliveries)})")
        report.append("")
        report.append("Genetik Algoritma:")
        report.append(f"  - Teorik: O(g * p * f) - g: jenerasyon, p: popülasyon, f: fitness")
        report.append(f"  - Pratik: O({self.genetic_algorithm.generations} * {self.genetic_algorithm.population_size} * {len(self.deliveries)})")
        report.append("")

        report.append("=" * 80)

        return "\n".join(report)


if __name__ == "__main__":
    generator = RandomDataGenerator()

    scenarios = [
        {"name": "senaryo_1", "drone_count": 5, "delivery_count": 20, "nfz_count": 3},
        {"name": "senaryo_2", "drone_count": 10, "delivery_count": 50, "nfz_count": 5},
    ]

    for s in scenarios:
        print(f"\n=== {s['name'].upper()} ===")
        scenario = generator.generate_scenario(
            name=s["name"],
            drone_count=s["drone_count"],
            delivery_count=s["delivery_count"],
            nfz_count=s["nfz_count"]
        )

        simulation = DroneDeliverySimulation(
            scenario["drones"],
            scenario["deliveries"],
            scenario["no_fly_zones"]
        )

        print("A* Algoritması çalıştırılıyor...")
        a_star_routes = simulation.run_a_star_simulation()

        print("Genetik Algoritma çalıştırılıyor...")
        genetic_routes = simulation.run_genetic_algorithm_simulation()

        report = simulation.generate_report()
        print("\n" + report)

        print("Görselleştirmeler oluşturuluyor...")
        fig_a_star = simulation.visualize_routes('a_star')
        plt.savefig(f"{s['name']}_routes_a_star.png", dpi=300, bbox_inches='tight')

        fig_genetic = simulation.visualize_routes('genetic')
        plt.savefig(f"{s['name']}_routes_genetic.png", dpi=300, bbox_inches='tight')

        print(f"{s['name']} görselleştirmeleri ve raporu tamamlandı.")