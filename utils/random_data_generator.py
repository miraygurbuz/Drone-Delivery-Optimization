import random
import json
from typing import List, Dict, Tuple
class RandomDataGenerator:

    def __init__(self, map_size: Tuple[int, int] = (100, 100)):
        self.map_size = map_size

    def generate_drones(self, count: int = 5) -> List[Dict]:
        drones = []

        for i in range(count):
            drone = {
                "id": i + 1,
                "max_weight": round(random.uniform(2.0, 6.0), 1),
                "battery": random.randint(8000, 20000),
                "speed": round(random.uniform(5.0, 15.0), 1),
                "start_pos": (
                    round(random.uniform(0, self.map_size[0]), 1),
                    round(random.uniform(0, self.map_size[1]), 1)
                )
            }
            drones.append(drone)

        return drones

    def generate_deliveries(self, count: int = 20) -> List[Dict]:
        deliveries = []

        for i in range(count):
            start_time = random.randint(0, 60)
            end_time = start_time + random.randint(20, 60)

            delivery = {
                "id": i + 1,
                "pos": (
                    round(random.uniform(0, self.map_size[0]), 1),
                    round(random.uniform(0, self.map_size[1]), 1)
                ),
                "weight": round(random.uniform(0.5, 5.0), 1),
                "priority": random.randint(1, 5),
                "time_window": (start_time, end_time)
            }
            deliveries.append(delivery)

        return deliveries

    def generate_no_fly_zones(self, count: int = 3) -> List[Dict]:
        no_fly_zones = []

        for i in range(count):
            center_x = random.uniform(20, self.map_size[0] - 20)
            center_y = random.uniform(20, self.map_size[1] - 20)

            width = random.uniform(10, 30)
            height = random.uniform(10, 30)

            coordinates = [
                (center_x - width / 2, center_y - height / 2),
                (center_x + width / 2, center_y - height / 2),
                (center_x + width / 2, center_y + height / 2),
                (center_x - width / 2, center_y + height / 2)
            ]

            coordinates = [(round(x, 1), round(y, 1)) for x, y in coordinates]

            start_time = random.randint(0, 60)
            end_time = start_time + random.randint(30, 60)

            nfz = {
                "id": i + 1,
                "coordinates": coordinates,
                "active_time": (start_time, min(end_time, 120))
            }
            no_fly_zones.append(nfz)

        return no_fly_zones

    def generate_scenario(self, name: str, drone_count: int, delivery_count: int,
                          nfz_count: int) -> Dict:
        scenario = {
            "name": name,
            "drones": self.generate_drones(drone_count),
            "deliveries": self.generate_deliveries(delivery_count),
            "no_fly_zones": self.generate_no_fly_zones(nfz_count)
        }
        return scenario

    def save_scenario(self, scenario: Dict, filename: str):
        """Senaryoyu JSON dosyasına kaydeder"""
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(scenario, f, indent=2, ensure_ascii=False)

def create_test_scenarios():
    """Farklı test senaryoları oluşturur"""
    generator = RandomDataGenerator()

    scenario1 = generator.generate_scenario(
        name="Küçük Ölçek Test",
        drone_count=5,
        delivery_count=20,
        nfz_count=2
    )

    scenario2 = generator.generate_scenario(
        name="Orta Ölçek Test",
        drone_count=10,
        delivery_count=50,
        nfz_count=5
    )

    scenarios = {
        'scenario1': scenario1,
        'scenario2': scenario2,
    }

    return scenarios

if __name__ == "__main__":
    scenarios = create_test_scenarios()

    generator = RandomDataGenerator()
    for name, scenario in scenarios.items():
        filename = f"{name}.json"
        generator.save_scenario(scenario, filename)
        print(f"  - Drone sayısı: {len(scenario['drones'])}")
        print(f"  - Teslimat sayısı: {len(scenario['deliveries'])}")
        print(f"  - No-fly zone sayısı: {len(scenario['no_fly_zones'])}")
        print()