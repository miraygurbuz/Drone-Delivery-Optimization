import os
from typing import List, Dict, Tuple

def load_data_from_file(filename: str = "data/veri_seti.txt") -> Tuple[List[Dict], List[Dict], List[Dict]]:
    try:
        if not os.path.exists(filename):
            print(f"Uyarı: {filename} bulunamadı. Varsayılan veriler kullanılıyor.")
            return get_default_data()

        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()

        namespace = {}
        exec(content, namespace)

        drones = namespace.get('drones', [])
        deliveries = namespace.get('deliveries', [])
        no_fly_zones = namespace.get('no_fly_zones', [])

        print(f"{len(drones)} drone")
        print(f"{len(deliveries)} teslimat noktası")
        print(f"{len(no_fly_zones)} no-fly zone")

        return drones, deliveries, no_fly_zones

    except Exception as e:
        print(f"Hata: {e}")
        print("Varsayılan veriler kullanılıyor.")
        return get_default_data()

def get_default_data() -> Tuple[List[Dict], List[Dict], List[Dict]]:
    default_drones = [
        {"id": 1, "max_weight": 4.0, "battery": 12000, "speed": 8.0, "start_pos": (10, 10)},
        {"id": 2, "max_weight": 3.5, "battery": 10000, "speed": 10.0, "start_pos": (20, 30)},
        {"id": 3, "max_weight": 5.0, "battery": 15000, "speed": 7.0, "start_pos": (50, 50)},
        {"id": 4, "max_weight": 2.0, "battery": 8000, "speed": 12.0, "start_pos": (80, 20)},
        {"id": 5, "max_weight": 6.0, "battery": 20000, "speed": 5.0, "start_pos": (40, 70)}
    ]

    default_deliveries = [
        {"id": 1, "pos": (15, 25), "weight": 1.5, "priority": 3, "time_window": (0, 60)},
        {"id": 2, "pos": (30, 40), "weight": 2.0, "priority": 5, "time_window": (0, 30)},
        {"id": 3, "pos": (70, 80), "weight": 3.0, "priority": 2, "time_window": (20, 80)},
        {"id": 4, "pos": (90, 10), "weight": 1.0, "priority": 4, "time_window": (10, 40)},
        {"id": 5, "pos": (45, 60), "weight": 4.0, "priority": 1, "time_window": (30, 90)},
        {"id": 6, "pos": (25, 15), "weight": 2.5, "priority": 3, "time_window": (0, 50)},
        {"id": 7, "pos": (60, 30), "weight": 1.0, "priority": 5, "time_window": (5, 25)},
        {"id": 8, "pos": (85, 90), "weight": 3.5, "priority": 2, "time_window": (40, 100)},
        {"id": 9, "pos": (10, 80), "weight": 2.0, "priority": 4, "time_window": (15, 45)},
        {"id": 10, "pos": (95, 50), "weight": 1.5, "priority": 3, "time_window": (0, 60)},
        {"id": 11, "pos": (55, 20), "weight": 0.5, "priority": 5, "time_window": (0, 20)},
        {"id": 12, "pos": (35, 75), "weight": 2.0, "priority": 1, "time_window": (50, 120)},
        {"id": 13, "pos": (75, 40), "weight": 3.0, "priority": 3, "time_window": (10, 50)},
        {"id": 14, "pos": (20, 90), "weight": 1.5, "priority": 4, "time_window": (30, 70)},
        {"id": 15, "pos": (65, 65), "weight": 4.5, "priority": 2, "time_window": (25, 75)},
        {"id": 16, "pos": (40, 10), "weight": 2.0, "priority": 5, "time_window": (0, 30)},
        {"id": 17, "pos": (5, 50), "weight": 1.0, "priority": 3, "time_window": (15, 55)},
        {"id": 18, "pos": (50, 85), "weight": 3.0, "priority": 1, "time_window": (60, 100)},
        {"id": 19, "pos": (80, 70), "weight": 2.5, "priority": 4, "time_window": (20, 60)},
        {"id": 20, "pos": (30, 55), "weight": 1.5, "priority": 2, "time_window": (40, 80)}
    ]

    default_no_fly_zones = [
        {
            "id": 1,
            "coordinates": [(40, 30), (60, 30), (60, 50), (40, 50)],
            "active_time": (0, 120)
        },
        {
            "id": 2,
            "coordinates": [(70, 10), (90, 10), (90, 30), (70, 30)],
            "active_time": (30, 90)
        },
        {
            "id": 3,
            "coordinates": [(10, 60), (30, 60), (30, 80), (10, 80)],
            "active_time": (0, 60)
        }
    ]

    return default_drones, default_deliveries, default_no_fly_zones

drones, deliveries, no_fly_zones = load_data_from_file()

def validate_data(drones: List[Dict], deliveries: List[Dict],
                  no_fly_zones: List[Dict]) -> bool:
    try:
        for drone in drones:
            assert 'id' in drone and isinstance(drone['id'], int)
            assert 'max_weight' in drone and drone['max_weight'] > 0
            assert 'battery' in drone and drone['battery'] > 0
            assert 'speed' in drone and drone['speed'] > 0
            assert 'start_pos' in drone and len(drone['start_pos']) == 2

        for delivery in deliveries:
            assert 'id' in delivery and isinstance(delivery['id'], int)
            assert 'pos' in delivery and len(delivery['pos']) == 2
            assert 'weight' in delivery and delivery['weight'] > 0
            assert 'priority' in delivery and 1 <= delivery['priority'] <= 5
            assert 'time_window' in delivery and len(delivery['time_window']) == 2

        for nfz in no_fly_zones:
            assert 'id' in nfz and isinstance(nfz['id'], int)
            assert 'coordinates' in nfz and len(nfz['coordinates']) >= 3
            assert 'active_time' in nfz and len(nfz['active_time']) == 2

        return True

    except AssertionError as e:
        print(f"Hata: {e}")
        return False

if not validate_data(drones, deliveries, no_fly_zones):
    drones, deliveries, no_fly_zones = get_default_data()