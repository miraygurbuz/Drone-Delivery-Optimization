from typing import List
from collections import defaultdict
from models.delivery import Delivery
from models.no_fly_zone import NoFlyZone
from utils.helpers import calculate_distance, line_intersects_polygon

class DeliveryGraph:
    def __init__(self, deliveries: List[Delivery], no_fly_zones: List[NoFlyZone]):
        self.deliveries = {d.id: d for d in deliveries}
        self.no_fly_zones = no_fly_zones
        self.adjacency_list = defaultdict(list)
        self._build_graph()

    def _build_graph(self):

        # Komşuluk Listesi
        for d1_id, d1 in self.deliveries.items():
            for d2_id, d2 in self.deliveries.items():
                if d1_id != d2_id:
                    # Maliyet Hesaplaması
                    distance = calculate_distance(d1.pos, d2.pos)
                    cost = distance * d2.weight + (d2.priority * 100)

                    # No Fly Zone Kontrolü
                    violates_nfz = False
                    for nfz in self.no_fly_zones:
                        if line_intersects_polygon(d1.pos, d2.pos, nfz.coordinates):
                            violates_nfz = True
                            cost += 10000
                            break

                    self.adjacency_list[d1_id].append({
                        'to': d2_id,
                        'cost': cost,
                        'distance': distance,
                        'violates_nfz': violates_nfz
                    })