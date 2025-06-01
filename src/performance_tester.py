import time
import matplotlib.pyplot as plt
import numpy as np
from typing import Dict
import json
from main import DroneDeliverySimulation
from utils.random_data_generator import RandomDataGenerator

class PerformanceTester:

    def __init__(self):
        self.results = {
            'scenarios': [],
            'a_star': {},
            'genetic': {},
            'comparison': {}
        }

    def run_performance_tests(self, scenarios: Dict) -> Dict:
        for scenario_name, scenario in scenarios.items():
            print(f"\n{'=' * 60}")
            print(f"Test Senaryosu: {scenario['name']}")
            print(f"{'=' * 60}")

            sim = DroneDeliverySimulation(
                scenario['drones'],
                scenario['deliveries'],
                scenario['no_fly_zones']
            )

            print("A* algoritması test ediliyor...")
            a_star_start = time.time()
            sim.run_a_star_simulation()
            a_star_time = time.time() - a_star_start

            print("Genetik algoritma test ediliyor...")
            ga_start = time.time()
            sim.run_genetic_algorithm_simulation()
            ga_time = time.time() - ga_start

            self.results['scenarios'].append({
                'name': scenario['name'],
                'drone_count': len(scenario['drones']),
                'delivery_count': len(scenario['deliveries']),
                'nfz_count': len(scenario['no_fly_zones'])
            })

            a_star_metrics = sim.results['a_star']['metrics']
            self.results['a_star'][scenario_name] = {
                'metrics': a_star_metrics,
                'execution_time': a_star_time
            }

            ga_metrics = sim.results['genetic']['metrics']
            self.results['genetic'][scenario_name] = {
                'metrics': ga_metrics,
                'execution_time': ga_time
            }

            self.results['comparison'][scenario_name] = {
                'completion_rate_diff': a_star_metrics['completion_rate'] - ga_metrics['completion_rate'],
                'time_ratio': a_star_time / ga_time if ga_time > 0 else 0,
            }

            print(f"  A* - Tamamlanma: {a_star_metrics['completion_rate']:.1f}%, "
                  f"Süre: {a_star_time:.3f}s")
            print(f"  GA - Tamamlanma: {ga_metrics['completion_rate']:.1f}%, "
                  f"Süre: {ga_time:.3f}s")

        return self.results

    def generate_performance_charts(self):
        scenario_names = list(self.results['a_star'].keys())

        # Tamamlanma Oranı Karşılaştırması
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        a_star_completion = [self.results['a_star'][s]['metrics']['completion_rate']
                             for s in scenario_names]
        ga_completion = [self.results['genetic'][s]['metrics']['completion_rate']
                         for s in scenario_names]

        x = np.arange(len(scenario_names))
        width = 0.35

        ax1.bar(x - width / 2, a_star_completion, width, label='A*', color='#1f77b4')
        ax1.bar(x + width / 2, ga_completion, width, label='GA', color='#ff7f0e')
        ax1.set_xlabel('Senaryo')
        ax1.set_ylabel('Tamamlanma Oranı (%)')
        ax1.set_title('Algoritma Tamamlanma Oranları')
        ax1.set_xticks(x)
        ax1.set_xticklabels([s.replace('scenario', 'S') for s in scenario_names])
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Çalışma Süresi Karşılaştırması
        a_star_times = [self.results['a_star'][s]['execution_time'] for s in scenario_names]
        ga_times = [self.results['genetic'][s]['execution_time'] for s in scenario_names]

        ax2.bar(x - width / 2, a_star_times, width, label='A*', color='#2ca02c')
        ax2.bar(x + width / 2, ga_times, width, label='GA', color='#d62728')
        ax2.set_xlabel('Senaryo')
        ax2.set_ylabel('Çalışma Süresi (saniye)')
        ax2.set_title('Algoritma Çalışma Süreleri')
        ax2.set_xticks(x)
        ax2.set_xticklabels([s.replace('scenario', 'S') for s in scenario_names])
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('performance_comparison.png', dpi=300, bbox_inches='tight')

    def generate_detailed_report(self) -> str:
        report = []

        total_scenarios = len(self.results['scenarios'])
        report.append(f"Toplam test edilen senaryo sayısı: {total_scenarios}")

        for i, scenario in enumerate(self.results['scenarios']):
            scenario_name = f"scenario{i + 1}"
            report.append(f"\n{'=' * 60}")
            report.append(f"SENARYO: {scenario['name']}")
            report.append(f"{'=' * 60}")
            report.append(f"Drone Sayısı: {scenario['drone_count']}")
            report.append(f"Teslimat Sayısı: {scenario['delivery_count']}")
            report.append(f"No-Fly Zone Sayısı: {scenario['nfz_count']}")
            report.append("")

            a_star = self.results['a_star'][scenario_name]
            report.append("A* Algoritması Sonuçları:")
            report.append(f"  - Tamamlanma Oranı: {a_star['metrics']['completion_rate']:.2f}%")
            report.append(f"  - Toplam Enerji: {a_star['metrics']['total_energy']:.2f} mAh")
            report.append(f"  - Ortalama Enerji/Teslimat: {a_star['metrics']['avg_energy_per_delivery']:.2f} mAh")
            report.append(f"  - Çalışma Süresi: {a_star['execution_time']:.4f} saniye")
            report.append("")

            ga = self.results['genetic'][scenario_name]
            report.append("Genetik Algoritma Sonuçları:")
            report.append(f"  - Tamamlanma Oranı: {ga['metrics']['completion_rate']:.2f}%")
            report.append(f"  - Toplam Enerji: {ga['metrics']['total_energy']:.2f} mAh")
            report.append(f"  - Ortalama Enerji/Teslimat: {ga['metrics']['avg_energy_per_delivery']:.2f} mAh")
            report.append(f"  - Final Fitness: {ga['metrics']['final_fitness']:.2f}")
            report.append(f"  - Çalışma Süresi: {ga['execution_time']:.4f} saniye")
            report.append("")

            # Karşılaştırma
            comp = self.results['comparison'][scenario_name]
            report.append("Karşılaştırma:")
            report.append(f"  - Tamamlanma Farkı: {comp['completion_rate_diff']:.2f}%")
            report.append(f"  - Süre Oranı (A*/GA): {comp['time_ratio']:.2f}")

        best_a_star_scenario = max(self.results['a_star'].items(),
                                   key=lambda x: x[1]['metrics']['completion_rate'])
        best_ga_scenario = max(self.results['genetic'].items(),
                               key=lambda x: x[1]['metrics']['completion_rate'])

        report.append(f"\nEn İyi A* Performansı: {best_a_star_scenario[0]}")
        report.append(f"  - Tamamlanma Oranı: {best_a_star_scenario[1]['metrics']['completion_rate']:.2f}%")

        report.append(f"\nEn İyi GA Performansı: {best_ga_scenario[0]}")
        report.append(f"  - Tamamlanma Oranı: {best_ga_scenario[1]['metrics']['completion_rate']:.2f}%")

        avg_a_star_completion = np.mean([r['metrics']['completion_rate']
                                         for r in self.results['a_star'].values()])
        avg_ga_completion = np.mean([r['metrics']['completion_rate']
                                     for r in self.results['genetic'].values()])

        report.append(f"\nOrtalama Tamamlanma Oranları:")
        report.append(f"  - A*: {avg_a_star_completion:.2f}%")
        report.append(f"  - GA: {avg_ga_completion:.2f}%")

        report.append(f"\nZaman Karmaşıklığı Değerlendirmesi:")
        delivery_counts = [s['delivery_count'] for s in self.results['scenarios']]
        a_star_times = [self.results['a_star'][f'scenario{i + 1}']['execution_time']
                        for i in range(len(self.results['scenarios']))]
        ga_times = [self.results['genetic'][f'scenario{i + 1}']['execution_time']
                    for i in range(len(self.results['scenarios']))]

        from scipy import stats

        a_star_slope, _, _, _, _ = stats.linregress(delivery_counts, a_star_times)
        ga_slope, _, _, _, _ = stats.linregress(delivery_counts, ga_times)

        report.append(f"  - A* zaman artış oranı: {a_star_slope:.6f} saniye/teslimat")
        report.append(f"  - GA zaman artış oranı: {ga_slope:.6f} saniye/teslimat")

        if a_star_slope < ga_slope:
            report.append(f"  - A* algoritması büyük veri setlerinde daha iyi ölçekleniyor")
        else:
            report.append(f"  - GA algoritması büyük veri setlerinde daha iyi ölçekleniyor")

        report.append("\n" + "=" * 80)

        return "\n".join(report)

    def save_results(self, filename: str = "performance_results.json"):
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)


def main():
    print("Performans testleri başlatılıyor...")
    generator = RandomDataGenerator()
    scenarios = {
        'scenario1': generator.generate_scenario("Senaryo 1", 5, 20, 2),
        'scenario2': generator.generate_scenario("Senaryo 2", 10, 50, 5),
    }
    tester = PerformanceTester()
    results = tester.run_performance_tests(scenarios)
    tester.generate_performance_charts()
    report = tester.generate_detailed_report()
    print("\n" + report)
    tester.save_results()

    with open("performance_report.txt", 'w', encoding='utf-8') as f:
        f.write(report)

    print("Oluşturulan dosyalar:")
    print("  - performance_comparison.png")
    print("  - performance_results.json")
    print("  - performance_report.txt")


if __name__ == "__main__":
    main()