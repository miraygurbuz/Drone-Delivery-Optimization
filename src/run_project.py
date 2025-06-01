import os
import sys
import matplotlib.pyplot as plt
from datetime import datetime
from main import DroneDeliverySimulation
from utils.data_loader import drones, deliveries, no_fly_zones
from utils.random_data_generator import RandomDataGenerator, create_test_scenarios
from performance_tester import PerformanceTester
from utils.helpers import *

def run_main_simulation():
    """Ana simülasyon"""

    simulation = DroneDeliverySimulation(drones, deliveries, no_fly_zones)

    print("\n1. A* Algoritması çalıştırılıyor...")
    a_star_routes = simulation.run_a_star_simulation()
    print(f"Tamamlandı - {simulation.results['a_star']['metrics']['completed_deliveries']} teslimat")

    print("\n2. Genetik Algoritma çalıştırılıyor...")
    genetic_routes = simulation.run_genetic_algorithm_simulation()
    print(f"Tamamlandı - {simulation.results['genetic']['metrics']['completed_deliveries']} teslimat")

    print("\n3. Görselleştirmeler oluşturuluyor...")
    fig_a_star = simulation.visualize_routes('a_star')
    plt.savefig('results/figures/routes_a_star.png', dpi=300, bbox_inches='tight')
    plt.close()
    fig_genetic = simulation.visualize_routes('genetic')
    plt.savefig('results/figures/routes_genetic.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("Rota görselleştirmeleri tamamlandı")

    print("\n4. Detaylı rapor oluşturuluyor...")
    report = simulation.generate_report()
    with open('results/reports/main_simulation_report.txt', 'w', encoding='utf-8') as f:
        f.write(report)
    print("Rapor kaydedildi")

    return simulation

def run_time_complexity_analysis():
    """Zaman karmaşıklığı analizi"""

    deliveries_count = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    print(f"Test edilecek teslimat sayıları: {deliveries_count}")

    complexity_results = analyze_time_complexity(deliveries_count, n_drones=5)

    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(deliveries_count, complexity_results['a_star'], 'o-', label='A* Algoritması',
             linewidth=2, markersize=8, color='#1f77b4')
    plt.plot(deliveries_count, complexity_results['genetic'], 's-', label='Genetik Algoritma',
             linewidth=2, markersize=8, color='#ff7f0e')
    plt.xlabel('Teslimat Sayısı', fontsize=12)
    plt.ylabel('Çalışma Süresi (saniye)', fontsize=12)
    plt.title('Algoritma Zaman Karmaşıklığı', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(2, 1, 2)
    plt.semilogy(deliveries_count, complexity_results['a_star'], 'o-', label='A* Algoritması',
                 linewidth=2, markersize=8, color='#1f77b4')
    plt.semilogy(deliveries_count, complexity_results['genetic'], 's-', label='Genetik Algoritma',
                 linewidth=2, markersize=8, color='#ff7f0e')
    plt.xlabel('Teslimat Sayısı', fontsize=12)
    plt.ylabel('Çalışma Süresi (saniye) - Log Ölçek', fontsize=12)
    plt.title('Algoritma Zaman Karmaşıklığı (Logaritmik Ölçek)', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('results/figures/time_complexity_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()

    print("Zaman karmaşıklığı analizi tamamlandı")

    return complexity_results

def run_performance_tests():
    """Performans testleri"""

    generator = RandomDataGenerator()
    scenarios = create_test_scenarios()

    tester = PerformanceTester()
    results = tester.run_performance_tests(scenarios)

    print("\nPerformans grafikleri oluşturuluyor...")
    tester.generate_performance_charts()

    import shutil
    for filename in ['performance_comparison.png', 'scalability_analysis.png']:
        if os.path.exists(filename):
            shutil.move(filename, f'results/figures/{filename}')

    performance_report = tester.generate_detailed_report()
    with open('results/reports/performance_test_report.txt', 'w', encoding='utf-8') as f:
        f.write(performance_report)
    tester.save_results('results/data/performance_results.json')
    print("Performans testleri tamamlandı")

    return results

def main():
    print(f"Başlangıç Zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    try:
        print("\nAna simülasyon çalıştırılıyor...")
        simulation = run_main_simulation()

        print("\nZaman karmaşıklığı analizi yapılıyor...")
        complexity_results = run_time_complexity_analysis()

        print("\nPerformans testleri çalıştırılıyor...")
        performance_results = run_performance_tests()

        print(f"Bitiş Zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"A* Tamamlanma Oranı: {simulation.results['a_star']['metrics']['completion_rate']:.1f}%")
        print(f"GA Tamamlanma Oranı: {simulation.results['genetic']['metrics']['completion_rate']:.1f}%")

    except Exception as e:
        print(f"\nHATA: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())