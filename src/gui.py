import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
from datetime import datetime
import matplotlib.pyplot as plt
from main import DroneDeliverySimulation
from utils.data_loader import drones, deliveries, no_fly_zones
from utils.random_data_generator import RandomDataGenerator
from performance_tester import PerformanceTester

class DroneSimulationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Teslimat Simülasyonu")
        self.root.geometry("800x600")

        self.setup_ui()

        self.current_simulation = None

    def setup_ui(self):
        title_label = tk.Label(self.root, text="Drone Teslimat Simülasyonu",
                               font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=5)

        self.main_frame = ttk.Frame(notebook)
        notebook.add(self.main_frame, text="Ana Simülasyon")
        self.setup_main_simulation_tab()

        self.test_frame = ttk.Frame(notebook)
        notebook.add(self.test_frame, text="Test Senaryoları")
        self.setup_test_scenarios_tab()

        self.results_frame = ttk.Frame(notebook)
        notebook.add(self.results_frame, text="Sonuçlar")
        self.setup_results_tab()

    def setup_main_simulation_tab(self):
        info_frame = ttk.LabelFrame(self.main_frame, text="Veri Seti Bilgileri")
        info_frame.pack(fill="x", padx=10, pady=5)

        tk.Label(info_frame, text=f"Drone Sayısı: {len(drones)}").pack(anchor="w", padx=5)
        tk.Label(info_frame, text=f"Teslimat Sayısı: {len(deliveries)}").pack(anchor="w", padx=5)
        tk.Label(info_frame, text=f"No-Fly Zone Sayısı: {len(no_fly_zones)}").pack(anchor="w", padx=5)

        button_frame = ttk.Frame(self.main_frame)
        button_frame.pack(fill="x", padx=10, pady=10)

        self.run_main_btn = ttk.Button(button_frame, text="Ana Simülasyonu Çalıştır",
                                       command=self.run_main_simulation)
        self.run_main_btn.pack(side="left", padx=5)

        self.show_routes_btn = ttk.Button(button_frame, text="Rotaları Göster",
                                          command=self.show_routes, state="disabled")
        self.show_routes_btn.pack(side="left", padx=5)

        self.main_status = scrolledtext.ScrolledText(self.main_frame, height=15)
        self.main_status.pack(fill="both", expand=True, padx=10, pady=5)

    def setup_test_scenarios_tab(self):
        predefined_frame = ttk.LabelFrame(self.test_frame, text="Önceden Tanımlı Senaryolar")
        predefined_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(predefined_frame, text="Senaryo 1 (5 Drone, 20 Teslimat, 3 NFZ)",
                   command=lambda: self.run_predefined_scenario(1)).pack(anchor="w", padx=5, pady=2)

        ttk.Button(predefined_frame, text="Senaryo 2 (10 Drone, 50 Teslimat, 5 NFZ)",
                   command=lambda: self.run_predefined_scenario(2)).pack(anchor="w", padx=5, pady=2)

        ttk.Button(predefined_frame, text="Her İki Senaryoyu Çalıştır",
                   command=self.run_performance_tests).pack(anchor="w", padx=5, pady=2)

        custom_frame = ttk.LabelFrame(self.test_frame, text="Özel Senaryo")
        custom_frame.pack(fill="x", padx=10, pady=5)

        input_frame = ttk.Frame(custom_frame)
        input_frame.pack(fill="x", padx=5, pady=5)

        tk.Label(input_frame, text="Drone Sayısı:").grid(row=0, column=0, sticky="w", padx=5)
        self.drone_count_var = tk.StringVar(value="5")
        tk.Entry(input_frame, textvariable=self.drone_count_var, width=10).grid(row=0, column=1, padx=5)

        tk.Label(input_frame, text="Teslimat Sayısı:").grid(row=0, column=2, sticky="w", padx=5)
        self.delivery_count_var = tk.StringVar(value="20")
        tk.Entry(input_frame, textvariable=self.delivery_count_var, width=10).grid(row=0, column=3, padx=5)

        tk.Label(input_frame, text="NFZ Sayısı:").grid(row=1, column=0, sticky="w", padx=5)
        self.nfz_count_var = tk.StringVar(value="3")
        tk.Entry(input_frame, textvariable=self.nfz_count_var, width=10).grid(row=1, column=1, padx=5)

        ttk.Button(input_frame, text="Özel Senaryoyu Çalıştır",
                   command=self.run_custom_scenario).grid(row=1, column=2, columnspan=2, padx=5, pady=5)

        self.test_status = scrolledtext.ScrolledText(self.test_frame, height=12)
        self.test_status.pack(fill="both", expand=True, padx=10, pady=5)

    def setup_results_tab(self):
        self.results_text = scrolledtext.ScrolledText(self.results_frame, height=25)
        self.results_text.pack(fill="both", expand=True, padx=10, pady=10)

        ttk.Button(self.results_frame, text="Sonuçları Temizle",
                   command=self.clear_results).pack(pady=5)

    def log_message(self, message, widget=None):
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"

        if widget is None:
            widget = self.main_status

        widget.insert(tk.END, formatted_message)
        widget.see(tk.END)
        self.root.update()

    def run_main_simulation(self):
        """Ana simülasyonu çalıştır"""

        def simulate():
            try:
                self.run_main_btn.config(state="disabled")
                self.log_message("Ana simülasyon başlatılıyor...")

                simulation = DroneDeliverySimulation(drones, deliveries, no_fly_zones)
                self.current_simulation = simulation

                self.log_message("A* algoritması çalıştırılıyor...")
                simulation.run_a_star_simulation()
                a_star_completed = simulation.results['a_star']['metrics']['completed_deliveries']
                self.log_message(f"A* tamamlandı - {a_star_completed} teslimat")

                self.log_message("Genetik algoritma çalıştırılıyor...")
                simulation.run_genetic_algorithm_simulation()
                ga_completed = simulation.results['genetic']['metrics']['completed_deliveries']
                self.log_message(f"Genetik algoritma tamamlandı - {ga_completed} teslimat")

                report = simulation.generate_report()
                self.show_results(report)

                self.log_message("Simülasyon tamamlandı!")
                self.show_routes_btn.config(state="normal")

            except Exception as e:
                self.log_message(f"HATA: {str(e)}")
                messagebox.showerror("Hata", f"Simülasyon sırasında hata oluştu:\n{str(e)}")
            finally:
                self.run_main_btn.config(state="normal")

        threading.Thread(target=simulate, daemon=True).start()

    def run_predefined_scenario(self, scenario_num):
        """Önceden tanımlı senaryoları çalıştır"""

        def simulate():
            try:
                self.log_message(f"Senaryo {scenario_num} başlatılıyor...", self.test_status)

                generator = RandomDataGenerator()

                if scenario_num == 1:
                    scenario = generator.generate_scenario("Senaryo 1", 5, 20, 3)
                else:
                    scenario = generator.generate_scenario("Senaryo 2", 10, 50, 5)

                simulation = DroneDeliverySimulation(
                    scenario["drones"],
                    scenario["deliveries"],
                    scenario["no_fly_zones"]
                )

                self.log_message("A* algoritması çalıştırılıyor...", self.test_status)
                simulation.run_a_star_simulation()

                self.log_message("Genetik algoritma çalıştırılıyor...", self.test_status)
                simulation.run_genetic_algorithm_simulation()

                report = simulation.generate_report()
                self.show_results(f"=== SENARYO {scenario_num} SONUÇLARI ===\n\n{report}")

                self.log_message(f"Senaryo {scenario_num} tamamlandı!", self.test_status)

            except Exception as e:
                self.log_message(f"HATA: {str(e)}", self.test_status)
                messagebox.showerror("Hata", f"Senaryo {scenario_num} sırasında hata oluştu:\n{str(e)}")

        threading.Thread(target=simulate, daemon=True).start()

    def run_custom_scenario(self):
        """Özel senaryoyu çalıştır"""
        try:
            drone_count = int(self.drone_count_var.get())
            delivery_count = int(self.delivery_count_var.get())
            nfz_count = int(self.nfz_count_var.get())

            if drone_count <= 0 or delivery_count <= 0 or nfz_count < 0:
                messagebox.showerror("Hata", "Lütfen geçerli sayılar girin!")
                return

        except ValueError:
            messagebox.showerror("Hata", "Lütfen sayısal değerler girin!")
            return

        def simulate():
            try:
                self.log_message(f"Özel senaryo başlatılıyor ({drone_count}D, {delivery_count}T, {nfz_count}NFZ)...",
                                 self.test_status)

                generator = RandomDataGenerator()
                scenario = generator.generate_scenario(
                    f"Özel Senaryo ({drone_count}D-{delivery_count}T-{nfz_count}NFZ)",
                    drone_count, delivery_count, nfz_count
                )

                simulation = DroneDeliverySimulation(
                    scenario["drones"],
                    scenario["deliveries"],
                    scenario["no_fly_zones"]
                )

                self.log_message("A* algoritması çalıştırılıyor...", self.test_status)
                simulation.run_a_star_simulation()

                self.log_message("Genetik algoritma çalıştırılıyor...", self.test_status)
                simulation.run_genetic_algorithm_simulation()

                report = simulation.generate_report()
                self.show_results(f"=== ÖZEL SENARYO SONUÇLARI ===\n\n{report}")

                self.log_message("Özel senaryo tamamlandı!", self.test_status)

            except Exception as e:
                self.log_message(f"HATA: {str(e)}", self.test_status)
                messagebox.showerror("Hata", f"Özel senaryo sırasında hata oluştu:\n{str(e)}")

        threading.Thread(target=simulate, daemon=True).start()

    def run_performance_tests(self):
        """Performans testlerini çalıştır"""

        def test():
            try:
                self.log_message("Performans testleri başlatılıyor...", self.test_status)

                generator = RandomDataGenerator()
                scenarios = {
                    'scenario1': generator.generate_scenario("Senaryo 1", 5, 20, 3),
                    'scenario2': generator.generate_scenario("Senaryo 2", 10, 50, 5)
                }

                tester = PerformanceTester()
                results = tester.run_performance_tests(scenarios)

                performance_report = tester.generate_detailed_report()
                self.show_results(f"=== PERFORMANS TESTİ SONUÇLARI ===\n\n{performance_report}")

                self.log_message("Performans testleri tamamlandı!", self.test_status)

            except Exception as e:
                self.log_message(f"HATA: {str(e)}", self.test_status)
                messagebox.showerror("Hata", f"Performans testleri sırasında hata oluştu:\n{str(e)}")

        threading.Thread(target=test, daemon=True).start()

    def show_routes(self):
        """Rotaları göster"""
        if self.current_simulation is None:
            messagebox.showwarning("Uyarı", "Önce ana simülasyonu çalıştırın!")
            return

        try:
            fig_a_star = self.current_simulation.visualize_routes('a_star')
            fig_a_star.suptitle("A* Algoritması Rotaları", fontsize=16)
            plt.show()

            fig_genetic = self.current_simulation.visualize_routes('genetic')
            fig_genetic.suptitle("Genetik Algoritma Rotaları", fontsize=16)
            plt.show()

        except Exception as e:
            messagebox.showerror("Hata", f"Rotalar gösterilirken hata oluştu:\n{str(e)}")

    def show_results(self, report):
        """Sonuçları göster"""
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, report)

    def clear_results(self):
        """Sonuçları temizle"""
        self.results_text.delete(1.0, tk.END)


def main():
    root = tk.Tk()
    app = DroneSimulationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()