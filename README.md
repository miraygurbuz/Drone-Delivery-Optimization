# Drone Delivery Optimization

This project focuses on route planning for drones under strict constraints such as battery limits, no-fly zones, weight capacity, and delivery time windows.  
Two main algorithms are implemented and compared: A* + CSP and Genetic Algorithm (GA).

[![Project Report](https://img.shields.io/badge/Report-grup28__rapor.pdf-blue)](https://github.com/miraygurbuz/Drone-Delivery-Optimization/blob/main/grup28_rapor.pdf)
[![Visualizations](https://img.shields.io/badge/Visualizations-PNGs-green)](https://github.com/miraygurbuz/Drone-Delivery-Optimization/tree/main/src/results/figures)

> You can find the full project report and route visualizations using the badges above.

## Setup & Run

* ### Clone the repository:

```
git clone https://github.com/miraygurbuz/Drone-Delivery-Optimization.git
cd Drone-Delivery-Optimization
```

* ### Install dependencies:

```
pip install -r requirements.txt
```

### Run Options:

* #### GUI Interface
Launch the graphical user interface for easy simulation management:
```bash
python src/gui.py
```
* #### Command Line Interface

If you're using Git Bash / Linux / macOS:

````
export PYTHONPATH=.
python src/run_project.py
````
If you're on Windows CMD:

```
set PYTHONPATH=.
py src/run_project.py
```
If you're on Windows PowerShell:

```
$env:PYTHONPATH="."
python src/run_project.py
```

## Algorithms Used

The following two optimization approaches are implemented and compared:

- A* + CSP (Constraint Satisfaction Problem)  
  Finds the most feasible route by evaluating weight, time, energy, and NFZ constraints.
 
  `heuristic = distance + nofly_zone_penalty`

- Genetic Algorithm (GA)  
  Produces near-optimal results using mutation, crossover, and a custom fitness function.

  `fitness = (deliveries × 50) − (energy × 0.1) − (violations × 100)`

## Graph Model
The delivery map is represented as a directed graph built from delivery nodes.

- **Nodes**: Delivery points
- **Edges**: All-to-all connections with custom cost calculations
- **Cost Formula**:
 
  `cost = distance × weight + (priority × 100)`

- **No-Fly Zone Penalty**:  If a path intersects an active NFZ, a penalty is added to discourage the route.

Each node stores:
- Target delivery ID (`to`)
- Distance between points
- Cost including weight and priority
- Whether the path violates an NFZ (`violates_nfz`)

## Data Models

The project uses three core data classes:

- **Drone**
  - `id`: Unique identifier
  - `max_weight`: Maximum load (kg)
  - `battery`: Battery capacity (mAh)
  - `speed`: Movement speed (m/s)
  - `start_pos`: Starting coordinates (x, y)

- **Delivery**
  - `id`: Unique delivery ID
  - `weight`: Package weight (kg)
  - `priority`: Importance level (1–5)
  - `time_window`: Acceptable delivery time (start, end)
  - `pos`: Delivery location (x, y)

- **NoFlyZone**
  - `id`: Zone identifier
  - `active_time`: Active period (start, end in minutes)
  - `coordinates`: Polygon corners [(x, y),..]

## Test Scenarios

You can run a predefined test suite using the `performance_tester.py` script:
```
python src/performance_tester.py
```

This script will:

- Run A* and GA for two randomized scenarios
- Generate a detailed performance report
- Save outputs to:
  
  - `performance_comparison.png` (chart)
  - `performance_report.txt` (summary text)
  - `performance_results.json` (raw data)
