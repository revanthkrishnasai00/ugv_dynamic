# UGV Navigation in a Dynamic, Partially Unknown Environment

## 1. Problem Statement
In the earlier static problem, all obstacles were known in advance, allowing a single A\* run to find the shortest path. In the real world, this assumption fails:
* Some obstacles are **unknown** until the UGV's sensors detect them.
* Some obstacles are **dynamic** (moving), meaning a previously clear path can become blocked unexpectedly.

**Goal:** Ensure the UGV reaches the goal safely and efficiently by the shortest possible distance, even as the map changes and remains only partially known.

---

## 2. Key Idea: Plan, Sense, Replan
A single offline plan cannot stay valid in a changing environment, so the UGV operates in a continuous loop. It maintains a **belief map** of what it currently knows and replans whenever that belief changes.

At every time step:
1. **Sense:** Detect obstacles within the sensor range and update the belief map.
2. **Check:** Determine if any cell on the remaining path is now blocked.
3. **Replan:** If yes, run A\* from the current position to the goal on the updated map.
4. **Act:** Move one step along the path, or wait if no safe path exists.
5. **World Changes:** Moving obstacles take their next step.

The loop repeats until the UGV reaches the goal or the maximum step limit is hit.

---

## 3. State-Space Formulation
The formulation mirrors the static problem, but adapts to runtime discovery:

| Element | Definition |
| :--- | :--- |
| **State** | $(x, y)$: the cell the UGV currently occupies |
| **Initial state** | The UGV's current cell (changes dynamically at each replan) |
| **Goal test** | `state == GOAL` |
| **Actions** | 8 moves (N, S, E, W and 4 diagonals) |
| **Transition model** | Illegal if the move leaves the grid, enters a cell the UGV believes is blocked, or cuts an obstacle corner. |
| **Step cost** | 1 (straight) or $\sqrt{2} \approx 1.414$ (diagonal) |
| **Belief map** | Known buildings + discovered hidden static obstacles + moving obstacles currently in sensor range. |

*Note: The environment is only partially observable. The UGV plans based on its belief map, not the true world.*

---

## 4. Handling Uncertainty

### Unknown Static Obstacles
These are hidden at the start. When one enters sensor range, it is permanently added to the belief map.

### Moving Obstacles
Because their positions change every step, they are **not remembered long-term**. Only those currently visible in sensor range are treated as obstacles. 
* A **safety margin of one cell** around each visible moving obstacle is also blocked to prevent the UGV from planning a path right next to something that may step into it.

---

## 5. Algorithm Used: A* with Replanning
Each plan is generated using standard A\* with $f(n) = g(n) + h(n)$ (where $h$ is the Euclidean straight-line distance to the goal). Replanning is triggered only when the active path becomes blocked, preventing needless computation.

### Pseudocode Outline
```python
belief = known_buildings
position = START

while position != GOAL and steps < STEP_LIMIT:
  # 1. Sense environment
  belief.update(hidden_obstacles_in_range)
  visible_moving = get_moving_obstacles_in_range_with_margin()

  # 2. Check if current plan is invalidated
  if no_plan or any(cell in plan for cell in visible_moving):
    plan = astar(position, GOAL, belief + visible_moving)

  # 3. Act or wait
  if plan exists:
    position = plan.next_step()
  else:
    wait()

  # 4. World updates
  update_moving_obstacles()

```

---

## 6. Alternative Algorithms

| Algorithm | Core Idea | Best Suited For |
| --- | --- | --- |
| **A* with Replanning (Used Here)** | Rerun A* when the plan is blocked | Small maps; simple to implement |
| **D* Lite** | Searches backward from the goal and reuses previous results, updating only affected cells | Large maps with unknown/changing terrain |
| **Anytime Repairing A*** | Generates a quick rough path first, improving it iteratively over time | Time-critical decisions |
| **Space-Time A*** | Adds time as a state dimension $(x, y, t)$ to avoid predicted obstacle positions | Predictable obstacle motion |
| **Local Planners (DWA, Potential Fields)** | React to nearby obstacles continuously in real-time | Fast-moving obstacles |

*(A practical deployment often combines a global planner like D* Lite with a local reactive planner. Only A* with replanning was implemented and tested here.)*

---

## 7. Implementation (`ugv_dynamic.py`)

| Part | Purpose |
| --- | --- |
| `BUILDINGS`, `known` | Known baseline obstacles from the static version |
| `actions()`, `astar()` | State-space transitions and A* search (modified to accept variable start coordinates) |
| `simulate(n_dynamic, seed)` | Executes a full trial: sense, replan, act, and update world state |
| `hidden` | 15 unknown static obstacles (~5% of the grid) |
| `dyn` | Moving obstacles; each takes one random step per time step |
| **Main loop** | Runs 50 random trials per density level, reports aggregate results, and plots a sample run |

### Settings

* **Grid Size:** $20 \times 20$ cells (25 m per cell)
* **Sensor Range:** 3 cells
* **Safety Margin:** 1 cell around moving obstacles
* **Step Limit:** 200 steps
* **Dynamic Obstacle Levels:** Low = 3, Medium = 8, High = 15 moving obstacles

---

## 8. Measures of Effectiveness (MOEs)

| MOE | Meaning |
| --- | --- |
| **Success rate (%)** | Share of trials in which the UGV successfully reached the goal |
| **Distance travelled (m)** | Actual length of the route driven |
| **Efficiency vs reference (%)** | Reference shortest distance ($\div$ distance travelled $\times 100$), where reference is the static optimum (281.1 m) |
| **Replans** | Number of times A* had to be re-run |
| **Waits** | Steps spent waiting because no safe path existed |
| **Collisions** | Total times a moving obstacle stepped into the UGV's cell across all 50 runs |

---

## 9. Results

*Each level was evaluated over 50 trials with distinct random seeds (averages shown for successful runs; collisions represent absolute totals over all 50 runs).*

| Metric | Low (3 Moving) | Medium (8 Moving) | High (15 Moving) |
| --- | --- | --- | --- |
| **Success rate (%)** | 100 | 100 | 100 |
| **Distance travelled (m)** | 322.5 | 345.7 | 443.5 |
| **Efficiency vs reference (%)** | 87.2 | 81.3 | 63.4 |
| **Average replans** | 2.1 | 3.6 | 7.4 |
| **Average waits** | 0.1 | 0.8 | 2.2 |
| **Collisions (50 runs)** | 0 | 4 | 11 |

### Observations

1. **Robust Success:** The UGV achieved a 100% success rate across all traffic densities, proving that sense-and-replan handles unknown static and dynamic blocks effectively.
2. **Increased Travel Distance:** Due to late obstacle discovery and required detours, travel distances exceeded the 281.1 m static reference, scaling upwards as crowd density increased.
3. **Computational Overhead:** Both replan frequency and waiting steps increased proportionally with the number of moving obstacles.
4. **Collision Risk:** Collisions occurred when unpredictable moving obstacles stepped directly into the UGV's position faster than the reaction/replan loop could clear. While the 1-cell safety margin mitigated this, it did not eliminate it entirely.
5. **Dynamic Optimality:** In a changing world, "optimal" shifts from a fixed global path to the best possible route given the UGV's current local awareness.

---

## 10. How to Run

1. Ensure Python 3 is installed.
2. Install the required plotting package:
```bash
pip install matplotlib

```


3. Run the simulation script:
```bash
python ugv_dynamic.py

```


* Performance metrics will print directly to your terminal, and the visual trajectory map will be saved as `ugv_dynamic.png`.



---

## 11. Assumptions and Limitations

* Moving obstacles follow a random walk (one cell per step) and do not actively react to the UGV's presence.
* Sensors are modeled as an idealized circle with a 3-cell radius, ignoring noise, occlusion, or blocked line-of-sight limits.
* The UGV moves at parity speed with dynamic obstacles (one cell per step).
* Results depend on the test seed range (`0` to `49`); alternate seeds will yield minor statistical variations.
* Collisions are logged for metrics tracking but carry no runtime physical penalty. A production-grade design would integrate trajectory prediction or expand safety margins.
* Advanced global/local architectures (like D* Lite or Space-Time A*) are outlined for theoretical comparison but left unimplemented.

```

```
