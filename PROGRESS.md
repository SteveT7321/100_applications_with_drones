# Progress Tracker

> Format: `- [ ]` Not started　`- [~]` In progress　`- [x]` Completed

---

## Notes: 2D vs 3D Classification (S001–S015)

雖然所有場景都用 Matplotlib 3D 呈現，實際上大多數是固定 z 軸的 2D 模擬：

| 分類 | Scenarios | 說明 |
|------|-----------|------|
| **真正 3D**（z 動態變化） | S001, S003 | S001 出發/目標 z 不同；S003 地形追蹤 |
| **固定 z=2m**（本質 2D） | S002, S004–S014 | 所有位置 z 硬編碼為 2.0；部分有明確歸零 acc[2]=0 |
| **純 2D**（無 z 座標） | S015 | 完全用 np.array([x,y])，感測器陣列為 2D |

> **待優化**：S016+ 可考慮對純 2D 場景直接使用 2D 視覺化（Matplotlib 2D），
> 或保留 3D 框架但明確標注 z 固定，避免誤導。

---

## Domain 1: Pursuit & Evasion — 18/20

### 1v1 Scenarios
- [x] **S001** Basic Intercept — Pursuer tracks a stationary target using Proportional Navigation Guidance
- [x] **S002** Evasive Maneuver — Evader executes optimal evasion, pursuer uses pure pursuit
- [x] **S003** Low-Altitude Tracking — Terrain-hugging pursuit under terrain occlusion
- [x] **S004** Obstacle-Course Chase — Pursuit in an environment with static obstacles
- [x] **S005** Stealth Approach — Pursuer attempts to enter from radar blind spots
- [x] **S006** Energy Race — Both sides have battery limits, optimal energy consumption strategy
- [x] **S007** Blind Pursuit under Jamming — Sensor noise maximized
- [x] **S008** Stochastic Pursuit — Target moves randomly, pursuer's optimal estimation
- [x] **S009** Differential Game 1v1 — Lion & Man problem numerical solution
- [x] **S010** Asymmetric Speed — Time-optimal pursuit: slow pursuer vs fast evader

### Nv1 Scenarios
- [x] **S011** Swarm Encirclement — Multiple drones converging simultaneously from different angles
- [x] **S012** Relay Pursuit — When battery runs low, another drone takes over
- [x] **S013** Pincer Movement — Three drones form a triangle formation to cut off escape routes
- [x] **S014** Decoy Lure — One drone acts as decoy, others flank
- [x] **S015** Communication Relay Tracking — Target exceeds single drone sensor range

### NvM Scenarios
- [x] **S016** Airspace Defense — Defending multi-drone formation vs attacking multi-drone breakthrough
- [x] **S017** Swarm vs Swarm — Self-organizing confrontation with 3~5 drones on each side
- [x] **S018** Multi-Target Interception — Optimal assignment problem (Hungarian algorithm)
- [ ] **S019** Dynamic Reassignment — Real-time adjustment when targets disappear or new ones appear
- [ ] **S020** Pursuit-Evasion Game — Full differential game solution

---

## Domain 2: Logistics & Delivery — 0/20

### 1v1 Scenarios
- [ ] **S021** Point Delivery — Shortest path from A to B with time-optimal landing
- [ ] **S022** Obstacle Avoidance Delivery — RRT* path planning
- [ ] **S023** Moving Landing Pad — Landing on a simulated moving truck
- [ ] **S024** Wind Compensation — Trajectory correction under crosswind
- [ ] **S025** Payload CoG Offset — Attitude compensation under asymmetric payload

### Nv1 Scenarios
- [ ] **S026** Cooperative Heavy Lift — 2~4 drones suspending a single heavy load
- [ ] **S027** Aerial Refueling Relay — Low-battery drone resupplied by tanker drone
- [ ] **S028** Cargo Escort Formation — Escort drone formation

### NvM Scenarios
- [ ] **S029** Urban Logistics Scheduling — Multi-depot multi-target VRP
- [ ] **S030** Multi-Depot Delivery — Which drone departs from which depot
- [ ] **S031** Path De-confliction — Time-based/altitude-based separation for multiple drones in the same airspace
- [ ] **S032** Charging Queue — Battery management + mission continuity
- [ ] **S033** Online Order Insertion — Inserting new targets mid-flight
- [ ] **S034** Weather Rerouting — Dynamic rerouting around strong wind zones
- [ ] **S035** UTM Simulation — Multiple drones requesting corridors, conflict alerts
- [ ] **S036** Last-Mile Relay — Long-distance delivery beyond single drone range
- [ ] **S037** Reverse Logistics — Recovering damaged drones or cargo
- [ ] **S038** Disaster Relief Drop — Precision airdrop without landing zones
- [ ] **S039** Offshore Platform Exchange — Landing under vessel pitch and roll
- [ ] **S040** Fleet Load Balancing — Dynamically redistributing mission load across drones

---

## Domain 3: Environmental & SAR — 0/20

- [ ] **S041** Wildfire Boundary Scan
- [ ] **S042** Missing Person Search
- [ ] **S043** Confined Space Exploration
- [ ] **S044** Wall Crack Inspection
- [ ] **S045** Chemical Plume Tracing
- [ ] **S046** 3D Trilateration
- [ ] **S047** Signal Relay Enhancement
- [ ] **S048** Lawnmower Coverage
- [ ] **S049** Dynamic Zone Assignment
- [ ] **S050** Swarm SLAM
- [ ] **S051** Post-Disaster Comm Network
- [ ] **S052** Glacier Area Monitoring
- [ ] **S053** Coral Reef 3D Mapping
- [ ] **S054** Minefield Detection
- [ ] **S055** Oil Spill Tracking
- [ ] **S056** Radiation Hotspot Detection
- [ ] **S057** Wildlife Census
- [ ] **S058** Typhoon Eye Penetration
- [ ] **S059** Sonar Buoy Relay
- [ ] **S060** Meteorological Profiling

---

## Domain 4: Industrial & Agriculture — 0/20

- [ ] **S061** Power Line Inspection
- [ ] **S062** Wind Turbine Blade Inspection
- [ ] **S063** Precision Per-Plant Spraying
- [ ] **S064** Greenhouse Interior Navigation
- [ ] **S065** Building 3D Scan Path
- [ ] **S066** Cooperative Crane Simulation
- [ ] **S067** Spray Overlap Optimization
- [ ] **S068** Large-Scale Field Spraying
- [ ] **S069** Automated Warehouse Inventory
- [ ] **S070** Swarm Weeding
- [ ] **S071** Bridge Underside Inspection
- [ ] **S072** Pipeline Leak Detection
- [ ] **S073** Solar Panel Thermal Inspection
- [ ] **S074** Mine 3D Mapping
- [ ] **S075** Container Yard Inventory
- [ ] **S076** Thermal Management in Hot Environment
- [ ] **S077** Precision Pollination
- [ ] **S078** Harvester Guidance
- [ ] **S079** Offshore Wind Installation
- [ ] **S080** Underground Pipe Inspection

---

## Domain 5: Special Ops & Entertainment — 0/20

- [ ] **S081** Selfie Follow Mode
- [ ] **S082** FPV Racing
- [ ] **S083** Light Show Single Drone Test
- [ ] **S084** High-Wind Endurance Test
- [ ] **S085** Light Matrix Positioning
- [ ] **S086** Multi-Angle Cinematography
- [ ] **S087** LED Formation Text
- [ ] **S088** Formation Shape Morphing
- [ ] **S089** Large-Scale Collision Avoidance
- [ ] **S090** Racing Optimal Path
- [ ] **S091** Acrobatic Maneuvers
- [ ] **S092** Movie Chase Scene
- [ ] **S093** Rubble Search and Rescue
- [ ] **S094** Counter-Drone Intercept
- [ ] **S095** Water Surface Takeoff/Landing
- [ ] **S096** Drone Relay Race
- [ ] **S097** Aerial Puzzle Assembly
- [ ] **S098** Swarm Synchronized Dance
- [ ] **S099** Obstacle Relay Pass
- [ ] **S100** Grand Challenge: Multi-Domain Integration Scenario

---

## Statistics

| Domain | Completed | Total |
|--------|-----------|-------|
| Pursuit & Evasion | 18 | 20 |
| Logistics & Delivery | 0 | 20 |
| Environmental & SAR | 0 | 20 |
| Industrial & Agriculture | 0 | 20 |
| Special Ops & Entertainment | 0 | 20 |
| **Total** | **18** | **100** |
