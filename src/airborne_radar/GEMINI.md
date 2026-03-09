# Module Background

The `airborne_radar` module is a high-fidelity simulation library designed to replace legacy monolithic radar models.This module adopts a "Control Plane vs. Data Plane" separation, ensuring microsecond-level deterministic performance while maintaining architectural flexibility for diverse simulation scenarios.    

# Module Structure

The radar system is partitioned into four distinct architectural layers, each with strict boundary constraints:

```text
src/airborne_radar/
├── common/                      # Internal utility classes
├── core/                        # [Core Processing Layer] System Mediator & Lifecycle
│   ├── controller/              # RadarController: The central mediator for scheduling
│   ├── event/                   # EventBus: Asynchronous decoupling for high-frequency data
│   └── context/                 # TacticalContext: Payload container for state and commands
├── decision/                    # [Behavior Decision Layer] Tactical Brain (Pipeline)
│   ├── pipeline/                # ITacticalProcessor: Logic for sequential tactical nodes
│   ├── classifier/              # TargetClassifier: Feature-based threat identification
│   ├── lpi/                     # LpiController: Low Probability of Intercept strategies
│   └── eccm/                    # EccmController: Electronic Counter-Countermeasures
├── signal/                      # [Signal Processing Layer] High-speed Data Pump
│   ├── pipeline/                # ISignalPipeline: Drivers for the detection-tracking loop
│   ├── detection/               # SignalDetector: Pulse compression and CFAR algorithms
│   └── tracking/                # TrackFilter: Kalman/JPDA tracking and association
└── environment/                 # [Environment Modeling Layer] Physical Infrastructure
    ├── scene/                   # SceneManager: Unified target and clutter management
    ├── database/                # FeatureRepository: Anti-corruption O(1) memory cache
    └── simulation/              # PropagationModel: Physical loss and interference simulation
```

# Module Architectural Design

- Control Plane vs Data Plane separation: decision logic stays pure, execution is centralized in core.
- Core layer acts as the mediator and lifecycle scheduler; environment layer is the unified infrastructure boundary.
- Decision layer is a strict pipeline (Chain of Responsibility) over `ITacticalProcessor` with `TacticalContext` as the payload carrier.
- Actions are collected as commands in context and executed by core in a unified step (CQRS-style separation).
- Signal layer queries environment only through abstract services (e.g., `IEnvironmentService`) to enforce DIP.
- EventBus delivers high-frequency, cross-layer data; repositories serve static features from in-memory cache.
- No Direct Implement Inclusion: decision/signal `.cpp` files must not include concrete classes from `environment`.














