# City Traffic Simulation

A minimalist, top-down city traffic simulation built in Godot 4.7, designed as a calm, ambient dynamic wallpaper. Autonomous vehicles navigate a procedurally generated grid-based road network using A\* pathfinding.

## Features

- **Procedural road network** — two map generators selectable via `generator_type`: a Manhattan-style `grid` (with optional `block_jitter` for varied block spacing and `obstacle_count`/`obstacle_radius` for park-filled holes that force A\* detours), or a `street_network` built on a coherent aligned grid with **superblock tiling** — the cell grid is tiled with 1×1, 2×1, 1×2, and 2×2 blocks at equal weights, and roads are created only on block boundaries, so larger blocks produce longer uninterrupted road stretches and natural T-junctions. Optional 45°/135° diagonal avenues snap to grid intersections. New generators plug in via the `MapGenerator` Resource seam.
- **A\* pathfinding** — continuous A→B routing over the road graph with Manhattan-distance heuristic; vehicles repath to a **fresh boundary spawn point** and a new far goal on every arrival, so each trip is independent of the previous.
- **Arc-length bezier turns** — turns through intersections are quadratic bezier arcs, G1-continuous with the incoming/outgoing straights, so position and heading stay perfectly coupled.
- **Right-hand lane following** — all trajectories are offset to the right-hand lane; the car never enters oncoming traffic.
- **Acceleration / deceleration** — S-curve ramp up from standstill, smoothstep deceleration to stop at the destination, and apex-based slowdown through turns (slowest in the middle of each arc), with a windowed look-ahead so the car brakes before the turn and sustains the corner speed.
- **Brake lights** — taillights brighten in proportion to braking intensity, glow gently whenever the car eases off (coasts down toward a lower target), and stay **bright while stopped** at a zero target (future-proofs for traffic lights). Brake lights fire on any deceleration reason, not only turns.
- **Turn indicators** — amber blinkers at the corners on the turning side fire 0.2s on / 0.2s off as soon as a turn enters the look-ahead window (while the car is already decelerating toward it) and cancel once the turn leaves the window.
- **Always-on route visualization** — soft cyan route line plus green start (A) and red goal (B) rings, drawn on the same right-lane offset trajectory the vehicle follows.
- **Debug overlay** — F1 toggles raw graph nodes/edges; F5 regenerates the grid.

## Status

- [x] Phase 1 — Procedural road grid generation
- [x] Phase 2 — Vehicle & basic movement
- [x] Phase 3 — A\* pathfinding A→B with continuous repathing
- [ ] Phase 4 — Traffic lights & right-of-way
- [ ] Phase 5 — Multi-vehicle simulation & collision avoidance
- [ ] Phase 6 — Visual polish & wallpaper export

See [ARCHITECTURE.md](ARCHITECTURE.md) for the full design and class contracts, and the **Roadmap** below for where Phases 4–6 plug in.

## Run

```bash
make run
```

Or open `project.godot` in the Godot 4.7 editor and press Play (F5).

> Requires Godot 4.7. Edit the `GODOT` variable at the top of the `Makefile` if your Godot binary lives elsewhere.

## Controls

| Key | Action |
|-----|--------|
| F1  | Toggle debug overlay (graph nodes, edges, active path) |
| F5  | Regenerate the road grid |

## Architecture

```
Main (Node2D, main.gd) — entry point; injects RoadGrid into SimulationManager
├── RoadGrid (Node2D) — owns MapGenerator + RoadGraph; renders roads + route
└── SimulationManager (Node2D) — owns VehicleSpawner; spawns + repaths vehicles
    └── VehicleController (Node2D) — thin orchestrator
        ├── VehicleMover (RefCounted) — pure motion model + signals
        └── VehicleBody (Node2D) — composed visual scene (body + lights)
            ├── BodyShape (Polygon2D)
              └── 8× CircleDrawer (headlights, taillights, indicators)

MapGenerator (Resource) → RoadGraph (RefCounted) → TrajectoryBuilder (RefCounted)
                                                                     ↓
                                                     Trajectory (RefCounted)
                                                     wraps Array[TrajectorySegment]
                                                     (LineSeg / BezierSeg)
```

`Main` wires the two siblings via dependency injection (no hardcoded node paths). `SimulationManager` delegates spawn/repath policy to a `VehicleSpawner` (RefCounted) and holds `Array[VehicleController]` — ready for multi-vehicle. `VehicleController` is a thin orchestrator: it owns a `VehicleMover` (pure motion, emits `braking_changed`/`turn_indicator_changed`/`arrived` signals) and a `VehicleBody` (composed of child nodes driven by those signals). `RoadGrid` and `VehicleMover` both build their trajectory from the **same** `TrajectoryBuilder` via a cached `Trajectory` wrapper (DRY): the vehicle drives it, the grid draws it. A `VehicleSpec` Resource on the controller's `spec` export swaps vehicle types (sedan/truck/bus) without code changes.

See [ARCHITECTURE.md](ARCHITECTURE.md) for layer-by-layer contracts, algorithm details, and design decisions.

### Class responsibilities

| File | Class | Extends | Role |
|------|-------|---------|------|
| `scripts/map_generator.gd` | `MapGenerator` | `Resource` | Contract for procedural map generation (nodes + edges); subclasses saved as `.tres` presets |
| `scripts/grid_generator.gd` | `GridGenerator` | `MapGenerator` | Screen-filling Manhattan grid (nodes + 4-neighbourhood edges, optional obstacle holes) |
| `scripts/street_network_generator.gd` | `StreetNetworkGenerator` | `MapGenerator` | Variable grid + superblock tiling + 45° diagonals |
| `scripts/road_graph.gd` | `RoadGraph` | `RefCounted` | Holds the graph and runs A\* pathfinding |
| `scripts/trajectory_builder.gd` | `TrajectoryBuilder` | `RefCounted` | Converts a grid path into right-lane-offset `LineSeg`/`BezierSeg` (shared by driving + rendering) |
| `scripts/trajectory.gd` | `Trajectory` | `RefCounted` | Arc-length-parametrized wrapper over segments; single source of truth for segment lookup (DRY) |
| `scripts/trajectory_segment.gd` | `TrajectorySegment` | `RefCounted` | Base class: parametric position + tangent by arc length |
| `scripts/line_seg.gd` | `LineSeg` | `TrajectorySegment` | Straight segment (constant heading) |
| `scripts/bezier_seg.gd` | `BezierSeg` | `TrajectorySegment` | Quadratic bezier arc with an arc-length lookup table |
| `scripts/vehicle_spec.gd` | `VehicleSpec` | `Resource` | Inspector-configurable tuning + visual config for a vehicle type (swappable via `.tres`) |
| `scripts/vehicle_mover.gd` | `VehicleMover` | `RefCounted` | Pure motion model: accel/decel, turn slowdown, snap-to-arrival; emits `braking_changed`/`turn_indicator_changed`/`arrived` signals |
| `scripts/vehicle_body.gd` | `VehicleBody` | `Node2D` | Composed visual scene: body polygon + headlights/taillights/indicators driven by mover signals |
| `scripts/circle_drawer.gd` | `CircleDrawer` | `Node2D` | Minimal leaf node drawing a filled circle (lights) |
| `scripts/vehicle_controller.gd` | `VehicleController` | `Node2D` | Thin orchestrator: owns a mover + body, calls `mover.update(delta)`, re-emits `arrived` |
| `scripts/vehicle_spawner.gd` | `VehicleSpawner` | `RefCounted` | Spawn/repath policy: picks start/goal, instantiates vehicles, injects spec; multi-vehicle ready |
| `scripts/road_grid.gd` | `RoadGrid` | `Node2D` | Renders roads, lane markings, route visualization, debug overlay; owns the `MapGenerator` |
| `scripts/simulation_manager.gd` | `SimulationManager` | `Node2D` | Owns `VehicleSpawner`; spawns N vehicles, repaths on arrival, forwards route to `RoadGrid` |
| `scripts/main.gd` | `Main` | `Node2D` | Entry point: injects `RoadGrid` into `SimulationManager` (sibling mediation) |

## Configuration

### Grid (editable in `scenes/road_grid.tscn`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `screen_size` | `1280×720` | Viewport size the grid fills |
| `margin_px` | `40` | Margin kept clear around the screen edge |
| `target_block_size` | `128` | Target distance between intersections; actual block size is recomputed to fill the inner area exactly |
| `road_width` | `48` | Total road surface width (px) |
| `lane_width` | `24` | Single lane width (px) |
| `lane_offset` | `12` | Right-hand perpendicular offset (half a lane) |
| `turn_radius_for_route` | `22` | Pull-back before intersection used by the route line (matches the vehicle) |
| `generator_type` | `street_network` | Which built-in generator to use: `grid` (Manhattan grid) or `street_network` (variable grid + T-junctions + 45° diagonals). A `map_generator` .tres preset always takes priority. |
| `block_jitter` | `0` | Fraction of block size to vary each step by (0 = uniform grid, 0.25 = ±25% per step; renormalized to fill the area). Used by both generators (GridGenerator default 0, StreetNetworkGenerator default 0.25). |
| `obstacle_count` | `3` | Grid only: number of obstacle hole clusters carved out of the grid interior (0 = no holes; holes force A* to detour) |
| `obstacle_radius` | `2` | Grid only: radius of each hole cluster in graph hops (boundary nodes are protected and never removed) |
| `partial_road_fraction` | *(removed)* | Replaced by superblock tiling: the generator tiles the cell grid with 1×1, 2×1, 1×2, and 2×2 blocks at equal weights, creating roads only on block boundaries. Interior edges of superblocks are skipped, producing varied block sizes, T-junctions, and longer uninterrupted road stretches — no configuration needed. |
| `diagonal_count` | `2` | Street network only: number of 45°/135° diagonal avenues crossing the grid (0, 1, or 2; diagonals snap to grid intersections) |
| `snap_tolerance` | `24` | Street network only: max distance (px) for a diagonal node to snap to an existing grid node (merges crossing roads into 5-way junctions) |

### Vehicle (configurable via `VehicleSpec` Resource on `vehicle.tscn`)

All vehicle tuning and visual config lives in a `VehicleSpec` Resource (`scripts/vehicle_spec.gd`). Drop a saved `.tres` (e.g. `sedan.tres`, `truck.tres`) on the controller's `spec` export to swap types without code changes. Defaults (the "final feel" values):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | `80` | Cruising speed (px/s) |
| `accel_rate` | `50` | Acceleration, gentle (px/s²) |
| `decel_rate` | `120` | Braking deceleration, gentle (px/s²) |
| `decel_distance` | `90` | Distance before destination to start braking (px) |
| `turn_slowdown_factor` | `0.5` | Speed reduction per radian of turn |
| `min_turn_speed_ratio` | `0.3` | Floor on speed while in a turn (fraction of `max_speed`) |
| `turn_look_ahead` | `50` | Look-ahead window for upcoming turns (px) |
| `snap_distance` | `5` | Snap-to-arrival distance threshold (px) |
| `snap_speed_threshold` | `15` | Below this speed, snap-to-arrival triggers (px/s) |
| `lane_offset` | `12` | Right-hand lane offset (must match `RoadGrid`) |
| `turn_radius` | `22` | Pull-back before intersection for bezier arcs (must match `RoadGrid`) |
| `body_length` / `body_width` | `36` / `18` | Body polygon dimensions (px) |
| `body_color` | `#6b7280` | Body fill |
| `headlight_color` | `#fff4d6` | Headlight fill |
| `taillight_color` | `#ff5a4d` | Taillight fill |
| `indicator_color` | `#ff9926` | Turn indicator fill (amber) |
| `indicator_blink_period` | `0.4` | Indicator blink period (s; 0.2 on / 0.2 off) |

### SimulationManager (`scenes/main.tscn`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `spawn_count` | `1` | Number of vehicles to spawn at start (multi-vehicle-ready; route viz shows the most recent) |

### Map generation (`RoadGrid`)

`RoadGrid` builds a default `GridGenerator` from the screen/margin/block exports. To swap topologies, drop a saved `MapGenerator` `.tres` preset (e.g. a `HexGenerator.tres`) on the `map_generator` export — `RoadGrid` uses a duplicate so a shared preset isn't mutated.

## Testing

Unit tests use [GUT](https://github.com/bitwes/Gut) (already vendored in `addons/gut/`).

```bash
make test
```

| Test file | Covers |
|-----------|--------|
| `tests/unit/test_grid_generator.gd` | Grid dimensions, world-pos formula, boundary nodes, `far_from` |
| `tests/unit/test_map_generator.gd` | MapGenerator contract, GridGenerator conformance, custom subclass through the seam |
| `tests/unit/test_road_graph.gd` | A\* optimality, path contiguity, heuristic + edge-cost contracts |
| `tests/unit/test_trajectory.gd` | Arc-length accumulation, segment lookup with hint, position/tangent coupling |
| `tests/unit/test_trajectory_builder.gd` | Straight→LineSeg, turn→Bezier, contiguity, empty paths |
| `tests/unit/test_trajectory_segment.gd` | Base-class curvature + progress fraction |
| `tests/unit/test_line_seg.gd` | Endpoints, constant tangent, zero-length guard |
| `tests/unit/test_bezier_seg.gd` | Endpoints, tangent at start/end, arc-length LUT accuracy, curvature |
| `tests/unit/test_vehicle_mover.gd` | Smoothstep, turn factor (apex/windowed monotonicity), braking intensity (hard/coast/hold-still), upcoming turn direction |
| `tests/unit/test_vehicle_spec.gd` | Spec defaults, colors/dimensions, mover spec injection + write-through |
| `tests/unit/test_vehicle_spawner.gd` | Pick start/goal, spawn + path assignment, repath from last node |
| `tests/unit/test_integration.gd` | Full-trip right-lane invariant, segment contiguity end-to-end |

## Roadmap

- **Phase 4 — Traffic lights & right-of-way.** Intersection-scoped `TrafficLight` nodes; vehicles query the intersection state and stop on red. Plugs into the `VehicleMover` target-speed model — the existing "bright while stopped" braking case already handles the visual side, so only the target-speed source needs to learn about lights.
- **Phase 5 — Multi-vehicle & collision avoidance.** `SimulationManager` already holds `Array[VehicleController]` and a `VehicleSpawner` with configurable `spawn_count`; a coordination layer prevents overlapping paths and rear-end collisions. The `vehicle_path_assigned` signal on the spawner is the seam for per-vehicle route viz.
- **Phase 6 — Visual polish & wallpaper export.** `shaders/` and `assets/textures/` (currently stubs) get filled in; `CircleDrawer` lights can be swapped for `Sprite2D`/`PointLight2D` without touching `VehicleBody`; `export/` produces a wallpaper-ready build.

## Palette

| Element | Color |
|---------|-------|
| Asphalt | `#2b2b30` |
| Lane markings | `#5a5a64` |
| Vehicle body | `#6b7280` |
| Headlights | `#fff4d6` |
| Taillights | `#ff5a4d` |
| Turn indicators | `#ff9926` (amber, blinking) |
| Route line | `#7fd8ff` (~40% alpha) |
| Start (A) ring | `#7fff9a` |
| Goal (B) ring | `#ff7f7f` |

## Project structure

```
city-traffic-sim/
├── project.godot              # Godot 4.7 config (input map: toggle_debug, regenerate_grid)
├── scenes/
│   ├── main.tscn              # Root: Main + RoadGrid + SimulationManager
│   ├── road_grid.tscn         # Draws roads, owns MapGenerator + RoadGraph + debug overlay
│   └── vehicle.tscn           # Composed vehicle scene (Body + 8 CircleDrawer lights)
├── scripts/
│   ├── map_generator.gd       # MapGenerator contract (Resource base for .tres presets)
│   ├── grid_generator.gd      # Screen-filling Manhattan grid (extends MapGenerator)
│   ├── street_network_generator.gd  # Variable grid + superblock tiling + 45° diagonals (extends MapGenerator)
│   ├── road_graph.gd          # Graph + A* pathfinding
│   ├── trajectory_builder.gd  # Right-lane offset trajectory (DRY)
│   ├── trajectory.gd          # Arc-length-parametrized trajectory wrapper (DRY)
│   ├── trajectory_segment.gd  # Base parametric segment
│   ├── line_seg.gd            # Straight segment
│   ├── bezier_seg.gd          # Quadratic bezier arc with arc-length LUT
│   ├── vehicle_spec.gd        # VehicleSpec Resource (tuning + visuals, swappable)
│   ├── vehicle_mover.gd       # Pure motion model + signals (RefCounted)
│   ├── vehicle_body.gd        # Composed visual scene driven by mover signals
│   ├── circle_drawer.gd       # Minimal leaf node drawing a filled circle
│   ├── vehicle_controller.gd  # Thin orchestrator (mover + body)
│   ├── vehicle_spawner.gd     # Spawn/repath policy (multi-vehicle ready)
│   ├── road_grid.gd           # Rendering, lane markings, debug overlay
│   ├── simulation_manager.gd  # Owns spawner; spawns N vehicles, repaths on arrival
│   └── main.gd                # Entry point: injects RoadGrid into SimulationManager
├── tests/unit/                # GUT unit tests (12 files, 83 tests)
├── shaders/                   # Passthrough stubs (Phase 6 polish)
├── assets/textures/           # (Phase 6)
└── export/                    # (Phase 6)
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for setup, the quality gate (`make check`), and commit conventions. [AGENTS.md](AGENTS.md) documents the workflow standard for AI agents working on this repo.

## License

[MIT](LICENSE) — © 2026 Kristof Sweerts.