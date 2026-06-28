# City Traffic Simulation

A minimalist, top-down city traffic simulation built in Godot 4.7, designed as a calm, ambient dynamic wallpaper. Autonomous vehicles navigate a procedurally generated grid-based road network using A\* pathfinding.

## Features

- **Procedural road grid** — a uniform Manhattan-style grid that fills the screen, recomputed from a target block size.
- **A\* pathfinding** — continuous A→B routing over the grid graph with Manhattan-distance heuristic; vehicles repath to a new random destination on every arrival.
- **Arc-length bezier turns** — turns through intersections are quadratic bezier arcs, G1-continuous with the incoming/outgoing straights, so position and heading stay perfectly coupled.
- **Right-hand lane following** — all trajectories are offset to the right-hand lane; the car never enters oncoming traffic.
- **Acceleration / deceleration** — S-curve ramp up from standstill, smoothstep deceleration to stop at the destination, and apex-based slowdown through turns (slowest in the middle of each arc).
- **Brake lights** — taillights brighten in proportion to braking intensity.
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
GridGenerator (RefCounted) → RoadGraph (RefCounted) → TrajectoryBuilder (RefCounted)
                                                                    ↓
                                                    Array[TrajectorySegment]
                                                    (LineSeg / BezierSeg)
                                                                    ↓
                                          VehicleController (Node2D) — drives
                                          RoadGrid (Node2D) — draws route line
                                          SimulationManager (Node2D) — orchestrates
```

`SimulationManager` spawns a vehicle, assigns an A\* path, and connects the vehicle's `arrived` signal to repath from the current location. `VehicleController` and `RoadGrid` both build their trajectory from the **same** `TrajectoryBuilder` (DRY): the vehicle drives it, the grid draws it.

See [ARCHITECTURE.md](ARCHITECTURE.md) for layer-by-layer contracts, algorithm details, and design decisions.

### Class responsibilities

| File | Class | Extends | Role |
|------|-------|---------|------|
| `scripts/grid_generator.gd` | `GridGenerator` | `RefCounted` | Builds the screen-filling Manhattan grid (nodes + 4-neighbourhood edges) |
| `scripts/road_graph.gd` | `RoadGraph` | `RefCounted` | Holds the graph and runs A\* pathfinding |
| `scripts/trajectory_builder.gd` | `TrajectoryBuilder` | `RefCounted` | Converts a grid path into right-lane-offset `LineSeg`/`BezierSeg` (shared by driving + rendering) |
| `scripts/trajectory_segment.gd` | `TrajectorySegment` | `RefCounted` | Base class: parametric position + tangent by arc length |
| `scripts/line_seg.gd` | `LineSeg` | `TrajectorySegment` | Straight segment (constant heading) |
| `scripts/bezier_seg.gd` | `BezierSeg` | `TrajectorySegment` | Quadratic bezier arc with an arc-length lookup table |
| `scripts/vehicle_controller.gd` | `VehicleController` | `Node2D` | Drives the trajectory: accel/decel, turn slowdown, brake lights, emits `arrived` |
| `scripts/road_grid.gd` | `RoadGrid` | `Node2D` | Renders roads, lane markings, route visualization, debug overlay |
| `scripts/simulation_manager.gd` | `SimulationManager` | `Node2D` | Spawns vehicles, assigns paths, repaths on arrival |

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

### Vehicle (editable in `scenes/vehicle.tscn`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | `120` | Cruising speed (px/s) |
| `accel_rate` | `180` | Acceleration (px/s²) |
| `decel_rate` | `300` | Braking deceleration, stronger than accel (px/s²) |
| `decel_distance` | `60` | Distance before destination to start braking (px) |
| `turn_slowdown_factor` | `0.3` | Speed reduction per radian of turn |
| `min_turn_speed_ratio` | `0.35` | Floor on speed while in a turn (fraction of `max_speed`) |
| `snap_distance` | `5` | Snap-to-arrival distance threshold (px) |
| `snap_speed_threshold` | `15` | Below this speed, snap-to-arrival triggers (px/s) |
| `lane_offset` | `12` | Right-hand lane offset (must match `RoadGrid`) |
| `turn_radius` | `22` | Pull-back before intersection for bezier arcs (must match `RoadGrid`) |

## Testing

Unit tests use [GUT](https://github.com/bitwes/Gut) (already vendored in `addons/gut/`).

```bash
make test
```

| Test file | Covers |
|-----------|--------|
| `tests/unit/test_grid_generator.gd` | Grid dimensions, world-pos formula, boundary nodes, `far_from` |
| `tests/unit/test_road_graph.gd` | A\* optimality, path contiguity, heuristic + edge-cost contracts |
| `tests/unit/test_trajectory_builder.gd` | Straight→LineSeg, turn→Bezier, contiguity, empty paths |
| `tests/unit/test_trajectory_segment.gd` | Base-class curvature + progress fraction |
| `tests/unit/test_line_seg.gd` | Endpoints, constant tangent, zero-length guard |
| `tests/unit/test_bezier_seg.gd` | Endpoints, tangent at start/end, arc-length LUT accuracy, curvature |
| `tests/unit/test_vehicle_logic.gd` | Smoothstep boundaries, braking intensity, point-to-segment distance |
| `tests/unit/test_integration.gd` | Full-trip right-lane invariant, segment contiguity end-to-end |

## Roadmap

- **Phase 4 — Traffic lights & right-of-way.** Intersection-scoped `TrafficLight` nodes; vehicles query the intersection state and stop on red. Plugs into the `SimulationManager` orchestration layer and the `VehicleController` target-speed model.
- **Phase 5 — Multi-vehicle & collision avoidance.** `SimulationManager` spawns many vehicles; a coordination layer prevents overlapping paths and rear-end collisions. The single-vehicle `arrived` signal contract generalizes directly.
- **Phase 6 — Visual polish & wallpaper export.** `shaders/` and `assets/textures/` (currently stubs) get filled in; `export/` produces a wallpaper-ready build.

## Palette

| Element | Color |
|---------|-------|
| Asphalt | `#2b2b30` |
| Lane markings | `#5a5a64` |
| Vehicle body | `#6b7280` |
| Headlights | `#fff4d6` |
| Taillights | `#ff5a4d` |
| Route line | `#7fd8ff` (~40% alpha) |
| Start (A) ring | `#7fff9a` |
| Goal (B) ring | `#ff7f7f` |

## Project structure

```
city-traffic-sim/
├── project.godot              # Godot 4.7 config
├── scenes/
│   ├── main.tscn              # Root: RoadGrid + SimulationManager
│   ├── road_grid.tscn         # Draws roads, owns RoadGraph + debug overlay
│   └── vehicle.tscn           # Vehicle scene (body + lights)
├── scripts/
│   ├── grid_generator.gd      # Screen-filling uniform Manhattan grid
│   ├── road_graph.gd          # Graph + A* pathfinding
│   ├── trajectory_builder.gd  # Right-lane offset trajectory (DRY)
│   ├── trajectory_segment.gd  # Base parametric segment
│   ├── line_seg.gd            # Straight segment
│   ├── bezier_seg.gd          # Quadratic bezier arc with arc-length LUT
│   ├── vehicle_controller.gd  # Path follow, lane offset, smooth turns
│   ├── road_grid.gd           # Rendering, lane markings, debug overlay
│   └── simulation_manager.gd  # Spawn, path assignment, repath on arrival
├── tests/unit/                # GUT unit tests (8 files, 40 tests)
├── shaders/                   # Passthrough stubs (Phase 6 polish)
├── assets/textures/           # (Phase 6)
└── export/                    # (Phase 6)
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for setup, the quality gate (`make check`), and commit conventions. [AGENTS.md](AGENTS.md) documents the workflow standard for AI agents working on this repo.

## License

[MIT](LICENSE) — © 2026 Kristof Sweerts.