# Architecture

This document describes the internal design of the City Traffic Simulation: the data flow, each layer's responsibilities and contracts, the key algorithms, and the design decisions behind them. For the user-facing overview, see [README.md](README.md); for the contributor workflow, see [CONTRIBUTING.md](CONTRIBUTING.md).

## 1. Overview

The simulation is a single-vehicle, top-down traffic demo built in Godot 4.7, designed as a calm ambient wallpaper. In its current state (Phase 3) one vehicle continuously drives A→B routes across a procedurally generated grid: it picks a far destination, follows an A\* path, arrives, and repaths from its current location — forever.

The design is deliberately layered and decoupled:

- **Data classes** (`RefCounted`) hold the grid, graph, and trajectory — they never enter the `SceneTree`.
- **Scene `Node`s** render and drive — they own references to the data classes but contain no pathfinding or geometry logic themselves.
- A single **trajectory builder** is the source of truth for both driving and rendering, so the car and the route line never disagree.

## 2. System diagram

```
                        ┌─────────────────────┐
                        │  GridGenerator      │  RefCounted
                        │  (nodes + edges)    │  no SceneTree
                        └──────────┬──────────┘
                                   │ build()
                                   ▼
                        ┌─────────────────────┐
                        │  RoadGraph          │  RefCounted
                        │  (A* pathfinding)   │  no SceneTree
                        └──────────┬──────────┘
                                   │ find_path() → Array[Vector2i]
                                   ▼
       ┌──────────────────────────────────────────────────────┐
       │  TrajectoryBuilder.build()  (static, RefCounted)    │
       │  → Array[TrajectorySegment]  (LineSeg / BezierSeg)   │
       └───────────────┬──────────────────────┬───────────────┘
                      │                      │
            used by driving          used by rendering
                      │                      │
                      ▼                      ▼
        ┌──────────────────────┐   ┌──────────────────────┐
        │ VehicleController    │   │ RoadGrid             │  both Node2D
        │ (Node2D)             │   │ (Node2D)             │  in SceneTree
        │ - drives the traj    │   │ - draws the traj     │
        │ - emits `arrived`    │   │ - draws roads/grid   │
        └──────────┬───────────┘   └──────────▲───────────┘
                   │  signal arrived            │ set_route()
                   ▼                            │
        ┌──────────────────────────────────────────────────┐
        │ SimulationManager (Node2D)                       │
        │ - spawns vehicles                                 │
        │ - assigns A* paths                                │
        │ - repaths on arrival (connects `arrived`)         │
        └────────────────────────────────────────────────────┘
```

Ownership: `SimulationManager` is a sibling of `RoadGrid` under the root `main.tscn`. `SimulationManager` reaches `RoadGrid` via `@onready var road_grid = $"../RoadGrid"` and reads its `graph`/`generator`. Child→parent communication uses the `arrived` **signal**, not string paths into the parent.

## 3. Layer by layer

### 3.1 Grid layer — `GridGenerator`

`scripts/grid_generator.gd` — `class_name GridGenerator extends RefCounted`

Generates a uniform Manhattan grid that fills the screen minus a margin.

**Algorithm (`generate()`):**
1. Inner area = `screen_size - 2 * margin_px`.
2. `cols = floor(inner_w / target_block_size) + 1`, same for `rows`.
3. Recompute `block_w = inner_w / (cols - 1)` so the grid fills the inner area exactly (the target is a hint, not a hard size).
4. Create one node per `(c, r)` at `world_pos(c, r) = margin + (c, r) * block`.
5. Wire a **4-neighbourhood**: for each cell, connect to `(c+1, r)` and `(c, r+1)` if in range. `_connect(a, b)` appends each to the other's edge list (undirected).

**Public surface:**
- `world_pos(c, r)` — grid coords → world `Vector2`.
- `grid_to_world(key)` — `Vector2i` → world `Vector2` via the `nodes` dict.
- `boundary_nodes()` — perimeter nodes (spawn candidates).
- `far_from(key, min_distance)` — nodes at least `min_distance` Manhattan steps away (destination candidates).
- `all_nodes()` — every key.

**Invariants:**
- `nodes` maps `Vector2i → Vector2` (world position).
- `edges` maps `Vector2i → Array[Vector2i]` (4-neighbourhood, symmetric).
- `cols`, `rows`, `block_w`, `block_h` are valid only after `generate()`.

### 3.2 Graph layer — `RoadGraph`

`scripts/road_graph.gd` — `class_name RoadGraph extends RefCounted`

Holds a deep copy of the grid's nodes/edges and runs A\*.

**A\* implementation (`find_path`):**
- `edge_cost(a, b)` = Euclidean distance between world positions.
- `heuristic(a, b)` = Manhattan distance (admissible for 4-connected grids with axis-aligned edges — never overestimates Euclidean distance on this grid).
- Open set is a plain `Array` with a **linear scan** for the lowest `f_score`. Grids are small (hundreds of nodes), so the O(n) scan is simpler and fast enough; a heap is YAGNI here.
- `g_score` / `f_score` / `came_from` are `Dictionary`s keyed by `Vector2i`.
- `_reconstruct` walks `came_from` backward and `push_front`s to build the start→goal path.
- Returns `[]` if either endpoint is absent or no path exists; returns `[start]` if `start == goal`.

**Contract:** `build(generator)` deep-copies (`duplicate(true)`) so later grid regeneration does not mutate a graph already in use.

### 3.3 Trajectory layer

The trajectory layer converts a grid path (a list of intersection keys) into a continuous, right-lane-offset sequence of parametric segments. This is the heart of the sim's "smooth motion" guarantee.

#### `TrajectorySegment` (base) — `scripts/trajectory_segment.gd`

`class_name TrajectorySegment extends RefCounted`

Abstract base. Holds `length` (total arc length, px) and declares the polymorphic interface:
- `position_at(s_local) → Vector2` — world position at arc-length `s_local` within this segment.
- `tangent_at(s_local) → float` — heading (radians) at `s_local`.
- `curvature_at(s_local) → float` — total turn angle (0 for straights, >0 for arcs); used for turn slowdown.
- `progress_fraction(s_local) → float` — `[0, 1]` position within the segment, for apex weighting.

Subclasses override `position_at` / `tangent_at` / `curvature_at`. The base `push_error`s if `position_at`/`tangent_at` are called unimplemented.

#### `LineSeg` — `scripts/line_seg.gd`

Straight segment. Trivial: `dir = (end - start).normalized()`, `length = (end - start).length()`.
- `position_at(s) = start + dir * s`
- `tangent_at(_) = dir.angle()` (constant)
- `curvature_at(_) = 0` (inherited default)

#### `BezierSeg` — `scripts/bezier_seg.gd`

Quadratic bezier arc `B(t) = (1-t)²·p0 + 2(1-t)t·control + t²·p1`.

The key problem: the vehicle moves at **constant arc-length speed**, but a bezier is parameterized by `t`, which is not arc-length. The solution is a **lookup table** built at construction:

- `_build_arc_length_lut()` samples `B(t)` at `LUT_SIZE = 64` uniform `t` values, accumulating chord lengths into `_cumulative_arc` and recording the corresponding `_t_values`. `length` = final cumulative value.
- `_t_for_arc(s_local)` **binary-searches** `_cumulative_arc` for the bracket containing `s_local`, then linearly interpolates `t` within the bracket for sub-sample precision.
- `position_at(s) = _eval_bezier(_t_for_arc(s))`
- `tangent_at(s) = _eval_derivative(_t_for_arc(s)).angle()`
- `curvature_at(_) = total_turn_angle` — cached at construction as `abs(angle_difference(tangent_at(0), tangent_at(1)))`.

Because position and tangent both come from the **same** `_t_for_arc(s)` evaluation, the car body is always exactly tangent to the curve it travels — no skating, no speed-dependent heading lag.

#### `TrajectoryBuilder` — `scripts/trajectory_builder.gd`

`class_name TrajectoryBuilder extends RefCounted` — a single **static** method `build()`.

This is the DRY linchpin: both `VehicleController` (driving) and `RoadGrid` (rendering the route line) call the same `TrajectoryBuilder.build()`, so the car and the drawn line can never diverge.

**Algorithm (`build()`):**
1. For each consecutive pair in the path, compute direction `d = (b - a).normalized()` and the **right-hand perpendicular** `perp = Vector2(-d.y, d.x)`. In y-down screen space, this points to the right of travel. Offset both endpoints: `entry = a + perp * lane_offset`, `exit = b + perp * lane_offset`.
2. Walk the segments. For each:
   - If it's the last segment, or the next segment continues in the same direction (`dot(dir[i], dir[i+1]) >= 0.99`), emit a single `LineSeg` from `current_pos` to `exits[i]`.
   - Otherwise it's a **turn**:
     - Compute `tr = min(turn_radius, seg_len * 0.4, next_seg_len * 0.4)` clamped to `>= 2.0` so the arc never overshoots a short segment.
     - `approach = exits[i] - dir[i] * tr` (pull back before the intersection).
     - `leave = entries[i+1] + dir[i+1] * tr` (push forward after).
     - Emit `LineSeg(current_pos → approach)` if non-trivial.
     - Compute the bezier **control point** as the intersection of the two offset tangent lines (the line through `approach` in direction `dir[i]`, and the line through `leave` in direction `dir[i+1]`). Solved analytically via a 2D cross-product formula; degenerate (parallel) cases fall back to a `LineSeg`. This produces a **G1-continuous** arc: the tangent matches the incoming straight at `approach` and the outgoing straight at `leave`.
     - Emit `BezierSeg(approach, control, leave)`.
     - `current_pos = leave`.

**Contract:** returns `Array[TrajectorySegment]` (empty if `path.size() < 2`). Segments are contiguous: each segment's end equals the next segment's start (verified by `test_trajectory_builder.gd` contiguity tests and `test_integration.gd`).

### 3.4 Vehicle layer — `VehicleController`

`scripts/vehicle_controller.gd` — `class_name VehicleController extends Node2D`

Drives the trajectory. The central abstraction is a single **arc-length cursor** `s` (px) that walks from `0` to `total_length` across the whole concatenated trajectory. Each frame:

1. Compute `target_speed = _target_speed_at(s)`.
2. Rate-limit `current_speed` toward `target` using `accel_rate` / `decel_rate` (no instantaneous jumps → no jerk).
3. Advance: `s += current_speed * delta`.
4. **Snap-to-arrival safeguard:** if within `snap_distance` of the end and below `snap_speed_threshold`, or within `_eff_decel` and below 5 px/s, clamp `s = total_length`. Prevents hovering at near-zero speed.
5. `_advance_segment_index()` walks the cached `seg_index` forward until `s` falls in the current segment (O(1) amortized; indices only move forward).
6. `local_s = s - seg_start_arc[seg_index]`; `position_on_road = seg.position_at(local_s)`; `heading = seg.tangent_at(local_s)`. Both from the same parametric evaluation.
7. `queue_redraw()`.

**Speed model (`_target_speed_at`):**
- **End ramp:** `end_factor = smoothstep((total_length - s) / _eff_decel)`. Smoothstep `t*t*(3-2t)` has zero derivative at `t=0` and `t=1`, giving a gentle stop with no kink. `_eff_decel = min(decel_distance, total_length * 0.4)` so short trips still fit the ramp. Sampled **only at the current `s`** — never at the look-ahead point — so the car never starts stopping early.
- **Turn slowdown (apex-based with look-ahead):** the turn factor is computed by the pure helper `_turn_factor_at(s_pos)`: locate the segment containing `s_pos`, and if its `curvature_at > 0`, compute `progress = progress_fraction(local_s)` and a **triangle weight** `apex_weight = 1 - |progress - 0.5| * 2` (0 at entry, 1 at the apex, 0 at exit), then `turn_factor = max(1 - turn_angle * turn_slowdown_factor * apex_weight, min_turn_speed_ratio)`. On straight segments (and past `total_length`) it returns `1.0`.
- **Look-ahead:** `_target_speed_at` takes `turn_factor = min(_turn_factor_at(s), _turn_factor_at(s + turn_look_ahead))`. Sampling `turn_look_ahead` px ahead and keeping the lower of the two makes the car **decelerate before entering a turn** (anticipative braking) instead of only reacting once inside the arc. Because `_turn_factor_at` returns `1.0` past the end, the look-ahead never spuriously lowers the target near the destination. Recovery after a turn is governed by the gentle `accel_rate`, so the car spools back up lazily rather than snapping to cruise.
- `target = max_speed * end_factor * turn_factor`.

Note the **start acceleration** is *not* modeled by `_target_speed_at` — there is no `start_factor` of 0 at `s=0`. This is deliberate: a target of 0 at the start would be a chicken-and-egg trap (the car could never begin moving). Instead, `current_speed` starts at 0 and ramps up via `accel_rate` toward `max_speed` naturally.

**Brake lights (`_braking_intensity`):** returns `clamp((current_speed - target) / (max_speed * 0.2), 0, 1)` — brightens taillights proportional to how hard the car is decelerating relative to its target. The denominator scales with `max_speed` so a modest overshoot lights the brakes fully; the baseline taillight alpha is low (0.35) so braking to 1.0 reads clearly.

**Current segment estimation (`_estimate_current_segment`):** nearest grid edge to `position_on_road` by point-to-segment distance, returning the originating `Vector2i`. Used for diagnostics and as a stable per-trip segment key.

**Signal contract:** emits `arrived` exactly once when `s >= total_length`. `_process` guards the emit: it sets `s = total_length + 1` before emitting so the handler's `assign_path` (which resets `s = 0`) is not immediately overwritten by the rest of `_process`.

#### `assign_path(new_path)`

Builds the trajectory via `TrajectoryBuilder.build`, precomputes `seg_start_arc` (cumulative arc length at each segment boundary), resets `s = 0`, `seg_index = 0`, `current_speed = 0`, and seeds `position_on_road`/`heading` from the first segment. Clamps `_eff_decel` for short trips.

### 3.5 Orchestration — `SimulationManager`

`scripts/simulation_manager.gd` — `extends Node2D`

Phase 3 orchestrates a single vehicle, but the design generalizes to many:

1. `_ready` — randomize RNG, ensure `RoadGrid` is ready, `_spawn_initial_vehicle`.
2. `_spawn_initial_vehicle` — pick a random `boundary_nodes()` entry, instantiate the vehicle scene, inject `graph`, connect `arrived` → `_on_vehicle_arrived`, assign the first path.
3. `_assign_new_path_from(current)` — pick a goal from `far_from(current, 6)` (falls back to `all_nodes()` if empty), run `find_path`, call `vehicle.assign_path(p)`, and `road_grid.set_route(current, goal, p)` so the route line updates.
4. `_on_vehicle_arrived` — read the last node of the current path, repath from there.

The `far_from(current, 6)` filter keeps trips reasonably long (≥6 Manhattan steps) so the car actually drives rather than hopping one block. The `arrived` signal is the only child→parent channel — no string-path coupling.

### 3.6 Rendering — `RoadGrid`

`scripts/road_grid.gd` — `extends Node2D`

Owns the `GridGenerator` and `RoadGraph`, and renders everything in `_draw`:

1. **Road surfaces:** iterate `graph.edges`; for each undirected edge (guarded by `key < n` to draw once), `draw_line` with `road_width` in asphalt color.
2. **Lane markings:** `_draw_dashed_line` overlays a dashed center line (dash 16, gap 12, width 1.5).
3. **Intersection squares:** `draw_rect` at each node masks the seam where two perpendicular roads meet.
4. **Route visualization (`_draw_route`):** builds the *same* right-lane-offset trajectory as the vehicle (via `TrajectoryBuilder.build`), samples it every ~4 px of arc length, and `draw_polyline`s it in soft cyan. Green start ring and red goal ring cap the ends. This is the DRY payoff: the drawn line is pixel-aligned with the car's actual path, including through bezier turns.
5. **Debug overlay (F1):** raw graph edges (thin blue) + nodes (yellow dots).
6. **F5** regenerates the grid via `_regenerate()` (new `GridGenerator`, new `RoadGraph`, redraw).

`set_route(start, goal, path)` is the entry point `SimulationManager` calls after each path assignment; it stores the route and `queue_redraw`s.

## 4. Key design decisions

- **`RefCounted` for data, `Node` for SceneTree.** `GridGenerator`, `RoadGraph`, `TrajectoryBuilder`, and the segment classes are `RefCounted` — they hold data and logic, not scene lifetime. Only `VehicleController`, `RoadGrid`, and `SimulationManager` are `Node2D`. This follows the [Godot best practice](https://docs.godotengine.org/en/stable/tutorials/best_practices/index.html) of not putting data structures in the tree.
- **Single arc-length parameter `s`.** Position and heading both derive from the same `s` each frame. This is what eliminates skating and speed-dependent heading artifacts — the common failure mode of "move forward, then rotate to face the path."
- **DRY `TrajectoryBuilder`.** Driving and rendering share one trajectory builder. If the turn geometry changes, both update; they cannot disagree.
- **Signals, not string paths.** `VehicleController` emits `arrived`; `SimulationManager` connects it. The parent never reaches into the child by path to detect completion.
- **Apex-based turn slowdown.** A triangle weight peaking at `progress = 0.5` matches where bezier curvature actually peaks, so the car slows in the middle of the turn and recovers on exit — more natural than a constant turn-speed factor.
- **Linear-scan A\* open set.** Grids are small; a heap would add complexity for negligible gain (YAGNI).
- **Smoothstep end ramp, not linear.** Zero derivative at both ends means the stop is gentle and the start of deceleration isn't a kink.

## 5. Coordinate systems

- **Screen space, y-down.** Standard Godot 2D: +x right, +y down.
- **Right-hand drive.** The right-hand perpendicular to a travel direction `d = (dx, dy)` is `perp = (-dy, dx)`. In y-down space this points to the right of travel (verify: for `d = (1, 0)` east, `perp = (0, 1)` = south = right of east when facing east in y-down). All trajectory points are offset by `+ perp * lane_offset`, putting the car on the right lane.
- **Grid coordinates** are `Vector2i(c, r)`, independent of world scale. `world_pos` maps them to pixels.
- **Arc length `s`** is in pixels, global to the whole trajectory (not per-segment). `seg_start_arc` translates global `s` to per-segment `local_s`.

## 6. Extending — where Phases 4–6 plug in

- **Phase 4 (Traffic lights):** add `TrafficLight` nodes positioned at intersection world positions (from `GridGenerator.nodes`). `VehicleController._target_speed_at` gains a red-light factor: query the upcoming intersection and clamp target speed to 0 within a stopping distance. `SimulationManager` owns the light cycle. No change to the trajectory layer.
- **Phase 5 (Multi-vehicle & collision avoidance):** `SimulationManager` spawns N vehicles. Add a coordination layer (either a centralized reservation at intersections, or per-vehicle lookahead that lowers `_target_speed_at` when a vehicle ahead is close). The `arrived` signal contract already scales — each vehicle connects its own.
- **Phase 6 (Visual polish & wallpaper):** `shaders/road_shader.shader` and `shaders/vehicle_shader.shader` (currently passthrough stubs) get real material; `assets/textures/` is populated; `export/` defines the wallpaper build presets.

In all three, the data class / `Node` split and the `TrajectoryBuilder` DRY boundary are intended to stay intact.

## 7. Testing strategy

Unit tests use [GUT](https://github.com/bitwes/Gut), vendored in `addons/gut/`, run headlessly via `make test`. The suite (40 tests across 8 files) targets each layer:

| Layer | Test file | What's verified |
|-------|-----------|-----------------|
| Grid | `test_grid_generator.gd` | Dimensions, world-pos formula, node/edge counts, boundary nodes, `far_from`, single-column edge case |
| Graph | `test_road_graph.gd` | A\* optimality, contiguity, missing-node handling, heuristic = Manhattan, edge cost = Euclidean |
| Trajectory builder | `test_trajectory_builder.gd` | Straight→LineSeg, turn→Bezier, contiguity, positive lengths, empty/single-node guards |
| Segments | `test_trajectory_segment.gd`, `test_line_seg.gd`, `test_bezier_seg.gd` | Base curvature/progress, constant tangent, arc-length LUT accuracy, 90°/180° curvature |
| Vehicle logic | `test_vehicle_logic.gd` | Smoothstep boundaries, braking intensity math, point-to-segment distance |
| Integration | `test_integration.gd` | Full-trip right-lane invariant (car stays on the right of its path) and end-to-end segment contiguity |

The quality gate (`make check` = `format-check` + `lint` + `test`) is enforced by a git pre-commit hook; no commit lands with failing checks.