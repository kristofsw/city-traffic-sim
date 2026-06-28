# AGENTS.md — Instructions for AI agents working on this project

> **Read this file at the start of every session.** It defines the mandatory
> workflow, quality standards, and conventions for this project.

## Project

City Traffic Simulation — a minimalist top-down traffic sim in Godot 4.7.
See `README.md` for the project overview and `CONTRIBUTING.md` for human
contributors.

## Mandatory workflow for ANY code change

You MUST run the full quality gate before committing:

```bash
make check    # format-check + lint + tests
```

If `make check` fails, fix the issue and re-run until it passes. Do not
commit with failing checks. The git pre-commit hook enforces this — a
commit will be blocked if checks fail.

### Step-by-step for any change

1. **Make the code change** (feature, fix, refactor, etc.)
2. **Format**: `make format` (runs `gdformat scripts/ tests/`)
3. **Lint**: `make lint` (runs `gdlint scripts/ tests/`) — fix all warnings
4. **Test**: `make test` (runs GUT headlessly) — all tests must pass
5. **Review** against the definition of done (below)
6. **Commit** with a conventional commit message

### One-time setup (already done, but if hooks are missing)

```bash
make install-hooks   # install the git pre-commit hook
```

## Definition of done

A change is complete only when ALL of these are true:

- [ ] `gdformat --check scripts/ tests/` passes (no formatting diffs)
- [ ] `gdlint scripts/ tests/` produces zero warnings
- [ ] `make test` passes (exit 0, all tests green)
- [ ] No dead code, no unused exports, no stale .tscn properties
- [ ] New logic has unit tests
- [ ] DRY: no duplicated logic (if duplicated, extract a shared class)
- [ ] RefCounted for data classes, Node only for SceneTree participants
- [ ] Signals for child→parent communication, not hardcoded string paths
- [ ] snake_case for files and folders, PascalCase for node names
- [ ] Commit message follows conventional format (see below)

## Commit conventions

- Prefix: `feat:`, `fix:`, `refactor:`, `test:`, `docs:`, `chore:`
- Present tense, imperative mood: "add", "fix", "extract", not "added"
- Subject line ≤ 72 characters
- Body explains *why*, not just *what*

Examples:
```
feat: acceleration/deceleration with apex-based turn slowdown
fix: correct right-hand perpendicular in y-down screen space
refactor: extract TrajectoryBuilder to eliminate duplicated logic
test: add GUT unit tests for BezierSeg arc-length LUT
docs: add AGENTS.md workflow standard
chore: add gdtoolkit formatting and linting
```

## Godot best practices reference

Follow the [Godot official best practices](https://docs.godotengine.org/en/stable/tutorials/best_practices/index.html).
Key principles for this project:

### Object-oriented principles
- Scripts and scenes are classes — apply SOLID, DRY, KISS, YAGNI
- **Single Responsibility**: each class does one thing well
- **DRY**: never duplicate logic; extract a shared class/function

### Node alternatives
- Use `RefCounted` for data classes (auto-freed, no SceneTree needed)
- Use `Node` only for things that must live in the SceneTree
- Our `TrajectorySegment`, `LineSeg`, `BezierSeg`, `GridGenerator`,
  `RoadGraph`, `TrajectoryBuilder` are all `RefCounted` — keep it that way

### Scene organization
- Scenes should have zero external dependencies (loose coupling)
- Child→parent communication via **signals**, not string paths
- Parent→child via dependency injection (`@export var` or method calls)
- Use `_get_configuration_warnings()` to self-document dependencies

### Data preferences
- `Dictionary` for key→value lookups (constant time)
- `Array` for ordered iteration
- Don't use `Node` for custom data structures — use `RefCounted`

### Project organization
- `snake_case` for files and folders
- `PascalCase` for node names
- Third-party code in `addons/` (excluded from formatting/linting)
- Group assets with the scenes that use them

## Commands

| Command | What it does |
|---------|-------------|
| `make test` | Run GUT unit tests headlessly |
| `make lint` | Lint all scripts (gdlint) |
| `make format` | Format all scripts in place (gdformat) |
| `make format-check` | Check formatting without modifying (CI gate) |
| `make check` | Full gate: format-check + lint + test |
| `make run` | Launch the simulation |
| `make import` | Regenerate .godot import cache |
| `make install-hooks` | Install the git pre-commit hook |

## Architecture overview

```
Main (Node2D, main.gd) — entry point; injects RoadGrid into SimulationManager
├── RoadGrid (Node2D) — owns MapGenerator + RoadGraph; renders roads + route
└── SimulationManager (Node2D) — owns VehicleSpawner; spawns + repaths vehicles
    └── VehicleController (Node2D) — thin orchestrator
        ├── VehicleMover (RefCounted) — pure motion model + signals
        └── VehicleBody (Node2D) — composed visual scene (body + CircleDrawer lights)

MapGenerator (Resource) → RoadGraph (RefCounted) → TrajectoryBuilder (RefCounted)
                                                            ↓
                                            Trajectory (RefCounted) wraps segments
                                            (LineSeg / BezierSeg)
```

- `MapGenerator`: Resource contract for procedural map generation (`.tres` presets)
- `GridGenerator`: builds the Manhattan grid (extends MapGenerator)
- `RoadGraph`: A* pathfinding over the grid
- `TrajectoryBuilder`: converts a path into LineSeg/BezierSeg segments
- `Trajectory`: arc-length-parametrized wrapper (DRY: single source of segment lookup)
- `TrajectorySegment` / `LineSeg` / `BezierSeg`: parametric trajectory segments
- `VehicleSpec`: Resource holding all tuning + visual config (swappable per vehicle type)
- `VehicleMover`: pure motion model (RefCounted); emits `braking_changed`/`turn_indicator_changed`/`arrived` signals
- `VehicleBody`: composed visual scene driven by mover signals
- `VehicleController`: thin orchestrator owning a mover + body
- `VehicleSpawner`: spawn/repath policy (multi-vehicle ready, RefCounted)
- `RoadGrid`: renders roads + route visualization; owns the MapGenerator
- `SimulationManager`: owns the spawner; spawns N vehicles, repaths on arrival
- `Main`: entry point; mediates siblings via dependency injection