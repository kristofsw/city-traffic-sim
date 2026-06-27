# City Traffic Simulation

A minimalist, top-down city traffic simulation built in Godot 4.7, designed as a calm, ambient dynamic wallpaper. Autonomous vehicles navigate a procedurally generated grid-based road network using A* pathfinding.

## Status

- [x] Phase 1 — Procedural road grid generation
- [x] Phase 2 — Vehicle & basic movement
- [x] Phase 3 — A* pathfinding A→B with continuous repathing
- [ ] Phase 4 — Traffic lights & right-of-way
- [ ] Phase 5 — Multi-vehicle simulation & collision avoidance
- [ ] Phase 6 — Visual polish & wallpaper export

## Run

```bash
/Applications/Godot.app/Contents/MacOS/Godot --path /Users/kristofsweerts/Projects/city-traffic-sim
```

Or open the `project.godot` file in the Godot editor and press Play (F5).

## Controls

| Key | Action |
|-----|--------|
| F1  | Toggle debug overlay (graph nodes, edges, active path) |
| F5  | Regenerate the road grid |

## Configuration

Default grid parameters (editable in `scenes/road_grid.tscn`):

| Parameter | Value |
|-----------|-------|
| Screen | 1280×720 |
| Edge margin | 40 px |
| Target block size | 128 px |
| Road width | 48 px |
| Lane width | 24 px |
| Lane offset | 12 px (right-hand drive) |

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
│   ├── road_grid.gd           # Rendering, lane markings, debug overlay
│   ├── vehicle_controller.gd  # Path follow, lane offset, smooth turns
│   └── simulation_manager.gd  # Spawn, path assignment, repath on arrival
├── shaders/                   # Passthrough stubs (Phase 6 polish)
│   ├── road_shader.shader
│   └── vehicle_shader.shader
├── assets/textures/           # (Phase 6)
└── export/                    # (Phase 6)
```

## Palette

| Element | Color |
|---------|-------|
| Asphalt | `#2b2b30` |
| Lane markings | `#5a5a64` |
| Vehicle body | `#6b7280` |
| Headlights | `#fff4d6` |
| Taillights | `#ff5a4d` |