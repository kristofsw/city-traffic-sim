class_name SimulationManager
extends Node2D
## Coordinates spawning, pathing, and lifecycle of vehicles.
## Owns a VehicleSpawner (RefCounted) that encapsulates spawn/repath policy;
## this manager only wires the spawner's signals to RoadGrid route viz and
## keeps the Array[VehicleController]. Structured for N vehicles (Phase 5):
## increase `spawn_count` and the spawner handles the rest.
##
## The RoadGrid dependency is injected by the Main entry-point node (see
## main.gd) via the `road_grid` export, not via a hardcoded sibling string
## path. Per the Godot scene-organization best practice, siblings should
## not reference each other directly; the ancestor mediates.

const VehicleScene := preload("res://scenes/vehicle.tscn")

@export var road_grid: RoadGrid = null
@export var spawn_count: int = 10  # number of vehicles to spawn at startup

var vehicles: Array[VehicleController] = []
var spawner: VehicleSpawner = null
var traffic: TrafficSystem = null
var rng := RandomNumberGenerator.new()


func _get_configuration_warnings() -> PackedStringArray:
	if road_grid == null:
		return PackedStringArray(["road_grid must be assigned (done by Main.gd)"])
	return PackedStringArray()


func _ready() -> void:
	rng.randomize()
	# Main.gd injects road_grid in its own _ready (runs after children are
	# ready), so defer initialization to the next frame to ensure the
	# reference is present. This replaces the old manual road_grid._ready()
	# lifecycle call -- each node now manages its own lifecycle naturally.
	await get_tree().process_frame
	if road_grid == null:
		printerr("[SimulationManager] road_grid not assigned — wire it via Main.gd")
		return
	print(
		(
			"[SimulationManager] map nodes=%d edges=%d"
			% [road_grid.generator.nodes.size(), road_grid.generator.edges.size()]
		)
	)
	_setup_spawner()
	_spawn_initial_vehicles()


func _setup_spawner() -> void:
	spawner = VehicleSpawner.new()
	spawner.vehicle_scene = VehicleScene
	spawner.graph = road_grid.get_graph()
	spawner.generator = road_grid.get_generator()
	spawner.rng = rng
	spawner.spec_pool = _build_spec_pool()
	# The spawner emits path assignments; we forward them to RoadGrid so the
	# route viz stays in sync without the spawner knowing about RoadGrid.
	spawner.vehicle_path_assigned.connect(_on_vehicle_path_assigned)
	traffic = TrafficSystem.new()


## Build a pool of VehicleSpecs with varied cruising speeds and colors so
## multi-vehicle traffic has natural speed variation: some cars hurry,
## others cruise. The spawner picks one at random per vehicle (duplicated
## so each vehicle owns its copy).
func _build_spec_pool() -> Array[VehicleSpec]:
	var pool: Array[VehicleSpec] = []
	# Cruiser: default speed, muted gray-blue.
	var cruiser := VehicleSpec.new()
	cruiser.max_speed = 70.0
	cruiser.body_color = Color(0.42, 0.45, 0.50, 1)
	pool.append(cruiser)
	# Sedan: standard speed, silver.
	var sedan := VehicleSpec.new()
	sedan.max_speed = 85.0
	sedan.body_color = Color(0.55, 0.57, 0.62, 1)
	pool.append(sedan)
	# Racer: fast, red, shorter following gap (aggressive).
	var racer := VehicleSpec.new()
	racer.max_speed = 105.0
	racer.body_color = Color(0.78, 0.25, 0.25, 1)
	racer.follow_time_gap = 1.1
	racer.follow_min_gap = 24.0
	pool.append(racer)
	# Bus: slow, long, green, wider following gap (cautious).
	var bus := VehicleSpec.new()
	bus.max_speed = 55.0
	bus.body_color = Color(0.30, 0.55, 0.35, 1)
	bus.body_length = 52.0
	bus.body_width = 22.0
	bus.follow_time_gap = 2.0
	bus.follow_min_gap = 40.0
	pool.append(bus)
	return pool


func _spawn_initial_vehicles() -> void:
	for i in range(spawn_count):
		var v: VehicleController = spawner.spawn(self, _on_vehicle_arrived)
		vehicles.append(v)


## Run the traffic system BEFORE the children's _process. Godot processes a
## parent's _process before its children's, so the ACC constraints are set
## just before each VehicleController._process calls mover.update(delta).
func _process(delta: float) -> void:
	if traffic != null and not vehicles.is_empty():
		traffic.update(vehicles, delta)


## Called when any vehicle arrives; the spawner repaths it from its last
## node. The vehicle is bound into the Callable by the spawner so the
## handler knows WHICH vehicle emitted the signal (multi-vehicle safe).
func _on_vehicle_arrived(vehicle: VehicleController) -> void:
	spawner.repath(vehicle)


## Forward a spawner path assignment to RoadGrid route viz. Only the most
## recent assignment drives the visible route line (single-vehicle behavior
## preserved; for multi-vehicle, RoadGrid.set_route would need to draw N
## routes -- a Phase 5 concern).
func _on_vehicle_path_assigned(
	_vehicle: VehicleController, start: Vector2i, goal: Vector2i, path: Array[Vector2i]
) -> void:
	road_grid.set_route(start, goal, path)
	print("[SimulationManager] path %s -> %s (%d hops)" % [start, goal, path.size()])
