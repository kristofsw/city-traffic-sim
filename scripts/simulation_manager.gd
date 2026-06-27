extends Node2D
## Coordinates spawning, pathing, and lifecycle of vehicles.
## Phase 2/3: spawns a single vehicle, assigns an A* path, and repaths to a
## new random destination whenever the vehicle arrives.

const VehicleScene := preload("res://scenes/vehicle.tscn")

@onready var road_grid: Node2D = $"../RoadGrid"

var vehicle: VehicleController = null
var rng := RandomNumberGenerator.new()

func _ready() -> void:
	rng.randomize()
	if road_grid and road_grid.graph == null:
		road_grid._ready()
	print("[SimulationManager] grid cols=%d rows=%d nodes=%d" % [road_grid.generator.cols, road_grid.generator.rows, road_grid.graph.nodes.size()])
	_spawn_initial_vehicle()

func _spawn_initial_vehicle() -> void:
	var graph: RoadGraph = road_grid.get_graph()
	var boundary: Array[Vector2i] = road_grid.get_generator().boundary_nodes()
	if boundary.is_empty():
		printerr("[SimulationManager] no boundary nodes to spawn at")
		return
	var start: Vector2i = boundary[rng.randi() % boundary.size()]
	vehicle = VehicleScene.instantiate() as VehicleController
	vehicle.graph = graph
	add_child(vehicle)
	vehicle.arrived.connect(_on_vehicle_arrived)
	_assign_new_path_from(start)

func _assign_new_path_from(current: Vector2i) -> void:
	var graph: RoadGraph = road_grid.get_graph()
	var candidates: Array[Vector2i] = road_grid.get_generator().far_from(current, 6)
	if candidates.is_empty():
		candidates = road_grid.get_generator().all_nodes()
	var goal: Vector2i = candidates[rng.randi() % candidates.size()]
	var p: Array[Vector2i] = graph.find_path(current, goal)
	if p.is_empty():
		printerr("[SimulationManager] no path %s -> %s" % [current, goal])
		return
	vehicle.assign_path(p)
	road_grid.set_route(current, goal, p)
	print("[SimulationManager] path %s -> %s (%d hops)" % [current, goal, p.size()])

func _on_vehicle_arrived() -> void:
	var last: Vector2i = vehicle.path[vehicle.path.size() - 1]
	_assign_new_path_from(last)