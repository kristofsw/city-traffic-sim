class_name SimulationManager
extends Node2D
## Coordinates spawning, pathing, and lifecycle of vehicles.
## Phase 2/3: spawns a single vehicle, assigns an A* path, and repaths to a
## new random destination whenever the vehicle arrives.
##
## The RoadGrid dependency is injected by the Main entry-point node (see
## main.gd) via the `road_grid` export, not via a hardcoded sibling string
## path. Per the Godot scene-organization best practice, siblings should
## not reference each other directly; the ancestor mediates.

const VehicleScene := preload("res://scenes/vehicle.tscn")

@export var road_grid: RoadGrid = null

var vehicle: VehicleController = null
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
			"[SimulationManager] grid cols=%d rows=%d nodes=%d"
			% [road_grid.generator.cols, road_grid.generator.rows, road_grid.graph.nodes.size()]
		)
	)
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
