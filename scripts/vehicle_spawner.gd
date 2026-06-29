class_name VehicleSpawner
extends RefCounted
## Owns vehicle spawn and repath policy. A RefCounted data/logic object per
## the Godot node-alternatives best practice -- no SceneTree dependency,
## fully testable. The spawner knows about the graph, the map generator,
## the vehicle scene and an optional spec pool, but NOT about RoadGrid or
## route visualization: it emits `vehicle_path_assigned` and whoever cares
## (SimulationManager) updates the route viz.
##
## Swappable policy: subclass or reconfigure to change spawn point
## selection, goal distance, spec distribution, etc. without touching
## SimulationManager. This is the seam for Phase 5 multi-vehicle: the
## manager just calls `spawn` N times and connects `arrived` per vehicle.

signal vehicle_spawned(vehicle: VehicleController)
signal vehicle_path_assigned(
	vehicle: VehicleController, start: Vector2i, goal: Vector2i, path: Array[Vector2i]
)

var vehicle_scene: PackedScene = null
var graph: RoadGraph = null
var generator: MapGenerator = null
var rng: RandomNumberGenerator = null
var min_trip_distance: int = 6  # Manhattan; fallback to all_nodes if too few candidates
var spec_pool: Array[VehicleSpec] = []  # empty -> each vehicle uses its default spec


## Spawn one vehicle into `parent`, inject the graph, connect `arrived` to
## `arrived_handler` (bound with the vehicle so the handler knows WHICH
## vehicle arrived), and assign its first path. Returns the vehicle.
func spawn(parent: Node, arrived_handler: Callable) -> VehicleController:
	var start := pick_start()
	var vehicle := vehicle_scene.instantiate() as VehicleController
	vehicle.graph = graph
	if spec_pool.size() > 0:
		# Duplicate so each vehicle gets its own copy of the spec (a shared
		# .tres would otherwise be mutated by any vehicle that writes back).
		vehicle.spec = spec_pool[rng.randi() % spec_pool.size()].duplicate(true)
	parent.add_child(vehicle)
	# Bind the vehicle so the arrived handler (which takes no args from the
	# signal) receives it as the first argument -- identifies the emitter
	# when multiple vehicles exist.
	vehicle.arrived.connect(arrived_handler.bind(vehicle))
	vehicle_spawned.emit(vehicle)
	assign_path_from(vehicle, start)
	return vehicle


## Pick a fresh boundary spawn point and assign a new path to a far goal.
## Called by SimulationManager when a vehicle's `arrived` signal fires.
## Each trip is a clean A->B unrelated to the previous one: the vehicle
## teleports to a new boundary node (assign_path resets s=0 and emits
## position_changed, moving the vehicle to the new start's world position).
func repath(vehicle: VehicleController) -> void:
	if not generator:
		push_error("[VehicleSpawner] no generator set")
		return
	var start := pick_start()
	assign_path_from(vehicle, start)


## Pick a boundary node as a spawn point.
func pick_start() -> Vector2i:
	var boundary: Array[Vector2i] = generator.boundary_nodes()
	if boundary.is_empty():
		push_error("[VehicleSpawner] generator has no boundary nodes")
		return Vector2i.ZERO
	return boundary[rng.randi() % boundary.size()]


## Pick a goal node at least `min_trip_distance` (Manhattan) from `current`.
func pick_goal(current: Vector2i) -> Vector2i:
	var candidates: Array = generator.far_from(current, min_trip_distance)
	if candidates.is_empty():
		candidates = generator.all_nodes()
	if candidates.is_empty():
		push_error("[VehicleSpawner] generator has no nodes")
		return current
	return candidates[rng.randi() % candidates.size()]


## Find a path and assign it to the vehicle; emit vehicle_path_assigned.
func assign_path_from(vehicle: VehicleController, current: Vector2i) -> void:
	var goal := pick_goal(current)
	var p: Array[Vector2i] = graph.find_path(current, goal)
	if p.is_empty():
		printerr("[VehicleSpawner] no path %s -> %s" % [current, goal])
		return
	vehicle.assign_path(p)
	vehicle_path_assigned.emit(vehicle, current, goal, p)
