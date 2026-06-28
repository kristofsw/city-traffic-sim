extends GutTest
## Unit tests for VehicleSpawner spawn/repath policy (RefCounted, no
## SceneTree dependency of its own; uses this test node as the parent for
## spawned vehicles).

const VehicleScene := preload("res://scenes/vehicle.tscn")


func _build_spawner() -> VehicleSpawner:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	var s := VehicleSpawner.new()
	s.vehicle_scene = VehicleScene
	s.graph = graph
	s.generator = gen
	s.rng = RandomNumberGenerator.new()
	s.rng.seed = 42
	return s


func test_pick_start_returns_boundary_node() -> void:
	var s := _build_spawner()
	var start := s.pick_start()
	assert_true(s.generator.boundary_nodes().has(start), "pick_start should return a boundary node")


func test_pick_goal_honors_min_distance() -> void:
	var s := _build_spawner()
	s.min_trip_distance = 6
	var goal := s.pick_goal(Vector2i(0, 0))
	var d: int = abs(goal.x) + abs(goal.y)
	assert_gte(d, 6, "pick_goal should be at least min_trip_distance away")


func test_pick_goal_falls_back_when_no_far_nodes() -> void:
	var s := _build_spawner()
	s.min_trip_distance = 999  # no node is that far
	var goal := s.pick_goal(Vector2i(0, 0))
	# Should fall back to all_nodes() and still return a valid node.
	assert_true(s.generator.all_nodes().has(goal), "pick_goal fallback should return a valid node")


func test_spawn_creates_vehicle_and_assigns_path() -> void:
	var s := _build_spawner()
	# Use a one-element Array as a mutable counter (GDScript lambdas capture
	# primitives by value; an Array element is mutated in place).
	var counter := [0]
	s.vehicle_path_assigned.connect(func(_v, _st, _go, _p): counter[0] += 1)
	var v: VehicleController = s.spawn(self, _dummy_arrived)
	autofree(v)
	assert_eq(vehicles_count(), 1, "spawn should add the vehicle to the parent")
	assert_true(v is VehicleController, "spawn should return a VehicleController")
	assert_gt(v.path.size(), 1, "spawn should assign a multi-hop path")
	assert_eq(counter[0], 1, "spawn should emit vehicle_path_assigned once")
	# The vehicle should have a graph and a non-empty trajectory.
	assert_ne(v.graph, null, "spawn should inject the graph")
	assert_ne(v.trajectory, null, "spawn should build a trajectory")


func test_repath_assigns_new_path_from_last_node() -> void:
	var s := _build_spawner()
	var v: VehicleController = s.spawn(self, _dummy_arrived)
	autofree(v)
	var first_path := v.path.duplicate()
	s.repath(v)
	assert_gt(v.path.size(), 1, "repath should assign a new multi-hop path")
	# The new path should start where the old one ended.
	assert_eq(
		v.path[0],
		first_path[first_path.size() - 1],
		"repath should start from the last node of the previous path"
	)


func _dummy_arrived(_v: VehicleController) -> void:
	pass


func vehicles_count() -> int:
	var n := 0
	for c in get_children():
		if c is VehicleController:
			n += 1
	return n
