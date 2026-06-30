extends GutTest
## Integration test: full pipeline from grid generation to vehicle arrival.
## Verifies the right-lane invariant holds throughout a complete trip.

const VehicleScene := preload("res://scenes/vehicle.tscn")


func test_full_trip_right_lane_invariant() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0  # predictable full grid for invariant test
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	# Find a path with at least one turn.
	var start := Vector2i(0, 0)
	var goal := Vector2i(5, 3)
	var path := graph.find_path(start, goal)
	assert_gt(path.size(), 1, "path should exist from (0,0) to (5,3)")

	var traj := TrajectoryBuilder.build_trajectory(graph, path, 12.0, 22.0)
	assert_false(traj.is_empty(), "trajectory should have segments")

	# Simulate driving the full trajectory at fixed speed.
	var total_length := traj.total_length
	var s := 0.0
	var speed := 120.0
	var delta := 1.0 / 60.0
	var min_signed_offset := INF
	var seg_hint := 0

	while s < total_length:
		seg_hint = traj.segment_index_at(s, seg_hint)
		var pos := traj.position_at(s, seg_hint)

		# Compute signed lane offset relative to the nearest path segment.
		var best_i := 0
		var best_d := INF
		for i in range(path.size() - 1):
			var a: Vector2 = graph.world_of(path[i])
			var b: Vector2 = graph.world_of(path[i + 1])
			var ab: Vector2 = b - a
			var t: float = clamp((pos - a).dot(ab) / ab.length_squared(), 0.0, 1.0)
			var proj: Vector2 = a + ab * t
			var dist: float = pos.distance_to(proj)
			if dist < best_d:
				best_d = dist
				best_i = i
		var a: Vector2 = graph.world_of(path[best_i])
		var b: Vector2 = graph.world_of(path[best_i + 1])
		var dir := (b - a).normalized()
		var perp := Vector2(-dir.y, dir.x)
		var signed_offset := (pos - a).dot(perp)
		if signed_offset < min_signed_offset:
			min_signed_offset = signed_offset

		s += speed * delta

	# The car should never enter the oncoming lane (signed offset >= ~0).
	# Allow a small tolerance for floating-point at the very start.
	assert_gte(
		min_signed_offset, -0.5, "car should never enter oncoming lane (signed offset >= -0.5)"
	)


func test_full_trip_segment_contiguity() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0  # predictable full grid for contiguity test
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	var path := graph.find_path(Vector2i(0, 0), Vector2i(9, 5))
	assert_gt(path.size(), 1, "path should exist across the grid")

	var traj := TrajectoryBuilder.build_trajectory(graph, path, 12.0, 22.0)
	var segs := traj.segments
	for i in range(segs.size() - 1):
		var end_pos := segs[i].position_at(segs[i].length)
		var start_next := segs[i + 1].position_at(0.0)
		assert_almost_eq(
			end_pos.distance_to(start_next),
			0.0,
			2.0,
			"segment %d end should connect to segment %d start" % [i, i + 1]
		)


func test_right_lane_invariant_with_obstacle_holes() -> void:
	# Verify the right-lane invariant holds when routing around obstacle
	# holes — the trajectory should still stay on the right lane through
	# the detour. Uses a seeded rng for reproducibility.
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 3
	gen.obstacle_radius = 2
	gen.rng.seed = 42
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	# Pick two boundary nodes far apart; the connectivity prune guarantees
	# a path exists between them.
	var boundary := gen.boundary_nodes()
	assert_gt(boundary.size(), 1, "should have boundary nodes")
	var start := boundary[0]
	var goal := boundary[boundary.size() - 1]
	var path := graph.find_path(start, goal)
	assert_gt(path.size(), 1, "path should exist around holes between boundary nodes")

	var traj := TrajectoryBuilder.build_trajectory(graph, path, 12.0, 22.0)
	assert_false(traj.is_empty(), "trajectory should have segments around holes")

	# Simulate driving the full trajectory; check the right-lane invariant.
	var total_length := traj.total_length
	var s := 0.0
	var speed := 120.0
	var delta := 1.0 / 60.0
	var min_signed_offset := INF
	var seg_hint := 0

	while s < total_length:
		seg_hint = traj.segment_index_at(s, seg_hint)
		var pos := traj.position_at(s, seg_hint)

		# Signed lane offset relative to the nearest path segment.
		var best_d := INF
		var best_a := Vector2.ZERO
		var best_b := Vector2.ZERO
		for i in range(path.size() - 1):
			var a: Vector2 = graph.world_of(path[i])
			var b: Vector2 = graph.world_of(path[i + 1])
			var ab: Vector2 = b - a
			var t: float = clamp((pos - a).dot(ab) / ab.length_squared(), 0.0, 1.0)
			var proj: Vector2 = a + ab * t
			var dist: float = pos.distance_to(proj)
			if dist < best_d:
				best_d = dist
				best_a = a
				best_b = b
		var dir := (best_b - best_a).normalized()
		var perp := Vector2(-dir.y, dir.x)
		var signed_offset := (pos - best_a).dot(perp)
		if signed_offset < min_signed_offset:
			min_signed_offset = signed_offset

		s += speed * delta

	# The car should never enter the oncoming lane, even around holes.
	assert_gte(
		min_signed_offset,
		-0.5,
		"car should never enter oncoming lane around holes (signed offset >= -0.5)"
	)


func test_right_lane_invariant_on_street_network() -> void:
	# Verify the right-lane invariant holds on the StreetNetworkGenerator
	# (variable grid + T-junctions + 45° diagonals). The trajectory builder
	# offsets to the right of each edge regardless of angle, so the car
	# should stay on the right lane through 45° diagonal turns.
	var gen := StreetNetworkGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.block_jitter = 0.25
	gen.diagonal_count = 2
	gen.snap_tolerance = 24.0
	gen.rng.seed = 42
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	var boundary := gen.boundary_nodes()
	assert_gt(boundary.size(), 1, "street network should have boundary nodes")
	var start := boundary[0]
	var goal := boundary[boundary.size() - 1]
	var path := graph.find_path(start, goal)
	assert_gt(path.size(), 1, "path should exist across the street network")

	var traj := TrajectoryBuilder.build_trajectory(graph, path, 12.0, 22.0)
	assert_false(traj.is_empty(), "trajectory should have segments on the street network")

	# Simulate driving the full trajectory; check the right-lane invariant.
	var total_length := traj.total_length
	var s := 0.0
	var speed := 120.0
	var delta := 1.0 / 60.0
	var min_signed_offset := INF
	var seg_hint := 0

	while s < total_length:
		seg_hint = traj.segment_index_at(s, seg_hint)
		var pos := traj.position_at(s, seg_hint)

		# Signed lane offset relative to the nearest path segment.
		var best_d := INF
		var best_a := Vector2.ZERO
		var best_b := Vector2.ZERO
		for i in range(path.size() - 1):
			var a: Vector2 = graph.world_of(path[i])
			var b: Vector2 = graph.world_of(path[i + 1])
			var ab: Vector2 = b - a
			var t: float = clamp((pos - a).dot(ab) / ab.length_squared(), 0.0, 1.0)
			var proj: Vector2 = a + ab * t
			var dist: float = pos.distance_to(proj)
			if dist < best_d:
				best_d = dist
				best_a = a
				best_b = b
		var dir := (best_b - best_a).normalized()
		var perp := Vector2(-dir.y, dir.x)
		var signed_offset := (pos - best_a).dot(perp)
		if signed_offset < min_signed_offset:
			min_signed_offset = signed_offset

		s += speed * delta

	# The car should never enter the oncoming lane on the street network.
	assert_gte(
		min_signed_offset,
		-0.5,
		"car should never enter oncoming lane on street network (signed offset >= -0.5)"
	)


func test_acc_prevents_overtaking() -> void:
	# Two vehicles on the same straight trajectory: a slow lead (max_speed=40)
	# ahead of a fast follower (max_speed=120). The follower should never
	# pass the lead -- the ACC constraint caps its speed to maintain the gap.
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	# A long straight path along the top row.
	var path := graph.find_path(Vector2i(0, 0), Vector2i(9, 0))
	assert_gt(path.size(), 1, "path should exist along the top row")

	# Lead: slow car. Follower: fast car that would naturally overtake.
	var lead_spec := VehicleSpec.new()
	lead_spec.max_speed = 40.0
	var follow_spec := VehicleSpec.new()
	follow_spec.max_speed = 120.0
	follow_spec.follow_time_gap = 1.5
	follow_spec.follow_min_gap = 40.0

	var lead_mover := VehicleMover.new()
	lead_mover.graph = graph
	lead_mover.apply_spec(lead_spec)
	lead_mover.assign_path(path, 12.0, 22.0)

	var follow_mover := VehicleMover.new()
	follow_mover.graph = graph
	follow_mover.apply_spec(follow_spec)
	# Start the follower 100px behind the lead on the same trajectory.
	follow_mover.assign_path(path, 12.0, 22.0)

	# Build lightweight controllers so TrafficSystem can read position/heading.
	var lead_v: VehicleController = VehicleScene.instantiate()
	add_child(lead_v)
	autofree(lead_v)
	lead_v.mover = lead_mover
	lead_v.mover.position_changed.connect(lead_v._on_mover_position_changed)
	lead_v.mover.speed_changed.connect(lead_v._on_mover_speed_changed)

	var follow_v: VehicleController = VehicleScene.instantiate()
	add_child(follow_v)
	autofree(follow_v)
	follow_v.mover = follow_mover
	follow_v.mover.position_changed.connect(follow_v._on_mover_position_changed)
	follow_v.mover.speed_changed.connect(follow_v._on_mover_speed_changed)

	# Place the lead 100px ahead on the trajectory.
	lead_mover.s = 100.0
	lead_mover.position_on_road = lead_mover.trajectory.position_at(lead_mover.s)
	lead_mover.heading = lead_mover.trajectory.tangent_at(lead_mover.s)

	var ts := TrafficSystem.new()
	var delta := 1.0 / 60.0
	var overtakes: bool = false
	var prev_gap: float = INF

	for _i in range(600):
		# TrafficSystem sets ACC constraints based on current positions.
		ts.update([lead_v, follow_v], delta)
		lead_mover.update(delta)
		follow_mover.update(delta)
		# Check gap: follower must never be ahead of the lead.
		var gap: float = lead_mover.s - follow_mover.s
		if gap < 0.0:
			overtakes = true
		# Track the minimum gap for diagnostics.
		if gap < prev_gap:
			prev_gap = gap
		# Stop if the lead arrives.
		if not lead_mover.is_busy():
			break

	assert_false(overtakes, "follower should never overtake the lead (ACC holds the gap)")


func test_junction_conflict_yields_and_resumes() -> void:
	# Two vehicles approaching the same junction from perpendicular
	# directions. The further one should yield (stop), and the closer one
	# should proceed through. Once the closer one clears the junction,
	# the yielding one resumes.
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	# Junction at node (3, 3). Vehicle A approaches from the west along
	# row 3; vehicle B approaches from the north along column 3.
	var junction := Vector2i(3, 3)
	var junction_pos: Vector2 = graph.world_of(junction)

	# A: heading east toward the junction, 150px away.
	var path_a: Array[Vector2i] = []
	for c in range(0, 6):
		path_a.append(Vector2i(c, 3))
	# B: heading south toward the junction, 50px away (closer → has priority).
	var path_b: Array[Vector2i] = []
	for r in range(0, 6):
		path_b.append(Vector2i(3, r))

	var spec := VehicleSpec.new()
	spec.max_speed = 80.0
	spec.accel_rate = 200.0  # fast accel so it gets moving quickly
	spec.decel_rate = 300.0

	var mover_a := VehicleMover.new()
	mover_a.graph = graph
	mover_a.apply_spec(spec)
	mover_a.assign_path(path_a, 12.0, 22.0)
	# Place A 150px before the junction.
	var a_offset: float = junction_pos.x - 150.0 - mover_a.position_on_road.x
	mover_a.s = max(0.0, a_offset)
	mover_a.position_on_road = mover_a.trajectory.position_at(mover_a.s)
	mover_a.heading = mover_a.trajectory.tangent_at(mover_a.s)

	var mover_b := VehicleMover.new()
	mover_b.graph = graph
	mover_b.apply_spec(spec)
	mover_b.assign_path(path_b, 12.0, 22.0)
	# Place B 50px before the junction (closer → priority).
	var b_offset: float = junction_pos.y - 50.0 - mover_b.position_on_road.y
	mover_b.s = max(0.0, b_offset)
	mover_b.position_on_road = mover_b.trajectory.position_at(mover_b.s)
	mover_b.heading = mover_b.trajectory.tangent_at(mover_b.s)

	var v_a: VehicleController = VehicleScene.instantiate()
	add_child(v_a)
	autofree(v_a)
	v_a.mover = mover_a
	v_a.mover.position_changed.connect(v_a._on_mover_position_changed)
	v_a.mover.speed_changed.connect(v_a._on_mover_speed_changed)
	v_a.graph = graph
	v_a.path = path_a

	var v_b: VehicleController = VehicleScene.instantiate()
	add_child(v_b)
	autofree(v_b)
	v_b.mover = mover_b
	v_b.mover.position_changed.connect(v_b._on_mover_position_changed)
	v_b.mover.speed_changed.connect(v_b._on_mover_speed_changed)
	v_b.graph = graph
	v_b.path = path_b

	# Emit initial position so controllers are synced.
	mover_a.position_changed.emit(mover_a.position_on_road, mover_a.heading)
	mover_b.position_changed.emit(mover_b.position_on_road, mover_b.heading)

	var ts := TrafficSystem.new()
	var delta := 1.0 / 60.0
	var a_yielded: bool = false
	var a_resumed: bool = false

	for _i in range(900):
		ts.update([v_a, v_b], delta)
		mover_a.update(delta)
		mover_b.update(delta)
		# Track if A yielded (junction_yield active).
		if mover_a._junction_yield_speed >= 0.0:
			a_yielded = true
		# After yielding, check if A resumed (yield cleared AND moving).
		if a_yielded and mover_a._junction_yield_speed < 0.0 and mover_a.current_speed > 1.0:
			a_resumed = true
		if not mover_a.is_busy() and not mover_b.is_busy():
			break

	assert_true(a_yielded, "further vehicle (A) should yield at the junction")
	assert_true(a_resumed, "A should resume after B clears the junction")
