extends GutTest
## Integration test: full pipeline from grid generation to vehicle arrival.
## Verifies the right-lane invariant holds throughout a complete trip.


func test_full_trip_right_lane_invariant() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	# Find a path with at least one turn.
	var start := Vector2i(0, 0)
	var goal := Vector2i(5, 3)
	var path := graph.find_path(start, goal)
	assert_gt(path.size(), 1, "path should exist from (0,0) to (5,3)")

	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	assert_gt(segs.size(), 0, "trajectory should have segments")

	# Simulate driving the full trajectory at fixed speed.
	var total_length := 0.0
	var seg_start_arc: Array[float] = []
	for seg in segs:
		seg_start_arc.append(total_length)
		total_length += seg.length

	var s := 0.0
	var speed := 120.0
	var delta := 1.0 / 60.0
	var min_signed_offset := INF

	while s < total_length:
		# Find current segment.
		var seg_idx := 0
		while seg_idx < segs.size() - 1 and s >= seg_start_arc[seg_idx] + segs[seg_idx].length:
			seg_idx += 1
		var local_s := s - seg_start_arc[seg_idx]
		var pos := segs[seg_idx].position_at(local_s)

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
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)

	var path := graph.find_path(Vector2i(0, 0), Vector2i(9, 5))
	assert_gt(path.size(), 1, "path should exist across the grid")

	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	for i in range(segs.size() - 1):
		var end_pos := segs[i].position_at(segs[i].length)
		var start_next := segs[i + 1].position_at(0.0)
		assert_almost_eq(
			end_pos.distance_to(start_next),
			0.0,
			2.0,
			"segment %d end should connect to segment %d start" % [i, i + 1]
		)
