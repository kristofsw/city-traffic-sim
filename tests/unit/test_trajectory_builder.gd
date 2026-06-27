extends GutTest
## Unit tests for TrajectoryBuilder.


func _build_test_graph() -> RoadGraph:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(400, 400)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	return graph


func test_straight_path_produces_only_line_segs() -> void:
	var graph := _build_test_graph()
	# Path: (0,0) -> (1,0) -> (2,0) — all same direction, no turns.
	var path: Array[Vector2i] = [Vector2i(0, 0), Vector2i(1, 0), Vector2i(2, 0)]
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	assert_gt(segs.size(), 0, "straight path should produce segments")
	for seg in segs:
		assert_true(seg is LineSeg, "straight path should produce only LineSeg segments")


func test_turning_path_produces_bezier() -> void:
	var graph := _build_test_graph()
	# Path: (0,0) -> (1,0) -> (1,1) — 90-degree turn at (1,0).
	var path: Array[Vector2i] = [Vector2i(0, 0), Vector2i(1, 0), Vector2i(1, 1)]
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	var has_bezier := false
	for seg in segs:
		if seg is BezierSeg:
			has_bezier = true
	assert_true(has_bezier, "turning path should produce at least one BezierSeg")


func test_segments_are_contiguous() -> void:
	var graph := _build_test_graph()
	var path: Array[Vector2i] = [Vector2i(0, 0), Vector2i(1, 0), Vector2i(1, 1), Vector2i(2, 1)]
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	for i in range(segs.size() - 1):
		var end_pos := segs[i].position_at(segs[i].length)
		var start_next := segs[i + 1].position_at(0.0)
		assert_almost_eq(
			end_pos.distance_to(start_next),
			0.0,
			1.0,
			"segment %d end should be within 1px of segment %d start" % [i, i + 1]
		)


func test_all_segments_have_positive_length() -> void:
	var graph := _build_test_graph()
	var path: Array[Vector2i] = [Vector2i(0, 0), Vector2i(1, 0), Vector2i(1, 1)]
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	for i in range(segs.size()):
		assert_gt(segs[i].length, 0.0, "segment %d should have positive length" % i)


func test_empty_path_returns_empty() -> void:
	var graph := _build_test_graph()
	var path: Array[Vector2i] = []
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	assert_eq(segs.size(), 0, "empty path should produce no segments")


func test_single_node_path_returns_empty() -> void:
	var graph := _build_test_graph()
	var path: Array[Vector2i] = [Vector2i(0, 0)]
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	assert_eq(segs.size(), 0, "single-node path should produce no segments")
