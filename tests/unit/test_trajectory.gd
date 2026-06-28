extends GutTest
## Unit tests for the Trajectory wrapper (arc-length bookkeeping + segment
## lookup). Verifies the DRY abstraction used by VehicleController, RoadGrid
## and the integration tests.


func _make_straight_trajectory() -> Trajectory:
	# Two LineSeg segments of 100px each, end to end.
	var seg0 := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	var seg1 := LineSeg.new(Vector2(100, 0), Vector2(200, 0))
	return Trajectory.from_segments([seg0, seg1])


func _make_turning_trajectory() -> Trajectory:
	# Straight (100px) -> Bezier arc -> Straight (100px).
	var seg0 := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	# 90-degree right turn (y-down): from (100,0) curving to (100,100).
	var ctrl := Vector2(100, 0)
	var seg1 := BezierSeg.new(Vector2(100, 0), ctrl, Vector2(100, 100))
	var seg2 := LineSeg.new(Vector2(100, 100), Vector2(100, 200))
	return Trajectory.from_segments([seg0, seg1, seg2])


func test_from_segments_accumulates_arc_length() -> void:
	var t := _make_straight_trajectory()
	assert_eq(t.segments.size(), 2, "should hold 2 segments")
	assert_almost_eq(t.seg_start_arc[0], 0.0, 0.001, "first segment starts at arc 0")
	assert_almost_eq(t.seg_start_arc[1], 100.0, 0.001, "second segment starts at arc 100")
	assert_almost_eq(t.total_length, 200.0, 0.001, "total length should be 200")


func test_is_empty_and_clear() -> void:
	var t := _make_straight_trajectory()
	assert_false(t.is_empty(), "non-empty trajectory reports not empty")
	t.clear()
	assert_true(t.is_empty(), "cleared trajectory reports empty")
	assert_eq(t.segments.size(), 0, "cleared segments gone")
	assert_eq(t.seg_start_arc.size(), 0, "cleared arc bookkeeping gone")
	assert_almost_eq(t.total_length, 0.0, 0.001, "cleared total length is 0")


func test_segment_index_at_walks_forward_with_hint() -> void:
	var t := _make_straight_trajectory()
	# At arc 50 -> segment 0.
	assert_eq(t.segment_index_at(50.0, 0), 0, "arc 50 in segment 0")
	# At arc 100 exactly -> advances to segment 1 (walk condition is
	# s >= start+len, and 100 >= 0+100 is true).
	assert_eq(t.segment_index_at(100.0, 0), 1, "arc 100 boundary advances to segment 1")
	# At arc 99.999 -> still segment 0.
	assert_eq(t.segment_index_at(99.999, 0), 0, "arc 99.999 in segment 0")
	# Hint resumes from last known index.
	assert_eq(t.segment_index_at(150.0, 1), 1, "arc 150 in segment 1 (hint=1)")


func test_segment_index_at_walks_back_when_hint_overshot() -> void:
	var t := _make_straight_trajectory()
	# Hint overshoots to 1 but we ask for arc in segment 0.
	assert_eq(t.segment_index_at(50.0, 1), 0, "overshot hint walks back to segment 0")


func test_segment_index_at_clamps_past_end() -> void:
	var t := _make_straight_trajectory()
	assert_eq(t.segment_index_at(9999.0, 0), 1, "past end clamps to last segment")


func test_segment_index_at_empty_returns_minus_one() -> void:
	var t := Trajectory.new()
	assert_eq(t.segment_index_at(0.0), -1, "empty trajectory returns -1")


func test_local_s_at() -> void:
	var t := _make_straight_trajectory()
	assert_almost_eq(t.local_s_at(50.0, 0), 50.0, 0.001, "local s at 50 in seg 0 = 50")
	assert_almost_eq(t.local_s_at(150.0, 1), 50.0, 0.001, "local s at 150 in seg 1 = 50")


func test_position_at_matches_segment_position() -> void:
	var t := _make_straight_trajectory()
	assert_almost_eq(t.position_at(50.0).x, 50.0, 0.001, "x at arc 50 = 50")
	assert_almost_eq(t.position_at(50.0).y, 0.0, 0.001, "y at arc 50 = 0")
	assert_almost_eq(t.position_at(150.0).x, 150.0, 0.001, "x at arc 150 = 150")
	# Clamping past end.
	var end_pos := t.position_at(9999.0)
	assert_almost_eq(end_pos.x, 200.0, 0.001, "past end clamps to final x")


func test_tangent_at_matches_segment_tangent() -> void:
	var t := _make_straight_trajectory()
	# Both segments point along +x, so tangent is 0.0 radians.
	assert_almost_eq(t.tangent_at(50.0), 0.0, 0.001, "tangent at arc 50 = 0 rad")
	assert_almost_eq(t.tangent_at(150.0), 0.0, 0.001, "tangent at arc 150 = 0 rad")


func test_is_at_end() -> void:
	var t := _make_straight_trajectory()
	assert_false(t.is_at_end(0.0), "at start is not at end")
	assert_false(t.is_at_end(199.0), "just before end is not at end")
	assert_true(t.is_at_end(200.0), "at total_length is at end")
	assert_true(t.is_at_end(201.0), "past total_length is at end")


func test_turning_trajectory_segment_lookup() -> void:
	var t := _make_turning_trajectory()
	# The bezier segment should be found in the middle of the trajectory.
	# Find which index has non-zero curvature (the bezier).
	var bezier_idx := -1
	for i in t.segments.size():
		if t.segments[i].curvature_at(0.0) > 0.001:
			bezier_idx = i
			break
	assert_ne(bezier_idx, -1, "should find a bezier segment")
	# Sample a point inside the bezier's arc range and confirm lookup lands
	# on the bezier segment.
	var s_in_bezier: float = t.seg_start_arc[bezier_idx] + t.segments[bezier_idx].length * 0.5
	var idx: int = t.segment_index_at(s_in_bezier, 0)
	assert_eq(idx, bezier_idx, "mid-bezier arc lands on bezier segment")


func test_position_and_tangent_couple_at_same_arc() -> void:
	# Mirrors the vehicle's invariant: pos and tangent come from the same
	# parametric evaluation of the same segment.
	var t := _make_turning_trajectory()
	var s := t.seg_start_arc[1] + 10.0  # 10px into the bezier
	var idx: int = t.segment_index_at(s, 0)
	var local_s := s - t.seg_start_arc[idx]
	var pos := t.position_at(s, idx)
	var tan := t.tangent_at(s, idx)
	# Position should be on the bezier, tangent should match the segment's
	# own tangent_at at the same local_s.
	assert_almost_eq(pos.x, t.segments[idx].position_at(local_s).x, 0.001, "pos x matches segment")
	assert_almost_eq(tan, t.segments[idx].tangent_at(local_s), 0.001, "tangent matches segment")


func test_build_trajectory_equals_build_plus_from_segments() -> void:
	# build_trajectory should produce the same segments as build wrapped in
	# Trajectory.from_segments.
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(400, 400)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	var path: Array[Vector2i] = [Vector2i(0, 0), Vector2i(1, 0), Vector2i(1, 1)]
	var segs := TrajectoryBuilder.build(graph, path, 12.0, 22.0)
	var traj := TrajectoryBuilder.build_trajectory(graph, path, 12.0, 22.0)
	assert_eq(traj.segments.size(), segs.size(), "same segment count")
	for i in segs.size():
		assert_almost_eq(
			traj.segments[i].length, segs[i].length, 0.001, "segment %d length matches" % i
		)
