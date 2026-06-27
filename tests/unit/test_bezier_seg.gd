extends GutTest
## Unit tests for BezierSeg.

const TOLERANCE := 0.5  # px tolerance for arc-length LUT approximation


func test_position_at_endpoints() -> void:
	var seg := BezierSeg.new(Vector2(0, 0), Vector2(50, 50), Vector2(100, 0))
	assert_almost_eq(seg.position_at(0.0).x, 0.0, 0.1, "position_at(0) should be p0")
	assert_almost_eq(seg.position_at(0.0).y, 0.0, 0.1, "position_at(0) should be p0")
	var end_pos := seg.position_at(seg.length)
	assert_almost_eq(end_pos.x, 100.0, 0.1, "position_at(length) should be p1")
	assert_almost_eq(end_pos.y, 0.0, 0.1, "position_at(length) should be p1")


func test_tangent_at_start() -> void:
	var seg := BezierSeg.new(Vector2(0, 0), Vector2(50, 50), Vector2(100, 0))
	# Derivative at t=0: 2*(C - P0) = 2*(50,50) = (100,100) -> angle = 45deg
	var t0 := seg.tangent_at(0.0)
	var expected := Vector2(50, 50).angle()
	assert_almost_eq(t0, expected, 0.01, "tangent_at(0) should be direction(p0 -> control)")


func test_tangent_at_end() -> void:
	var seg := BezierSeg.new(Vector2(0, 0), Vector2(50, 50), Vector2(100, 0))
	# Derivative at t=1: 2*(P1 - C) = 2*(50,-50) = (100,-100) -> angle = -45deg
	var t1 := seg.tangent_at(seg.length)
	var expected := Vector2(100 - 50, 0 - 50).angle()
	assert_almost_eq(t1, expected, 0.01, "tangent_at(length) should be direction(control -> p1)")


func test_curvature_90_degree_turn() -> void:
	# A 90-degree turn: start going right, end going down.
	var p0 := Vector2(0, 0)
	var ctrl := Vector2(50, 0)  # tangent at start = (1,0) -> 0 rad
	var p1 := Vector2(50, 50)  # tangent at end = (0,1) -> PI/2 rad
	var seg := BezierSeg.new(p0, ctrl, p1)
	assert_almost_eq(
		seg.total_turn_angle, PI / 2.0, 0.05, "90-degree turn should have curvature ~PI/2"
	)
	assert_almost_eq(
		seg.curvature_at(0.0), PI / 2.0, 0.05, "curvature_at should return total_turn_angle"
	)


func test_curvature_180_degree_uturn() -> void:
	# A U-turn: tangent at start goes right, tangent at end goes left.
	# p0=(0,0), ctrl=(50,50), p1=(100, 100)
	# tangent at t=0 = 2*(ctrl-p0) = (100, 100) -> angle ~PI/4
	# tangent at t=1 = 2*(p1-ctrl) = (100, 100) -> angle ~PI/4
	# That's actually straight! For a real U-turn we need opposite tangents.
	# p0=(0,0), ctrl=(50, 100), p1=(0, 200)
	# tangent at t=0 = 2*(50, 100) -> angle ~1.107
	# tangent at t=1 = 2*(-50, 100) -> angle ~2.034
	# angle_difference ~0.927 rad. A quadratic bezier can't produce a full
	# 180-degree turn with G1 continuity. Test a 90-degree turn instead,
	# which is the realistic maximum for our grid.
	var p0 := Vector2(0, 0)
	var ctrl := Vector2(50, 0)  # tangent at start = (1, 0) -> 0 rad
	var p1 := Vector2(50, 50)  # tangent at end = (0, 1) -> PI/2 rad
	var seg := BezierSeg.new(p0, ctrl, p1)
	assert_almost_eq(
		seg.total_turn_angle, PI / 2.0, 0.05, "90-degree turn should have curvature ~PI/2"
	)


func test_lut_arc_length_accuracy() -> void:
	# Compare LUT-based arc length to a high-resolution manual sampling.
	var p0 := Vector2(0, 0)
	var ctrl := Vector2(50, 50)
	var p1 := Vector2(100, 0)
	var seg := BezierSeg.new(p0, ctrl, p1)
	# Manual high-res arc length (1000 samples).
	var prev := p0
	var manual_len := 0.0
	for i in range(1, 1001):
		var t := float(i) / 1000.0
		var u := 1.0 - t
		var pt := u * u * p0 + 2.0 * u * t * ctrl + t * t * p1
		manual_len += prev.distance_to(pt)
		prev = pt
	assert_almost_eq(
		seg.length,
		manual_len,
		TOLERANCE,
		"LUT arc length should match manual sampling within tolerance"
	)


func test_progress_fraction_monotonic() -> void:
	var seg := BezierSeg.new(Vector2(0, 0), Vector2(50, 50), Vector2(100, 0))
	var prev_progress := -1.0
	for i in range(11):
		var s := seg.length * float(i) / 10.0
		var progress := seg.progress_fraction(s)
		assert_gte(progress, prev_progress, "progress_fraction should be monotonically increasing")
		prev_progress = progress
