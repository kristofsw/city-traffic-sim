extends GutTest
## Unit tests for LineSeg.


func test_position_at_endpoints() -> void:
	var seg := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	assert_almost_eq(seg.position_at(0.0).x, 0.0, 0.01, "position_at(0) should be start")
	assert_almost_eq(seg.position_at(100.0).x, 100.0, 0.01, "position_at(length) should be end")


func test_tangent_is_constant() -> void:
	var seg := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	var t0 := seg.tangent_at(0.0)
	var t50 := seg.tangent_at(50.0)
	var t100 := seg.tangent_at(100.0)
	assert_almost_eq(t0, t50, 0.001, "tangent should be constant on a line")
	assert_almost_eq(t50, t100, 0.001, "tangent should be constant on a line")


func test_curvature_is_zero() -> void:
	var seg := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	assert_eq(seg.curvature_at(0.0), 0.0, "curvature should be 0 on a straight line")
	assert_eq(seg.curvature_at(50.0), 0.0, "curvature should be 0 on a straight line")


func test_progress_fraction() -> void:
	var seg := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	assert_almost_eq(seg.progress_fraction(0.0), 0.0, 0.001, "progress at start should be 0")
	assert_almost_eq(seg.progress_fraction(50.0), 0.5, 0.001, "progress at middle should be 0.5")
	assert_almost_eq(seg.progress_fraction(100.0), 1.0, 0.001, "progress at end should be 1")


func test_zero_length() -> void:
	var seg := LineSeg.new(Vector2(5, 5), Vector2(5, 5))
	assert_eq(seg.length, 0.0, "zero-length segment should have length 0")
	assert_almost_eq(seg.progress_fraction(0.0), 0.0, 0.001, "progress on zero-length should be 0")
