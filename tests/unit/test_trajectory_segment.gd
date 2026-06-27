extends GutTest
## Unit tests for TrajectorySegment base class.


func test_base_curvature_is_zero() -> void:
	var seg := TrajectorySegment.new()
	assert_eq(seg.curvature_at(0.0), 0.0, "base curvature_at should return 0")


func test_base_progress_fraction() -> void:
	var seg := TrajectorySegment.new()
	seg.length = 100.0
	assert_almost_eq(seg.progress_fraction(0.0), 0.0, 0.001, "progress at start should be 0")
	assert_almost_eq(seg.progress_fraction(50.0), 0.5, 0.001, "progress at middle should be 0.5")
	assert_almost_eq(seg.progress_fraction(100.0), 1.0, 0.001, "progress at end should be 1")


func test_base_progress_fraction_zero_length() -> void:
	var seg := TrajectorySegment.new()
	seg.length = 0.0
	assert_almost_eq(seg.progress_fraction(0.0), 0.0, 0.001, "progress on zero-length should be 0")
