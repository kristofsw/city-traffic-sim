extends GutTest
## Unit tests for VehicleController logic (no _process/_draw, pure methods).


func test_smoothstep_boundaries() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	assert_almost_eq(vc._smoothstep(0.0), 0.0, 0.001, "smoothstep(0) = 0")
	assert_almost_eq(vc._smoothstep(1.0), 1.0, 0.001, "smoothstep(1) = 1")
	assert_almost_eq(vc._smoothstep(0.5), 0.5, 0.001, "smoothstep(0.5) = 0.5")
	# Outside [0,1] should clamp.
	assert_almost_eq(vc._smoothstep(-0.5), 0.0, 0.001, "smoothstep(-) = 0")
	assert_almost_eq(vc._smoothstep(1.5), 1.0, 0.001, "smoothstep(+) = 1")


func test_braking_intensity_zero_when_not_braking() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	# No segments assigned -> braking intensity should be 0.
	assert_eq(vc._braking_intensity(), 0.0, "braking intensity should be 0 with no segments")


func test_dist_point_to_segment() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	# Point directly above the midpoint of a horizontal segment.
	var d := vc._dist_point_to_segment(Vector2(50, 10), Vector2(0, 0), Vector2(100, 0))
	assert_almost_eq(d, 10.0, 0.01, "distance from point to segment should be 10")
	# Point at the start of the segment.
	var d2 := vc._dist_point_to_segment(Vector2(0, 0), Vector2(0, 0), Vector2(100, 0))
	assert_almost_eq(d2, 0.0, 0.01, "distance from point on segment should be 0")
