extends GutTest
## Tests for the VehicleSpec Resource and its wiring into VehicleMover/VehicleBody.


func test_spec_defaults_match_final_feel() -> void:
	# The spec defaults are the "final feel" tuning previously scattered
	# between vehicle_controller.gd script defaults and vehicle.tscn
	# overrides. The scene values (accel=50, decel=120, decel_dist=90,
	# min_turn_ratio=0.3, look_ahead=50) won and are now unified here.
	var s := VehicleSpec.new()
	assert_almost_eq(s.accel_rate, 50.0, 0.001, "accel_rate default should be the scene value 50")
	assert_almost_eq(s.decel_rate, 120.0, 0.001, "decel_rate default should be the scene value 120")
	assert_almost_eq(s.decel_distance, 90.0, 0.001, "decel_distance default should be 90")
	assert_almost_eq(
		s.min_turn_speed_ratio, 0.3, 0.001, "min_turn_speed_ratio default should be 0.3"
	)
	assert_almost_eq(s.turn_look_ahead, 50.0, 0.001, "turn_look_ahead default should be 50")
	assert_almost_eq(s.max_speed, 80.0, 0.001, "max_speed default should be 80")
	assert_almost_eq(s.lane_offset, 12.0, 0.001, "lane_offset default should be 12")
	assert_almost_eq(s.turn_radius, 22.0, 0.001, "turn_radius default should be 22")


func test_spec_colors_and_dimensions() -> void:
	var s := VehicleSpec.new()
	assert_almost_eq(s.body_length, 36.0, 0.001, "body_length default should be 36")
	assert_almost_eq(s.body_width, 18.0, 0.001, "body_width default should be 18")
	assert_almost_eq(s.headlight_radius, 2.2, 0.001, "headlight_radius default should be 2.2")
	assert_almost_eq(s.indicator_radius, 2.5, 0.001, "indicator_radius default should be 2.5")
	assert_almost_eq(
		s.indicator_blink_period, 0.4, 0.001, "indicator_blink_period default should be 0.4"
	)


func test_mover_uses_injected_spec() -> void:
	var s := VehicleSpec.new()
	s.max_speed = 120.0
	s.accel_rate = 70.0
	var m := VehicleMover.new()
	m.apply_spec(s)
	assert_almost_eq(m.max_speed, 120.0, 0.001, "mover should read max_speed from injected spec")
	assert_almost_eq(m.accel_rate, 70.0, 0.001, "mover should read accel_rate from injected spec")


func test_mover_falls_back_to_default_spec() -> void:
	var m := VehicleMover.new()
	# No spec injected -> default spec created on first access.
	assert_almost_eq(m.max_speed, 80.0, 0.001, "mover default max_speed should be 80")
	# Writing through the accessor writes to the default spec.
	m.max_speed = 100.0
	assert_almost_eq(m.max_speed, 100.0, 0.001, "mover max_speed setter should persist")


func test_mover_spec_setter_writes_through_to_spec() -> void:
	var s := VehicleSpec.new()
	s.accel_rate = 50.0
	var m := VehicleMover.new()
	m.apply_spec(s)
	m.accel_rate = 75.0
	assert_almost_eq(s.accel_rate, 75.0, 0.001, "mover setter should write through to the spec")
