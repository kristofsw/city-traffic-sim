extends GutTest
## Unit tests for VehicleMover's Adaptive Cruise Control (ACC) constraint.
## Extracted from test_vehicle_mover.gd so that file stays under gdlint's
## max-public-methods limit. The mover is a RefCounted object; tests call
## its pure methods directly.


func _setup_line_then_bezier(m: VehicleMover) -> void:
	var line := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	var bez := BezierSeg.new(Vector2(100, 0), Vector2(150, 0), Vector2(150, 50))
	m.trajectory = Trajectory.from_segments([line, bez])
	m._eff_decel = min(m.decel_distance, m.trajectory.total_length * 0.4)


func test_set_lead_constraint_caps_target_speed() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Natural target at s=0 is max_speed (no turn/end effect at the start).
	var natural: float = m.target_speed_at(0.0)
	assert_almost_eq(natural, 80.0, 0.001, "natural target at start should be max_speed")
	# Set a lead constraint: gap=90px, lead_speed=40, min_gap=40, time_gap=1.5
	# safe = 40 + (90-40)/1.5 = 40 + 33.33 = 73.33 -> caps natural 80.
	m.set_lead_constraint(90.0, 40.0)
	assert_almost_eq(
		m.target_speed_at(0.0), 73.33, 0.01, "ACC should cap target to safe speed 73.33"
	)
	# Tighter gap: gap=60, lead=40 -> safe = 40 + (60-40)/1.5 = 40 + 13.33 = 53.33
	m.set_lead_constraint(60.0, 40.0)
	assert_almost_eq(
		m.target_speed_at(0.0), 53.33, 0.01, "ACC should cap target to safe speed 53.33"
	)
	# Very tight: gap < min_gap -> safe = 0
	m.set_lead_constraint(20.0, 40.0)
	assert_almost_eq(m.target_speed_at(0.0), 0.0, 0.001, "gap below min_gap should cap target to 0")


func test_clear_lead_constraint_restores_natural_target() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	m.set_lead_constraint(60.0, 40.0)
	assert_lt(m.target_speed_at(0.0), 80.0, "constrained target should be below natural")
	m.clear_lead_constraint()
	assert_almost_eq(
		m.target_speed_at(0.0), 80.0, 0.001, "clear_lead_constraint should restore natural target"
	)


func test_lead_constraint_negative_gap_clears() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	m.set_lead_constraint(60.0, 40.0)
	# Passing gap < 0 should act as "no lead" (clear).
	m.set_lead_constraint(-1.0, 0.0)
	assert_almost_eq(m.target_speed_at(0.0), 80.0, 0.001, "negative gap should clear constraint")
