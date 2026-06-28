extends GutTest
## Unit tests for VehicleMover logic (pure motion model, no Node/_process/_draw).
## The mover is a RefCounted object; tests call its pure methods directly.


func test_smoothstep_boundaries() -> void:
	var m := VehicleMover.new()
	# _smoothstep is private; exercise it via the end-ramp branch of
	# target_speed_at by building a trajectory and sampling near the end.
	_setup_line_then_bezier(m)
	# At the very end (s = total_length), end_factor = smoothstep(0) = 0.
	var end_target: float = m.target_speed_at(m.trajectory.total_length)
	assert_almost_eq(end_target, 0.0, 0.001, "target at end should be 0 (smoothstep(0)=0)")
	# Far before the end, end_factor = smoothstep(>=1) = 1.
	var start_target: float = m.target_speed_at(0.0)
	assert_almost_eq(start_target, m.max_speed, 0.001, "target at start should be max_speed")


func test_braking_intensity_zero_when_not_braking() -> void:
	var m := VehicleMover.new()
	# No trajectory assigned -> braking intensity should be 0.
	assert_eq(
		m.braking_intensity(0.0, 0.0), 0.0, "braking intensity should be 0 with no trajectory"
	)


func test_turn_factor_at_apex() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	m.turn_slowdown_factor = 0.5
	m.min_turn_speed_ratio = 0.25
	_setup_line_then_bezier(m)
	var apex_s: float = m.trajectory.seg_start_arc[1] + m.trajectory.segments[1].length / 2.0
	var factor: float = m.turn_factor_at(apex_s)
	var expected: float = max(1.0 - (PI / 2.0) * 0.5 * 1.0, 0.25)
	assert_almost_eq(factor, expected, 0.01, "turn factor at apex should match apex formula")


func test_turn_factor_at_entry_exit() -> void:
	var m := VehicleMover.new()
	m.turn_slowdown_factor = 0.5
	m.min_turn_speed_ratio = 0.25
	_setup_line_then_bezier(m)
	var entry_s: float = m.trajectory.seg_start_arc[1]
	assert_almost_eq(
		m.turn_factor_at(entry_s), 1.0, 0.001, "turn factor at arc entry should be 1.0"
	)
	var exit_s: float = m.trajectory.seg_start_arc[1] + m.trajectory.segments[1].length
	assert_almost_eq(m.turn_factor_at(exit_s), 1.0, 0.001, "turn factor at arc exit should be 1.0")


func test_turn_factor_at_line_seg() -> void:
	var m := VehicleMover.new()
	_setup_line_then_bezier(m)
	var mid_line: float = m.trajectory.seg_start_arc[0] + m.trajectory.segments[0].length / 2.0
	assert_almost_eq(m.turn_factor_at(mid_line), 1.0, 0.001, "turn factor on LineSeg should be 1.0")


func test_turn_factor_past_end_is_one() -> void:
	var m := VehicleMover.new()
	_setup_line_then_bezier(m)
	assert_almost_eq(
		m.turn_factor_at(m.trajectory.total_length + 100.0),
		1.0,
		0.001,
		"turn factor past end should be 1.0"
	)


func test_look_ahead_lowers_target_before_turn() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	m.turn_slowdown_factor = 0.5
	m.min_turn_speed_ratio = 0.25
	m.turn_look_ahead = 60.0
	m.decel_distance = 9999.0  # push end-ramp far away so it doesn't interfere
	_setup_line_then_bezier(m)
	var end_of_line: float = m.trajectory.seg_start_arc[0] + m.trajectory.segments[0].length
	var target: float = m.target_speed_at(end_of_line)
	assert_lt(target, m.max_speed, "look-ahead should lower target below max_speed before the turn")
	assert_lt(target, m.max_speed * 1.0, "target should be pulled below max_speed")


func test_windowed_factor_enforces_apex_floor() -> void:
	var m := VehicleMover.new()
	m.turn_slowdown_factor = 0.5
	m.min_turn_speed_ratio = 0.25
	m.turn_look_ahead = 60.0
	_setup_line_then_bezier(m)
	var arc_start: float = m.trajectory.seg_start_arc[1]
	var apex_arc: float = arc_start + m.trajectory.segments[1].length / 2.0
	var s: float = apex_arc - 20.0
	var windowed: float = m.turn_factor_windowed(s)
	var apex_factor: float = m.turn_factor_at(apex_arc)
	assert_almost_eq(
		windowed,
		apex_factor,
		0.001,
		"windowed factor should clamp to apex floor when apex in window"
	)
	assert_lt(
		windowed,
		m.turn_factor_at(s),
		"windowed factor should be below the entry-side endpoint sample"
	)
	assert_lt(
		windowed,
		m.turn_factor_at(s + m.turn_look_ahead),
		"windowed factor should be below the exit-side endpoint sample"
	)


func test_windowed_factor_monotonic_brake_in() -> void:
	var m := VehicleMover.new()
	m.turn_slowdown_factor = 0.5
	m.min_turn_speed_ratio = 0.25
	m.turn_look_ahead = 60.0
	_setup_line_then_bezier(m)
	var prev: float = m.turn_factor_windowed(0.0)
	var s: float = 5.0
	var arc_start: float = m.trajectory.seg_start_arc[1]
	while s <= arc_start:
		var cur: float = m.turn_factor_windowed(s)
		assert_true(
			cur <= prev + 0.0001,
			"windowed factor should be non-increasing while approaching turn (s=%.1f)" % s
		)
		prev = cur
		s += 5.0


func test_windowed_factor_monotonic_accel_out() -> void:
	var m := VehicleMover.new()
	m.turn_slowdown_factor = 0.5
	m.min_turn_speed_ratio = 0.25
	m.turn_look_ahead = 60.0
	_setup_line_then_bezier(m)
	var apex_arc: float = m.trajectory.seg_start_arc[1] + m.trajectory.segments[1].length / 2.0
	var s: float = apex_arc + 5.0
	var prev: float = m.turn_factor_windowed(s)
	s += 5.0
	while s <= m.trajectory.total_length:
		var cur: float = m.turn_factor_windowed(s)
		assert_true(
			cur >= prev - 0.0001, "windowed factor should be non-decreasing after apex (s=%.1f)" % s
		)
		prev = cur
		s += 5.0


func test_braking_intensity_coast_down_glow() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Coasting down: speed delta is negative (rate-limiter subtracted speed)
	# but the decel is gentle (small fraction of decel_rate) -> glows at or
	# above the COAST_DOWN_GLOW floor (any decel lifts the taillights).
	m.s = 0.0
	m.current_speed = m.max_speed  # == target on the line at s=0
	# A small speed delta, well below decel_rate*0.033 (~4 px/s).
	var intensity: float = m.braking_intensity(-0.5, m.max_speed)
	assert_gte(
		intensity, m.COAST_DOWN_GLOW, "gentle coast-down should glow at least at COAST_DOWN_GLOW"
	)


func test_braking_intensity_zero_when_accelerating() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Accelerating: speed delta is positive -> 0.0.
	m.s = 0.0
	m.current_speed = 40.0  # below target (max_speed on the line)
	assert_eq(
		m.braking_intensity(2.0, m.max_speed),
		0.0,
		"braking intensity should be 0 when accelerating"
	)


func test_braking_intensity_zero_when_cruising() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Cruising: speed delta ~0 (speed matches target, no change) -> 0.0.
	# This is the bug the user reported: cruising at constant speed must NOT
	# light the brake lights.
	m.s = 0.0
	m.current_speed = m.max_speed  # == target on the line at s=0
	assert_eq(
		m.braking_intensity(0.0, m.max_speed),
		0.0,
		"braking intensity should be 0 when cruising at constant speed"
	)


func test_braking_intensity_hard_brake_ramp() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Hard brake: speed overshoots the target by a large margin -> clamped
	# to 1.0 regardless of the per-frame delta.
	m.s = 0.0
	m.current_speed = m.max_speed  # target on the line is max_speed
	var target: float = m.max_speed * 0.5  # pretend a turn lowered the target
	var intensity: float = m.braking_intensity(-2.0, target)
	# (max_speed - 0.5*max_speed) / (max_speed * 0.2) = 2.5 -> clamped to 1.0
	assert_almost_eq(intensity, 1.0, 0.001, "hard brake above the 20% band should clamp to 1.0")
	# Just barely above target: small overshoot -> at least COAST_DOWN_GLOW.
	m.current_speed = target + m.max_speed * 0.1
	var intensity2: float = m.braking_intensity(-2.0, target)
	assert_gte(
		intensity2, m.COAST_DOWN_GLOW, "10% overshoot should glow at least at COAST_DOWN_GLOW"
	)


func test_braking_intensity_full_decel_rate_bright() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Full decel_rate braking (end of trip): the per-frame speed delta equals
	# decel_rate * delta. With delta=1/60 and decel_rate=120, that's 2 px/s.
	# The decel_fraction = -speed_delta / (decel_rate * 0.033) should be ~0.5
	# so the glow should be well above the floor (lerp(0.5, 1.0, 0.5) ~ 0.75).
	# Set current_speed == target so the hard-brake branch doesn't intercept;
	# we want to exercise the decelerating branch.
	m.s = 0.0
	m.current_speed = m.max_speed * 0.5
	var target: float = m.max_speed * 0.5
	var speed_delta: float = -m.decel_rate * (1.0 / 60.0)  # -2.0 px/s
	var intensity: float = m.braking_intensity(speed_delta, target)
	# decel_fraction = 2.0 / (120 * 0.033) = 2.0 / 3.96 ~ 0.505 -> lerp(0.5, 1.0, 0.505) ~ 0.75
	assert_gte(intensity, 0.7, "full decel_rate braking should glow bright (>= 0.7)")


func test_braking_intensity_bright_while_stopped() -> void:
	var m := VehicleMover.new()
	m.max_speed = 80.0
	_setup_line_then_bezier(m)
	# Hold-still: target ~0 and speed ~0 -> full bright (future traffic lights).
	m.s = 0.0
	m.current_speed = 0.0
	var intensity: float = m.braking_intensity(-0.1, 0.0)
	assert_almost_eq(
		intensity, m.HOLD_STILL_GLOW, 0.001, "stopped at zero target should be full bright"
	)


func test_upcoming_turn_direction_before_turn() -> void:
	var m := VehicleMover.new()
	m.turn_look_ahead = 60.0
	_setup_line_then_bezier(m)
	var s: float = m.trajectory.seg_start_arc[1] - 20.0
	assert_eq(
		m.upcoming_turn_direction(s),
		1,
		"should report the upcoming right turn before entering the arc"
	)


func test_upcoming_turn_direction_no_turn_in_window() -> void:
	var m := VehicleMover.new()
	m.turn_look_ahead = 10.0  # small window so the arc is out of range
	_setup_line_then_bezier(m)
	assert_eq(
		m.upcoming_turn_direction(0.0),
		0,
		"should report no turn when the arc is outside the look-ahead window"
	)


func test_upcoming_turn_direction_inside_arc() -> void:
	var m := VehicleMover.new()
	m.turn_look_ahead = 60.0
	_setup_line_then_bezier(m)
	var apex_s: float = m.trajectory.seg_start_arc[1] + m.trajectory.segments[1].length / 2.0
	assert_eq(
		m.upcoming_turn_direction(apex_s),
		1,
		"should report the turn direction while inside the arc"
	)


## Build a LineSeg (100px) followed by a 90-degree right-turn BezierSeg arc
## and wire up the mover's trajectory. The bezier goes right then down
## (clockwise in y-down screen space) -> turn_direction() == +1,
## total_turn_angle ~ PI/2. Also sets _eff_decel so the end-ramp is active
## (assign_path would normally do this).
func _setup_line_then_bezier(m: VehicleMover) -> void:
	var line := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	var bez := BezierSeg.new(Vector2(100, 0), Vector2(150, 0), Vector2(150, 50))
	m.trajectory = Trajectory.from_segments([line, bez])
	m._eff_decel = min(m.decel_distance, m.trajectory.total_length * 0.4)
