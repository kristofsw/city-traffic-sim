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


func test_turn_factor_at_apex() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.max_speed = 80.0
	vc.turn_slowdown_factor = 0.5
	vc.min_turn_speed_ratio = 0.25
	_setup_line_then_bezier(vc)
	# Apex of the bezier is at its midpoint: seg_start_arc[1] + bezier.length/2.
	var apex_s: float = vc.seg_start_arc[1] + vc.segments[1].length / 2.0
	var factor: float = vc._turn_factor_at(apex_s)
	# At apex apex_weight=1, turn_angle=PI/2 -> factor = 1 - (PI/2)*0.5*1.
	var expected: float = max(1.0 - (PI / 2.0) * 0.5 * 1.0, 0.25)
	assert_almost_eq(factor, expected, 0.01, "turn factor at apex should match apex formula")


func test_turn_factor_at_entry_exit() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_slowdown_factor = 0.5
	vc.min_turn_speed_ratio = 0.25
	_setup_line_then_bezier(vc)
	# At the entry of the bezier (progress=0) apex_weight=0 -> factor=1.0.
	var entry_s: float = vc.seg_start_arc[1]
	assert_almost_eq(
		vc._turn_factor_at(entry_s), 1.0, 0.001, "turn factor at arc entry should be 1.0"
	)
	# At the exit of the bezier (progress=1) apex_weight=0 -> factor=1.0.
	var exit_s: float = vc.seg_start_arc[1] + vc.segments[1].length
	assert_almost_eq(
		vc._turn_factor_at(exit_s), 1.0, 0.001, "turn factor at arc exit should be 1.0"
	)


func test_turn_factor_at_line_seg() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	_setup_line_then_bezier(vc)
	# On the straight LineSeg, curvature is 0 -> factor 1.0 everywhere.
	var mid_line: float = vc.seg_start_arc[0] + vc.segments[0].length / 2.0
	assert_almost_eq(
		vc._turn_factor_at(mid_line), 1.0, 0.001, "turn factor on LineSeg should be 1.0"
	)


func test_turn_factor_past_end_is_one() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	_setup_line_then_bezier(vc)
	# Past total_length there is no turn -> 1.0 (so look-ahead doesn't spuriously lower).
	assert_almost_eq(
		vc._turn_factor_at(vc.total_length + 100.0),
		1.0,
		0.001,
		"turn factor past end should be 1.0"
	)


func test_look_ahead_lowers_target_before_turn() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.max_speed = 80.0
	vc.turn_slowdown_factor = 0.5
	vc.min_turn_speed_ratio = 0.25
	vc.turn_look_ahead = 60.0
	vc.decel_distance = 9999.0  # push end-ramp far away so it doesn't interfere
	_setup_line_then_bezier(vc)
	# At the END of the LineSeg (just before the bezier), with no look-ahead the
	# turn factor would be 1.0 (still on a straight). With look-ahead, the
	# future bezier pulls the target down -> target < max_speed.
	var end_of_line: float = vc.seg_start_arc[0] + vc.segments[0].length
	var target: float = vc._target_speed_at(end_of_line)
	assert_lt(
		target, vc.max_speed, "look-ahead should lower target below max_speed before the turn"
	)
	# And it should be strictly less than the turn factor at the current
	# position alone (1.0 on the line) would give -- i.e. max_speed.
	assert_lt(target, vc.max_speed * 1.0, "target should be pulled below max_speed")


func test_windowed_factor_enforces_apex_floor() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_slowdown_factor = 0.5
	vc.min_turn_speed_ratio = 0.25
	vc.turn_look_ahead = 60.0
	_setup_line_then_bezier(vc)
	# Place the window so the bezier apex is strictly inside it: start on the
	# line, 20px before the arc. The apex is at seg_start_arc[1] + length/2.
	var arc_start: float = vc.seg_start_arc[1]
	var apex_arc: float = arc_start + vc.segments[1].length / 2.0
	var s: float = apex_arc - 20.0
	# The two-point min(T(s), T(s+L)) would miss the apex (both endpoints sit
	# on the sides of the V, above the apex value). The windowed min must clamp
	# to the apex factor -- the real corner floor.
	var windowed: float = vc._turn_factor_windowed(s)
	var apex_factor: float = vc._turn_factor_at(apex_arc)
	assert_almost_eq(
		windowed,
		apex_factor,
		0.001,
		"windowed factor should clamp to apex floor when apex in window"
	)
	assert_lt(
		windowed,
		vc._turn_factor_at(s),
		"windowed factor should be below the entry-side endpoint sample"
	)
	assert_lt(
		windowed,
		vc._turn_factor_at(s + vc.turn_look_ahead),
		"windowed factor should be below the exit-side endpoint sample"
	)


func test_windowed_factor_monotonic_brake_in() -> void:
	# Regression guard against the W-shape / double brake: as the car
	# approaches a turn, _turn_factor_windowed(s) must be NON-INCREASING.
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_slowdown_factor = 0.5
	vc.min_turn_speed_ratio = 0.25
	vc.turn_look_ahead = 60.0
	_setup_line_then_bezier(vc)
	# Walk s from the start of the line to the arc entry in 5px steps.
	var prev: float = vc._turn_factor_windowed(0.0)
	var s: float = 5.0
	var arc_start: float = vc.seg_start_arc[1]
	while s <= arc_start:
		var cur: float = vc._turn_factor_windowed(s)
		assert_true(
			cur <= prev + 0.0001,
			"windowed factor should be non-increasing while approaching turn (s=%.1f)" % s
		)
		prev = cur
		s += 5.0


func test_windowed_factor_monotonic_accel_out() -> void:
	# After the apex leaves the window, _turn_factor_windowed(s) must be
	# NON-DECREASING -- the car spools back up toward cruise, single release.
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_slowdown_factor = 0.5
	vc.min_turn_speed_ratio = 0.25
	vc.turn_look_ahead = 60.0
	_setup_line_then_bezier(vc)
	# Start just after the apex has left the window: s > apex_arc.
	var apex_arc: float = vc.seg_start_arc[1] + vc.segments[1].length / 2.0
	var s: float = apex_arc + 5.0
	var prev: float = vc._turn_factor_windowed(s)
	s += 5.0
	while s <= vc.total_length:
		var cur: float = vc._turn_factor_windowed(s)
		assert_true(
			cur >= prev - 0.0001, "windowed factor should be non-decreasing after apex (s=%.1f)" % s
		)
		prev = cur
		s += 5.0


## Build a LineSeg (100px) followed by a 90-degree right-turn BezierSeg arc
## and wire up the vehicle's segment bookkeeping (segments, seg_start_arc,
## total_length). The bezier goes right then down (clockwise in y-down screen
## space) -> turn_direction() == +1, total_turn_angle ~ PI/2.
func _setup_line_then_bezier(vc: VehicleController) -> void:
	var line := LineSeg.new(Vector2(0, 0), Vector2(100, 0))
	# Right turn: p0=(100,0), control=(150,0) [entry tangent right], p1=(150,50)
	# [exit tangent down]. cross((50,0),(0,50)) = 2500 > 0 -> right (+1).
	var bez := BezierSeg.new(Vector2(100, 0), Vector2(150, 0), Vector2(150, 50))
	vc.segments = [line, bez]
	vc.seg_start_arc = [0.0, line.length]
	vc.total_length = line.length + bez.length


func test_braking_intensity_coast_down_glow() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.max_speed = 80.0
	_setup_line_then_bezier(vc)
	# Coasting down: speed == target (no overshoot) but the rate-limiter is
	# subtracting this frame -> gentle glow, not zero.
	vc.current_speed = 50.0
	vc._decelerating = true
	# Place s where target is below current_speed so the coast-down branch is
	# the one taken: set target by choosing s near the end ramp. Simpler: set
	# current_speed == target by picking s on the line (target == max_speed)
	# and current_speed == max_speed with _decelerating true.
	vc.s = 0.0
	vc.current_speed = vc.max_speed  # == target on the line at s=0
	var intensity: float = vc._braking_intensity()
	assert_almost_eq(
		intensity, vc.COAST_DOWN_GLOW, 0.001, "coast-down should glow at COAST_DOWN_GLOW"
	)


func test_braking_intensity_zero_when_accelerating() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.max_speed = 80.0
	_setup_line_then_bezier(vc)
	# Accelerating: speed <= target and NOT decelerating -> 0.0.
	vc.s = 0.0
	vc.current_speed = 40.0  # below target (max_speed on the line)
	vc._decelerating = false
	assert_eq(vc._braking_intensity(), 0.0, "braking intensity should be 0 when accelerating")


func test_upcoming_turn_direction_before_turn() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_look_ahead = 60.0
	_setup_line_then_bezier(vc)
	# On the line, 20px before the arc, the bezier is within the look-ahead
	# window -> direction should be the bezier's (right, +1), not 0.
	var s: float = vc.seg_start_arc[1] - 20.0
	assert_eq(
		vc._upcoming_turn_direction(s),
		1,
		"should report the upcoming right turn before entering the arc"
	)


func test_upcoming_turn_direction_no_turn_in_window() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_look_ahead = 10.0  # small window so the arc is out of range
	_setup_line_then_bezier(vc)
	# At the start of the line, with a 10px window, the bezier (100px ahead)
	# is outside -> 0.
	assert_eq(
		vc._upcoming_turn_direction(0.0),
		0,
		"should report no turn when the arc is outside the look-ahead window"
	)


func test_upcoming_turn_direction_inside_arc() -> void:
	var vc := VehicleController.new()
	autofree(vc)
	vc.turn_look_ahead = 60.0
	_setup_line_then_bezier(vc)
	# At the apex of the bezier itself, the segment overlaps the window -> +1.
	var apex_s: float = vc.seg_start_arc[1] + vc.segments[1].length / 2.0
	assert_eq(
		vc._upcoming_turn_direction(apex_s),
		1,
		"should report the turn direction while inside the arc"
	)
