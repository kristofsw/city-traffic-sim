extends GutTest
## Unit tests for TrafficSystem (ACC coordinator). Uses lightweight
## VehicleController stand-ins: real controllers are Nodes, but the
## TrafficSystem only reads position_on_road, heading, current_speed, and
## mover. We build minimal controllers via the vehicle scene so movers +
## specs are real, then override their position/heading/speed directly.

const VehicleScene := preload("res://scenes/vehicle.tscn")


func _build_vehicle(pos: Vector2, heading: float, speed: float) -> VehicleController:
	var v: VehicleController = VehicleScene.instantiate()
	add_child(v)
	autofree(v)
	v.position_on_road = pos
	v.heading = heading
	v.current_speed = speed
	return v


func test_no_lead_when_alone() -> void:
	var ts := TrafficSystem.new()
	var v := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	ts.update([v], 1.0 / 60.0)
	assert_eq(v.mover._acc_target_speed, -1.0, "lone vehicle should have no ACC constraint")


func test_lead_detected_directly_ahead() -> void:
	var ts := TrafficSystem.new()
	# Follower at origin heading east (0 rad); lead 80px ahead, same lane.
	# Both cars body_length=36 -> bumper-to-bumper gap = 80 - 36 = 44px.
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	var lead := _build_vehicle(Vector2(80, 0), 0.0, 40.0)
	ts.update([follower, lead], 1.0 / 60.0)
	# gap=44, lead_speed=40, min_gap=40, time_gap=1.5
	# safe = 40 + (44-40)/1.5 = 40 + 2.67 = 42.67
	assert_almost_eq(
		follower.mover._acc_target_speed,
		40.0 + (44.0 - 40.0) / 1.5,
		0.01,
		"follower should match lead speed + gap term"
	)
	# Lead has no one ahead -> no constraint.
	assert_eq(lead.mover._acc_target_speed, -1.0, "lead (frontmost) should have no constraint")


func test_no_lead_when_behind() -> void:
	var ts := TrafficSystem.new()
	# Follower at origin heading east; other vehicle 80px BEHIND (west).
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	var other := _build_vehicle(Vector2(-80, 0), 0.0, 40.0)
	ts.update([follower, other], 1.0 / 60.0)
	assert_eq(
		follower.mover._acc_target_speed, -1.0, "vehicle behind should not be detected as a lead"
	)


func test_no_lead_outside_cone() -> void:
	var ts := TrafficSystem.new()
	# Follower heading east; other 80px ahead but 90° off-axis (north).
	# 90° > CONE_HALF_ANGLE (~30°) -> not a lead.
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	var other := _build_vehicle(Vector2(0, -80), 0.0, 40.0)
	ts.update([follower, other], 1.0 / 60.0)
	assert_eq(
		follower.mover._acc_target_speed,
		-1.0,
		"vehicle outside the forward cone should not be a lead"
	)


func test_no_lead_beyond_look_ahead() -> void:
	var ts := TrafficSystem.new()
	# Follower at standstill (speed=0) -> look_ahead = 0 + 40 + 60 = 100px.
	# Other 200px ahead -> beyond look-ahead -> no lead.
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 0.0)
	var other := _build_vehicle(Vector2(200, 0), 0.0, 40.0)
	ts.update([follower, other], 1.0 / 60.0)
	assert_eq(
		follower.mover._acc_target_speed,
		-1.0,
		"vehicle beyond look-ahead range should not be a lead"
	)


func test_nearest_lead_wins() -> void:
	var ts := TrafficSystem.new()
	# Two leads ahead: near at 60px (speed 20), far at 120px (speed 60).
	# bumper-to-bumper gaps: 60-36=24, 120-36=84.
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	var near := _build_vehicle(Vector2(60, 0), 0.0, 20.0)
	var far := _build_vehicle(Vector2(120, 0), 0.0, 60.0)
	ts.update([follower, near, far], 1.0 / 60.0)
	# safe from near: 20 + (24-40)/1.5 -> negative -> 0 (gap < min_gap)
	assert_eq(
		follower.mover._acc_target_speed,
		0.0,
		"nearest lead (bumper gap 24 < min_gap 40) should force stop"
	)


func test_gap_below_min_gap_stops() -> void:
	var ts := TrafficSystem.new()
	# Two cars 50px center-to-center -> bumper gap = 50-36 = 14px < min_gap 40 -> stop.
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	var lead := _build_vehicle(Vector2(50, 0), 0.0, 40.0)
	ts.update([follower, lead], 1.0 / 60.0)
	assert_eq(
		follower.mover._acc_target_speed, 0.0, "gap below min_gap should force target to 0 (stop)"
	)
