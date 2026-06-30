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


## Build a small grid graph and two vehicles with paths sharing a junction
## node. Vehicle A heads east along row 0; vehicle B heads south along
## column 2. They share node (2,0).
func _build_grid_and_vehicles() -> Array:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1200, 900)
	gen.margin_px = 40.0
	gen.target_block_size = 160.0
	gen.obstacle_count = 0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	# A: (0,0) -> (5,0) heading east. B: (2,-1)->(2,3) heading south.
	# Shared node (2,0).
	var a := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	a.graph = graph
	a.path = [
		Vector2i(0, 0),
		Vector2i(1, 0),
		Vector2i(2, 0),
		Vector2i(3, 0),
		Vector2i(4, 0),
		Vector2i(5, 0)
	]
	var b := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	b.graph = graph
	b.path = [Vector2i(2, 0), Vector2i(2, 1), Vector2i(2, 2), Vector2i(2, 3)]
	return [graph, a, b]


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


func test_oncoming_traffic_not_a_lead() -> void:
	var ts := TrafficSystem.new()
	# Follower heading east (0 rad); oncoming bus heading west (PI rad), 80px
	# ahead in the other lane (slight lateral offset). Oncoming traffic must
	# not be treated as a lead -- the follower should NOT slow down.
	var follower := _build_vehicle(Vector2(0, 0), 0.0, 80.0)
	var oncoming := _build_vehicle(Vector2(80, 32), PI, 50.0)
	oncoming.mover._ensure_spec().body_width = 22.0
	oncoming.mover._ensure_spec().body_length = 52.0
	ts.update([follower, oncoming], 1.0 / 60.0)
	assert_eq(
		follower.mover._acc_target_speed,
		-1.0,
		"oncoming traffic (opposite heading) should not be treated as a lead"
	)


# ---------------------------------------------------------------------------
# Junction conflict tests
# ---------------------------------------------------------------------------


func test_junction_further_vehicle_yields() -> void:
	var ts := TrafficSystem.new()
	var result := _build_grid_and_vehicles()
	var a: VehicleController = result[1]
	var b: VehicleController = result[2]
	# Shared node (2,0). A is at (1,0)-ish heading east, B is at (2,-1)-ish
	# heading south — but we set positions directly.
	var node_pos: Vector2 = a.graph.world_of(Vector2i(2, 0))
	# A is 60px from the junction (heading east toward it).
	a.position_on_road = node_pos - Vector2(60, 0)
	a.heading = 0.0
	# B is 120px from the junction (heading south toward it).
	b.position_on_road = node_pos - Vector2(0, 120)
	b.heading = PI * 0.5
	ts.update([a, b], 1.0 / 60.0)
	# A is closer (60 < 120) -> B yields.
	assert_eq(b.mover._junction_yield_speed, 0.0, "further vehicle (B) should yield to stop")
	assert_eq(a.mover._junction_yield_speed, -1.0, "closer vehicle (A) should NOT yield")


func test_junction_closer_vehicle_proceeds() -> void:
	var ts := TrafficSystem.new()
	var result := _build_grid_and_vehicles()
	var a: VehicleController = result[1]
	var b: VehicleController = result[2]
	var node_pos: Vector2 = a.graph.world_of(Vector2i(2, 0))
	# B is closer (40px), A is further (150px) -> A yields.
	b.position_on_road = node_pos - Vector2(0, 40)
	b.heading = PI * 0.5
	a.position_on_road = node_pos - Vector2(150, 0)
	a.heading = 0.0
	ts.update([a, b], 1.0 / 60.0)
	assert_eq(a.mover._junction_yield_speed, 0.0, "further vehicle (A) should yield")
	assert_eq(b.mover._junction_yield_speed, -1.0, "closer vehicle (B) should NOT yield")


func test_junction_same_direction_no_conflict() -> void:
	var ts := TrafficSystem.new()
	var result := _build_grid_and_vehicles()
	var a: VehicleController = result[1]
	var b: VehicleController = result[2]
	var node_pos: Vector2 = a.graph.world_of(Vector2i(2, 0))
	# Both heading east (parallel) -> no junction conflict (ACC handles).
	a.position_on_road = node_pos - Vector2(60, 0)
	a.heading = 0.0
	b.position_on_road = node_pos - Vector2(120, 0)
	b.heading = 0.0
	# B's path also goes east (same direction as A).
	b.path = a.path.duplicate()
	ts.update([a, b], 1.0 / 60.0)
	assert_eq(
		a.mover._junction_yield_speed,
		-1.0,
		"same-direction traffic should not trigger junction yield"
	)
	assert_eq(
		b.mover._junction_yield_speed,
		-1.0,
		"same-direction traffic should not trigger junction yield"
	)


func test_junction_beyond_look_ahead_no_conflict() -> void:
	var ts := TrafficSystem.new()
	var result := _build_grid_and_vehicles()
	var a: VehicleController = result[1]
	var b: VehicleController = result[2]
	var node_pos: Vector2 = a.graph.world_of(Vector2i(2, 0))
	# A is 60px from junction, B is 300px (beyond CONFLICT_LOOK_AHEAD=200).
	a.position_on_road = node_pos - Vector2(60, 0)
	a.heading = 0.0
	b.position_on_road = node_pos - Vector2(0, 300)
	b.heading = PI * 0.5
	ts.update([a, b], 1.0 / 60.0)
	assert_eq(a.mover._junction_yield_speed, -1.0, "A within range, B beyond range -> no yield")
	assert_eq(b.mover._junction_yield_speed, -1.0, "B beyond look-ahead should not yield")


func test_junction_already_passed_no_conflict() -> void:
	var ts := TrafficSystem.new()
	var result := _build_grid_and_vehicles()
	var a: VehicleController = result[1]
	var b: VehicleController = result[2]
	var node_pos: Vector2 = a.graph.world_of(Vector2i(2, 0))
	# A is past the junction (heading east, node is behind it).
	a.position_on_road = node_pos + Vector2(50, 0)
	a.heading = 0.0
	b.position_on_road = node_pos - Vector2(0, 60)
	b.heading = PI * 0.5
	ts.update([a, b], 1.0 / 60.0)
	assert_eq(
		b.mover._junction_yield_speed, -1.0, "B should not yield when A already passed junction"
	)


func test_junction_tiebreaker_lower_index_proceeds() -> void:
	var ts := TrafficSystem.new()
	var result := _build_grid_and_vehicles()
	var a: VehicleController = result[1]
	var b: VehicleController = result[2]
	var node_pos: Vector2 = a.graph.world_of(Vector2i(2, 0))
	# Both 100px from the junction, perpendicular headings -> tie.
	a.position_on_road = node_pos - Vector2(100, 0)
	a.heading = 0.0
	b.position_on_road = node_pos - Vector2(0, 100)
	b.heading = PI * 0.5
	ts.update([a, b], 1.0 / 60.0)
	# a is index 0, b is index 1 -> b yields.
	assert_eq(b.mover._junction_yield_speed, 0.0, "tie: higher index (B) should yield")
	assert_eq(a.mover._junction_yield_speed, -1.0, "tie: lower index (A) should proceed")
