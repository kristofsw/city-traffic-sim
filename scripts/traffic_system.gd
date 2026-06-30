class_name TrafficSystem
extends RefCounted
## Adaptive Cruise Control (ACC) coordinator for multi-vehicle traffic.
##
## A pure RefCounted logic object (no SceneTree dependency) owned by
## SimulationManager. Each frame, BEFORE the vehicles' movers update, the
## manager calls `update(vehicles, delta)`. For every vehicle the system
## scans the others for the nearest one ahead within a forward cone and
## pushes a lead constraint onto that vehicle's mover via
## `set_lead_constraint(gap, lead_speed)`. The mover then caps its target
## speed to the ACC safe speed, so the car eases off to match the lead and
## maintain the gap. When no lead is found, the constraint is cleared.
##
## The "ahead" test uses the follower's heading to define a forward cone
## (~30° half-angle), so oncoming traffic in the other lane is ignored.
## The look-ahead distance scales with the follower's speed
## (current_speed * follow_time_gap + follow_min_gap) so faster cars look
## further ahead, plus a fixed detection margin. Each vehicle reads its own
## spec's ACC fields, so a sporty spec can tailgate while a cautious one
## keeps a wide berth.

## Half-angle of the forward cone (radians). ~30°; a vehicle is "ahead" if
## the angle between the follower's heading and the vector to it is below
## this. Keeps oncoming-lane traffic out of the lead detection.
const CONE_HALF_ANGLE := 0.52  # radians (~30°)
## Extra detection range beyond the speed-scaled look-ahead (px). Ensures a
## slow/stopped car still detects a lead a short distance ahead.
const DETECTION_MARGIN := 60.0
## Two vehicles whose headings differ by more than this are oncoming (opposite
## directions), not lead/follower. Excludes oncoming traffic from ACC so a
## wide bus in the other lane doesn't make following cars stop.
const _ONCOMING_HEADING_THRESHOLD := 2.4  # radians (~137°, half-pi + margin)
## How far ahead to look for junction conflicts (px). ~1.5 blocks. Both
## vehicles must be within this distance of a shared junction node for the
## conflict to trigger.
const CONFLICT_LOOK_AHEAD := 200.0
## Minimum heading difference (radians) for a junction conflict. Below this
## the vehicles are parallel (same direction) — ACC handles following. Above
## PI minus this they are oncoming (same road, different lanes) — the ACC
## oncoming filter handles that. Only cross-traffic (30°..150°) conflicts.
const CONFLICT_ANGLE_THRESHOLD := 0.52  # radians (~30°)


## Update all vehicles' ACC constraints and junction-yield constraints for
## this frame. Call BEFORE the vehicles' movers update (Godot processes the
## parent's _process before its children's, so SimulationManager._process
## runs before each VehicleController._process -- the constraints are set
## just in time).
func update(vehicles: Array[VehicleController], _delta: float) -> void:
	var n: int = vehicles.size()
	# Pass 1: Adaptive Cruise Control (forward following).
	for i in n:
		var follower: VehicleController = vehicles[i]
		if follower == null or follower.mover == null:
			continue
		var lead := _find_lead(follower, vehicles, i)
		if not lead.has("vehicle"):
			follower.mover.clear_lead_constraint()
		else:
			follower.mover.set_lead_constraint(lead["gap"], lead["speed"])
	# Pass 2: Junction conflict resolution (cross-traffic yielding).
	# Clear all junction yields first, then resolve pairwise conflicts.
	for i in n:
		var v: VehicleController = vehicles[i]
		if v != null and v.mover != null:
			v.mover.clear_junction_yield()
	_resolve_junction_conflicts(vehicles)


## Find the nearest vehicle ahead of `follower` within its forward cone.
## Returns a Dictionary {vehicle, gap, speed} or empty if no lead is found.
func _find_lead(
	follower: VehicleController, vehicles: Array[VehicleController], skip: int
) -> Dictionary:
	var f_pos: Vector2 = follower.position_on_road
	var f_heading: float = follower.heading
	var f_dir: Vector2 = Vector2.from_angle(f_heading)
	var f_speed: float = follower.current_speed
	var spec: VehicleSpec = follower.mover._ensure_spec()
	var look_ahead: float = f_speed * spec.follow_time_gap + spec.follow_min_gap + DETECTION_MARGIN
	var best_gap: float = INF
	var best_vehicle: VehicleController = null
	var best_speed: float = 0.0
	for j in vehicles.size():
		if j == skip:
			continue
		var other: VehicleController = vehicles[j]
		if other == null or other.mover == null:
			continue
		var offset: Vector2 = other.position_on_road - f_pos
		var dist: float = offset.length()
		if dist > look_ahead or dist < 0.001:
			continue
		# Forward cone: the angle between the follower's heading and the
		# vector to the other vehicle must be within CONE_HALF_ANGLE.
		var angle: float = abs(f_dir.angle_to(offset.normalized()))
		if angle > CONE_HALF_ANGLE:
			continue
		# Oncoming-traffic filter: if the other vehicle is heading toward
		# the follower (opposite direction, > 90° apart), it is not a
		# lead to follow -- it's oncoming traffic in the other lane. A wide
		# bus at close range falls inside the position cone but is never
		# something the follower should trail behind.
		var other_heading: float = other.heading
		var heading_delta: float = abs(angle_difference(f_heading, other_heading))
		if heading_delta > _ONCOMING_HEADING_THRESHOLD:
			continue
		# Convert center-to-center distance to bumper-to-bumper gap so the
		# ACC min-gap is measured edge-to-edge, not center-to-center.
		# Without this, two 36px cars 30px apart "gap" are actually
		# overlapping by 6px (rear-ending).
		var gap: float = (
			dist - spec.body_length * 0.5 - other.mover._ensure_spec().body_length * 0.5
		)
		if gap > look_ahead or gap < 0.001:
			continue
		if gap < best_gap:
			best_gap = gap
			best_vehicle = other
			best_speed = other.current_speed
	if best_vehicle == null:
		return {}
	return {"vehicle": best_vehicle, "gap": best_gap, "speed": best_speed}


## Resolve cross-traffic conflicts at shared junctions. For each pair of
## vehicles, find shared path nodes (junctions both will pass through). If
## both are approaching the same junction within CONFLICT_LOOK_AHEAD and
## their headings differ by more than CONFLICT_ANGLE_THRESHOLD (cross-traffic,
## not parallel), the vehicle further from the junction yields (stops). The
## closer vehicle proceeds. Tiebreaker: lower index proceeds (deterministic,
## prevents both stopping forever).
func _resolve_junction_conflicts(vehicles: Array[VehicleController]) -> void:
	var n: int = vehicles.size()
	for i in n:
		var a: VehicleController = vehicles[i]
		if a == null or a.mover == null or a.path.is_empty():
			continue
		for j in range(i + 1, n):
			var b: VehicleController = vehicles[j]
			if b == null or b.mover == null or b.path.is_empty():
				continue
			_resolve_pair(a, b, i, j)


## Resolve a single pair of vehicles. Finds their nearest shared junction
## node ahead and yields the further one if their headings differ enough.
func _resolve_pair(a: VehicleController, b: VehicleController, idx_a: int, idx_b: int) -> void:
	# Find shared path nodes (nodes that appear in both vehicles' paths).
	var shared := _shared_path_nodes(a.path, b.path)
	if shared.is_empty():
		return
	# For each shared node, check if both vehicles are approaching it.
	for node_key in shared:
		var node_pos: Vector2 = a.graph.world_of(node_key)
		var dist_a: float = a.position_on_road.distance_to(node_pos)
		var dist_b: float = b.position_on_road.distance_to(node_pos)
		# Both must be within look-ahead range of the junction.
		if dist_a > CONFLICT_LOOK_AHEAD or dist_b > CONFLICT_LOOK_AHEAD:
			continue
		# Both must still be approaching (not already passed). We check if
		# the node is still ahead in the vehicle's path.
		if not _is_ahead_in_path(a, node_key):
			continue
		if not _is_ahead_in_path(b, node_key):
			continue
		# Heading difference: only cross-traffic conflicts (not parallel
		# same-direction, not oncoming opposite-direction).
		var heading_delta: float = abs(angle_difference(a.heading, b.heading))
		if heading_delta < CONFLICT_ANGLE_THRESHOLD:
			continue  # parallel — ACC handles same-direction following
		if heading_delta > PI - CONFLICT_ANGLE_THRESHOLD:
			continue  # oncoming — ACC oncoming filter handles
		# Conflict detected: the further vehicle yields.
		if dist_a < dist_b:
			b.mover.set_junction_yield(0.0)
		elif dist_b < dist_a:
			a.mover.set_junction_yield(0.0)
		else:
			# Tie: lower index proceeds, higher index yields.
			if idx_a < idx_b:
				b.mover.set_junction_yield(0.0)
			else:
				a.mover.set_junction_yield(0.0)
		return  # one conflict per pair per frame is enough


## Return the set of Vector2i nodes that appear in both paths.
func _shared_path_nodes(path_a: Array[Vector2i], path_b: Array[Vector2i]) -> Array[Vector2i]:
	var set_b: Dictionary = {}
	for k in path_b:
		set_b[k] = true
	var shared: Array[Vector2i] = []
	for k in path_a:
		if set_b.has(k):
			shared.append(k)
	return shared


## True if `node_key` is still ahead of the vehicle in its path (hasn't
## passed it yet). A node is "ahead" if it appears in the path at or after
## the vehicle's current segment. We approximate by checking if the node
## world position is in front of the vehicle (positive projection onto
## the heading vector).
func _is_ahead_in_path(v: VehicleController, node_key: Vector2i) -> bool:
	# Simple forward check: the node is ahead if the vector from the vehicle
	# to the node has a positive projection onto the vehicle's heading.
	var node_pos: Vector2 = v.graph.world_of(node_key)
	var to_node: Vector2 = node_pos - v.position_on_road
	var f_dir: Vector2 = Vector2.from_angle(v.heading)
	return f_dir.dot(to_node) > 0.0
