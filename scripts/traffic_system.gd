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


## Update all vehicles' ACC constraints for this frame. Call BEFORE the
## vehicles' movers update (Godot processes the parent's _process before
## its children's, so SimulationManager._process runs before each
## VehicleController._process -- the constraints are set just in time).
func update(vehicles: Array[VehicleController], _delta: float) -> void:
	var n: int = vehicles.size()
	for i in n:
		var follower: VehicleController = vehicles[i]
		if follower == null or follower.mover == null:
			continue
		var lead := _find_lead(follower, vehicles, i)
		if not lead.has("vehicle"):
			follower.mover.clear_lead_constraint()
		else:
			follower.mover.set_lead_constraint(lead["gap"], lead["speed"])


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
		if dist < best_gap:
			best_gap = dist
			best_vehicle = other
			best_speed = other.current_speed
	if best_vehicle == null:
		return {}
	return {"vehicle": best_vehicle, "gap": best_gap, "speed": best_speed}
