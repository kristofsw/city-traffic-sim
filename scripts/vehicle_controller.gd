class_name VehicleController
extends Node2D
## Drives a vehicle along a path of grid intersections. The trajectory is
## precomputed as offset waypoints on the RIGHT-hand side of each segment,
## connected by quadratic bezier arcs at intersections so the car rounds
## corners on the right side (Belgium-style right-hand traffic) and never
## swerves into the oncoming lane.

signal arrived

const BODY_COLOR := Color(0.42, 0.45, 0.50, 1)        # #6b7280 muted gray-blue
const HEADLIGHT_COLOR := Color(1.0, 0.96, 0.84, 1)    # #fff4d6
const TAILLIGHT_COLOR := Color(1.0, 0.35, 0.30, 1)    # #ff5a4d

@export var speed: float = 120.0            # px / sec
@export var rotation_speed: float = 8.0     # rad / sec (lerp factor for heading)
@export var lane_offset: float = 12.0       # half-lane, right-hand drive
@export var arrival_radius: float = 3.0     # px tolerance for "reached waypoint"
@export var turn_radius: float = 18.0       # px; pull-back before intersection for arc
@export var bezier_samples: int = 14        # samples per turn arc
@export var debug_lane: bool = false        # log signed lane offset each frame

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var trajectory: Array[Vector2] = []         # world-space points to drive through
var traj_index: int = 0
var heading: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO
var _current_segment_key: Vector2i = Vector2i.ZERO  # for lane-invariant logging

func _ready() -> void:
	position = position_on_road

func assign_path(new_path: Array[Vector2i]) -> void:
	path = new_path
	traj_index = 0
	trajectory = _build_trajectory(new_path)
	if path.size() >= 1:
		position_on_road = graph.world_of(path[0])
		position = position_on_road
		_current_segment_key = path[0]
	if trajectory.size() >= 2:
		heading = (trajectory[1] - trajectory[0]).angle()
	elif path.size() >= 2:
		heading = _direction_to(path[0], path[1]).angle()

func _build_trajectory(p: Array[Vector2i]) -> Array[Vector2]:
	var out: Array[Vector2] = []
	if p.size() < 2:
		if p.size() == 1:
			out.append(graph.world_of(p[0]))
		return out

	# Per-segment entry/exit on the right-hand side.
	var entries: Array[Vector2] = []
	var exits: Array[Vector2] = []
	var perps: Array[Vector2] = []
	for i in range(p.size() - 1):
		var a: Vector2 = graph.world_of(p[i])
		var b: Vector2 = graph.world_of(p[i + 1])
		var d: Vector2 = (b - a).normalized()
		var perp: Vector2 = Vector2(d.y, -d.x)  # right-hand perpendicular (y-down)
		perps.append(perp)
		entries.append(a + perp * lane_offset)
		exits.append(b + perp * lane_offset)

	# Assemble: straight(entry -> approach) + bezier(approach -> leave) + straight(leave -> exit)
	for i in range(p.size() - 1):
		var entry: Vector2 = entries[i]
		var exit: Vector2 = exits[i]
		var seg_len: float = entry.distance_to(exit)

		if i < p.size() - 2:
			# Straight portion up to 'approach' (pulled back by turn_radius from exit).
			var approach: Vector2 = exit - (exit - entry).normalized() * turn_radius
			if entry.distance_to(approach) > arrival_radius:
				out.append(entry)
				if approach.distance_to(entry) > turn_radius:
					out.append(approach)
				else:
					out.append(exit)  # segment too short; just go to exit
			else:
				out.append(entry)
			# Bezier from approach to next segment's 'leave' point.
			var next_perp: Vector2 = perps[i + 1]
			var next_entry: Vector2 = entries[i + 1]
			var leave: Vector2 = next_entry + (next_entry - exits[i + 1]).normalized() * turn_radius
			# Control point: the right-hand corner of the intersection = intersection
			# center offset by perp of the CURRENT segment (keeps arc on the right
			# side as the car enters the turn) -- this yields a smooth tangent handoff.
			var center: Vector2 = graph.world_of(p[i + 1])
			var control: Vector2 = center + perps[i] * lane_offset
			# If the next segment is straight-on (same direction), skip the arc.
			var cur_dir: Vector2 = (graph.world_of(p[i + 1]) - graph.world_of(p[i])).normalized()
			var nxt_dir: Vector2 = (graph.world_of(p[i + 2]) - graph.world_of(p[i + 1])).normalized()
			if cur_dir.dot(nxt_dir) > 0.99:
				# No turn; just continue straight through.
				out.append(exit)
			else:
				# Sample the quadratic bezier: B(t) = (1-t)^2 P0 + 2(1-t)t C + t^2 P1
				var p0: Vector2 = approach if approach.distance_to(entry) > turn_radius else exit
				var p1: Vector2 = leave
				for s in range(1, bezier_samples + 1):
					var t: float = float(s) / float(bezier_samples)
					var pt: Vector2 = p0.lerp(control, t).lerp(control.lerp(p1, t), t)
					out.append(pt)
		else:
			# Last segment: straight to the destination exit point.
			out.append(entry)
			out.append(exit)
	return out

func _process(delta: float) -> void:
	if graph == null or trajectory.size() < 2 or traj_index >= trajectory.size() - 1:
		# Check arrival.
		if trajectory.size() > 0 and traj_index >= trajectory.size() - 1:
			arrived.emit()
			traj_index = trajectory.size()  # guard against re-emit
		return

	var target: Vector2 = trajectory[traj_index + 1]
	var to_target: Vector2 = target - position_on_road
	var dist: float = to_target.length()

	# Update current segment key for lane-invariant logging (find which segment
	# the car is currently on based on closest entry/exit pair).
	_current_segment_key = _estimate_current_segment()

	if debug_lane:
		print("[Vehicle] lane_offset_signed=%.2f seg=%s pos=%s" % [_signed_lane_offset(), _current_segment_key, position_on_road])

	if dist <= arrival_radius:
		traj_index += 1
		return

	var step: float = min(speed * delta, dist)
	position_on_road += to_target.normalized() * step
	position = position_on_road
	# Smoothly rotate heading toward travel direction.
	var desired: float = to_target.angle()
	heading = lerp_angle(heading, desired, clamp(rotation_speed * delta, 0.0, 1.0))
	queue_redraw()

func _estimate_current_segment() -> Vector2i:
	# Map current position back to the path segment whose centerline is closest.
	var best: Vector2i = path[0]
	var best_d: float = INF
	for i in range(path.size() - 1):
		var a: Vector2 = graph.world_of(path[i])
		var b: Vector2 = graph.world_of(path[i + 1])
		var d: float = _dist_point_to_segment(position_on_road, a, b)
		if d < best_d:
			best_d = d
			best = path[i]
	return best

func _dist_point_to_segment(p: Vector2, a: Vector2, b: Vector2) -> float:
	var ab: Vector2 = b - a
	var t: float = clamp((p - a).dot(ab) / ab.length_squared(), 0.0, 1.0)
	var proj: Vector2 = a + ab * t
	return p.distance_to(proj)

func _signed_lane_offset() -> float:
	# Signed perpendicular distance from current segment centerline.
	# Positive = right-hand side (correct). Negative = oncoming lane (bug).
	if path.size() < 2:
		return 0.0
	# Find segment nearest to current position.
	var best_i: int = 0
	var best_d: float = INF
	for i in range(path.size() - 1):
		var a: Vector2 = graph.world_of(path[i])
		var b: Vector2 = graph.world_of(path[i + 1])
		var d: float = _dist_point_to_segment(position_on_road, a, b)
		if d < best_d:
			best_d = d
			best_i = i
	var a: Vector2 = graph.world_of(path[best_i])
	var b: Vector2 = graph.world_of(path[best_i + 1])
	var d: Vector2 = (b - a).normalized()
	var perp: Vector2 = Vector2(d.y, -d.x)  # right-hand
	return (position_on_road - a).dot(perp)

func _draw() -> void:
	var length: float = 36.0
	var width: float = 18.0
	var cos_h: float = cos(heading)
	var sin_h: float = sin(heading)
	var corners := PackedVector2Array([
		Vector2(-length * 0.5, -width * 0.5),
		Vector2( length * 0.5, -width * 0.5),
		Vector2( length * 0.5,  width * 0.5),
		Vector2(-length * 0.5,  width * 0.5),
	])
	var rotated := PackedVector2Array()
	for c in corners:
		rotated.append(Vector2(c.x * cos_h - c.y * sin_h, c.x * sin_h + c.y * cos_h))
	draw_colored_polygon(rotated, BODY_COLOR)
	# Headlights at front edge.
	var front_local := Vector2(length * 0.5, 0.0)
	var front_world := Vector2(front_local.x * cos_h - front_local.y * sin_h, front_local.x * sin_h + front_local.y * cos_h)
	draw_circle(front_world + Vector2(0, -width * 0.3).rotated(heading), 2.2, HEADLIGHT_COLOR)
	draw_circle(front_world + Vector2(0,  width * 0.3).rotated(heading), 2.2, HEADLIGHT_COLOR)
	# Taillights at rear edge.
	var rear_local := Vector2(-length * 0.5, 0.0)
	var rear_world := Vector2(rear_local.x * cos_h - rear_local.y * sin_h, rear_local.x * sin_h + rear_local.y * cos_h)
	draw_circle(rear_world + Vector2(0, -width * 0.3).rotated(heading), 1.8, TAILLIGHT_COLOR)
	draw_circle(rear_world + Vector2(0,  width * 0.3).rotated(heading), 1.8, TAILLIGHT_COLOR)

func _direction_to(a: Vector2i, b: Vector2i) -> Vector2:
	return (graph.world_of(b) - graph.world_of(a)).normalized()

func is_busy() -> bool:
	return traj_index < trajectory.size() - 1