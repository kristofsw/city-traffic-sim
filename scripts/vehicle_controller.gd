class_name VehicleController
extends Node2D
## Drives a vehicle along a parametric trajectory of LineSeg and BezierSeg
## segments. Position and heading are both derived from the SAME arc-length
## parameter each frame, so the car body is always exactly tangent to the
## curve -- no skating, no choppiness, no speed-dependent artifacts.
##
## Speed is not constant: the car accelerates from standstill at the start
## of each trip (S-curve ramp), decelerates to a stop at the destination,
## and slows proportionally to turn sharpness (apex-based: slowest in the
## middle of each arc). Actual speed is rate-limited toward the target so
## changes are always smooth. Taillights brighten when braking.

signal arrived

const BODY_COLOR := Color(0.42, 0.45, 0.50, 1)        # #6b7280 muted gray-blue
const HEADLIGHT_COLOR := Color(1.0, 0.96, 0.84, 1)    # #fff4d6
const TAILLIGHT_COLOR := Color(1.0, 0.35, 0.30, 1)    # #ff5a4d

@export var max_speed: float = 120.0                 # px/s (cruising speed)
@export var accel_rate: float = 180.0                # px/s^2 (acceleration)
@export var decel_rate: float = 300.0                # px/s^2 (braking, stronger than accel)
@export var accel_distance: float = 80.0             # px over which to ramp up at start
@export var decel_distance: float = 60.0             # px before destination to start braking
@export var turn_slowdown_factor: float = 0.3        # speed reduction per radian of turn
@export var min_turn_speed_ratio: float = 0.35       # never slower than this fraction in a turn
@export var snap_distance: float = 5.0               # px; snap to arrival when this close
@export var snap_speed_threshold: float = 15.0       # px/s; below this, snap to arrival
@export var lane_offset: float = 12.0                # half-lane, right-hand drive
@export var turn_radius: float = 22.0                # px; pull-back before intersection for arc
@export var debug_lane: bool = false                 # log signed lane offset each frame

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var segments: Array[TrajectorySegment] = []
var seg_start_arc: Array[float] = []        # cumulative arc length at start of each segment
var total_length: float = 0.0
var s: float = 0.0                          # arc-length position along whole trajectory
var seg_index: int = 0                      # cached current segment index
var heading: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO
var current_speed: float = 0.0              # actual speed (px/s), rate-limited toward target
var _current_segment_key: Vector2i = Vector2i.ZERO
var _eff_accel: float = 0.0                 # clamped accel distance for current trip
var _eff_decel: float = 0.0                 # clamped decel distance for current trip

func _ready() -> void:
	position = position_on_road

func assign_path(new_path: Array[Vector2i]) -> void:
	path = new_path
	segments = _build_segments(new_path)
	seg_start_arc.clear()
	var cum: float = 0.0
	for seg in segments:
		seg_start_arc.append(cum)
		cum += seg.length
	total_length = cum
	s = 0.0
	seg_index = 0
	current_speed = 0.0
	# Clamp decel distance for very short trips so the end ramp fits.
	_eff_decel = min(decel_distance, total_length * 0.4)
	if segments.size() > 0:
		position_on_road = segments[0].position_at(0.0)
		heading = segments[0].tangent_at(0.0)
		position = position_on_road
	if path.size() >= 1:
		_current_segment_key = path[0]

func _build_segments(p: Array[Vector2i]) -> Array[TrajectorySegment]:
	var out: Array[TrajectorySegment] = []
	if p.size() < 2:
		return out

	# Per-segment data: direction, right-hand perpendicular, entry/exit offsets.
	var dirs: Array[Vector2] = []
	var entries: Array[Vector2] = []
	var exits: Array[Vector2] = []
	for i in range(p.size() - 1):
		var a: Vector2 = graph.world_of(p[i])
		var b: Vector2 = graph.world_of(p[i + 1])
		var d: Vector2 = (b - a).normalized()
		var perp: Vector2 = Vector2(-d.y, d.x)  # right-hand perpendicular (y-down)
		dirs.append(d)
		entries.append(a + perp * lane_offset)
		exits.append(b + perp * lane_offset)

	# Build a continuous trajectory: alternating LineSeg (straights) and
	# BezierSeg (turn arcs) at intersections. All points are offset to the
	# right-hand lane, so the car never enters the oncoming lane.
	var current_pos: Vector2 = entries[0]

	for i in range(p.size() - 1):
		var is_last: bool = (i == p.size() - 2)
		var has_turn: bool = false
		if not is_last:
			has_turn = dirs[i].dot(dirs[i + 1]) < 0.99

		if is_last or not has_turn:
			# Straight through to the exit of this segment.
			if current_pos.distance_to(exits[i]) > 0.5:
				out.append(LineSeg.new(current_pos, exits[i]))
			current_pos = exits[i]
		else:
			# Turn: straight to approach point, then bezier arc to leave point.
			var seg_len: float = entries[i].distance_to(exits[i])
			var next_seg_len: float = entries[i + 1].distance_to(exits[i + 1])
			var tr: float = min(turn_radius, seg_len * 0.4, next_seg_len * 0.4)
			tr = max(tr, 2.0)

			var approach: Vector2 = exits[i] - dirs[i] * tr
			var leave: Vector2 = entries[i + 1] + dirs[i + 1] * tr

			# Straight portion: current_pos -> approach
			if current_pos.distance_to(approach) > 0.5:
				out.append(LineSeg.new(current_pos, approach))

			# Bezier control point: intersection of the two offset tangent lines.
			var cross: float = dirs[i].x * dirs[i + 1].y - dirs[i].y * dirs[i + 1].x
			if abs(cross) < 0.001:
				out.append(LineSeg.new(approach, leave))
			else:
				var delta: Vector2 = leave - approach
				var t_ctrl: float = (delta.x * dirs[i + 1].y - delta.y * dirs[i + 1].x) / cross
				var control: Vector2 = approach + dirs[i] * t_ctrl
				out.append(BezierSeg.new(approach, control, leave))

			current_pos = leave
	return out

func _process(delta: float) -> void:
	if graph == null or segments.is_empty() or s >= total_length:
		# Arrival check. Guard BEFORE emit so the handler's assign_path
		# (which resets s=0 and segments) is not overwritten.
		if segments.size() > 0 and s >= total_length:
			s = total_length + 1.0  # guard against re-emit
			arrived.emit()
		return

	# Compute target speed at current position.
	var target: float = _target_speed_at(s)

	# Smoothly approach target speed (rate-limited -> no jerk).
	if current_speed < target:
		current_speed = min(current_speed + accel_rate * delta, target)
	else:
		current_speed = max(current_speed - decel_rate * delta, target)

	# Advance arc length by actual speed * delta.
	s += current_speed * delta

	# Snap-to-arrival safeguard: prevent hovering at near-zero speed.
	var dist_to_end_arc: float = total_length - s
	if (dist_to_end_arc <= snap_distance and current_speed < snap_speed_threshold) \
			or (dist_to_end_arc <= _eff_decel and current_speed < 5.0):
		s = total_length

	if s >= total_length:
		s = total_length

	# Find the segment containing the current arc length.
	_advance_segment_index()
	var local_s: float = s - seg_start_arc[seg_index]
	var seg: TrajectorySegment = segments[seg_index]
	# Position and heading both come from the same parametric evaluation.
	position_on_road = seg.position_at(local_s)
	heading = seg.tangent_at(local_s)
	position = position_on_road

	_current_segment_key = _estimate_current_segment()
	if debug_lane:
		print("[Vehicle] v=%.1f target=%.1f lane_offset_signed=%.2f seg=%s pos=%s" % [current_speed, target, _signed_lane_offset(), _current_segment_key, position_on_road])

	queue_redraw()

## Target speed at a given arc-length position. Combines:
## - End ramp (decelerate to stop at destination, S-curve)
## - Apex-based turn slowdown (slowest in the middle of each bezier arc)
## The START acceleration is handled by the accel_rate itself (the car
## ramps up from 0 naturally), so no start_factor is needed -- this avoids
## a chicken-and-egg trap where target=0 at s=0 prevents the car from
## ever moving. All factors are continuous (no jumps at segment boundaries).
func _target_speed_at(s_pos: float) -> float:
	# End ramp: smoothstep from 1 to 0 over _eff_decel px before destination.
	var end_factor: float = 1.0
	if _eff_decel > 0.001:
		end_factor = _smoothstep((total_length - s_pos) / _eff_decel)
	# Turn slowdown: apex-based (slowest in the middle of the arc).
	var turn_factor: float = 1.0
	if seg_index < segments.size():
		var seg: TrajectorySegment = segments[seg_index]
		var turn_angle: float = seg.curvature_at(0.0)
		if turn_angle > 0.001:
			var progress: float = seg.progress_fraction(s_pos - seg_start_arc[seg_index])
			# Triangle weight: 0 at entry, 1 at apex (progress=0.5), 0 at exit.
			var apex_weight: float = 1.0 - abs(progress - 0.5) * 2.0
			turn_factor = 1.0 - turn_angle * turn_slowdown_factor * apex_weight
			turn_factor = max(turn_factor, min_turn_speed_ratio)
	return max_speed * end_factor * turn_factor

## Smoothstep: S-curve interpolation. Zero derivative at t=0 and t=1
## (gentle start/end, no kinks). t is clamped to [0, 1].
func _smoothstep(t: float) -> float:
	t = clamp(t, 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)

func _advance_segment_index() -> void:
	# Walk forward from the cached index until we find the segment containing s.
	while seg_index < segments.size() - 1 and s >= seg_start_arc[seg_index] + segments[seg_index].length:
		seg_index += 1

func _estimate_current_segment() -> Vector2i:
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
	if path.size() < 2:
		return 0.0
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
	var perp: Vector2 = Vector2(-d.y, d.x)
	return (position_on_road - a).dot(perp)

## Braking intensity for taillight visual (0 = not braking, 1 = hard braking).
func _braking_intensity() -> float:
	if segments.is_empty() or s >= total_length:
		return 0.0
	var target: float = _target_speed_at(s)
	if current_speed <= target:
		return 0.0
	return clamp((current_speed - target) / (decel_rate * 0.1), 0.0, 1.0)

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
	# Taillights at rear edge -- brighten when braking.
	var braking: float = _braking_intensity()
	var brake_radius: float = 1.8 + braking * 1.5
	var taillight_alpha: float = 0.6 + braking * 0.4
	var taillight := Color(TAILLIGHT_COLOR.r, TAILLIGHT_COLOR.g, TAILLIGHT_COLOR.b, taillight_alpha)
	var rear_local := Vector2(-length * 0.5, 0.0)
	var rear_world := Vector2(rear_local.x * cos_h - rear_local.y * sin_h, rear_local.x * sin_h + rear_local.y * cos_h)
	draw_circle(rear_world + Vector2(0, -width * 0.3).rotated(heading), brake_radius, taillight)
	draw_circle(rear_world + Vector2(0,  width * 0.3).rotated(heading), brake_radius, taillight)

func _direction_to(a: Vector2i, b: Vector2i) -> Vector2:
	return (graph.world_of(b) - graph.world_of(a)).normalized()

func is_busy() -> bool:
	return s < total_length