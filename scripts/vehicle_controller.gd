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
## middle of each arc, with a windowed look-ahead so it brakes BEFORE the
## turn and sustains the corner speed). Actual speed is rate-limited toward
## the target so changes are always smooth. Taillights brighten when braking.

signal arrived

const BODY_COLOR := Color(0.42, 0.45, 0.50, 1)  # #6b7280 muted gray-blue
const HEADLIGHT_COLOR := Color(1.0, 0.96, 0.84, 1)  # #fff4d6
const TAILLIGHT_COLOR := Color(1.0, 0.35, 0.30, 1)  # #ff5a4d
const INDICATOR_COLOR := Color(1.0, 0.6, 0.15, 1)  # #ff9926 amber turn signal
const INDICATOR_BLINK_PERIOD := 0.4  # seconds (0.2s on / 0.2s off)
const COAST_DOWN_GLOW := 0.25  # taillight intensity while coasting down

@export var max_speed: float = 80.0  # px/s (cruising speed)
@export var accel_rate: float = 90.0  # px/s^2 (acceleration, gentle)
@export var decel_rate: float = 130.0  # px/s^2 (braking, gentle)
@export var decel_distance: float = 120.0  # px before destination to start braking
@export var turn_slowdown_factor: float = 0.5  # speed reduction per radian of turn
@export var min_turn_speed_ratio: float = 0.25  # never slower than this fraction in a turn
@export var turn_look_ahead: float = 60.0  # px; look ahead for upcoming turns
@export var snap_distance: float = 5.0  # px; snap to arrival when this close
@export var snap_speed_threshold: float = 15.0  # px/s; below this, snap to arrival
@export var lane_offset: float = 12.0  # half-lane, right-hand drive
@export var turn_radius: float = 22.0  # px; pull-back before intersection for arc
@export var debug_lane: bool = false  # log signed lane offset each frame

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var segments: Array[TrajectorySegment] = []
var seg_start_arc: Array[float] = []  # cumulative arc length at start of each segment
var total_length: float = 0.0
var s: float = 0.0  # arc-length position along whole trajectory
var seg_index: int = 0  # cached current segment index
var heading: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO
var current_speed: float = 0.0  # actual speed (px/s), rate-limited toward target
var _decelerating: bool = false  # true the frame the rate-limiter subtracts speed
var _blink_phase: float = 0.0  # accumulator for indicator blink cadence
var _current_segment_key: Vector2i = Vector2i.ZERO
var _eff_decel: float = 0.0  # clamped decel distance for current trip


func _ready() -> void:
	position = position_on_road


func assign_path(new_path: Array[Vector2i]) -> void:
	path = new_path
	segments = TrajectoryBuilder.build(graph, new_path, lane_offset, turn_radius)
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
	# Track whether we are decelerating this frame so the taillights can glow
	# during coast-down (speed easing toward a lower target), not only during
	# hard braking (speed above target).
	if current_speed < target:
		current_speed = min(current_speed + accel_rate * delta, target)
		_decelerating = false
	else:
		current_speed = max(current_speed - decel_rate * delta, target)
		_decelerating = true

	# Advance arc length by actual speed * delta.
	s += current_speed * delta

	# Indicator blink phase accumulator (runs always so the cadence is stable).
	_blink_phase += delta

	# Snap-to-arrival safeguard: prevent hovering at near-zero speed.
	var dist_to_end_arc: float = total_length - s
	if (
		(dist_to_end_arc <= snap_distance and current_speed < snap_speed_threshold)
		or (dist_to_end_arc <= _eff_decel and current_speed < 5.0)
	):
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
		print(
			(
				"[Vehicle] v=%.1f target=%.1f lane_offset_signed=%.2f seg=%s pos=%s"
				% [
					current_speed,
					target,
					_signed_lane_offset(),
					_current_segment_key,
					position_on_road
				]
			)
		)

	queue_redraw()


## Target speed at a given arc-length position. Combines:
## - End ramp (decelerate to stop at destination, S-curve), sampled ONLY at
##   the current position so the car never starts stopping early.
## - Apex-based turn slowdown with LOOK-AHEAD: we sample the turn factor at
##   the current position AND at `turn_look_ahead` px ahead, then keep the
##   lower of the two. This makes the car slow BEFORE entering a turn
## - Apex-based turn slowdown with a WINDOWED look-ahead: we take the
##   minimum turn factor over the whole window [s, s + turn_look_ahead].
##   This makes the car slow BEFORE entering a turn (anticipative braking)
##   and sustain the corner speed while the apex is in the window, then
##   spool back up lazily via the gentle accel_rate. A two-point min would
##   miss an apex between the samples and produce a W-shaped target
##   (brake -> release -> brake, i.e. a double taillight flash); the
##   windowed min yields a trapezoid -- single sustained slowdown.
## The START acceleration is handled by accel_rate itself (the car ramps up
## from 0 naturally), so no start_factor is needed -- this avoids a
## chicken-and-egg trap where target=0 at s=0 prevents the car from ever
## moving. All factors are continuous (no jumps at segment boundaries).
func _target_speed_at(s_pos: float) -> float:
	# End ramp: smoothstep from 1 to 0 over _eff_decel px before destination.
	# Sampled ONLY at the current position (never at the look-ahead point),
	# otherwise the car would begin stopping turn_look_ahead px early.
	var end_factor: float = 1.0
	if _eff_decel > 0.001:
		end_factor = _smoothstep((total_length - s_pos) / _eff_decel)
	# Turn slowdown with a WINDOWED look-ahead: minimum of T over
	# [s_pos, s_pos + turn_look_ahead]. A trapezoidal target profile
	# (monotonic brake-in, flat floor, monotonic accel-out) -- no W-shape.
	var turn_factor: float = _turn_factor_windowed(s_pos)
	return max_speed * end_factor * turn_factor


## Turn slowdown factor at a given arc-length position. Returns 1.0 on
## straight segments, and an apex-weighted reduction on bezier arcs (slowest
## in the middle of the arc). Past the end of the trajectory returns 1.0
## (no turn there), which keeps the look-ahead from spuriously lowering the
## target near the destination. Pure and side-effect free for testability.
func _turn_factor_at(s_pos: float) -> float:
	if segments.is_empty():
		return 1.0
	# Clamp to trajectory bounds: past the end there is no turn.
	var s_clamped: float = clamp(s_pos, 0.0, total_length)
	# Locate the segment containing s_clamped (linear scan; small arrays).
	var idx: int = 0
	while idx < segments.size() - 1 and s_clamped >= seg_start_arc[idx] + segments[idx].length:
		idx += 1
	var seg: TrajectorySegment = segments[idx]
	var turn_angle: float = seg.curvature_at(0.0)
	if turn_angle <= 0.001:
		return 1.0
	var progress: float = seg.progress_fraction(s_clamped - seg_start_arc[idx])
	# Triangle weight: 0 at entry, 1 at apex (progress=0.5), 0 at exit.
	var apex_weight: float = 1.0 - abs(progress - 0.5) * 2.0
	var factor: float = 1.0 - turn_angle * turn_slowdown_factor * apex_weight
	return max(factor, min_turn_speed_ratio)


## Minimum turn factor over the window [s_pos, s_pos + turn_look_ahead].
## For the piecewise-linear triangle T, the minimum over an interval is the
## minimum of: the two endpoint samples, plus the triangle apex value for
## any apex (bezier midpoint) lying strictly inside the interval. This
## produces a trapezoidal target profile (monotonic brake-in, flat floor at
## the apex factor, monotonic accel-out) instead of the W-shape a two-point
## min would give -- so the car brakes once and sustains the corner speed.
func _turn_factor_windowed(s_pos: float) -> float:
	var s_end: float = s_pos + turn_look_ahead
	var best: float = _turn_factor_at(s_pos)
	best = min(best, _turn_factor_at(s_end))
	# Any bezier apex (segment midpoint) strictly inside (s_pos, s_end)
	# contributes its apex factor -- the floor the corner enforces.
	for i in segments.size():
		var seg: TrajectorySegment = segments[i]
		if seg.curvature_at(0.0) <= 0.001:
			continue
		var apex_arc: float = seg_start_arc[i] + seg.length / 2.0
		if apex_arc > s_pos and apex_arc < s_end:
			best = min(best, _turn_factor_at(apex_arc))
	return best


## Smoothstep: S-curve interpolation. Zero derivative at t=0 and t=1
## (gentle start/end, no kinks). t is clamped to [0, 1].
func _smoothstep(t: float) -> float:
	t = clamp(t, 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)


func _advance_segment_index() -> void:
	# Walk forward from the cached index until we find the segment containing s.
	while (
		seg_index < segments.size() - 1
		and s >= seg_start_arc[seg_index] + segments[seg_index].length
	):
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


## Braking intensity for taillight visual (0 = coasting/free, 1 = hard brake).
## Two regimes:
## - Hard braking (speed > target): proportional ramp up to 1.0, scaled by
##   max_speed so a modest overshoot fully lights the brakes.
## - Coast-down (speed <= target but decelerating this frame): a gentle glow
##   (COAST_DOWN_GLOW) so the taillights visibly lift whenever the car eases
##   off toward a lower target -- e.g. before a turn or a stop -- not only
##   when actively braking hard.
func _braking_intensity() -> float:
	if segments.is_empty() or s >= total_length:
		return 0.0
	var target: float = _target_speed_at(s)
	if current_speed > target:
		return clamp((current_speed - target) / (max_speed * 0.2), 0.0, 1.0)
	if _decelerating:
		return COAST_DOWN_GLOW
	return 0.0


## Signed direction of the next turn within the look-ahead window
## [s_pos, s_pos + turn_look_ahead]: -1 = left, +1 = right, 0 = none.
## Uses the same window as the speed model so indicators fire the moment the
## car begins decelerating toward an upcoming turn, and cancel once the turn
## leaves the window. Returns the direction of the FIRST turn in the window
## (the next one the car will take). Pure, side-effect free for testability.
func _upcoming_turn_direction(s_pos: float) -> int:
	if segments.is_empty():
		return 0
	var s_end: float = s_pos + turn_look_ahead
	for i in segments.size():
		var seg: TrajectorySegment = segments[i]
		var seg_start: float = seg_start_arc[i]
		var seg_end: float = seg_start + seg.length
		# Segment overlaps the window and is a real turn.
		if seg_end > s_pos and seg_start < s_end and seg.curvature_at(0.0) > 0.001:
			return seg.turn_direction()
	return 0


## True when the indicator blinker is in its ON phase (0.2s on / 0.2s off).
func _indicator_on_phase() -> bool:
	return fmod(_blink_phase, INDICATOR_BLINK_PERIOD) < INDICATOR_BLINK_PERIOD * 0.5


func _draw() -> void:
	var length: float = 36.0
	var width: float = 18.0
	var cos_h: float = cos(heading)
	var sin_h: float = sin(heading)
	var corners := PackedVector2Array(
		[
			Vector2(-length * 0.5, -width * 0.5),
			Vector2(length * 0.5, -width * 0.5),
			Vector2(length * 0.5, width * 0.5),
			Vector2(-length * 0.5, width * 0.5),
		]
	)
	var rotated := PackedVector2Array()
	for c in corners:
		rotated.append(Vector2(c.x * cos_h - c.y * sin_h, c.x * sin_h + c.y * cos_h))
	draw_colored_polygon(rotated, BODY_COLOR)
	# Headlights at front edge.
	var front_local := Vector2(length * 0.5, 0.0)
	var front_world := Vector2(
		front_local.x * cos_h - front_local.y * sin_h, front_local.x * sin_h + front_local.y * cos_h
	)
	draw_circle(front_world + Vector2(0, -width * 0.3).rotated(heading), 2.2, HEADLIGHT_COLOR)
	draw_circle(front_world + Vector2(0, width * 0.3).rotated(heading), 2.2, HEADLIGHT_COLOR)
	# Taillights at rear edge -- brighten when braking.
	var braking: float = _braking_intensity()
	var brake_radius: float = 1.8 + braking * 1.5
	# Low baseline alpha so braking (up to 1.0) is clearly visible against it.
	var taillight_alpha: float = 0.35 + braking * 0.65
	var taillight := Color(TAILLIGHT_COLOR.r, TAILLIGHT_COLOR.g, TAILLIGHT_COLOR.b, taillight_alpha)
	var rear_local := Vector2(-length * 0.5, 0.0)
	var rear_world := Vector2(
		rear_local.x * cos_h - rear_local.y * sin_h, rear_local.x * sin_h + rear_local.y * cos_h
	)
	draw_circle(rear_world + Vector2(0, -width * 0.3).rotated(heading), brake_radius, taillight)
	draw_circle(rear_world + Vector2(0, width * 0.3).rotated(heading), brake_radius, taillight)
	# Turn indicators: amber blinkers at the corners on the turning side.
	# Fire when a turn is within the look-ahead window (i.e. while the car is
	# already decelerating toward it) and blink 0.2s on / 0.2s off. Right turn
	# -> right-side corners (front-right + rear-right), left -> left-side.
	var turn_dir: int = _upcoming_turn_direction(s)
	if turn_dir != 0 and _indicator_on_phase():
		var side: float = float(turn_dir) * width * 0.5  # +1 right, -1 left
		var front_corner := Vector2(length * 0.5, side)
		var rear_corner := Vector2(-length * 0.5, side)
		var front_corner_world := Vector2(
			front_corner.x * cos_h - front_corner.y * sin_h,
			front_corner.x * sin_h + front_corner.y * cos_h
		)
		var rear_corner_world := Vector2(
			rear_corner.x * cos_h - rear_corner.y * sin_h,
			rear_corner.x * sin_h + rear_corner.y * cos_h
		)
		draw_circle(front_corner_world, 2.5, INDICATOR_COLOR)
		draw_circle(rear_corner_world, 2.5, INDICATOR_COLOR)


func is_busy() -> bool:
	return s < total_length
