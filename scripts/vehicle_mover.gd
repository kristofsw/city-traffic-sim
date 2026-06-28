class_name VehicleMover
extends RefCounted
## Pure motion model for a vehicle driving along a parametric Trajectory.
##
## Owns the trajectory, speed state, and the turn/end slowdown math. Has no
## rendering and no SceneTree dependency -- it is a RefCounted data/logic
## object per the Godot "node alternatives" best practice. A controller node
## calls `update(delta)` each frame and reads the resulting state (or
## subscribes to the signals) to drive visuals.
##
## Signals are the integration seam: anything (a renderer, a following car's
## adaptive cruise, a debug overlay) can observe speed, braking, turn
## intent and arrival without knowing *why* the speed changed. In
## particular `braking_changed` fires for any deceleration reason -- hard
## brake, coast-down before a turn/stop, or holding still at a zero target
## (e.g. a future traffic light) -- so brake lights are correct in every
## case without the renderer re-deriving the cause.

signal position_changed(pos: Vector2, heading: float)
signal speed_changed(speed: float)
signal braking_changed(intensity: float)
signal turn_indicator_changed(direction: int)
signal arrived

const INDICATOR_BLINK_PERIOD := 0.4  # seconds (0.2s on / 0.2s off)
const COAST_DOWN_GLOW := 0.25  # taillight intensity while coasting down
const HOLD_STILL_GLOW := 1.0  # taillight intensity while stopped at zero target

# Convenience accessors read/write through the spec so call sites stay short
# and the spec remains the single source of truth. Tests can set these
# directly (a default spec is created on first access).
var max_speed: float:
	get:
		return _ensure_spec().max_speed
	set(v):
		_ensure_spec().max_speed = v

var accel_rate: float:
	get:
		return _ensure_spec().accel_rate
	set(v):
		_ensure_spec().accel_rate = v

var decel_rate: float:
	get:
		return _ensure_spec().decel_rate
	set(v):
		_ensure_spec().decel_rate = v

var decel_distance: float:
	get:
		return _ensure_spec().decel_distance
	set(v):
		_ensure_spec().decel_distance = v

var turn_slowdown_factor: float:
	get:
		return _ensure_spec().turn_slowdown_factor
	set(v):
		_ensure_spec().turn_slowdown_factor = v

var min_turn_speed_ratio: float:
	get:
		return _ensure_spec().min_turn_speed_ratio
	set(v):
		_ensure_spec().min_turn_speed_ratio = v

var turn_look_ahead: float:
	get:
		return _ensure_spec().turn_look_ahead
	set(v):
		_ensure_spec().turn_look_ahead = v

var snap_distance: float:
	get:
		return _ensure_spec().snap_distance
	set(v):
		_ensure_spec().snap_distance = v

var snap_speed_threshold: float:
	get:
		return _ensure_spec().snap_speed_threshold
	set(v):
		_ensure_spec().snap_speed_threshold = v

# Motion spec (injected by the controller). When null, the mover falls back
# to a built-in default spec so the pure methods remain testable without a
# Node. Fields are read through the spec so a single VehicleSpec Resource is
# the source of truth for all tuning.
var spec: VehicleSpec = null

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var trajectory: Trajectory = null

# Live state (read by the renderer / tests).
var s: float = 0.0  # arc-length position along whole trajectory
var seg_index: int = 0  # cached current segment index (hint for trajectory)
var heading: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO
var current_speed: float = 0.0  # actual speed (px/s), rate-limited toward target

# Internal state.
var _blink_phase: float = 0.0  # accumulator for indicator blink cadence
var _eff_decel: float = 0.0  # clamped decel distance for current trip
var _arrived_emitted: bool = false
var _last_braking: float = 0.0
var _last_turn_dir: int = 0
var _default_spec: VehicleSpec = null  # lazily created for spec-less tests


## Apply a VehicleSpec to this mover. The controller calls this on _ready.
## Tests can also call it directly; if not called, a default spec is used.
func apply_spec(p_spec: VehicleSpec) -> void:
	spec = p_spec


## Lazy default spec so pure methods work without explicit injection.
func _ensure_spec() -> VehicleSpec:
	if spec != null:
		return spec
	if _default_spec == null:
		_default_spec = VehicleSpec.new()
	return _default_spec


func assign_path(new_path: Array[Vector2i], lane_offset: float, turn_radius: float) -> void:
	path = new_path
	trajectory = TrajectoryBuilder.build_trajectory(graph, new_path, lane_offset, turn_radius)
	s = 0.0
	seg_index = 0
	current_speed = 0.0
	_eff_decel = min(decel_distance, trajectory.total_length * 0.4)
	_arrived_emitted = false
	if not trajectory.is_empty():
		position_on_road = trajectory.position_at(0.0)
		heading = trajectory.tangent_at(0.0)
		position_changed.emit(position_on_road, heading)
	speed_changed.emit(current_speed)
	_recompute_braking()
	_recompute_turn_indicator()


## Advance the motion model by `delta` seconds. Called by the controller's
## _process. Emits signals for any state that changed this frame.
func update(delta: float) -> void:
	if trajectory == null or trajectory.is_empty():
		_emit_arrived_if_done()
		return

	if s >= trajectory.total_length:
		_emit_arrived_if_done()
		return

	var target: float = target_speed_at(s)

	# Rate-limited speed approach (no jerk). Track coast-down for the
	# braking signal: the frame the rate-limiter subtracts speed we are
	# "decelerating" even if speed ~= target (easing toward a lower target).
	var decelerating: bool = false
	if current_speed < target:
		current_speed = min(current_speed + accel_rate * delta, target)
	else:
		current_speed = max(current_speed - decel_rate * delta, target)
		decelerating = true

	s += current_speed * delta
	_blink_phase += delta

	# Snap-to-arrival safeguard: prevent hovering at near-zero speed.
	var dist_to_end: float = trajectory.total_length - s
	if (
		(dist_to_end <= snap_distance and current_speed < snap_speed_threshold)
		or (dist_to_end <= _eff_decel and current_speed < 5.0)
	):
		s = trajectory.total_length
	if s >= trajectory.total_length:
		s = trajectory.total_length

	# Position + heading from the SAME parametric eval (tangent invariant).
	seg_index = trajectory.segment_index_at(s, seg_index)
	var local_s: float = s - trajectory.seg_start_arc[seg_index]
	var seg: TrajectorySegment = trajectory.segments[seg_index]
	position_on_road = seg.position_at(local_s)
	heading = seg.tangent_at(local_s)

	position_changed.emit(position_on_road, heading)
	speed_changed.emit(current_speed)
	_recompute_braking(decelerating, target)
	_recompute_turn_indicator()

	_emit_arrived_if_done()


func is_busy() -> bool:
	return trajectory != null and not trajectory.is_empty() and s < trajectory.total_length


# ---------------------------------------------------------------------------
# Pure motion math (no side effects, unit-testable)
# ---------------------------------------------------------------------------


## Target speed at a given arc-length position. Combines the end ramp
## (S-curve decel to stop at destination, sampled only at the current
## position so the car never starts stopping early) with the windowed
## apex-based turn slowdown (look-ahead so it brakes BEFORE the turn and
## sustains the corner speed -- trapezoidal target, no W-shape).
func target_speed_at(s_pos: float) -> float:
	var end_factor: float = 1.0
	if _eff_decel > 0.001:
		end_factor = _smoothstep((trajectory.total_length - s_pos) / _eff_decel)
	var turn_factor: float = turn_factor_windowed(s_pos)
	return max_speed * end_factor * turn_factor


## Turn slowdown factor at a given arc-length position. 1.0 on straights
## and past the end; an apex-weighted reduction on bezier arcs (slowest at
## the midpoint). Pure for testability.
func turn_factor_at(s_pos: float) -> float:
	if trajectory == null or trajectory.is_empty():
		return 1.0
	var total_length: float = trajectory.total_length
	var s_clamped: float = clamp(s_pos, 0.0, total_length)
	var idx: int = trajectory.segment_index_at(s_clamped, seg_index)
	var seg: TrajectorySegment = trajectory.segments[idx]
	var turn_angle: float = seg.curvature_at(0.0)
	if turn_angle <= 0.001:
		return 1.0
	var progress: float = seg.progress_fraction(s_clamped - trajectory.seg_start_arc[idx])
	var apex_weight: float = 1.0 - abs(progress - 0.5) * 2.0
	var factor: float = 1.0 - turn_angle * turn_slowdown_factor * apex_weight
	return max(factor, min_turn_speed_ratio)


## Minimum turn factor over [s_pos, s_pos + turn_look_ahead]. Includes any
## bezier apex strictly inside the window so the profile is trapezoidal
## (single brake-in, flat floor, single accel-out) instead of W-shaped.
func turn_factor_windowed(s_pos: float) -> float:
	if trajectory == null or trajectory.is_empty():
		return 1.0
	var s_end: float = s_pos + turn_look_ahead
	var best: float = turn_factor_at(s_pos)
	best = min(best, turn_factor_at(s_end))
	for i in trajectory.segments.size():
		var seg: TrajectorySegment = trajectory.segments[i]
		if seg.curvature_at(0.0) <= 0.001:
			continue
		var apex_arc: float = trajectory.seg_start_arc[i] + seg.length / 2.0
		if apex_arc > s_pos and apex_arc < s_end:
			best = min(best, turn_factor_at(apex_arc))
	return best


func _smoothstep(t: float) -> float:
	t = clamp(t, 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)


## Braking intensity 0..1 from current state. Three regimes:
## - Hard brake (speed > target): proportional ramp up to 1.0.
## - Coast-down (decelerating this frame, speed <= target): gentle glow.
## - Hold-still (target ~0 and speed ~0): full bright (future traffic lights).
func braking_intensity(decelerating: bool, target: float) -> float:
	if trajectory == null or trajectory.is_empty() or s >= trajectory.total_length:
		return 0.0
	# Hold-still: stopped at a zero target (red light / stop sign / arrived).
	if target <= 0.001 and current_speed <= 0.001:
		return HOLD_STILL_GLOW
	if current_speed > target:
		return clamp((current_speed - target) / (max_speed * 0.2), 0.0, 1.0)
	if decelerating:
		return COAST_DOWN_GLOW
	return 0.0


## Signed direction of the next turn within the look-ahead window
## [s_pos, s_pos + turn_look_ahead]: -1 left, +1 right, 0 none. Pure.
func upcoming_turn_direction(s_pos: float) -> int:
	if trajectory == null or trajectory.is_empty():
		return 0
	var s_end: float = s_pos + turn_look_ahead
	for i in trajectory.segments.size():
		var seg: TrajectorySegment = trajectory.segments[i]
		var seg_start: float = trajectory.seg_start_arc[i]
		var seg_end: float = seg_start + seg.length
		if seg_end > s_pos and seg_start < s_end and seg.curvature_at(0.0) > 0.001:
			return seg.turn_direction()
	return 0


## True when the indicator blinker is in its ON phase (0.2s on / 0.2s off).
func indicator_on_phase() -> bool:
	return fmod(_blink_phase, INDICATOR_BLINK_PERIOD) < INDICATOR_BLINK_PERIOD * 0.5


# ---------------------------------------------------------------------------
# Signal recompute helpers (called from update)
# ---------------------------------------------------------------------------


func _recompute_braking(decelerating: bool = false, target: float = -1.0) -> void:
	if target < 0.0:
		if trajectory == null or trajectory.is_empty():
			target = 0.0
		else:
			target = target_speed_at(s)
	var intensity: float = braking_intensity(decelerating, target)
	if abs(intensity - _last_braking) > 0.001:
		_last_braking = intensity
		braking_changed.emit(intensity)


func _recompute_turn_indicator() -> void:
	var dir: int = upcoming_turn_direction(s)
	if dir != _last_turn_dir:
		_last_turn_dir = dir
		turn_indicator_changed.emit(dir)


func _emit_arrived_if_done() -> void:
	if _arrived_emitted:
		return
	if trajectory != null and not trajectory.is_empty() and s >= trajectory.total_length:
		_arrived_emitted = true
		arrived.emit()
