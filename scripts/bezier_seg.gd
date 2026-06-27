class_name BezierSeg
extends TrajectorySegment
## A quadratic bezier arc segment. Position and tangent are evaluated
## analytically from the bezier formula, parameterized by arc length via
## a lookup table built at construction time. This gives the car perfectly
## smooth, speed-independent heading changes through turns -- both position
## and heading come from the same parametric evaluation, so the car body
## is always exactly tangent to the curve it travels.

const LUT_SIZE: int = 64

var p0: Vector2
var control: Vector2
var p1: Vector2
var total_turn_angle: float = 0.0  # cached: total heading change across the arc (rad)

# Arc-length lookup table: cumulative_arc[i] = arc length at t = i/LUT_SIZE.
var _cumulative_arc: Array[float] = []
var _t_values: Array[float] = []

func _init(p_p0: Vector2, p_control: Vector2, p_p1: Vector2) -> void:
	p0 = p_p0
	control = p_control
	p1 = p_p1
	_build_arc_length_lut()
	# Cache the total turn angle for apex-based slowdown.
	var t0: float = _eval_derivative(0.0).angle()
	var t1: float = _eval_derivative(1.0).angle()
	total_turn_angle = abs(_angle_diff(t0, t1))

func _build_arc_length_lut() -> void:
	_cumulative_arc.clear()
	_t_values.clear()
	var prev: Vector2 = _eval_bezier(0.0)
	var cum: float = 0.0
	_cumulative_arc.append(0.0)
	_t_values.append(0.0)
	for i in range(1, LUT_SIZE + 1):
		var t: float = float(i) / float(LUT_SIZE)
		var pt: Vector2 = _eval_bezier(t)
		cum += prev.distance_to(pt)
		_cumulative_arc.append(cum)
		_t_values.append(t)
		prev = pt
	length = cum

func _eval_bezier(t: float) -> Vector2:
	var u: float = 1.0 - t
	return u * u * p0 + 2.0 * u * t * control + t * t * p1

func _eval_derivative(t: float) -> Vector2:
	var u: float = 1.0 - t
	return 2.0 * u * (control - p0) + 2.0 * t * (p1 - control)

## Given an arc-length position, binary-search the LUT for the corresponding
## bezier parameter t, then evaluate position and tangent analytically.
func _t_for_arc(s_local: float) -> float:
	s_local = clamp(s_local, 0.0, length)
	# Binary search in _cumulative_arc.
	var lo: int = 0
	var hi: int = _cumulative_arc.size() - 1
	while lo < hi - 1:
		var mid: int = (lo + hi) / 2
		if _cumulative_arc[mid] < s_local:
			lo = mid
		else:
			hi = mid
	# Linear interpolation between lo and hi for sub-sample precision.
	var s_lo: float = _cumulative_arc[lo]
	var s_hi: float = _cumulative_arc[hi]
	var frac: float = 0.0
	if s_hi - s_lo > 0.001:
		frac = (s_local - s_lo) / (s_hi - s_lo)
	return lerp(_t_values[lo], _t_values[hi], frac)

func position_at(s_local: float) -> Vector2:
	return _eval_bezier(_t_for_arc(s_local))

func tangent_at(s_local: float) -> float:
	var t: float = _t_for_arc(s_local)
	return _eval_derivative(t).angle()

func curvature_at(s_local: float) -> float:
	return total_turn_angle

## Smallest absolute angular difference between two angles (radians).
func _angle_diff(a: float, b: float) -> float:
	var diff: float = a - b
	while diff > PI:
		diff -= TAU
	while diff < -PI:
		diff += TAU
	return diff