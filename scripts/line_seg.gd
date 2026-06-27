class_name LineSeg
extends TrajectorySegment
## A straight-line segment of a trajectory. Position and tangent are both
## trivial and constant -- the heading never changes on a LineSeg.

var start: Vector2
var end: Vector2
var dir: Vector2


func _init(p_start: Vector2, p_end: Vector2) -> void:
	start = p_start
	end = p_end
	var diff: Vector2 = end - start
	length = diff.length()
	dir = diff.normalized() if length > 0.001 else Vector2.ZERO


func position_at(s_local: float) -> Vector2:
	return start + dir * s_local


func tangent_at(_s_local: float) -> float:
	return dir.angle()
