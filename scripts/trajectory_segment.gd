class_name TrajectorySegment
extends RefCounted
## Base class for a segment of a vehicle trajectory. Subclasses (LineSeg,
## BezierSeg) implement parametric position and tangent evaluation by
## arc length so the car's position and heading are always perfectly coupled.

var length: float = 0.0  # total arc length of this segment (px)

func position_at(s_local: float) -> Vector2:
	push_error("TrajectorySegment.position_at not implemented")
	return Vector2.ZERO

func tangent_at(s_local: float) -> float:
	push_error("TrajectorySegment.tangent_at not implemented")
	return 0.0