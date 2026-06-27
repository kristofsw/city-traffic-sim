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

## Total turn angle of this segment in radians (0 for straight, >0 for arcs).
## Used by the vehicle to compute turn-based slowdown.
func curvature_at(s_local: float) -> float:
	return 0.0

## Fractional position within this segment (0.0 at start, 1.0 at end).
## Used for apex-based turn slowdown weighting.
func progress_fraction(s_local: float) -> float:
	if length <= 0.001:
		return 0.0
	return clamp(s_local / length, 0.0, 1.0)