class_name Trajectory
extends RefCounted
## Immutable arc-length-parametrized trajectory over a sequence of
## TrajectorySegment (LineSeg / BezierSeg). Single source of truth for the
## cumulative-arc bookkeeping and the "find segment containing arc s" lookup
## that was previously duplicated in VehicleController, RoadGrid and the
## integration tests (DRY).
##
## A Trajectory is built once (typically via TrajectoryBuilder.build_trajectory)
## and then queried by any consumer that needs a position, tangent, segment
## index or local arc-length at a given global arc length.

var segments: Array[TrajectorySegment] = []
var seg_start_arc: Array[float] = []  # cumulative arc length at start of each segment
var total_length: float = 0.0


## Build a Trajectory from an existing segment list (does not copy).
static func from_segments(segs: Array[TrajectorySegment]) -> Trajectory:
	var t := Trajectory.new()
	t.segments = segs
	var cum := 0.0
	for seg in segs:
		t.seg_start_arc.append(cum)
		cum += seg.length
	t.total_length = cum
	return t


## Empty trajectory (zero length, no segments). Used as a safe default.
func clear() -> void:
	segments.clear()
	seg_start_arc.clear()
	total_length = 0.0


func is_empty() -> bool:
	return segments.is_empty()


## Index of the segment containing the global arc length `s_arc`. Cached
## forward walk: callers that advance monotonically (the vehicle) can pass
## `hint` to resume from the last known index. Clamps past the end to the
## final segment and before the start to 0.
func segment_index_at(s_arc: float, hint: int = 0) -> int:
	if segments.is_empty():
		return -1
	var idx: int = clamp(hint, 0, segments.size() - 1)
	while idx < segments.size() - 1 and s_arc >= seg_start_arc[idx] + segments[idx].length:
		idx += 1
	# Walk back if the hint overshot (s_arc moved backwards, e.g. after a
	# path reassignment mid-frame).
	while idx > 0 and s_arc < seg_start_arc[idx]:
		idx -= 1
	return idx


## The segment containing `s_arc`. Convenience wrapper around
## segment_index_at for consumers that don't need the cached hint.
func segment_at(s_arc: float, hint: int = 0) -> TrajectorySegment:
	var idx: int = segment_index_at(s_arc, hint)
	if idx < 0:
		return null
	return segments[idx]


## Arc length local to the current segment for a global arc length `s_arc`.
func local_s_at(s_arc: float, hint: int = 0) -> float:
	var idx: int = segment_index_at(s_arc, hint)
	if idx < 0:
		return 0.0
	return s_arc - seg_start_arc[idx]


## World position at a global arc length. Clamps past the end to the final
## point and before the start to the first point.
func position_at(s_arc: float, hint: int = 0) -> Vector2:
	if segments.is_empty():
		return Vector2.ZERO
	var s_clamped: float = clamp(s_arc, 0.0, total_length)
	var idx: int = segment_index_at(s_clamped, hint)
	return segments[idx].position_at(s_clamped - seg_start_arc[idx])


## Heading (radians) at a global arc length. Clamps like position_at.
func tangent_at(s_arc: float, hint: int = 0) -> float:
	if segments.is_empty():
		return 0.0
	var s_clamped: float = clamp(s_arc, 0.0, total_length)
	var idx: int = segment_index_at(s_clamped, hint)
	return segments[idx].tangent_at(s_clamped - seg_start_arc[idx])


## True when the global arc length has reached (or passed) the end.
func is_at_end(s_arc: float) -> bool:
	return s_arc >= total_length
