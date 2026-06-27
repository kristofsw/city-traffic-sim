class_name TrajectoryBuilder
extends RefCounted
## Single source of truth for building right-lane offset trajectories from a
## path of grid intersections. Used by both VehicleController (for driving)
## and RoadGrid (for route visualization) to avoid duplicated logic (DRY).
##
## Produces a continuous trajectory of LineSeg (straights) and BezierSeg
## (turn arcs) at intersections. All points are offset to the right-hand
## lane so the car never enters the oncoming lane.


## Build a trajectory from a grid path.
## Parameters:
##   graph:       the road graph containing node world positions
##   path:        ordered list of grid intersection coordinates
##   lane_offset: perpendicular offset to the right-hand side (px)
##   turn_radius: pull-back distance before intersection for bezier arcs (px)
## Returns: array of TrajectorySegment (LineSeg / BezierSeg)
static func build(
	graph: RoadGraph, path: Array[Vector2i], lane_offset: float, turn_radius: float
) -> Array[TrajectorySegment]:
	var out: Array[TrajectorySegment] = []
	if path.size() < 2:
		return out

	# Per-segment data: direction, right-hand perpendicular, entry/exit offsets.
	var dirs: Array[Vector2] = []
	var entries: Array[Vector2] = []
	var exits: Array[Vector2] = []
	for i in range(path.size() - 1):
		var a: Vector2 = graph.world_of(path[i])
		var b: Vector2 = graph.world_of(path[i + 1])
		var d: Vector2 = (b - a).normalized()
		var perp: Vector2 = Vector2(-d.y, d.x)  # right-hand perpendicular (y-down)
		dirs.append(d)
		entries.append(a + perp * lane_offset)
		exits.append(b + perp * lane_offset)

	# Build alternating LineSeg (straights) and BezierSeg (turn arcs).
	var current_pos: Vector2 = entries[0]

	for i in range(path.size() - 1):
		var is_last: bool = i == path.size() - 2
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
			# Line 1: through approach, direction dirs[i]
			# Line 2: through leave, direction dirs[i+1]
			# Produces a G1-continuous arc with correct tangents on both sides.
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
