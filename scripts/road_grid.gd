class_name RoadGrid
extends Node2D
## Renders the procedural road grid and owns the RoadGraph used by the
## simulation. The active A->B route (soft cyan line + green start ring +
## red goal ring) is ALWAYS visible. Press F1 to toggle the raw graph
## debug overlay (nodes + edges). F5 regenerates the grid.

const ASPHALT_COLOR := Color(0.168, 0.168, 0.188, 1)  # #2b2b30
const LANE_MARK_COLOR := Color(0.353, 0.353, 0.392, 1)  # #5a5a64
const PARK_COLOR := Color(0.165, 0.227, 0.165, 1)  # #2a3a2a subtle park fill
const DEBUG_NODE_COLOR := Color(1.0, 0.85, 0.30, 0.9)
const DEBUG_EDGE_COLOR := Color(0.30, 0.70, 1.0, 0.6)
const ROUTE_LINE_COLOR := Color(0.50, 0.85, 1.0, 0.40)  # #7fd8ff ~40%
const ROUTE_START_COLOR := Color(0.50, 1.0, 0.60, 0.9)  # #7fff9a
const ROUTE_GOAL_COLOR := Color(1.0, 0.50, 0.50, 0.9)  # #ff7f7f
const ROUTE_RING_INNER := Color(0.168, 0.168, 0.188, 1)  # asphalt core for ring look

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0
@export var show_debug: bool = false
@export var lane_offset: float = 12.0  # right-hand lane offset, matches vehicle
@export var turn_radius_for_route: float = 22.0  # matches vehicle turn_radius
## Which built-in generator to use when no `map_generator` preset is assigned.
## "grid" = Manhattan grid (GridGenerator); "street_network" = variable grid
## with T-junctions and 45° diagonals (StreetNetworkGenerator). The
## `map_generator` export always takes priority if set.
@export var generator_type: StringName = &"grid"
## StreetNetworkGenerator params (used only when generator_type == "street_network"
## and no map_generator preset is assigned).
@export var block_jitter: float = 0.25
@export var diagonal_count: int = 2
@export var snap_tolerance: float = 24.0
## Optional map-generator preset (.tres). When assigned, RoadGrid uses a
## duplicate of it (so a shared preset isn't mutated) instead of building a
## default generator from the exports above. This is the seam for
## expandable map generation: drop a HexGenerator.tres / RadialGenerator.tres
## here to swap topologies without touching this script.
@export var map_generator: MapGenerator = null

var generator: MapGenerator = null
var graph: RoadGraph = null
var route_path: Array[Vector2i] = []
var route_start: Vector2i = Vector2i.ZERO
var route_goal: Vector2i = Vector2i.ZERO
var _route_trajectory: Trajectory = null  # cached; rebuilt on set_route/_regenerate
# Cached polyline; rebuilt alongside _route_trajectory.
var _route_samples: PackedVector2Array = PackedVector2Array()


func _ready() -> void:
	_regenerate()


func _regenerate() -> void:
	if map_generator != null:
		# Use a duplicate so a shared preset resource isn't mutated by generate().
		generator = map_generator.duplicate(true)
	elif generator_type == &"street_network":
		var sn := StreetNetworkGenerator.new()
		sn.screen_size = screen_size
		sn.margin_px = margin_px
		sn.target_block_size = target_block_size
		sn.road_width = road_width
		sn.lane_width = lane_width
		sn.block_jitter = block_jitter
		sn.diagonal_count = diagonal_count
		sn.snap_tolerance = snap_tolerance
		generator = sn
	else:
		var grid := GridGenerator.new()
		grid.screen_size = screen_size
		grid.margin_px = margin_px
		grid.target_block_size = target_block_size
		grid.road_width = road_width
		grid.lane_width = lane_width
		generator = grid
	generator.generate()
	graph = RoadGraph.new()
	graph.build(generator)
	_route_trajectory = null  # invalidate cache; graph changed
	_route_samples = PackedVector2Array()
	queue_redraw()


func _draw() -> void:
	if graph == null:
		return
	# Park fills for carved obstacle holes (drawn under roads so road
	# edges around holes render cleanly).
	if generator is GridGenerator:
		var grid := generator as GridGenerator
		for center in grid.hole_centers:
			var r: float = grid.hole_radius_px + road_width * 0.5
			draw_circle(center, r, PARK_COLOR)
	# Road surface + lane markings: single pass over edges (each edge once).
	for key in graph.edges:
		var from: Vector2 = graph.world_of(key)
		for n in graph.edges[key]:
			if key < n:
				var to: Vector2 = graph.world_of(n)
				draw_line(from, to, ASPHALT_COLOR, road_width, true)
				_draw_dashed_line(from, to, LANE_MARK_COLOR, 1.5, 16.0, 12.0)
	# Intersection circles to mask seam artifacts (angle-agnostic; works for
	# any intersection geometry, not just 90° grid crossings).
	for key in graph.nodes:
		var p: Vector2 = graph.world_of(key)
		draw_circle(p, road_width * 0.5, ASPHALT_COLOR)
	# Always-on route visualization.
	_draw_route()
	# Debug overlay (F1): raw graph nodes + edges.
	if show_debug:
		for key in graph.edges:
			var from: Vector2 = graph.world_of(key)
			for n in graph.edges[key]:
				if key < n:
					var to: Vector2 = graph.world_of(n)
					draw_line(from, to, DEBUG_EDGE_COLOR, 1.0, true)
		for key in graph.nodes:
			draw_circle(graph.world_of(key), 4.0, DEBUG_NODE_COLOR)


func _draw_route() -> void:
	if route_path.size() < 2:
		return
	# Use the cached right-lane offset trajectory (same one the vehicle follows:
	# LineSeg + BezierSeg). Rebuild only if missing. Sampling at fixed
	# arc-length intervals yields a smooth polyline that stays on the right
	# lane through intersections and draws smooth bezier curves at turns.
	if _route_trajectory == null or _route_trajectory.is_empty():
		_route_trajectory = _build_route_trajectory()
		_route_samples = _sample_route(_route_trajectory)
	if _route_trajectory == null or _route_trajectory.is_empty():
		return
	if _route_samples.size() < 2:
		return
	draw_polyline(_route_samples, ROUTE_LINE_COLOR, 3.0, true)
	# Start (A) ring: green, on the right lane.
	_draw_ring(_route_trajectory.position_at(0.0), 10.0, ROUTE_START_COLOR)
	# Goal (B) ring: red, on the right lane.
	_draw_ring(
		_route_trajectory.position_at(_route_trajectory.total_length), 10.0, ROUTE_GOAL_COLOR
	)


## Sample the trajectory at fixed arc-length intervals into a polyline.
## Extracted so the sample cache can be rebuilt alongside the trajectory
## without re-running the sampling loop on every queue_redraw.
func _sample_route(traj: Trajectory) -> PackedVector2Array:
	var total: float = traj.total_length
	var sample_interval: float = 4.0
	var pts: PackedVector2Array = PackedVector2Array()
	var arc: float = 0.0
	var hint: int = 0
	while arc <= total:
		hint = traj.segment_index_at(arc, hint)
		pts.append(traj.position_at(arc, hint))
		arc += sample_interval
	return pts


func _build_route_trajectory() -> Trajectory:
	return TrajectoryBuilder.build_trajectory(graph, route_path, lane_offset, turn_radius_for_route)


func _draw_ring(center: Vector2, radius: float, color: Color) -> void:
	draw_circle(center, radius, color)
	draw_circle(center, radius - 4.0, ROUTE_RING_INNER)


func _draw_dashed_line(
	from: Vector2, to: Vector2, color: Color, width: float, dash: float, gap: float
) -> void:
	var total: float = from.distance_to(to)
	if total <= 0.0:
		return
	var dir: Vector2 = (to - from).normalized()
	var step: float = dash + gap
	var d: float = 0.0
	while d < total:
		var a: Vector2 = from + dir * d
		var b: Vector2 = from + dir * min(d + dash, total)
		draw_line(a, b, color, width, true)
		d += step


func _input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_debug"):
		show_debug = not show_debug
		queue_redraw()
		get_viewport().set_input_as_handled()
	elif event.is_action_pressed("regenerate_grid"):
		_regenerate()
		get_viewport().set_input_as_handled()


func get_graph() -> RoadGraph:
	return graph


func get_generator() -> MapGenerator:
	return generator


func set_route(start: Vector2i, goal: Vector2i, path: Array[Vector2i]) -> void:
	route_start = start
	route_goal = goal
	route_path = path
	_route_trajectory = null  # invalidate cache; route changed
	_route_samples = PackedVector2Array()
	queue_redraw()


func clear_route() -> void:
	route_path = []
	route_start = Vector2i.ZERO
	route_goal = Vector2i.ZERO
	_route_trajectory = null
	_route_samples = PackedVector2Array()
	queue_redraw()
