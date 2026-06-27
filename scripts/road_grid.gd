extends Node2D
## Renders the procedural road grid and owns the RoadGraph used by the
## simulation. The active A->B route (soft cyan line + green start ring +
## red goal ring) is ALWAYS visible. Press F1 to toggle the raw graph
## debug overlay (nodes + edges). F5 regenerates the grid.

const ASPHALT_COLOR := Color(0.168, 0.168, 0.188, 1)      # #2b2b30
const LANE_MARK_COLOR := Color(0.353, 0.353, 0.392, 1)    # #5a5a64
const DEBUG_NODE_COLOR := Color(1.0, 0.85, 0.30, 0.9)
const DEBUG_EDGE_COLOR := Color(0.30, 0.70, 1.0, 0.6)
const ROUTE_LINE_COLOR := Color(0.50, 0.85, 1.0, 0.40)    # #7fd8ff ~40%
const ROUTE_START_COLOR := Color(0.50, 1.0, 0.60, 0.9)    # #7fff9a
const ROUTE_GOAL_COLOR := Color(1.0, 0.50, 0.50, 0.9)     # #ff7f7f
const ROUTE_RING_INNER := Color(0.168, 0.168, 0.188, 1)   # asphalt core for ring look

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0
@export var show_debug: bool = false
@export var lane_offset: float = 12.0  # right-hand lane offset, matches vehicle
@export var turn_radius_for_route: float = 22.0  # matches vehicle turn_radius

var generator: GridGenerator = null
var graph: RoadGraph = null
var route_path: Array[Vector2i] = []
var route_start: Vector2i = Vector2i.ZERO
var route_goal: Vector2i = Vector2i.ZERO

func _ready() -> void:
	_regenerate()

func _regenerate() -> void:
	generator = GridGenerator.new()
	generator.screen_size = screen_size
	generator.margin_px = margin_px
	generator.target_block_size = target_block_size
	generator.road_width = road_width
	generator.lane_width = lane_width
	generator.generate()
	graph = RoadGraph.new()
	graph.build(generator)
	queue_redraw()

func _draw() -> void:
	if graph == null:
		return
	# Road surface: thick asphalt line per edge.
	for key in graph.edges:
		var from: Vector2 = graph.world_of(key)
		for n in graph.edges[key]:
			if key < n:
				var to: Vector2 = graph.world_of(n)
				draw_line(from, to, ASPHALT_COLOR, road_width, true)
	# Lane center markings: thin dashed line per edge.
	for key in graph.edges:
		var from: Vector2 = graph.world_of(key)
		for n in graph.edges[key]:
			if key < n:
				var to: Vector2 = graph.world_of(n)
				_draw_dashed_line(from, to, LANE_MARK_COLOR, 1.5, 16.0, 12.0)
	# Intersection squares to mask seam artifacts.
	for key in graph.nodes:
		var p: Vector2 = graph.world_of(key)
		var r: float = road_width * 0.5
		draw_rect(Rect2(p.x - r, p.y - r, r * 2.0, r * 2.0), ASPHALT_COLOR)
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
	# Build the same right-lane offset trajectory the vehicle follows:
	# straight portions on the right lane + bezier arcs at turns.
	# This keeps the route line on the right lane through intersections
	# instead of cutting across the center.
	var pts: Array[Vector2] = _build_route_offset_points()
	if pts.size() < 2:
		return
	for i in range(pts.size() - 1):
		draw_line(pts[i], pts[i + 1], ROUTE_LINE_COLOR, 3.0, true)
	# Start (A) ring: green, on the right lane.
	_draw_ring(pts[0], 10.0, ROUTE_START_COLOR)
	# Goal (B) ring: red, on the right lane.
	_draw_ring(pts[pts.size() - 1], 10.0, ROUTE_GOAL_COLOR)

func _build_route_offset_points() -> Array[Vector2]:
	var p: Array[Vector2i] = route_path
	var out: Array[Vector2] = []
	if p.size() < 2:
		return out
	# Per-segment direction, right-hand perpendicular, entry/exit offsets.
	var dirs: Array[Vector2] = []
	var entries: Array[Vector2] = []
	var exits: Array[Vector2] = []
	for i in range(p.size() - 1):
		var a: Vector2 = graph.world_of(p[i])
		var b: Vector2 = graph.world_of(p[i + 1])
		var d: Vector2 = (b - a).normalized()
		var perp: Vector2 = Vector2(-d.y, d.x)
		dirs.append(d)
		entries.append(a + perp * lane_offset)
		exits.append(b + perp * lane_offset)
	var current_pos: Vector2 = entries[0]
	out.append(current_pos)
	for i in range(p.size() - 1):
		var is_last: bool = (i == p.size() - 2)
		var has_turn: bool = false
		if not is_last:
			has_turn = dirs[i].dot(dirs[i + 1]) < 0.99
		if is_last or not has_turn:
			if current_pos.distance_to(exits[i]) > 1.0:
				out.append(exits[i])
			current_pos = exits[i]
		else:
			var seg_len: float = entries[i].distance_to(exits[i])
			var next_seg_len: float = entries[i + 1].distance_to(exits[i + 1])
			var tr: float = min(turn_radius_for_route, seg_len * 0.4, next_seg_len * 0.4)
			tr = max(tr, 2.0)
			var approach: Vector2 = exits[i] - dirs[i] * tr
			var leave: Vector2 = entries[i + 1] + dirs[i + 1] * tr
			if current_pos.distance_to(approach) > 1.0:
				out.append(approach)
			var cross: float = dirs[i].x * dirs[i + 1].y - dirs[i].y * dirs[i + 1].x
			if abs(cross) < 0.001:
				out.append(leave)
			else:
				var delta: Vector2 = leave - approach
				var t_ctrl: float = (delta.x * dirs[i + 1].y - delta.y * dirs[i + 1].x) / cross
				var control: Vector2 = approach + dirs[i] * t_ctrl
				var samples: int = 18
				for s in range(1, samples + 1):
					var bt: float = float(s) / float(samples)
					var pt: Vector2 = approach.lerp(control, bt).lerp(control.lerp(leave, bt), bt)
					out.append(pt)
			current_pos = leave
	return out

func _draw_ring(center: Vector2, radius: float, color: Color) -> void:
	draw_circle(center, radius, color)
	draw_circle(center, radius - 4.0, ROUTE_RING_INNER)

func _draw_dashed_line(from: Vector2, to: Vector2, color: Color, width: float, dash: float, gap: float) -> void:
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
	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_F1:
			show_debug = not show_debug
			queue_redraw()
		elif event.keycode == KEY_F5:
			_regenerate()

func get_graph() -> RoadGraph:
	return graph

func get_generator() -> GridGenerator:
	return generator

func set_route(start: Vector2i, goal: Vector2i, path: Array[Vector2i]) -> void:
	route_start = start
	route_goal = goal
	route_path = path
	queue_redraw()

func clear_route() -> void:
	route_path = []
	route_start = Vector2i.ZERO
	route_goal = Vector2i.ZERO
	queue_redraw()