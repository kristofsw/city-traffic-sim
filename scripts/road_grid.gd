extends Node2D
## Renders the procedural road grid and owns the RoadGraph used by the
## simulation. Press F1 to toggle the debug overlay (graph nodes + edges).

const ASPHALT_COLOR := Color(0.168, 0.168, 0.188, 1)      # #2b2b30
const LANE_MARK_COLOR := Color(0.353, 0.353, 0.392, 1)    # #5a5a64
const DEBUG_NODE_COLOR := Color(1.0, 0.85, 0.30, 0.9)
const DEBUG_EDGE_COLOR := Color(0.30, 0.70, 1.0, 0.6)
const DEBUG_PATH_COLOR := Color(1.0, 0.40, 0.55, 0.9)

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0
@export var show_debug: bool = false

var generator: GridGenerator = null
var graph: RoadGraph = null
var debug_path: Array[Vector2i] = []

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
			# Only draw each edge once (avoid double-draw).
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
	# Debug overlay.
	if show_debug:
		for key in graph.edges:
			var from: Vector2 = graph.world_of(key)
			for n in graph.edges[key]:
				if key < n:
					var to: Vector2 = graph.world_of(n)
					draw_line(from, to, DEBUG_EDGE_COLOR, 1.0, true)
		for key in graph.nodes:
			draw_circle(graph.world_of(key), 4.0, DEBUG_NODE_COLOR)
		if debug_path.size() >= 2:
			for i in range(debug_path.size() - 1):
				draw_line(graph.world_of(debug_path[i]), graph.world_of(debug_path[i + 1]), DEBUG_PATH_COLOR, 2.5, true)

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

func set_debug_path(path: Array[Vector2i]) -> void:
	debug_path = path
	queue_redraw()

func clear_debug_path() -> void:
	debug_path = []
	queue_redraw()