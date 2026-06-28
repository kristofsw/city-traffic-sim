class_name GridGenerator
extends MapGenerator
## Procedurally generates a uniform Manhattan-style road grid that fills the
## screen between a margin. Produces intersection coordinates and the edge
## list consumed by RoadGraph.
##
## A MapGenerator (Resource) subclass: the seam for expandable map
## generation. Drop a saved GridGenerator.tres onto RoadGrid's
## `map_generator` export, or let RoadGrid create one with its own exports.

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0

var cols: int = 0
var rows: int = 0
var block_w: float = 0.0
var block_h: float = 0.0


func generate() -> void:
	var inner_w: float = screen_size.x - 2.0 * margin_px
	var inner_h: float = screen_size.y - 2.0 * margin_px
	cols = max(1, int(floor(inner_w / target_block_size)) + 1)
	rows = max(1, int(floor(inner_h / target_block_size)) + 1)
	# Recompute block size so the grid fills the inner area exactly.
	block_w = inner_w / float(cols - 1)
	block_h = inner_h / float(rows - 1)
	nodes.clear()
	edges.clear()
	for c in range(cols):
		for r in range(rows):
			var key := Vector2i(c, r)
			nodes[key] = world_pos(c, r)
			edges[key] = []
	# 4-neighbourhood edges.
	for c in range(cols):
		for r in range(rows):
			var key := Vector2i(c, r)
			if c + 1 < cols:
				_connect(key, Vector2i(c + 1, r))
			if r + 1 < rows:
				_connect(key, Vector2i(c, r + 1))


func world_pos(c: int, r: int) -> Vector2:
	return Vector2(margin_px + float(c) * block_w, margin_px + float(r) * block_h)


func grid_to_world(key: Vector2i) -> Vector2:
	return nodes[key]


func _connect(a: Vector2i, b: Vector2i) -> void:
	edges[a].append(b)
	edges[b].append(a)


func boundary_nodes() -> Array[Vector2i]:
	var out: Array[Vector2i] = []
	for c in range(cols):
		out.append(Vector2i(c, 0))
		if rows > 1:
			out.append(Vector2i(c, rows - 1))
	for r in range(1, rows - 1):
		out.append(Vector2i(0, r))
		if cols > 1:
			out.append(Vector2i(cols - 1, r))
	return out
