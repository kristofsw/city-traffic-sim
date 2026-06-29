class_name GridGenerator
extends MapGenerator
## Procedurally generates a Manhattan-style road grid that fills the screen
## between a margin. Produces intersection coordinates and the edge list
## consumed by RoadGraph.
##
## `block_jitter` varies block sizes so intersections aren't evenly spaced
## (visual variety; topology stays a 4-neighbourhood grid). 0 = uniform.
##
## A MapGenerator (Resource) subclass: the seam for expandable map
## generation. Drop a saved GridGenerator.tres onto RoadGrid's
## `map_generator` export, or let RoadGrid create one with its own exports.

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0
## Fraction of block size to vary each step by (0..1). 0 = uniform grid,
## 0.25 = each step is 75%-125% of the base. Renormalized to fill the area.
@export var block_jitter: float = 0.0

var cols: int = 0
var rows: int = 0
var block_w: float = 0.0
var block_h: float = 0.0
var rng := RandomNumberGenerator.new()
# Cumulative x/y offsets from the margin origin (index 0 = 0.0). When
# block_jitter == 0 these are computed uniformly (identical to the old
# formula); when jitter > 0 each step varies and the array is renormalized
# to fill the inner area exactly.
var _col_x: Array[float] = []
var _row_y: Array[float] = []


func generate() -> void:
	var inner_w: float = screen_size.x - 2.0 * margin_px
	var inner_h: float = screen_size.y - 2.0 * margin_px
	cols = max(1, int(floor(inner_w / target_block_size)) + 1)
	rows = max(1, int(floor(inner_h / target_block_size)) + 1)
	# Recompute block size so a uniform grid fills the inner area exactly.
	block_w = inner_w / float(cols - 1) if cols > 1 else 0.0
	block_h = inner_h / float(rows - 1) if rows > 1 else 0.0
	_build_offsets(inner_w, inner_h)
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
	return Vector2(margin_px + _col_x[c], margin_px + _row_y[r])


## Build cumulative x/y offset arrays. Index 0 is always 0.0 (the margin is
## added in world_pos). Each subsequent step is base*(1 ± jitter*rng) and the
## array is renormalized so the last entry fills inner_w/inner_h exactly.
func _build_offsets(inner_w: float, inner_h: float) -> void:
	_col_x = _build_axis(cols, block_w, inner_w)
	_row_y = _build_axis(rows, block_h, inner_h)


func _build_axis(count: int, base: float, total: float) -> Array[float]:
	var out: Array[float] = []
	out.resize(count)
	if count == 0:
		return out
	out[0] = 0.0
	if count == 1:
		return out
	# Step i is the gap between node i-1 and node i.
	var raw_sum: float = 0.0
	var steps: Array[float] = []
	steps.resize(count - 1)
	for i in range(count - 1):
		var s: float = base
		if block_jitter > 0.0:
			# 1 ± jitter*randf() in [1-jitter, 1+jitter]
			var f: float = 1.0 + (rng.randf() * 2.0 - 1.0) * block_jitter
			s = base * f
		steps[i] = s
		raw_sum += s
	# Renormalize so the cumulative sum fills `total` exactly. This preserves
	# the relative variation while guaranteeing the grid fits the inner area.
	var scale: float = total / raw_sum if raw_sum > 0.0 else 0.0
	var cum: float = 0.0
	for i in range(count - 1):
		cum += steps[i] * scale
		out[i + 1] = cum
	return out


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
