class_name StreetNetworkGenerator
extends MapGenerator
## Structured variable-grid street network: an alternative to the uniform
## Manhattan grid. Builds a coherent aligned grid with variable block
## lengths, partial roads that create T-junctions, and optional 45°
## diagonal avenues cutting across -- the Barcelona Eixample pattern.
##
## Unlike the free-walk organic approach it replaced (which produced a
## jumbled mess of roads at arbitrary angles), this generator keeps a
## coherent structure: all rows share column x-positions, all columns
## share row y-positions, so roads are perfectly straight. Block sizes
## vary via `block_jitter`; `partial_road_fraction` of roads stop short
## of the boundary (T-junctions); `diagonal_count` 45° avenues cross
## the grid and snap to grid intersections where they pass nearby.
##
## A MapGenerator (Resource) subclass: the seam for expandable map
## generation. Drop a saved StreetNetworkGenerator.tres onto RoadGrid's
## `map_generator` export, or select it via the `generator_type` toggle.
##
## The entire driving pipeline (TrajectoryBuilder bezier arcs, VehicleMover
## turn-slowdown, RoadGraph A*, VehicleBody indicators) is angle-agnostic,
## so the 45° diagonal turns work without changes to those systems.

# Diagonal node keys use a large offset to avoid colliding with grid keys
# Vector2i(c, r). Diagonal nodes that snap to a grid node reuse that grid key.
const _DIAG_KEY_OFFSET: int = 10000

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0
## Fraction of block size to vary each step by (0..1). 0 = uniform grid,
## 0.25 = each step is 75%-125% of the base. Renormalized to fill the area.
@export var block_jitter: float = 0.25
## Fraction of roads that are partial (stop short of the boundary, creating
## T-junctions where they meet full roads). 0 = all roads span the map,
## 0.3 = ~30% of roads are partial. Boundary nodes on partial roads are
## removed (they don't reach the edge, so they're not spawn points).
@export var partial_road_fraction: float = 0.3
## Number of 45°/135° diagonal avenues crossing the grid (0, 1, or 2).
## Diagonals snap to nearby grid intersections, creating 5-way junctions
## where they cross grid roads.
@export var diagonal_count: int = 2
## Max distance (px) for a diagonal node to snap to an existing grid node.
@export var snap_tolerance: float = 24.0

var cols: int = 0
var rows: int = 0
var block_w: float = 0.0
var block_h: float = 0.0
var rng := RandomNumberGenerator.new()
# Cumulative x/y offsets from the margin origin (index 0 = 0.0).
var _col_x: Array[float] = []
var _row_y: Array[float] = []
# Cached boundary (nodes near the screen edge) for spawn selection.
var _boundary: Array[Vector2i] = []


func generate() -> void:
	var inner_w: float = screen_size.x - 2.0 * margin_px
	var inner_h: float = screen_size.y - 2.0 * margin_px
	cols = max(2, int(floor(inner_w / target_block_size)) + 1)
	rows = max(2, int(floor(inner_h / target_block_size)) + 1)
	block_w = inner_w / float(cols - 1)
	block_h = inner_h / float(rows - 1)
	_build_offsets(inner_w, inner_h)
	nodes.clear()
	edges.clear()
	_boundary = []
	# Pass 1: aligned variable grid.
	_build_grid()
	# Pass 2: partial roads (T-junctions).
	_make_partial_roads()
	# Pass 3: 45° diagonal avenues.
	_add_diagonals()
	# Finalize: prune dead-ends + isolated pockets, compute boundary.
	_prune_dead_ends()
	_prune_unreachable()
	_compute_boundary()


# ----------------------------------------------------------------- pass 1: grid


## Build the full aligned grid: all cols x rows nodes at variable offsets,
## connected with 4-neighbourhood edges. Roads are perfectly straight
## because all rows share _col_x and all columns share _row_y.
func _build_grid() -> void:
	for c in range(cols):
		for r in range(rows):
			var key := Vector2i(c, r)
			nodes[key] = world_pos(c, r)
			edges[key] = []
	for c in range(cols):
		for r in range(rows):
			var key := Vector2i(c, r)
			if c + 1 < cols:
				_connect(key, Vector2i(c + 1, r))
			if r + 1 < rows:
				_connect(key, Vector2i(c, r + 1))


func world_pos(c: int, r: int) -> Vector2:
	return Vector2(margin_px + _col_x[c], margin_px + _row_y[r])


# ------------------------------------------------------- pass 2: partial roads


## Make `partial_road_fraction` of vertical and horizontal roads partial:
## they span only a subset of rows/columns, stopping at interior
## intersections (T-junctions) instead of reaching the boundary. Nodes on
## partial roads outside the span are removed; boundary nodes on partial
## roads are removed (they're not spawn points).
func _make_partial_roads() -> void:
	# Vertical roads (columns): pick a contiguous row span [r_start, r_end].
	var partial_cols: int = int(float(cols) * partial_road_fraction * 0.5)
	for _i in range(partial_cols):
		var c: int = rng.randi_range(1, cols - 2)
		if not nodes.has(Vector2i(c, 0)):
			continue  # already partial
		var span_start: int = rng.randi_range(1, max(1, rows - 3))
		var span_end: int = rng.randi_range(span_start + 1, rows - 2)
		_remove_column_outside_span(c, span_start, span_end)
	# Horizontal roads (rows): pick a contiguous column span [c_start, c_end].
	var partial_rows: int = int(float(rows) * partial_road_fraction * 0.5)
	for _i in range(partial_rows):
		var r: int = rng.randi_range(1, rows - 2)
		if not nodes.has(Vector2i(0, r)):
			continue  # already partial
		var span_start: int = rng.randi_range(1, max(1, cols - 3))
		var span_end: int = rng.randi_range(span_start + 1, cols - 2)
		_remove_row_outside_span(r, span_start, span_end)


## Remove all nodes on column `c` outside [r_start, r_end], cleaning edges.
func _remove_column_outside_span(c: int, r_start: int, r_end: int) -> void:
	for r in range(rows):
		if r < r_start or r > r_end:
			_remove_node(Vector2i(c, r))


## Remove all nodes on row `r` outside [c_start, c_end], cleaning edges.
func _remove_row_outside_span(r: int, c_start: int, c_end: int) -> void:
	for c in range(cols):
		if c < c_start or c > c_end:
			_remove_node(Vector2i(c, r))


# ------------------------------------------------------ pass 3: diagonal avenues


## Add `diagonal_count` 45°/135° diagonal avenues crossing the map. Each
## diagonal steps at ~target_block_size in a fixed diagonal direction,
## snapping to existing grid nodes within snap_tolerance. Consecutive
## diagonal nodes are connected, merging with the grid where they cross.
func _add_diagonals() -> void:
	if diagonal_count <= 0:
		return
	var inner: Rect2 = _inner_rect()
	# Diagonal 1: 45° (southeast) from left edge near the top.
	if diagonal_count >= 1:
		var start1: Vector2 = Vector2(
			inner.position.x, inner.position.y + inner.size.y * rng.randf_range(0.1, 0.4)
		)
		_walk_diagonal(start1, PI / 4.0, inner)  # 45° southeast
	# Diagonal 2: 135° (southwest) from right edge near the top.
	if diagonal_count >= 2:
		var start2: Vector2 = Vector2(
			inner.position.x + inner.size.x,
			inner.position.y + inner.size.y * rng.randf_range(0.1, 0.4)
		)
		_walk_diagonal(start2, PI * 3.0 / 4.0, inner)  # 135° southwest


## Walk a diagonal avenue from `start` at fixed `heading` (45° or 135°),
## stepping ~target_block_size each node. At each position, snap to an
## existing grid node within snap_tolerance if one exists (creating 5-way
## junctions); otherwise add a new diagonal node. Connect consecutive nodes.
func _walk_diagonal(start: Vector2, heading: float, bounds: Rect2) -> void:
	var step: float = target_block_size * sqrt(2.0) * 0.5  # ~one grid cell diagonally
	var pos: Vector2 = start
	var prev_key: Vector2i = _add_or_snap_diagonal_node(pos)
	while bounds.has_point(pos):
		pos += Vector2.from_angle(heading) * step
		if not bounds.has_point(pos):
			break
		var key: Vector2i = _add_or_snap_diagonal_node(pos)
		if key != prev_key:
			_connect(prev_key, key)
		prev_key = key


## Add a diagonal node at `pos`, OR return the key of an existing grid node
## within snap_tolerance (so the diagonal merges with the grid at crossings).
func _add_or_snap_diagonal_node(pos: Vector2) -> Vector2i:
	for k in nodes.keys():
		if nodes[k].distance_to(pos) <= snap_tolerance:
			return k
	var key := Vector2i(_DIAG_KEY_OFFSET, _DIAG_KEY_OFFSET)
	# Find a unique diagonal key (offset both axes so it never collides
	# with grid keys Vector2i(c, r) where c,r < cols,rows).
	while nodes.has(key):
		key.x += 1
	nodes[key] = pos
	edges[key] = []
	return key


# ----------------------------------------------------------- finalize: prune


## Remove any node not reachable from the first surviving node. Guarantees
## the remaining graph is fully connected (A* always finds a path).
func _prune_unreachable() -> void:
	if nodes.is_empty():
		return
	var start: Vector2i = nodes.keys()[0]
	var visited: Dictionary = {start: true}
	var queue: Array[Vector2i] = [start]
	while not queue.is_empty():
		var cur: Vector2i = queue.pop_front()
		for n in edges[cur]:
			if not visited.has(n):
				visited[n] = true
				queue.append(n)
	var to_remove: Array[Vector2i] = []
	for k in nodes.keys():
		if not visited.has(k):
			to_remove.append(k)
	for k in to_remove:
		_remove_node(k)


## Iteratively remove degree-1 nodes (dead-ends). Removing a dead-end may
## turn its neighbor into a new dead-end, so loop until none remain.
func _prune_dead_ends() -> void:
	var changed: bool = true
	while changed:
		changed = false
		var to_remove: Array[Vector2i] = []
		for k in nodes.keys():
			if edges.has(k) and edges[k].size() <= 1:
				to_remove.append(k)
		for k in to_remove:
			_remove_node(k)
			changed = true


## Compute the boundary: nodes within margin_px + road_width of the screen
## edge. These are the spawn-suitable nodes for VehicleSpawner.pick_start.
func _compute_boundary() -> void:
	_boundary = []
	var edge_dist: float = margin_px + road_width
	for k in nodes.keys():
		var p: Vector2 = nodes[k]
		if (
			p.x <= edge_dist
			or p.x >= screen_size.x - edge_dist
			or p.y <= edge_dist
			or p.y >= screen_size.y - edge_dist
		):
			_boundary.append(k)


# -------------------------------------------------------------- helpers


func _inner_rect() -> Rect2:
	return Rect2(
		margin_px, margin_px, screen_size.x - 2.0 * margin_px, screen_size.y - 2.0 * margin_px
	)


## Build cumulative x/y offset arrays. Index 0 is always 0.0 (the margin is
## added in world_pos). Each subsequent step is base*(1 ± jitter*rng) and the
## array is renormalized so the last entry fills inner_w/inner_h exactly.
## Same renormalization math as GridGenerator._build_axis.
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
	var raw_sum: float = 0.0
	var steps: Array[float] = []
	steps.resize(count - 1)
	for i in range(count - 1):
		var s: float = base
		if block_jitter > 0.0:
			var f: float = 1.0 + (rng.randf() * 2.0 - 1.0) * block_jitter
			s = base * f
		steps[i] = s
		raw_sum += s
	var scale: float = total / raw_sum if raw_sum > 0.0 else 0.0
	var cum: float = 0.0
	for i in range(count - 1):
		cum += steps[i] * scale
		out[i + 1] = cum
	return out


func _connect(a: Vector2i, b: Vector2i) -> void:
	if a == b:
		return
	if not edges.has(a):
		edges[a] = []
	if not edges.has(b):
		edges[b] = []
	if not edges[a].has(b):
		edges[a].append(b)
	if not edges[b].has(a):
		edges[b].append(a)


func _remove_node(key: Vector2i) -> void:
	if not nodes.has(key):
		return
	for n in edges[key]:
		if edges.has(n):
			edges[n].erase(key)
	edges.erase(key)
	nodes.erase(key)


# -------------------------------------------------------------- overrides


func boundary_nodes() -> Array[Vector2i]:
	return _boundary


## World Euclidean distance (inherits the base MapGenerator.far_from behavior
## but kept explicit as a documented contract since this generator uses a
## mix of grid keys and diagonal keys).
func far_from(key: Vector2i, min_distance: float) -> Array:
	var out: Array = []
	if not nodes.has(key):
		return out
	var ref_pos: Vector2 = nodes[key]
	for k in nodes.keys():
		if nodes[k].distance_to(ref_pos) >= min_distance:
			out.append(k)
	return out
