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
## Number of obstacle hole clusters to carve out of the grid interior. 0 =
## no holes. Holes force A* to detour, producing multi-turn paths instead
## of trivial one-turn L-shapes.
@export var obstacle_count: int = 3
## Radius of each hole cluster in graph hops (adjacency steps). Radius 2
## removes ~13 nodes in a diamond. Seeds are always interior (boundary is
## preserved) and a connectivity prune removes any unreachable pockets.
@export var obstacle_radius: int = 2

var cols: int = 0
var rows: int = 0
var block_w: float = 0.0
var block_h: float = 0.0
var rng := RandomNumberGenerator.new()
# World-space centers of carved hole clusters, for RoadGrid to draw as
# park fills. Each entry is the average world position of the removed
# nodes in that cluster.
var hole_centers: Array[Vector2] = []
# Approximate radius (px) of carved holes, for RoadGrid fill sizing.
var hole_radius_px: float = 0.0
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
	# Carve obstacle holes out of the interior (boundary preserved). Each
	# hole removes a cluster of nodes within obstacle_radius hops of a
	# random interior seed, then a connectivity prune removes any pockets
	# isolated by the carving. This forces A* to detour, producing multi-
	# turn paths instead of trivial one-turn L-shapes.
	_carve_holes()


func world_pos(c: int, r: int) -> Vector2:
	return Vector2(margin_px + _col_x[c], margin_px + _row_y[r])


## Carve obstacle_count hole clusters out of the interior. Each hole is a
## BFS ball of obstacle_radius hops around a random interior seed. Edges
## to removed nodes are cleaned from both directions. After carving, a
## connectivity prune BFSs from the boundary and removes any node not
## reachable, guaranteeing the remaining graph is fully connected.
func _carve_holes() -> void:
	hole_centers = []
	hole_radius_px = 0.0
	if obstacle_count <= 0 or obstacle_radius <= 0:
		return
	if cols < 3 or rows < 3:
		return  # too small to carve safely
	var boundary := boundary_nodes()
	var boundary_set: Dictionary = {}
	for b in boundary:
		boundary_set[b] = true
	# Candidate seeds: interior nodes only (not boundary).
	var interior: Array[Vector2i] = []
	for c in range(1, cols - 1):
		for r in range(1, rows - 1):
			interior.append(Vector2i(c, r))
	if interior.is_empty():
		return
	# Estimate the pixel radius of a hole (average of block_w/block_h * radius).
	var avg_block: float = (block_w + block_h) * 0.5
	hole_radius_px = avg_block * float(obstacle_radius)
	for i in range(obstacle_count):
		if interior.is_empty():
			break
		var seed: Vector2i = interior[rng.randi() % interior.size()]
		# BFS up to obstacle_radius hops to collect the cluster.
		var cluster := _bfs_ball(seed, obstacle_radius)
		# Protect boundary nodes: never remove them (preserves the
		# connected perimeter ring the spawner relies on). Only interior
		# nodes in the cluster are carved.
		var to_remove: Array[Vector2i] = []
		for k in cluster:
			if not boundary_set.has(k) and nodes.has(k):
				to_remove.append(k)
		# World-space center = average position of the nodes we'll remove.
		var sum: Vector2 = Vector2.ZERO
		var counted: int = 0
		for k in to_remove:
			sum += nodes[k]
			counted += 1
		if counted > 0:
			hole_centers.append(sum / float(counted))
		# Remove the cluster nodes + clean edges both directions.
		for k in to_remove:
			_remove_node(k)
		# Remove carved nodes from the interior pool so the next seed
		# doesn't land inside an existing hole.
		interior = interior.filter(func(k): return not cluster.has(k))
	# Connectivity prune: BFS from every boundary node, remove any node
	# not reached (collapses isolated pockets a hole might create).
	_prune_unreachable(boundary_set)


## BFS ball of up to `radius` hops around `seed` on the current adjacency.
## Returns the set of nodes (including the seed) within that radius.
func _bfs_ball(seed: Vector2i, radius: int) -> Array[Vector2i]:
	var out: Array[Vector2i] = []
	if not edges.has(seed):
		return out
	var visited: Dictionary = {seed: 0}
	var queue: Array[Vector2i] = [seed]
	while not queue.is_empty():
		var cur: Vector2i = queue.pop_front()
		var d: int = visited[cur]
		if d >= radius:
			continue
		for n in edges[cur]:
			if not visited.has(n):
				visited[n] = d + 1
				queue.append(n)
	for k in visited.keys():
		out.append(k)
	return out


## Remove a node and clean its edges from both directions.
func _remove_node(key: Vector2i) -> void:
	if not nodes.has(key):
		return
	# Remove this node from every neighbor's edge list.
	for n in edges[key]:
		if edges.has(n):
			edges[n].erase(key)
	edges.erase(key)
	nodes.erase(key)


## Remove any node not reachable from the boundary. Guarantees the
## remaining graph is fully connected (A* always finds a path).
func _prune_unreachable(boundary_set: Dictionary) -> void:
	# BFS from every surviving boundary node.
	var visited: Dictionary = {}
	var queue: Array[Vector2i] = []
	for b in boundary_set.keys():
		if nodes.has(b):
			visited[b] = true
			queue.append(b)
	while not queue.is_empty():
		var cur: Vector2i = queue.pop_front()
		for n in edges[cur]:
			if not visited.has(n):
				visited[n] = true
				queue.append(n)
	# Remove any node not visited.
	var to_remove: Array[Vector2i] = []
	for k in nodes.keys():
		if not visited.has(k):
			to_remove.append(k)
	for k in to_remove:
		_remove_node(k)


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


## Grid-specific: Manhattan distance on Vector2i keys (hops), preserving
## the hop-count semantics GridGenerator tests rely on. The base
## MapGenerator.far_from now uses world Euclidean distance.
func far_from(key: Vector2i, min_distance: float) -> Array:
	var out: Array = []
	var min_int: int = int(min_distance)
	for k in nodes.keys():
		if abs(k.x - key.x) + abs(k.y - key.y) >= min_int:
			out.append(k)
	return out


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
