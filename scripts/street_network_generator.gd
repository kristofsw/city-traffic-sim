class_name StreetNetworkGenerator
extends MapGenerator
## Organic street network generator: an alternative to the Manhattan grid.
## Builds avenues (wavering polylines crossing the map) and side streets
## (branches off avenues at varied angles), producing non-aligned
## intersections, T-junctions, and varied block lengths -- realistic city
## variety that the single-global-grid GridGenerator cannot express.
##
## A MapGenerator (Resource) subclass: the seam for expandable map
## generation. Drop a saved StreetNetworkGenerator.tres onto RoadGrid's
## `map_generator` export, or select it via the `generator_type` toggle.
##
## The entire driving pipeline (TrajectoryBuilder bezier arcs, VehicleMover
## turn-slowdown, RoadGraph A*, VehicleBody indicators) is angle-agnostic,
## so roads at any angle work without changes to those systems.

@export var screen_size: Vector2 = Vector2(1280, 720)
@export var margin_px: float = 40.0
@export var target_block_size: float = 128.0
@export var road_width: float = 48.0
@export var lane_width: float = 24.0
## Number of primary avenues (long wavering roads crossing the map).
@export var avenue_count: int = 5
## Probability per avenue node that a side street branches off it.
@export var side_street_density: float = 0.5
## Max angular deviation (radians) per step. ~0.35 rad = ±20°. Avenues
## waver by ±angle_jitter each step; side streets branch at perpendicular
## ± angle_jitter (70°-110° from the avenue).
@export var angle_jitter: float = 0.35
## Max distance (px) for two nodes to be snapped into one. Merges
## near-coincident intersections produced by crossing roads.
@export var snap_tolerance: float = 24.0

var rng := RandomNumberGenerator.new()
# Sequential key counter; keys are Vector2i(id, 0) with no grid meaning.
var _next_id: int = 0
# Cached boundary (nodes near the screen edge) for spawn selection.
var _boundary: Array[Vector2i] = []


func generate() -> void:
	nodes.clear()
	edges.clear()
	_next_id = 0
	_boundary = []
	# Do NOT randomize() here: the caller may have set rng.seed for
	# reproducibility (tests, .tres presets). RandomNumberGenerator is
	# already randomized on construction; only re-randomize if desired.
	# Pass 1: avenues -- long wavering polylines crossing the map.
	_generate_avenues()
	# Pass 2: side streets -- branches off avenues at varied angles.
	_generate_side_streets()
	# Pass 3: snap near-coincident nodes, prune unreachable pockets.
	_snap_nearby_nodes()
	_prune_unreachable()
	_prune_dead_ends()
	_compute_boundary()


# ------------------------------------------------------------------ avenues


## Build `avenue_count` polylines that waver across the map. Half start on
## the left edge heading right; half start on the top edge heading down.
## Each step advances ~target_block_size and rotates the heading by
## ±angle_jitter. Nodes are added along the polyline; consecutive nodes
## are connected.
func _generate_avenues() -> void:
	var inner: Rect2 = _inner_rect()
	for i in range(avenue_count):
		var horizontal: bool = i % 2 == 0
		var start: Vector2
		var heading: float
		if horizontal:
			start = Vector2(inner.position.x, inner.position.y + rng.randf() * inner.size.y)
			heading = 0.0  # east
		else:
			start = Vector2(inner.position.x + rng.randf() * inner.size.x, inner.position.y)
			heading = PI / 2.0  # south
		_walk_polyline(start, heading, inner)


## Walk a wavering polyline from `start` in `heading` until it exits the
## inner rect. At each step: add a node (or snap to an existing nearby
## node), connect to the previous node, jitter the heading by ±angle_jitter.
func _walk_polyline(start: Vector2, heading: float, bounds: Rect2) -> void:
	var step: float = target_block_size
	var prev_key: Vector2i = _add_or_snap_node(start)
	var pos: Vector2 = start
	while bounds.has_point(pos):
		heading += (rng.randf() * 2.0 - 1.0) * angle_jitter
		pos += Vector2.from_angle(heading) * step
		pos = _clamp_to_bounds(pos, bounds)
		var key: Vector2i = _add_or_snap_node(pos)
		_connect(prev_key, key)
		# If we snapped onto an existing node, the polyline has merged with
		# another road -- stop here (the avenue has reached existing fabric).
		if nodes[key] == pos or _degree(key) > 1:
			# Snapped to an existing node (degree already > 1); keep walking
			# so the avenue continues through the intersection.
			pass
		prev_key = key


# -------------------------------------------------------------- side streets


## Branch side streets off avenue nodes. For each avenue node, with
## probability side_street_density, start a side street heading
## perpendicular ± angle_jitter and walk it until it snaps to an existing
## road node (no dead-ends: if it wanders too far without snapping, steer
## toward the nearest existing node).
func _generate_side_streets() -> void:
	var inner: Rect2 = _inner_rect()
	# Snapshot avenue node keys so we don't iterate nodes we add mid-loop.
	var avenue_keys: Array[Vector2i] = []
	for k in nodes.keys():
		avenue_keys.append(k)
	for k in avenue_keys:
		if rng.randf() > side_street_density:
			continue
		var pos: Vector2 = nodes[k]
		# Perpendicular to the first edge off this node (if any), ± angle_jitter.
		var base_heading: float = 0.0
		if edges[k].size() > 0:
			var nb: Vector2 = nodes[edges[k][0]]
			base_heading = (nb - pos).angle()
		var branch_heading: float = base_heading + PI / 2.0
		branch_heading += (rng.randf() * 2.0 - 1.0) * angle_jitter
		_walk_side_street(pos, branch_heading, inner)


## Walk a side street from `start` in `heading` until it snaps to an
## existing road node. Each step: advance ~target_block_size, jitter
## heading, add/snap a node, connect. If no snap after several steps, steer
## toward the nearest existing node to guarantee no dead-ends.
func _walk_side_street(start: Vector2, heading: float, bounds: Rect2) -> void:
	var step: float = target_block_size
	var max_steps_no_snap: int = 12
	var prev_key: Vector2i = _add_or_snap_node(start)
	var pos: Vector2 = start
	var steps_no_snap: int = 0
	for _i in range(60):
		heading += (rng.randf() * 2.0 - 1.0) * angle_jitter
		# If we haven't snapped in a while, steer toward the nearest
		# existing node to avoid dead-ends.
		if steps_no_snap >= max_steps_no_snap:
			var nearest := _nearest_other_node(pos, prev_key)
			if nearest != Vector2.ZERO:
				heading = (nearest - pos).angle()
		pos += Vector2.from_angle(heading) * step
		if not bounds.has_point(pos):
			pos = _clamp_to_bounds(pos, bounds)
		var key: Vector2i = _add_or_snap_node(pos)
		if key != prev_key:
			_connect(prev_key, key)
		# If we snapped to an existing node, the street has joined the
		# network -- stop (no dead-end).
		if _is_existing_node(key) and _degree(key) > 1:
			return
		prev_key = key
		steps_no_snap += 1


# ----------------------------------------------------------- snap + prune


## Merge nodes within snap_tolerance of each other into the lower-id node.
## Cleans edges from both directions and re-points references.
func _snap_nearby_nodes() -> void:
	var keys: Array[Vector2i] = []
	for k in nodes.keys():
		keys.append(k)
	for i in range(keys.size()):
		var a: Vector2i = keys[i]
		if not nodes.has(a):
			continue
		for j in range(i + 1, keys.size()):
			var b: Vector2i = keys[j]
			if not nodes.has(b):
				continue
			if nodes[a].distance_to(nodes[b]) <= snap_tolerance:
				_merge_nodes(a, b)


## Merge node b into node a (keep a). Re-point all b's edges to a.
func _merge_nodes(a: Vector2i, b: Vector2i) -> void:
	if a == b:
		return
	for n in edges[b]:
		if n == a:
			continue
		# Re-point neighbor n from b to a (avoid duplicates).
		if not edges[a].has(n):
			edges[a].append(n)
		if edges.has(n):
			var idx: int = edges[n].find(b)
			if idx >= 0:
				edges[n][idx] = a
	# Remove the b-a self-edge that may have been added.
	edges[a].erase(b)
	edges.erase(b)
	nodes.erase(b)


## Remove any node not reachable from the boundary. Guarantees the
## remaining graph is fully connected (A* always finds a path).
func _prune_unreachable() -> void:
	if nodes.is_empty():
		return
	# BFS from an arbitrary node; any node not reached is isolated.
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
## turn its neighbor into a new dead-end, so loop until none remain. This
## guarantees every node has degree >= 2, so every road is part of a loop
## and A* always has an alternate route.
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


func _clamp_to_bounds(pos: Vector2, bounds: Rect2) -> Vector2:
	return Vector2(
		clamp(pos.x, bounds.position.x, bounds.position.x + bounds.size.x),
		clamp(pos.y, bounds.position.y, bounds.position.y + bounds.size.y)
	)


## Add a node at `pos` with a fresh sequential key, OR return the key of
## an existing node within snap_tolerance (so crossing roads merge).
func _add_or_snap_node(pos: Vector2) -> Vector2i:
	# Check for an existing node within snap_tolerance.
	for k in nodes.keys():
		if nodes[k].distance_to(pos) <= snap_tolerance:
			return k
	var key := Vector2i(_next_id, 0)
	_next_id += 1
	nodes[key] = pos
	edges[key] = []
	return key


## True if the node existed before this side-street walk began (degree > 0
## from prior connections). Used to detect snapping onto existing fabric.
func _is_existing_node(key: Vector2i) -> bool:
	return edges.has(key) and edges[key].size() > 0


func _degree(key: Vector2i) -> int:
	if not edges.has(key):
		return 0
	return edges[key].size()


## Nearest node world position to `pos` excluding the node `exclude_key`.
func _nearest_other_node(pos: Vector2, exclude_key: Vector2i) -> Vector2:
	var best: Vector2 = Vector2.ZERO
	var best_d: float = INF
	for k in nodes.keys():
		if k == exclude_key:
			continue
		var d: float = nodes[k].distance_to(pos)
		if d < best_d:
			best_d = d
			best = nodes[k]
	return best


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


## World Euclidean distance override (the base uses Manhattan on Vector2i
## keys, which is meaningless for sequential-id keys). NOTE: Step 2 will
## unify the base signature to float; for now we match the base int
## signature and cast.
func far_from(key: Vector2i, min_distance: int) -> Array:
	var out: Array = []
	if not nodes.has(key):
		return out
	var ref_pos: Vector2 = nodes[key]
	var min_d: float = float(min_distance)
	for k in nodes.keys():
		if nodes[k].distance_to(ref_pos) >= min_d:
			out.append(k)
	return out
