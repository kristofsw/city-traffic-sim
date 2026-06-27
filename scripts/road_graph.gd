class_name RoadGraph
extends RefCounted

## Road graph built from a GridGenerator. Stores intersection world positions
## and adjacency, and exposes A* pathfinding over the grid.

var nodes: Dictionary = {} # Vector2i -> Vector2
var edges: Dictionary = {} # Vector2i -> Array[Vector2i]

func build(generator: GridGenerator) -> void:
	nodes = generator.nodes.duplicate(true)
	edges = generator.edges.duplicate(true)

func has_node(key: Vector2i) -> bool:
	return nodes.has(key)

func world_of(key: Vector2i) -> Vector2:
	return nodes[key]

func neighbors_of(key: Vector2i) -> Array:
	if not edges.has(key):
		return []
	return edges[key]

func edge_cost(a: Vector2i, b: Vector2i) -> float:
	return nodes[a].distance_to(nodes[b])

func heuristic(a: Vector2i, b: Vector2i) -> float:
	var pa: Vector2 = nodes[a]
	var pb: Vector2 = nodes[b]
	return abs(pa.x - pb.x) + abs(pa.y - pb.y)

## A* pathfinding. Returns an Array[Vector2i] from start to goal inclusive,
## or an empty Array if no path exists.
func find_path(start: Vector2i, goal: Vector2i) -> Array[Vector2i]:
	if not nodes.has(start) or not nodes.has(goal):
		return []
	if start == goal:
		return [start]

	var open: Array[Vector2i] = [start]
	var g_score: Dictionary = {start: 0.0}
	var f_score: Dictionary = {start: heuristic(start, goal)}
	var came_from: Dictionary = {}

	while not open.is_empty():
		# Pick the node with the lowest f_score (linear scan; grids are small).
		var current: Vector2i = open[0]
		var best_f: float = f_score[current]
		for k in open:
			var f: float = f_score[k]
			if f < best_f:
				best_f = f
				current = k

		if current == goal:
			return _reconstruct(came_from, current)

		open.erase(current)

		for n in edges[current]:
			var tentative: float = g_score[current] + edge_cost(current, n)
			if not g_score.has(n) or tentative < g_score[n]:
				came_from[n] = current
				g_score[n] = tentative
				f_score[n] = tentative + heuristic(n, goal)
				if not open.has(n):
					open.append(n)
	return []

func _reconstruct(came_from: Dictionary, current: Vector2i) -> Array[Vector2i]:
	var path: Array[Vector2i] = [current]
	while came_from.has(current):
		current = came_from[current]
		path.push_front(current)
	return path