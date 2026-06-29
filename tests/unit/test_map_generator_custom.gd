extends MapGenerator
## Minimal custom MapGenerator subclass used by test_map_generator.gd to prove
## the seam accepts new topologies without touching RoadGraph or
## SimulationManager. Builds a tiny 2x2 square of nodes at 100px spacing.


func generate() -> void:
	nodes.clear()
	edges.clear()
	for c in range(2):
		for r in range(2):
			var key := Vector2i(c, r)
			nodes[key] = Vector2(float(c) * 100.0, float(r) * 100.0)
			edges[key] = []
	_connect(Vector2i(0, 0), Vector2i(1, 0))
	_connect(Vector2i(1, 0), Vector2i(1, 1))
	_connect(Vector2i(1, 1), Vector2i(0, 1))
	_connect(Vector2i(0, 1), Vector2i(0, 0))


func _connect(a: Vector2i, b: Vector2i) -> void:
	edges[a].append(b)
	edges[b].append(a)


func boundary_nodes() -> Array[Vector2i]:
	# All 4 nodes are on the perimeter of a 2x2 grid.
	return nodes.keys()


## Grid-style Manhattan-on-keys override (this fixture mimics a grid, so
## the hop-distance semantics are meaningful for the contract test).
func far_from(key: Vector2i, min_distance: float) -> Array:
	var out: Array = []
	var min_int: int = int(min_distance)
	for k in nodes.keys():
		if abs(k.x - key.x) + abs(k.y - key.y) >= min_int:
			out.append(k)
	return out
