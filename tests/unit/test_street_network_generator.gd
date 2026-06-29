extends GutTest
## Unit tests for StreetNetworkGenerator (organic street network).


func _build_gen() -> StreetNetworkGenerator:
	var gen := StreetNetworkGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.avenue_count = 5
	gen.side_street_density = 0.5
	gen.angle_jitter = 0.35
	gen.snap_tolerance = 24.0
	gen.rng.seed = 42
	return gen


func test_generate_populates_nodes_and_edges() -> void:
	var gen := _build_gen()
	gen.generate()
	assert_gt(gen.nodes.size(), 0, "generate should populate nodes")
	assert_gt(gen.edges.size(), 0, "generate should populate edges")


func test_graph_is_connected() -> void:
	var gen := _build_gen()
	gen.generate()
	# BFS from any node; all nodes must be reachable.
	var keys: Array = gen.nodes.keys()
	assert_gt(keys.size(), 0, "should have nodes")
	var start: Vector2i = keys[0]
	var visited: Dictionary = {start: true}
	var queue: Array[Vector2i] = [start]
	while not queue.is_empty():
		var cur: Vector2i = queue.pop_front()
		for n in gen.edges[cur]:
			if not visited.has(n):
				visited[n] = true
				queue.append(n)
	assert_eq(visited.size(), gen.nodes.size(), "all nodes should be reachable (graph connected)")


func test_boundary_non_empty_and_near_edge() -> void:
	var gen := _build_gen()
	gen.generate()
	var boundary := gen.boundary_nodes()
	assert_gt(boundary.size(), 0, "boundary should be non-empty")
	var edge_dist: float = gen.margin_px + gen.road_width
	for k in boundary:
		var p: Vector2 = gen.nodes[k]
		var near: bool = (
			p.x <= edge_dist
			or p.x >= gen.screen_size.x - edge_dist
			or p.y <= edge_dist
			or p.y >= gen.screen_size.y - edge_dist
		)
		assert_true(near, "boundary node %s should be near a screen edge" % k)


func test_far_from_uses_world_distance() -> void:
	var gen := _build_gen()
	gen.generate()
	# Pick a node; far_from should return only nodes >= min_distance in
	# world Euclidean distance (not Manhattan-on-keys).
	var keys: Array = gen.nodes.keys()
	var key: Vector2i = keys[0]
	var ref_pos: Vector2 = gen.nodes[key]
	# Use an int here; Step 2 unifies far_from on float world distance.
	var far: Array = gen.far_from(key, 400)
	for k in far:
		var d: float = gen.nodes[k].distance_to(ref_pos)
		assert_gte(d, 400.0, "far_from node should be >= 400px in world distance")


func test_no_dangling_edges() -> void:
	var gen := _build_gen()
	gen.generate()
	for key in gen.edges:
		for n in gen.edges[key]:
			assert_true(
				gen.nodes.has(n),
				"edge target %s of %s should still exist (no dangling edges)" % [n, key]
			)


func test_no_dead_ends_min_degree_2() -> void:
	var gen := _build_gen()
	gen.generate()
	# Every node should have degree >= 2 (no dead-ends: side streets must
	# snap to another road before terminating).
	for key in gen.edges:
		assert_gte(gen.edges[key].size(), 2, "node %s should have degree >= 2 (no dead-ends)" % key)


func test_has_junctions_degree_3_or_more() -> void:
	var gen := _build_gen()
	gen.generate()
	# The network should have at least some T-junctions (degree 3) or
	# 4-way intersections (degree 4) -- this is the whole point of the
	# organic network vs the uniform grid.
	var junctions: int = 0
	for key in gen.edges:
		if gen.edges[key].size() >= 3:
			junctions += 1
	assert_gt(junctions, 0, "should have at least one junction (degree >= 3)")


func test_reproducibility_same_seed_same_graph() -> void:
	var gen1 := _build_gen()
	gen1.generate()
	var gen2 := _build_gen()
	gen2.generate()
	assert_eq(gen1.nodes.size(), gen2.nodes.size(), "same seed -> same node count")
	assert_eq(gen1.edges.size(), gen2.edges.size(), "same seed -> same edge count")
