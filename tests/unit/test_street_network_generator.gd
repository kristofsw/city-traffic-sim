extends GutTest
## Unit tests for StreetNetworkGenerator (variable grid + superblock tiling
## for varied block sizes + T-junctions + 45° diagonals).


func _build_gen() -> StreetNetworkGenerator:
	var gen := StreetNetworkGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.block_jitter = 0.25
	gen.diagonal_count = 2
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
	var keys: Array = gen.nodes.keys()
	var key: Vector2i = keys[0]
	var ref_pos: Vector2 = gen.nodes[key]
	var far: Array = gen.far_from(key, 400.0)
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
	for key in gen.edges:
		assert_gte(gen.edges[key].size(), 2, "node %s should have degree >= 2 (no dead-ends)" % key)


func test_has_t_junctions_degree_3() -> void:
	var gen := _build_gen()
	gen.generate()
	var t_junctions: int = 0
	for key in gen.edges:
		if gen.edges[key].size() == 3:
			t_junctions += 1
	assert_gt(
		t_junctions, 0, "should have at least one T-junction (degree 3) from superblock boundaries"
	)


func test_has_superblock_gaps() -> void:
	# Superblock tiling should leave at least one pair of adjacent cells
	# with no edge between them (the interior of a 2x1/1x2/2x2 block).
	# Disable diagonals to isolate the grid for a clean check.
	var gen := _build_gen()
	gen.diagonal_count = 0
	gen.generate()
	var has_gap: bool = false
	for r in range(gen.rows):
		for c in range(gen.cols):
			var key := Vector2i(c, r)
			if not gen.nodes.has(key):
				continue
			# Horizontal neighbor (c+1, r): edge should exist for a 1x1
			# boundary but be missing if both cells are in the same block.
			if c + 1 < gen.cols and gen.nodes.has(Vector2i(c + 1, r)):
				if not gen.edges[key].has(Vector2i(c + 1, r)):
					has_gap = true
			# Vertical neighbor (c, r+1).
			if r + 1 < gen.rows and gen.nodes.has(Vector2i(c, r + 1)):
				if not gen.edges[key].has(Vector2i(c, r + 1)):
					has_gap = true
	assert_true(
		has_gap, "superblock tiling should leave at least one gap (interior of a superblock)"
	)


func test_has_diagonal_edges() -> void:
	var gen := _build_gen()
	gen.generate()
	# At least one edge should be non-axis-aligned (a 45° diagonal segment).
	var has_diagonal: bool = false
	for key in gen.edges:
		var from: Vector2 = gen.nodes[key]
		for n in gen.edges[key]:
			if key < n:
				var to: Vector2 = gen.nodes[n]
				var dx: float = abs(to.x - from.x)
				var dy: float = abs(to.y - from.y)
				# Axis-aligned edges have one dimension ~0; diagonals have both.
				if dx > 1.0 and dy > 1.0:
					has_diagonal = true
	assert_true(has_diagonal, "should have at least one non-axis-aligned (diagonal) edge")


func test_aligned_grid_shares_positions() -> void:
	# Grid nodes (key.x < _DIAG_KEY_OFFSET) should share column x-positions
	# and row y-positions: all nodes with the same c have the same x.
	var gen := _build_gen()
	gen.diagonal_count = 0  # isolate grid for this check
	gen.generate()
	var col_x: Dictionary = {}  # c -> x
	var row_y: Dictionary = {}  # r -> y
	for k in gen.nodes.keys():
		var key: Vector2i = k
		if key.x >= gen._DIAG_KEY_OFFSET:
			continue  # skip diagonal nodes
		var p: Vector2 = gen.nodes[k]
		if col_x.has(key.x):
			assert_almost_eq(
				p.x, col_x[key.x], 0.01, "column %d x should be shared across rows" % key.x
			)
		else:
			col_x[key.x] = p.x
		if row_y.has(key.y):
			assert_almost_eq(
				p.y, row_y[key.y], 0.01, "row %d y should be shared across columns" % key.y
			)
		else:
			row_y[key.y] = p.y


func test_reproducibility_same_seed_same_graph() -> void:
	var gen1 := _build_gen()
	gen1.generate()
	var gen2 := _build_gen()
	gen2.generate()
	assert_eq(gen1.nodes.size(), gen2.nodes.size(), "same seed -> same node count")
	assert_eq(gen1.edges.size(), gen2.edges.size(), "same seed -> same edge count")
