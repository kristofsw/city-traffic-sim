extends GutTest
## Unit tests for GridGenerator.


func test_grid_dimensions() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	# inner_w = 1200, inner_h = 640; cols = floor(1200/128)+1 = 10, rows = floor(640/128)+1 = 6
	assert_eq(gen.cols, 10, "cols should be 10 for 1280px width with 128px blocks")
	assert_eq(gen.rows, 6, "rows should be 6 for 720px height with 128px blocks")


func test_world_pos_formula() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	var inner_w := 1280.0 - 80.0
	var inner_h := 720.0 - 80.0
	var expected_block_w := inner_w / 9.0  # cols-1 = 9
	var expected_block_h := inner_h / 5.0  # rows-1 = 5
	assert_almost_eq(gen.block_w, expected_block_w, 0.01, "block_w should fill inner width exactly")
	assert_almost_eq(
		gen.block_h, expected_block_h, 0.01, "block_h should fill inner height exactly"
	)
	var p0 := gen.world_pos(0, 0)
	assert_almost_eq(p0.x, 40.0, 0.01, "world_pos(0,0).x should be margin_px")
	assert_almost_eq(p0.y, 40.0, 0.01, "world_pos(0,0).y should be margin_px")


func test_node_count() -> void:
	var gen := GridGenerator.new()
	gen.obstacle_count = 0
	gen.generate()
	assert_eq(gen.nodes.size(), gen.cols * gen.rows, "node count should be cols * rows")


func test_edge_counts() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	# Corner node (0,0) should have 2 neighbors.
	assert_eq(gen.edges[Vector2i(0, 0)].size(), 2, "corner node should have 2 neighbors")
	# Edge node (5,0) should have 3 neighbors (not corner, on top row).
	assert_eq(gen.edges[Vector2i(5, 0)].size(), 3, "edge node should have 3 neighbors")
	# Interior node (5,3) should have 4 neighbors.
	assert_eq(gen.edges[Vector2i(5, 3)].size(), 4, "interior node should have 4 neighbors")


func test_boundary_nodes() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	var boundary := gen.boundary_nodes()
	# Boundary count = 2*cols + 2*(rows-2) = perimeter
	var expected := 2 * gen.cols + 2 * (gen.rows - 2)
	assert_eq(boundary.size(), expected, "boundary_nodes should return perimeter nodes only")
	# All boundary nodes should be on the perimeter.
	for key in boundary:
		var is_perimeter: bool = (
			key.x == 0 or key.x == gen.cols - 1 or key.y == 0 or key.y == gen.rows - 1
		)
		assert_true(is_perimeter, "boundary node %s should be on perimeter" % key)


func test_far_from() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	var far := gen.far_from(Vector2i(5, 3), 6)
	for key in far:
		var dist: int = abs(key.x - 5) + abs(key.y - 3)
		assert_gte(dist, 6, "far_from node should be at least min_distance away")


func test_single_column_grid() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(100, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0
	gen.generate()
	assert_gte(gen.cols, 1, "cols should be at least 1")
	assert_gte(gen.rows, 1, "rows should be at least 1")


func test_uniform_block_spacing_when_jitter_zero() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.block_jitter = 0.0
	gen.generate()
	# With jitter 0 every x-gap should equal block_w.
	for c in range(1, gen.cols):
		var gap: float = gen._col_x[c] - gen._col_x[c - 1]
		assert_almost_eq(gap, gen.block_w, 0.001, "jitter=0 x-gaps should equal block_w")
	for r in range(1, gen.rows):
		var gap: float = gen._row_y[r] - gen._row_y[r - 1]
		assert_almost_eq(gap, gen.block_h, 0.001, "jitter=0 y-gaps should equal block_h")


func test_non_uniform_block_spacing_when_jitter_nonzero() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.block_jitter = 0.5
	gen.rng.seed = 42
	gen.generate()
	# With jitter > 0 at least one x-gap should differ from block_w.
	var x_uniform: bool = true
	for c in range(1, gen.cols):
		var gap: float = gen._col_x[c] - gen._col_x[c - 1]
		if abs(gap - gen.block_w) > 0.01:
			x_uniform = false
	assert_false(x_uniform, "jitter=0.5 should produce non-uniform x-gaps")
	# The cumulative width must still fill the inner area exactly.
	var inner_w: float = gen.screen_size.x - 2.0 * gen.margin_px
	assert_almost_eq(gen._col_x[gen.cols - 1], inner_w, 0.5, "x offsets must fill inner_w")


func test_non_uniform_total_height_fits() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.block_jitter = 0.3
	gen.rng.seed = 7
	gen.generate()
	var inner_h: float = gen.screen_size.y - 2.0 * gen.margin_px
	assert_almost_eq(gen._row_y[gen.rows - 1], inner_h, 0.5, "y offsets must fill inner_h")


func test_obstacle_holes_remove_nodes() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 3
	gen.obstacle_radius = 2
	gen.rng.seed = 42
	gen.generate()
	var full_count: int = gen.cols * gen.rows
	assert_lt(
		gen.nodes.size(),
		full_count,
		"obstacle holes should remove nodes (got %d of %d)" % [gen.nodes.size(), full_count]
	)
	assert_gt(gen.hole_centers.size(), 0, "hole_centers should be populated")


func test_obstacle_holes_preserve_boundary() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 3
	gen.obstacle_radius = 2
	gen.rng.seed = 42
	gen.generate()
	var boundary := gen.boundary_nodes()
	for b in boundary:
		assert_true(gen.nodes.has(b), "boundary node %s should survive hole carving" % b)


func test_obstacle_holes_graph_stays_connected() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 3
	gen.obstacle_radius = 2
	gen.rng.seed = 42
	gen.generate()
	# BFS from the first surviving boundary node; every remaining node
	# must be reachable (the connectivity prune guarantees this).
	var boundary := gen.boundary_nodes()
	assert_gt(boundary.size(), 0, "should have boundary nodes")
	var start: Vector2i = boundary[0]
	var visited: Dictionary = {start: true}
	var queue: Array[Vector2i] = [start]
	while not queue.is_empty():
		var cur: Vector2i = queue.pop_front()
		for n in gen.edges[cur]:
			if not visited.has(n):
				visited[n] = true
				queue.append(n)
	assert_eq(
		visited.size(),
		gen.nodes.size(),
		"all remaining nodes should be reachable from boundary (graph connected)"
	)


func test_obstacle_holes_no_dangling_edges() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 3
	gen.obstacle_radius = 2
	gen.rng.seed = 42
	gen.generate()
	for key in gen.edges:
		for n in gen.edges[key]:
			assert_true(
				gen.nodes.has(n),
				"edge target %s of %s should still exist (no dangling edges)" % [n, key]
			)
