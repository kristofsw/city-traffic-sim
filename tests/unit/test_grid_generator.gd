extends GutTest
## Unit tests for GridGenerator.


func test_grid_dimensions() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.generate()
	# inner_w = 1200, inner_h = 640; cols = floor(1200/128)+1 = 10, rows = floor(640/128)+1 = 6
	assert_eq(gen.cols, 10, "cols should be 10 for 1280px width with 128px blocks")
	assert_eq(gen.rows, 6, "rows should be 6 for 720px height with 128px blocks")


func test_world_pos_formula() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
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
	gen.generate()
	assert_eq(gen.nodes.size(), gen.cols * gen.rows, "node count should be cols * rows")


func test_edge_counts() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
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
	gen.generate()
	assert_gte(gen.cols, 1, "cols should be at least 1")
	assert_gte(gen.rows, 1, "rows should be at least 1")
