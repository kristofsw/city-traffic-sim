extends GutTest
## Unit tests for RoadGraph and A* pathfinding.


func _build_test_graph() -> RoadGraph:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(400, 400)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	return graph


func test_build_copies_data() -> void:
	var gen := GridGenerator.new()
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	assert_eq(
		graph.nodes.size(), gen.nodes.size(), "graph should have same node count as generator"
	)
	assert_eq(
		graph.edges.size(), gen.edges.size(), "graph should have same edge count as generator"
	)


func test_find_path_same_start_goal() -> void:
	var graph := _build_test_graph()
	var path := graph.find_path(Vector2i(0, 0), Vector2i(0, 0))
	assert_eq(path.size(), 1, "path to same node should be [start]")
	assert_eq(path[0], Vector2i(0, 0), "path[0] should be start")


func test_find_path_unknown_nodes() -> void:
	var graph := _build_test_graph()
	var path := graph.find_path(Vector2i(-1, -1), Vector2i(0, 0))
	assert_eq(path.size(), 0, "path from unknown node should be empty")
	var path2 := graph.find_path(Vector2i(0, 0), Vector2i(99, 99))
	assert_eq(path2.size(), 0, "path to unknown node should be empty")


func test_find_path_optimal() -> void:
	var graph := _build_test_graph()
	var path := graph.find_path(Vector2i(0, 0), Vector2i(2, 2))
	assert_gt(path.size(), 0, "path should exist on connected grid")
	# Manhattan distance from (0,0) to (2,2) is 4; path length in hops = 4+1=5
	assert_eq(path.size(), 5, "optimal path should have 5 nodes (4 hops + start)")
	assert_eq(path[0], Vector2i(0, 0), "path should start at start")
	assert_eq(path[path.size() - 1], Vector2i(2, 2), "path should end at goal")


func test_path_contiguity() -> void:
	var graph := _build_test_graph()
	var path := graph.find_path(Vector2i(0, 0), Vector2i(2, 2))
	for i in range(path.size() - 1):
		var neighbors: Array[Vector2i] = graph.neighbors_of(path[i])
		assert_true(
			neighbors.has(path[i + 1]), "path[%d] should be adjacent to path[%d]" % [i, i + 1]
		)


func test_heuristic_is_manhattan() -> void:
	var graph := _build_test_graph()
	# Use nodes that exist in the 3x3 grid: (0,0) and (2,1)
	var h := graph.heuristic(Vector2i(0, 0), Vector2i(2, 1))
	var pa := graph.world_of(Vector2i(0, 0))
	var pb := graph.world_of(Vector2i(2, 1))
	var expected: float = abs(pa.x - pb.x) + abs(pa.y - pb.y)
	assert_almost_eq(h, expected, 0.01, "heuristic should be Manhattan distance in world coords")


func test_edge_cost_is_euclidean() -> void:
	var graph := _build_test_graph()
	var cost := graph.edge_cost(Vector2i(0, 0), Vector2i(1, 0))
	var a := graph.world_of(Vector2i(0, 0))
	var b := graph.world_of(Vector2i(1, 0))
	assert_almost_eq(cost, a.distance_to(b), 0.01, "edge_cost should be euclidean distance")
