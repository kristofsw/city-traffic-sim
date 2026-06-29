extends GutTest
## Unit tests for the MapGenerator seam. Verifies that GridGenerator
## satisfies the base contract and that RoadGraph.build / the simulation
## helpers work through the MapGenerator interface, not the concrete type.

## A minimal custom MapGenerator subclass with a tiny 2x2 topology, to prove
## the seam accepts new generators without touching RoadGraph/SimulationManager.
const CustomGen = preload("res://tests/unit/test_map_generator_custom.gd")


func test_grid_generator_is_map_generator() -> void:
	var gen := GridGenerator.new()
	assert_true(gen is MapGenerator, "GridGenerator should be a MapGenerator")


func test_grid_generator_satisfies_contract() -> void:
	var gen := GridGenerator.new()
	gen.screen_size = Vector2(1280, 720)
	gen.margin_px = 40.0
	gen.target_block_size = 128.0
	gen.obstacle_count = 0  # predictable full grid for contract test
	gen.generate()
	# Contract: nodes and edges populated.
	assert_gt(gen.nodes.size(), 0, "generate() should populate nodes")
	assert_gt(gen.edges.size(), 0, "generate() should populate edges")
	# Contract: all_nodes returns the keys.
	assert_eq(gen.all_nodes().size(), gen.nodes.size(), "all_nodes should return all keys")
	# Contract: boundary_nodes returns a non-empty perimeter.
	assert_gt(gen.boundary_nodes().size(), 0, "boundary_nodes should return perimeter")
	# Contract: far_from honors min_distance.
	var far := gen.far_from(Vector2i(0, 0), 6)
	for k in far:
		var d: int = abs(k.x) + abs(k.y)
		assert_gte(d, 6, "far_from node should be at least min_distance away")


func test_road_graph_build_accepts_map_generator() -> void:
	# RoadGraph.build is typed MapGenerator; pass a GridGenerator (subclass)
	# through the base type to verify the seam.
	var gen: MapGenerator = GridGenerator.new()
	(gen as GridGenerator).obstacle_count = 0  # predictable full grid
	gen.generate()
	var graph := RoadGraph.new()
	graph.build(gen)
	assert_eq(graph.nodes.size(), gen.nodes.size(), "graph should copy generator nodes")
	assert_eq(graph.edges.size(), gen.edges.size(), "graph should copy generator edges")
	# Pathfinding works through the base-typed graph.
	var p := graph.find_path(Vector2i(0, 0), Vector2i(2, 2))
	assert_gt(p.size(), 0, "pathfinding should work through MapGenerator-typed graph")


func test_custom_subclass_works_through_seam() -> void:
	var gen: MapGenerator = CustomGen.new()
	gen.generate()
	assert_eq(gen.nodes.size(), 4, "custom 2x2 generator should have 4 nodes")
	assert_eq(gen.boundary_nodes().size(), 4, "custom generator boundary should be all 4 nodes")
	var graph := RoadGraph.new()
	graph.build(gen)
	# All four nodes are mutually reachable via the square edges.
	var p := graph.find_path(Vector2i(0, 0), Vector2i(1, 1))
	assert_gte(p.size(), 3, "custom generator should pathfind across the square")
	# far_from uses the base implementation (Manhattan on Vector2i keys).
	var far := gen.far_from(Vector2i(0, 0), 2)
	assert_eq(far.size(), 1, "only (1,1) is >=2 Manhattan from (0,0) in a 2x2 grid")
	assert_eq(far[0], Vector2i(1, 1), "far node should be (1,1)")
