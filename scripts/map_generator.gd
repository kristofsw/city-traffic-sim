class_name MapGenerator
extends Resource
## Contract for a procedural map generator. Subclasses produce a road
## network (intersection world positions keyed by Vector2i + adjacency)
## consumable by RoadGraph and the simulation.
##
## Extends Resource (not RefCounted) so concrete subclasses -- GridGenerator,
## and future HexGenerator / RadialGenerator / OSM-import generators -- can
## be saved as .tres presets and dropped onto RoadGrid's `map_generator`
## export. This is the seam for expandable map generation: new topologies
## plug in without touching RoadGraph, RoadGrid or SimulationManager, which
## all depend on this interface rather than the concrete GridGenerator.
##
## Per the Godot node-alternatives best practice, Resource is the right base
## for Inspector-configurable data on a lightweight object.

var nodes: Dictionary = {}  # Vector2i -> Vector2 world position
var edges: Dictionary = {}  # Vector2i -> Array[Vector2i] (adjacency)


## Build the node/edge network into `nodes` and `edges`. Override in
## subclasses. The base implementation pushes an error -- MapGenerator is
## purely a contract.
func generate() -> void:
	push_error("MapGenerator.generate() must be overridden by a subclass")


## Perimeter / spawn-suitable nodes. Override in subclasses whose topology
## has a meaningful boundary. The base returns empty.
func boundary_nodes() -> Array[Vector2i]:
	push_error("MapGenerator.boundary_nodes() must be overridden by a subclass")
	return []


## All node keys. Default works for any generator using Vector2i keys.
func all_nodes() -> Array:
	return nodes.keys()


## Nodes at least `min_distance` (Manhattan, on Vector2i keys) from `key`.
## Default works for any grid using Vector2i keys; subclasses with
## non-gridded topologies should override with an appropriate metric.
func far_from(key: Vector2i, min_distance: int) -> Array:
	var out: Array = []
	for k in nodes.keys():
		if abs(k.x - key.x) + abs(k.y - key.y) >= min_distance:
			out.append(k)
	return out
