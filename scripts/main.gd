class_name Main
extends Node2D
## Entry-point node for the simulation. Owns the RoadGrid and
## SimulationManager children and wires them together via dependency
## injection. Per the Godot scene-organization best practice, siblings
## should not reference each other directly; the ancestor (this node)
## mediates. This removes the hardcoded \$"../RoadGrid" string path from
## SimulationManager and the manual road_grid._ready() lifecycle call.
##
## Child _ready() order is not guaranteed in Godot, so Main injects the
## road_grid reference in its own _ready (which runs after all children
## are ready), letting each child manage its own lifecycle naturally.

@onready var road_grid: RoadGrid = $RoadGrid
@onready var simulation_manager: SimulationManager = $SimulationManager


func _ready() -> void:
	simulation_manager.road_grid = road_grid
