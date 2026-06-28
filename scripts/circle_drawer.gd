class_name CircleDrawer
extends Node2D
## Minimal leaf node that draws a single filled circle. Used by VehicleBody
## for headlights, taillights and turn indicators. Keeping lights as nodes
## (rather than procedural draw_circle calls inside a monolithic _draw)
## makes them individually addressable -- their color, size and visibility
## can be driven directly by mover signals, and they can be swapped for
## Sprite2D / PointLight2D later without touching the body script.

var radius: float = 2.0
var color: Color = Color.WHITE


func set_circle(r: float, c: Color) -> void:
	radius = r
	color = c
	queue_redraw()


func _draw() -> void:
	draw_circle(Vector2.ZERO, radius, color)
