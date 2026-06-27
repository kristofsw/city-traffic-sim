class_name VehicleController
extends Node2D
## Drives a vehicle along a path of grid intersections. Maintains a
## right-hand lane offset, smooth heading rotation at turns, and reports
## arrival so the manager can assign a new path.

signal arrived

const BODY_COLOR := Color(0.42, 0.45, 0.50, 1)        # #6b7280 muted gray-blue
const HEADLIGHT_COLOR := Color(1.0, 0.96, 0.84, 1)    # #fff4d6
const TAILLIGHT_COLOR := Color(1.0, 0.35, 0.30, 1)    # #ff5a4d

@export var speed: float = 120.0            # px / sec
@export var rotation_speed: float = 6.0     # rad / sec (lerp factor)
@export var lane_offset: float = 12.0       # half-lane, right-hand drive
@export var arrival_radius: float = 6.0     # px tolerance for "reached node"

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var path_index: int = 0
var heading: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO  # actual world pos of the Node2D

func _ready() -> void:
	position = position_on_road

func assign_path(new_path: Array[Vector2i]) -> void:
	path = new_path
	path_index = 0
	if path.size() >= 1:
		position_on_road = graph.world_of(path[0])
		position = position_on_road
	if path.size() >= 2:
		heading = _direction_to(path[0], path[1]).angle()

func _process(delta: float) -> void:
	if graph == null or path.size() < 2:
		return
	# Target world pos for the current segment end, offset to right-hand lane.
	var from_key: Vector2i = path[path_index]
	var to_key: Vector2i = path[path_index + 1]
	var from_world: Vector2 = graph.world_of(from_key)
	var to_world: Vector2 = graph.world_of(to_key)
	var seg_dir: Vector2 = (to_world - from_world).normalized()
	# Right-hand perpendicular (in screen space, y-down): rotate seg_dir by -90deg
	var perp: Vector2 = Vector2(seg_dir.y, -seg_dir.x)
	var target: Vector2 = to_world + perp * lane_offset
	# Move toward target.
	var to_target: Vector2 = target - position_on_road
	var dist: float = to_target.length()
	if dist <= arrival_radius:
		# Reached this intersection; advance.
		path_index += 1
		if path_index >= path.size() - 1:
			# Final node reached.
			position_on_road = target
			position = position_on_road
			arrived.emit()
			return
		# Continue toward next segment; the next iteration will recompute target.
		return
	var step: float = min(speed * delta, dist)
	position_on_road += to_target.normalized() * step
	position = position_on_road
	# Smoothly rotate heading toward segment direction.
	var desired: float = seg_dir.angle()
	heading = lerp_angle(heading, desired, clamp(rotation_speed * delta, 0.0, 1.0))
	queue_redraw()

func _draw() -> void:
	# Body: rounded rectangle approximation via two rects (simple). Length ~36, width ~18.
	var length: float = 36.0
	var width: float = 18.0
	var body_rect := Rect2(-length * 0.5, -width * 0.5, length, width)
	# We rotate the canvas by heading for body orientation.
	var prev_transform := get_canvas_transform()
	# Drawing happens in local space already; we just rotate the rect by heading.
	var cos_h: float = cos(heading)
	var sin_h: float = sin(heading)
	var corners := PackedVector2Array([
		Vector2(-length * 0.5, -width * 0.5),
		Vector2( length * 0.5, -width * 0.5),
		Vector2( length * 0.5,  width * 0.5),
		Vector2(-length * 0.5,  width * 0.5),
	])
	var rotated := PackedVector2Array()
	for c in corners:
		rotated.append(Vector2(c.x * cos_h - c.y * sin_h, c.x * sin_h + c.y * cos_h))
	draw_colored_polygon(rotated, BODY_COLOR)
	# Headlights: small dots at the front edge.
	var front_local := Vector2(length * 0.5, 0.0)
	var front_world := Vector2(front_local.x * cos_h - front_local.y * sin_h, front_local.x * sin_h + front_local.y * cos_h)
	draw_circle(front_world + Vector2(0, -width * 0.3).rotated(heading), 2.2, HEADLIGHT_COLOR)
	draw_circle(front_world + Vector2(0,  width * 0.3).rotated(heading), 2.2, HEADLIGHT_COLOR)
	# Taillights: small dots at the rear edge.
	var rear_local := Vector2(-length * 0.5, 0.0)
	var rear_world := Vector2(rear_local.x * cos_h - rear_local.y * sin_h, rear_local.x * sin_h + rear_local.y * cos_h)
	draw_circle(rear_world + Vector2(0, -width * 0.3).rotated(heading), 1.8, TAILLIGHT_COLOR)
	draw_circle(rear_world + Vector2(0,  width * 0.3).rotated(heading), 1.8, TAILLIGHT_COLOR)

func _direction_to(a: Vector2i, b: Vector2i) -> Vector2:
	return (graph.world_of(b) - graph.world_of(a)).normalized()

func is_busy() -> bool:
	return path.size() >= 2 and path_index < path.size() - 1