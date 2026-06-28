class_name VehicleController
extends Node2D
## Thin orchestrator that wires a VehicleMover (pure motion model) to a
## VehicleBody (composed visual scene). The controller is a Node2D so it
## lives in the SceneTree and can be parented by SimulationManager; all
## motion logic lives in the mover (RefCounted) and all rendering lives in
## the body (Node2D with child nodes). Per Godot best practices this is
## single-responsibility: the controller only owns lifecycle + wiring.
##
## Backward-compat: exposes `assign_path`, `is_busy`, `graph`, `path` and
## the `arrived` signal so SimulationManager and tests keep working. The
## motion tuning exports remain here for the inspector; they are copied
## into the mover on _ready and on change (Step 3 will replace this with a
## VehicleSpec Resource).

signal arrived

@export var max_speed: float = 80.0
@export var accel_rate: float = 90.0
@export var decel_rate: float = 130.0
@export var decel_distance: float = 120.0
@export var turn_slowdown_factor: float = 0.5
@export var min_turn_speed_ratio: float = 0.25
@export var turn_look_ahead: float = 60.0
@export var snap_distance: float = 5.0
@export var snap_speed_threshold: float = 15.0
@export var lane_offset: float = 12.0  # half-lane, right-hand drive
@export var turn_radius: float = 22.0  # px; pull-back before intersection for arc
@export var debug_lane: bool = false

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var mover: VehicleMover = null
var body: VehicleBody = null

# Cached aliases for tests that still read these through the controller.
var trajectory: Trajectory = null
var s: float = 0.0
var current_speed: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO
var heading: float = 0.0


func _ready() -> void:
	_ensure_mover()
	_ensure_body()
	# Wire mover -> controller transform + aliases.
	mover.position_changed.connect(_on_mover_position_changed)
	mover.speed_changed.connect(_on_mover_speed_changed)
	mover.arrived.connect(_on_mover_arrived)


func _process(delta: float) -> void:
	mover.update(delta)
	# Keep the blink phase on the body in sync with the mover so indicators
	# blink at the motion cadence.
	if body:
		body.set_blink_phase(mover._blink_phase)


func assign_path(new_path: Array[Vector2i]) -> void:
	path = new_path
	mover.assign_path(new_path, lane_offset, turn_radius)
	trajectory = mover.trajectory
	s = mover.s
	position_on_road = mover.position_on_road
	heading = mover.heading


func is_busy() -> bool:
	return mover.is_busy()


# ---------------------------------------------------------------------------
# Wiring helpers
# ---------------------------------------------------------------------------


func _ensure_mover() -> void:
	if mover == null:
		mover = VehicleMover.new()
		mover.graph = graph
		mover.max_speed = max_speed
		mover.accel_rate = accel_rate
		mover.decel_rate = decel_rate
		mover.decel_distance = decel_distance
		mover.turn_slowdown_factor = turn_slowdown_factor
		mover.min_turn_speed_ratio = min_turn_speed_ratio
		mover.turn_look_ahead = turn_look_ahead
		mover.snap_distance = snap_distance
		mover.snap_speed_threshold = snap_speed_threshold


func _ensure_body() -> void:
	if body == null:
		body = $Body
	if body:
		body.bind_mover(mover)


# ---------------------------------------------------------------------------
# Mover signal handlers
# ---------------------------------------------------------------------------


func _on_mover_position_changed(pos: Vector2, head: float) -> void:
	position_on_road = pos
	heading = head
	s = mover.s
	# The controller node is the positioned/rotated root; body + lights are
	# children with local offsets only.
	global_position = pos
	rotation = head
	if debug_lane:
		print("[Vehicle] v=%.1f s=%.1f pos=%s" % [mover.current_speed, mover.s, pos])


func _on_mover_speed_changed(speed: float) -> void:
	current_speed = speed


func _on_mover_arrived() -> void:
	arrived.emit()
