class_name VehicleController
extends Node2D
## Thin orchestrator that wires a VehicleMover (pure motion model) to a
## VehicleBody (composed visual scene). The controller is a Node2D so it
## lives in the SceneTree and can be parented by SimulationManager; all
## motion logic lives in the mover (RefCounted) and all rendering lives in
## the body (Node2D with child nodes). Per Godot best practices this is
## single-responsibility: the controller only owns lifecycle + wiring.
##
## All tuning (motion + visual) lives in a single VehicleSpec Resource on
## the `spec` export. Drop a sedan.tres / truck.tres / bus.tres here to
## swap vehicle types without touching code. If no spec is assigned, a
## default is created at runtime.
##
## Backward-compat: exposes `assign_path`, `is_busy`, `graph`, `path` and
## the `arrived` signal so SimulationManager keeps working.

signal arrived

@export var spec: VehicleSpec = null

var graph: RoadGraph = null
var path: Array[Vector2i] = []
var mover: VehicleMover = null
var body: VehicleBody = null

# Cached aliases for tests / SimulationManager that read through the controller.
var trajectory: Trajectory = null
var s: float = 0.0
var current_speed: float = 0.0
var position_on_road: Vector2 = Vector2.ZERO
var heading: float = 0.0

var _debug_lane: bool = false


func _get_configuration_warnings() -> PackedStringArray:
	if spec == null:
		return PackedStringArray(
			["spec is unassigned — a default VehicleSpec will be created at runtime"]
		)
	return PackedStringArray()


func _ready() -> void:
	_ensure_spec()
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
	var p_spec: VehicleSpec = _ensure_spec()
	mover.assign_path(new_path, p_spec.lane_offset, p_spec.turn_radius)
	trajectory = mover.trajectory
	s = mover.s
	position_on_road = mover.position_on_road
	heading = mover.heading


func is_busy() -> bool:
	return mover.is_busy()


func set_debug_lane(enabled: bool) -> void:
	_debug_lane = enabled


# ---------------------------------------------------------------------------
# Wiring helpers
# ---------------------------------------------------------------------------


## Ensure a spec exists (use the @export one, else create a default).
func _ensure_spec() -> VehicleSpec:
	if spec == null:
		spec = VehicleSpec.new()
	return spec


func _ensure_mover() -> void:
	if mover == null:
		mover = VehicleMover.new()
		mover.graph = graph
		mover.apply_spec(_ensure_spec())


func _ensure_body() -> void:
	if body == null:
		body = $Body
	if body:
		body.apply_spec(_ensure_spec())
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
	if _debug_lane:
		print("[Vehicle] v=%.1f s=%.1f pos=%s" % [mover.current_speed, mover.s, pos])


func _on_mover_speed_changed(speed: float) -> void:
	current_speed = speed


func _on_mover_arrived() -> void:
	arrived.emit()
