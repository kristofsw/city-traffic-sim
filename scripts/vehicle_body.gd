class_name VehicleBody
extends Node2D
## Pure renderer for a vehicle. Composed of child nodes (body polygon,
## headlights, taillights, turn indicators) whose properties are driven by
## the VehicleMover's state via signals. This is the reusable visual half
## of the car: any node that owns a VehicleMover and wires its signals to a
## VehicleBody gets correct brake lights, headlights and turn indicators
## without re-implementing any motion logic.
##
## The body never reads the trajectory or computes speed -- it only reacts
## to mover signals (braking_changed, turn_indicator_changed) and the blink
## phase. All visual config (colors, sizes, body dimensions) comes from a
## VehicleSpec Resource injected via apply_spec. Per Godot best practices,
## the scene is self-contained: all visual assets are child nodes, no
## external dependencies, no hardcoded paths.

# Visual spec (injected by the controller). When null, falls back to a
# default VehicleSpec so the body renders even without explicit injection.
var spec: VehicleSpec = null

# Current visual state (driven by mover signals).
var _braking: float = 0.0
var _turn_dir: int = 0
var _blink_phase: float = 0.0
var _blink_period: float = 0.4  # updated from spec via apply_spec

# Mover reference (optional): when set, the body drives its own blink phase
# from the mover's so indicators blink in sync with the motion update.
var _mover: VehicleMover = null
var _default_spec: VehicleSpec = null

# Child nodes (assigned in _ready; the .tscn defines them).
@onready var _body: Polygon2D = $BodyShape
@onready var _headlight_l: CircleDrawer = $HeadlightL
@onready var _headlight_r: CircleDrawer = $HeadlightR
@onready var _taillight_l: CircleDrawer = $TaillightL
@onready var _taillight_r: CircleDrawer = $TaillightR
@onready var _indicator_fl: CircleDrawer = $IndicatorFL
@onready var _indicator_fr: CircleDrawer = $IndicatorFR
@onready var _indicator_rl: CircleDrawer = $IndicatorRL
@onready var _indicator_rr: CircleDrawer = $IndicatorRR


func _ready() -> void:
	_ensure_spec()
	_build_body_polygon()
	_layout_lights()
	_apply_colors()
	_set_indicators_visible(false)


## Apply a VehicleSpec to this body. The controller calls this on _ready.
func apply_spec(p_spec: VehicleSpec) -> void:
	spec = p_spec
	_blink_period = p_spec.indicator_blink_period
	# Rebuild visuals if already in the tree (child nodes available).
	if _body != null:
		_build_body_polygon()
		_layout_lights()
		_apply_colors()


## Bind this body to a mover. The body subscribes to the mover's signals
## and drives its child nodes accordingly. Call once after the body is in
## the tree and the mover is owned by the controller.
func bind_mover(mover: VehicleMover) -> void:
	_mover = mover
	mover.braking_changed.connect(_on_braking_changed)
	mover.turn_indicator_changed.connect(_on_turn_indicator_changed)
	# Initialize visuals from current mover state.
	_on_braking_changed(0.0)
	_on_turn_indicator_changed(0)


func set_blink_phase(phase: float) -> void:
	_blink_phase = phase
	_update_indicators()


func _on_braking_changed(intensity: float) -> void:
	_braking = intensity
	var s: VehicleSpec = _ensure_spec()
	var radius: float = s.taillight_base_radius + intensity * s.taillight_brake_radius_gain
	var alpha: float = s.taillight_base_alpha + intensity * (1.0 - s.taillight_base_alpha)
	var col := Color(s.taillight_color.r, s.taillight_color.g, s.taillight_color.b, alpha)
	_taillight_l.set_circle(radius, col)
	_taillight_r.set_circle(radius, col)


func _on_turn_indicator_changed(direction: int) -> void:
	_turn_dir = direction
	_update_indicators()


func _update_indicators() -> void:
	if _turn_dir == 0:
		_set_indicators_visible(false)
		return
	var on: bool = fmod(_blink_phase, _blink_period) < _blink_period * 0.5
	_set_indicators_visible(false)
	if on:
		if _turn_dir > 0:  # right turn -> right-side corners
			_indicator_fr.visible = true
			_indicator_rr.visible = true
		else:  # left turn -> left-side corners
			_indicator_fl.visible = true
			_indicator_rl.visible = true


func _set_indicators_visible(vis: bool) -> void:
	_indicator_fl.visible = vis
	_indicator_fr.visible = vis
	_indicator_rl.visible = vis
	_indicator_rr.visible = vis


func _ensure_spec() -> VehicleSpec:
	if spec != null:
		return spec
	if _default_spec == null:
		_default_spec = VehicleSpec.new()
		_blink_period = _default_spec.indicator_blink_period
	return _default_spec


func _build_body_polygon() -> void:
	var s: VehicleSpec = _ensure_spec()
	var w: float = s.body_width * 0.5
	var l: float = s.body_length * 0.5
	_body.polygon = PackedVector2Array(
		[Vector2(-l, -w), Vector2(l, -w), Vector2(l, w), Vector2(-l, w)]
	)
	_body.color = s.body_color


func _layout_lights() -> void:
	var s: VehicleSpec = _ensure_spec()
	var l: float = s.body_length * 0.5
	var w: float = s.body_width * 0.3
	# Front edge headlights.
	_headlight_l.position = Vector2(l, -w)
	_headlight_r.position = Vector2(l, w)
	# Rear edge taillights.
	_taillight_l.position = Vector2(-l, -w)
	_taillight_r.position = Vector2(-l, w)
	# Corner indicators.
	var hw: float = s.body_width * 0.5
	_indicator_fl.position = Vector2(l, -hw)
	_indicator_fr.position = Vector2(l, hw)
	_indicator_rl.position = Vector2(-l, -hw)
	_indicator_rr.position = Vector2(-l, hw)


func _apply_colors() -> void:
	var s: VehicleSpec = _ensure_spec()
	_headlight_l.set_circle(s.headlight_radius, s.headlight_color)
	_headlight_r.set_circle(s.headlight_radius, s.headlight_color)
	_indicator_fl.set_circle(s.indicator_radius, s.indicator_color)
	_indicator_fr.set_circle(s.indicator_radius, s.indicator_color)
	_indicator_rl.set_circle(s.indicator_radius, s.indicator_color)
	_indicator_rr.set_circle(s.indicator_radius, s.indicator_color)
	# Taillights set by _on_braking_changed; initialize at rest.
	_on_braking_changed(0.0)
