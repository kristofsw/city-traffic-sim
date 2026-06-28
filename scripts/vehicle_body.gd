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
## to mover signals (position_changed, braking_changed,
## turn_indicator_changed) and the blink phase. Per Godot best practices,
## the scene is self-contained: all visual assets are child nodes, no
## external dependencies, no hardcoded paths.

const BODY_COLOR := Color(0.42, 0.45, 0.50, 1)  # #6b7280 muted gray-blue
const HEADLIGHT_COLOR := Color(1.0, 0.96, 0.84, 1)  # #fff4d6
const TAILLIGHT_COLOR := Color(1.0, 0.35, 0.30, 1)  # #ff5a4d
const INDICATOR_COLOR := Color(1.0, 0.6, 0.15, 1)  # #ff9926 amber
const TAILLIGHT_BASE_ALPHA := 0.35  # resting taillight visibility
const TAILLIGHT_BRAKE_RADIUS := 1.8  # base radius (grows with braking)
const TAILLIGHT_BRAKE_RADIUS_GAIN := 1.5  # radius added at full brake
const INDICATOR_RADIUS := 2.5
const HEADLIGHT_RADIUS := 2.2

# Body dimensions. Step 3 moves these into a VehicleSpec Resource.
var body_length: float = 36.0
var body_width: float = 18.0

# Current visual state (driven by mover signals).
var _braking: float = 0.0
var _turn_dir: int = 0
var _blink_phase: float = 0.0
var _blink_period: float = 0.4  # must match VehicleMover.INDICATOR_BLINK_PERIOD

# Mover reference (optional): when set, the body drives its own blink phase
# from the mover's so indicators blink in sync with the motion update.
var _mover: VehicleMover = null

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
	_build_body_polygon()
	_layout_lights()
	_apply_colors()
	_set_indicators_visible(false)


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
	var radius: float = TAILLIGHT_BRAKE_RADIUS + intensity * TAILLIGHT_BRAKE_RADIUS_GAIN
	var alpha: float = TAILLIGHT_BASE_ALPHA + intensity * (1.0 - TAILLIGHT_BASE_ALPHA)
	var col := Color(TAILLIGHT_COLOR.r, TAILLIGHT_COLOR.g, TAILLIGHT_COLOR.b, alpha)
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


func _build_body_polygon() -> void:
	var w: float = body_width * 0.5
	var l: float = body_length * 0.5
	_body.polygon = PackedVector2Array(
		[Vector2(-l, -w), Vector2(l, -w), Vector2(l, w), Vector2(-l, w)]
	)
	_body.color = BODY_COLOR


func _layout_lights() -> void:
	var l: float = body_length * 0.5
	var w: float = body_width * 0.3
	# Front edge headlights.
	_headlight_l.position = Vector2(l, -w)
	_headlight_r.position = Vector2(l, w)
	# Rear edge taillights.
	_taillight_l.position = Vector2(-l, -w)
	_taillight_r.position = Vector2(-l, w)
	# Corner indicators.
	var hw: float = body_width * 0.5
	_indicator_fl.position = Vector2(l, -hw)
	_indicator_fr.position = Vector2(l, hw)
	_indicator_rl.position = Vector2(-l, -hw)
	_indicator_rr.position = Vector2(-l, hw)


func _apply_colors() -> void:
	_headlight_l.set_circle(HEADLIGHT_RADIUS, HEADLIGHT_COLOR)
	_headlight_r.set_circle(HEADLIGHT_RADIUS, HEADLIGHT_COLOR)
	_indicator_fl.set_circle(INDICATOR_RADIUS, INDICATOR_COLOR)
	_indicator_fr.set_circle(INDICATOR_RADIUS, INDICATOR_COLOR)
	_indicator_rl.set_circle(INDICATOR_RADIUS, INDICATOR_COLOR)
	_indicator_rr.set_circle(INDICATOR_RADIUS, INDICATOR_COLOR)
	# Taillights set by _on_braking_changed; initialize at rest.
	_on_braking_changed(0.0)
