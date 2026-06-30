class_name VehicleSpec
extends Resource
## Inspector-configurable specification for a vehicle type. Holds all
## motion tuning AND visual config in one Resource so vehicle types are
## swappable without touching code -- create a `sedan.tres`, `truck.tres`,
## `bus.tres` and drop them into the controller's `spec` export.
##
## Per the Godot "node alternatives" best practice, `Resource` is the right
## base for Inspector-configurable data on a lightweight object: it is
## RefCounted (no SceneTree needed), shows its @export fields in the
## Inspector, and can be saved as a .tres file for shareable presets.
##
## The controller injects the spec into both the VehicleMover (motion
## fields) and the VehicleBody (visual fields) via their `apply_spec`
## methods. Defaults are the "final feel" tuning previously scattered
## between vehicle_controller.gd script defaults and the vehicle.tscn
## overrides (now unified here -- the scene-vs-script mismatch is gone).

# --- Motion tuning ---
@export var max_speed: float = 80.0  # px/s (cruising speed)
@export var accel_rate: float = 50.0  # px/s^2 (acceleration, gentle)
@export var decel_rate: float = 120.0  # px/s^2 (braking, gentle)
@export var decel_distance: float = 90.0  # px before destination to start braking
@export var turn_slowdown_factor: float = 0.5  # speed reduction per radian of turn
@export var min_turn_speed_ratio: float = 0.3  # never slower than this fraction in a turn
@export var turn_look_ahead: float = 50.0  # px; look ahead for upcoming turns
@export var snap_distance: float = 5.0  # px; snap to arrival when this close
@export var snap_speed_threshold: float = 15.0  # px/s; below this, snap to arrival

# --- Adaptive Cruise Control (ACC) ---
## Desired following time gap to the vehicle ahead (seconds). The look-ahead
## distance scales with speed: gap_target = current_speed * follow_time_gap
## + follow_min_gap. A shorter gap = more aggressive tailgating.
@export var follow_time_gap: float = 1.5  # seconds
## Minimum following distance kept even at standstill (px, bumper-to-bumper).
## Below this the car holds still (target drops to 0).
@export var follow_min_gap: float = 40.0  # px

# --- Lane / trajectory ---
@export var lane_offset: float = 12.0  # half-lane, right-hand drive
@export var turn_radius: float = 22.0  # px; pull-back before intersection for arc

# --- Body dimensions ---
@export var body_length: float = 36.0
@export var body_width: float = 18.0

# --- Colors ---
@export var body_color: Color = Color(0.42, 0.45, 0.50, 1)  # #6b7280 muted gray-blue
@export var headlight_color: Color = Color(1.0, 0.96, 0.84, 1)  # #fff4d6
@export var taillight_color: Color = Color(1.0, 0.35, 0.30, 1)  # #ff5a4d
@export var indicator_color: Color = Color(1.0, 0.6, 0.15, 1)  # #ff9926 amber

# --- Light geometry ---
@export var headlight_radius: float = 2.2
@export var taillight_base_radius: float = 1.8  # resting radius (grows with braking)
@export var taillight_brake_radius_gain: float = 1.5  # radius added at full brake
@export var taillight_base_alpha: float = 0.35  # resting taillight visibility
@export var indicator_radius: float = 2.5

# --- Turn signal blink ---
@export var indicator_blink_period: float = 0.4  # seconds (0.2s on / 0.2s off)
