extends Node2D


const SPEED = 300.0

# delare main body
@onready var body:CharacterBody2D = $spider_body

# stuff for animation
@onready var comfort_positions:Array[Vector2] = [%target_0.position]
@onready var targets:Array[IKTargetMarker] = [%target_0]

@onready var leg:IKLeg = %ik_leg

func _ready() -> void:
	# collect comfort positions from scene
	pass

func target_position_from_body_position(body_position:Vector2, idx:int) -> Vector2:
	return body_position + comfort_positions[idx]

func _physics_process(delta: float) -> void:

	# body movement
	var direction := Input.get_axis("left", "right")
	if direction:
		body.velocity.x = direction * SPEED
	else:
		body.velocity.x = move_toward(body.velocity.x, 0, SPEED)

	body.move_and_slide()
	
