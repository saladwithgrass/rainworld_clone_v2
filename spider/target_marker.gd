extends Polygon2D
class_name IKTargetMarker

@export var controlled_leg:IKLeg
@export var stride_length:float = 300

@onready var anchor_position = controlled_leg.get_end_global_position()

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if (anchor_position - self.global_position).length_squared() >= stride_length*stride_length:
		print('update')
		# update previous anchor
		anchor_position = self.global_position
	controlled_leg.inverse_kinematics_global(anchor_position, 1)
