extends Polygon2D
class_name IKTargetMarker

@export var controlled_leg:IKLeg
@export var stride_length:float = 150
var anchor_position = self.global_position

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	anchor_position = global_position
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	$prev_marker.global_position = anchor_position
	if (anchor_position - self.global_position).length_squared() >= stride_length*stride_length:
		# update previous anchor
		anchor_position = self.global_position
		controlled_leg.create_and_run_step(anchor_position)
	# controlled_leg.inverse_kinematics_global(anchor_position, 0.001)
