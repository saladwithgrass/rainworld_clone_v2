extends PinJoint2D

@onready var joints:Array = [self, %joint1, %joint2, %end]
@onready var links:Array[RigidBody2D] = [%link1, %link2, %link3]

@export var max_velocities:Array[float] = [PI/2, PI/2, PI/2]

func disable_gravity(node:Node2D):
	if node is RigidBody2D:
		node.gravity_scale = 0
		node.freeze = true
	for child in node.get_children():
		disable_gravity(child)

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	disable_gravity(self)

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func get_joint_angles():
	var result = []
	result.resize(len(links))
	for idx in range(len(links)):
		result[idx] = links[idx].rotation
	return result

func forward_kinematics(angles:Array):
	var main_transform = Transform2D(0, Vector2.ZERO)
	for idx in len(angles):
		var cur_rotation = angles[idx]
		var cur_translation = joints[idx].position
		main_transform *= Transform2D(cur_rotation, cur_translation)
	return main_transform * joints[len(angles)].position

func error(current_position:Vector2, target_position:Vector2):
	return (target_position - current_position).length_squared()

func error_gradient(cur_angles:Array, target_position:Vector2, delta_step:float = 0.005):
	var cur_position = forward_kinematics(cur_angles)
	var gradient = []
	gradient.resize(len(cur_angles))
	gradient.fill(0)
	var cur_error = error(cur_position, target_position)
	for idx in len(cur_angles):
		cur_angles[idx] += delta_step
		gradient[idx] = \
		(error(forward_kinematics(cur_angles), target_position) - cur_error) / delta_step
		cur_angles[idx] -= delta_step
	return gradient

func inverse_kinematics(target_pos:Vector2, epsilon:float = 0.1):
	const max_iterations = 1024
	var learning_rate = 0.00005
	var cur_angles = get_joint_angles()
	for iter in range(max_iterations):
		if error(forward_kinematics(cur_angles), target_pos) <= epsilon:
			break
		var grad = error_gradient(cur_angles, target_pos)
		for i in len(grad):
			cur_angles[i] -= learning_rate * grad[i]
		
	return cur_angles

func ik_with_error(target_pos:Vector2, epsilon:float = 0.1):
	var ik_angles = inverse_kinematics(target_pos, epsilon)
	var ik_error = error(forward_kinematics(ik_angles), target_pos)
	return [ik_angles, ik_error]

func set_joints(angles:Array):
	for idx in range(len(angles)):
		links[idx].rotation = angles[idx]

func get_end_position():
	return to_local(%end.global_position)
