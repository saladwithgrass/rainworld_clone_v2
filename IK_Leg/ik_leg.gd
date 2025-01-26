extends PinJoint2D
class_name IKLeg

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
	return (target_position - current_position).length()

func error_gradient(cur_angles:Array, target_position:Vector2, delta_step:float = 0.005):
	# get position in current configuration
	var cur_position = forward_kinematics(cur_angles)
	
	# get error in current configuration
	var cur_error = error(cur_position, target_position)
	
	# initialize gradient
	var gradient = []
	gradient.resize(len(cur_angles))
	gradient.fill(0)
	
	# setup variables for calculations
	var new_position:Vector2
	var new_error:float
	
	# get derivative for each joint
	for idx in len(cur_angles):
		# modify element
		cur_angles[idx] += delta_step
		
		# get new position
		new_position = forward_kinematics(cur_angles)
		
		# get new error
		new_error = error(new_position, target_position)
		
		# calculate i-th derivative
		gradient[idx] = (new_error - cur_error) / delta_step
		
		# revert changes to original vector
		cur_angles[idx] -= delta_step
		
	return gradient

func inverse_kinematics(target_pos:Vector2, epsilon:float = 3):
	
	# initialize variables
	const max_iterations = 1024
	
	var cur_angles = get_joint_angles()
	var cur_error = error(
		forward_kinematics(cur_angles),
		target_pos
		)
	
	# if gradient norm is less than grad_delta, then break
	var gradient_delta = 0.1
	
	var iteration_count = 0
	var learning_rate
	var grad:Array = []
	var grad_norm = 0
	var cur_percent
	while cur_error >= epsilon and iteration_count < max_iterations:
		# set learning rate 
		cur_percent = (max_iterations - iteration_count + 1.) / (max_iterations)
		learning_rate = pow(cur_percent, 2) / 4900
		# print(cur_error, ' ', epsilon, ' ', cur_error > epsilon)
		
		# get gradient
		grad = error_gradient(cur_angles, target_pos, learning_rate)
		
		# move in the direction of gradient
		for i in len(grad):
			grad_norm += grad[i]*grad[i]
			cur_angles[i] -= learning_rate * grad[i]
		
		if grad_norm < gradient_delta:
			break
		
		# update error
		cur_error = error(
			forward_kinematics(cur_angles),
			target_pos
		)

		# update counter
		iteration_count += 1
		set_joints(cur_angles)
		# await get_tree().create_timer(0.05).timeout
		
	# report results
	# print('IK finished in %d iterations with error %f;' % [iteration_count, cur_error])
	
	return cur_angles

func inverse_kinematics_global(target_pos_global:Vector2, epsilon:float=3):
	return inverse_kinematics(to_local(target_pos_global), epsilon)

func ik_with_error(target_pos:Vector2):
	var ik_angles = await inverse_kinematics(target_pos)
	var ik_error = error(forward_kinematics(ik_angles), target_pos)
	return [ik_angles, ik_error]

func set_joints(angles:Array):
	for idx in range(len(angles)):
		links[idx].rotation = angles[idx]

func get_end_position():
	return to_local(%end.global_position)
	
func get_end_global_position():
	return %end.global_position
