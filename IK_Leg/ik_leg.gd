extends PinJoint2D
class_name IKLeg

@onready var joints:Array = [self, %joint1, %joint2, %end]
@onready var links:Array[RigidBody2D] = [%link1, %link2, %link3]

@export var max_velocities:Array[float] = [PI/2, PI/2, PI/2]

@export var comfort_angles:Array[float] = [
	deg_to_rad(-20),
	deg_to_rad(45),
	deg_to_rad(45)
	]

var MAX_DISTANCE

#region -------------ANIMATION PARAMETERS------------------- 
var is_in_animation:bool = false
var current_animation:Array[Array] = []
var cur_animation_step:int = 0
var animation_framerate:float = 10 / 0.05

#region --------------KEEP IN PLACE VARIABLES-------------------
var is_anchored:bool = true
var anchor_position:Vector2 = Vector2.ZERO

func disable_gravity(node:Node2D):
	if node is RigidBody2D:
		node.gravity_scale = 0
		node.freeze = true
	for child in node.get_children():
		if child is Node2D:
			disable_gravity(child)

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	disable_gravity(self)
	MAX_DISTANCE = forward_kinematics([0, 0, 0]).length() * 2
	anchor_position = get_end_global_position()

#region -----------------KINEMATICS----------------
func get_joint_angles():
	var result = []
	result.resize(len(links))
	for idx in range(len(links)):
		result[idx] = links[idx].rotation
	return result

func forward_kinematics(angles:Array):
	var main_transform = Transform2D(0, Vector2.ZERO)
	var cur_rotation:float
	var cur_translation:Vector2
	for idx in len(angles):
		cur_rotation = angles[idx]
		if idx != 0:
			cur_translation = joints[idx].position
		else:
			cur_translation = Vector2.ZERO
		main_transform *= Transform2D(cur_rotation, cur_translation)
	return main_transform * joints[len(angles)].position

func error(current_angles:Array, target_position:Vector2):
	# also error should be normalized
	# error incorporates both how close to the target
	# and how comfortable the position is
	var comfort_loss = 0
	const max_diff_squared = PI**2
	for idx in len(current_angles):
		var differnece_squared = (comfort_angles[idx] - current_angles[idx]) ** 2
		comfort_loss += differnece_squared/max_diff_squared
	
	var position_loss = forward_kinematics(current_angles)
	position_loss -= target_position
	position_loss = position_loss.length()
	position_loss /= MAX_DISTANCE
	return position_loss + 0.05 * comfort_loss

func error_gradient(cur_angles:Array, target_position:Vector2, delta_step:float = 0.001):
	# get position in current configuration
	
	# get error in current configuration
	var cur_error = error(cur_angles, target_position)
	
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
		
		# get new error
		new_error = error(cur_angles, target_position)
		
		# calculate i-th derivative
		gradient[idx] = (new_error - cur_error) / delta_step
		
		# revert changes to original vector
		cur_angles[idx] -= delta_step
		
	return gradient

func inverse_kinematics(target_pos:Vector2, epsilon:float = 0.005):
	
	# initialize variables
	const max_iterations = 1024
	
	var cur_angles = get_joint_angles()
	var cur_error = error(
		cur_angles,
		target_pos
		)
	
	# if gradient norm is less than grad_delta, then break
	var gradient_delta = 0.001
	
	var iteration_count = 0
	var learning_rate
	var grad:Array = []
	var grad_norm = 0
	var cur_percent
	while cur_error >= epsilon and iteration_count < max_iterations:
		# set learning rate 
		cur_percent = (max_iterations - iteration_count + 1.) / (max_iterations)
		# print(cur_percent)
		learning_rate = 1
		# print(cur_error, ' ', epsilon, ' ', cur_error > epsilon)
		
		# get gradient
		grad = error_gradient(cur_angles, target_pos, cur_percent*cur_percent*0.005)
		
		# move in the direction of gradient
		for i in len(grad):
			# grad_norm += grad[i]*grad[i]
			# print(cur_percent*cur_percent * grad[i])
			cur_angles[i] -= cur_percent*cur_percent * grad[i]
		
		# if grad_norm < gradient_delta*gradient_delta:
		#	break
		
		# update error
		cur_error = error(
			cur_angles,
			target_pos
		)

		# update counter
		iteration_count += 1
		# set_joints(cur_angles)
		# await get_tree().create_timer(0.05).timeout
		
	# report results
	# print('IK finished in %d iterations with error %f;' % [iteration_count, cur_error])
	
	return cur_angles

func inverse_kinematics_global(target_pos_global:Vector2, epsilon:float=0.009):
	return inverse_kinematics(to_local(target_pos_global), epsilon)

func ik_with_error(target_pos:Vector2):
	var ik_angles = await inverse_kinematics(target_pos)
	var ik_error = error(ik_angles, target_pos)
	return [ik_angles, ik_error]

func keep_in_place(target_pos:Vector2):
	is_anchored = true
	anchor_position = target_pos

#region --------------STEP ANIMATION-----------------

func create_trajectory_texture(trajectory:Array[Vector2]):
	var viewport = get_viewport_rect()
	var image = Image.create(viewport.size.x, viewport.size.y, false, Image.FORMAT_RGBA8)
	for point in trajectory:
		image.set_pixelv(point, Color.MAGENTA)
	var texture = ImageTexture.create_from_image(image)
	$CanvasLayer/TextureRect.texture = texture

func step_trajectory(start_pos:Vector2, target_pos:Vector2, division_steps:int=10):

	# we consider that the two points
	# we want to traverse are diametrically
	# opposed to each other on a circle
	var center = (target_pos + start_pos) / 2
	var current_vector  = start_pos - center
	var delta_step = PI / division_steps
	var result:Array[Array] = []
	var positions:Array[Vector2] = []
	positions.resize(division_steps)
	result.resize(division_steps)
	
	var direction = sign(current_vector.angle_to(global_position - center))
	
	
	for angle_idx in range(division_steps):
		current_vector = current_vector.rotated(direction * delta_step)
		positions.append(current_vector + center)
		result[angle_idx] = inverse_kinematics_global(current_vector + center)
	create_trajectory_texture(positions)
	return result

func run_animation(trajectory:Array[Array]):
	# print('running anuimation')
	is_anchored = false
	self.current_animation = trajectory
	self.is_in_animation = true
	self.cur_animation_step = 0
	$CanvasLayer/TextureRect.visible = true
	%animation_timer.start(1 / animation_framerate)

func stop_animation():
	# print('stopping animation')
	is_anchored = true
	self.current_animation = []
	self.is_in_animation = false
	$CanvasLayer/TextureRect.visible = false
	%animation_timer.stop()

func create_and_run_step(next_global_position:Vector2):
	var new_trajectory = step_trajectory(
		get_end_global_position(),
		next_global_position
	)
	run_animation(new_trajectory)

func execute_next_animation_step():
	# anchor_position = current_animation[cur_animation_step]
	var next_angles = current_animation[cur_animation_step]
	set_joints(next_angles)
	anchor_position = to_global(forward_kinematics(next_angles))
	cur_animation_step += 1

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if is_anchored:
		set_joints(inverse_kinematics_global(anchor_position))

func _on_animation_timer_timeout() -> void:
	if cur_animation_step < len(current_animation):
		execute_next_animation_step()
	else:
		stop_animation()

func set_joints(angles:Array):
	for idx in range(len(angles)):
		links[idx].rotation = angles[idx]

func get_end_position():
	return to_local(%end.global_position)
	
func get_end_global_position():
	return %end.global_position
