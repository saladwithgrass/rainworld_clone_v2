extends Control

@export var player_entity:Node2D

const PLAYER_NULL_MSG = 'Call failed, player is null'

var window_layers = []

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func _input(event: InputEvent) -> void:
	# toggle command line check
	if event.is_action_released("command_line"):
		if $cmd.visible:
			window_layers.pop_back()
			$cmd.visible = false
		else:
			window_layers.push_back($cmd)
			$cmd.visible = true		
	# exit check
	if event.is_action_released("exit"):
		if len(window_layers) == 0:
			get_tree().quit()
		else:
			window_layers[-1].visible = false
			window_layers.pop_back()

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func report_error(msg:String):
	$cmd.add_error(msg)
	
func report_success(msg:String):
	$cmd.add_success(msg)

func player_is_null(report:bool = true):
	if player_entity == null:
		report_error(PLAYER_NULL_MSG)
		return true
	return false

func _on_cmd_forward_kinematics(angles: Array) -> void:
	if player_is_null():
		return
	if player_entity.has_method('forward_kinematics'):
		var fk_result = player_entity.forward_kinematics(angles)
		report_success('FK result: %v' % fk_result)

func _on_cmd_end_position() -> void:
	if player_is_null():
		return
	if player_entity.has_method('get_end_position'):
		report_success('End position: %v' % player_entity.get_end_position())

func _on_cmd_set_joints(angles: Array) -> void:
	if player_is_null():
		return
	if player_entity.has_method('set_joints'):
		player_entity.set_joints(angles)


func _on_cmd_inverse_kinematics(pos: Vector2) -> void:
	if player_is_null():
		return
	if player_entity.has_method('ik_with_error'):
		var ik_with_error = await player_entity.ik_with_error(pos)
		var flattened = ik_with_error[0]
		for i in len(flattened):
			flattened[i] = rad_to_deg(flattened[i])
		flattened.append(ik_with_error[1])
		report_success('IK angles: %f %f %f with error %f' % flattened)


func _on_cmd_step_trajectory(start_pos: Vector2, end_pos: Vector2) -> void:
	if player_is_null():
		return
	if player_entity.has_method("step_trajectory"):
		await player_entity.step_trajectory(start_pos, end_pos)
