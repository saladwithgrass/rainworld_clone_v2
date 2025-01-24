extends Control
class_name CommandLine

var history:Array = []
const log_limit = 100
var history_pointer = 0


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	%cmd_input.grab_focus()

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("up"):
		if -history_pointer == len(history):
			return
		history_pointer -= 1
		set_cmd(history[history_pointer])

func set_cmd(text:String):
	%cmd_input.text = text
	print(%cmd_input.caret_column)

func _on_cmd_input_text_submitted(new_text: String) -> void:
	print('command entered')
	history.append(new_text)
	add_log('> ' + new_text)
	parse_cmd(new_text)
	%cmd_input.clear()
	history_pointer = 0

func add_log(text:String, color:Color=Color.WHITE):
	if history.size() >= log_limit:
		history.remove_at(0)
		%log_contatiner.get_children()[0].queue_free()
	var new_log_entry = Label.new()
	new_log_entry.text = text
	new_log_entry.label_settings = LabelSettings.new()
	new_log_entry.label_settings.font_color = color
	%log_contatiner.add_child(new_log_entry)

func add_error(text:String):
	add_log('    ERROR:' + text, Color.RED)

func add_success(text:String):
	add_log('    ' + text, Color.GREEN)

func str_list_to_float_list(str_list:Array):
	var args = []
	for arg in str_list:
		if arg.is_valid_float():
			args.append(deg_to_rad(arg.to_float()))
		else:
			add_error('Ivalid argument *%s*. Could not be converted to float.' % arg)
			return null
	return args

func str_list_to_vec(str_list:Array):
	var result = null
	if len(str_list) == 2:
		result = Vector2.ZERO
	elif len(str_list) == 3:
		result = Vector3.ZERO
	else:
		add_error('Could not convert string list to vector. Too many elements.')
		return null
	for idx in len(str_list):
		if str_list[idx].is_valid_float:
			result[idx] = str_list[idx].to_float()
		else:
			add_error('Could not convert string list to vector. \
			Argument *%s* is not valid float.' % str_list[idx])
			return null
	return result

signal forward_kinematics(angles:Array)
signal end_position()
signal set_joints(angles:Array)
signal inverse_kinematics(pos:Vector2)

var cmd_to_func = {
	"fk" : parse_fk,
	"ep" : parse_end_point,
	"end_point" : parse_end_point,
	"sj" : parse_set_joints,
	"set_joints" : parse_set_joints,
	"ik" : parse_ik,
	"eval" : parse_eval,
	"help" : parse_help
}

func parse_help(str_args:Array):
	var available_cmd = PackedStringArray(cmd_to_func.keys())
	add_success('Available commands: \n        ' + "\n        ".join(available_cmd))


func parse_fk(str_args:Array):
	# gather angles
	if len(str_args) == 0:
		add_error('Arguments expected')
		return
	var args = str_list_to_float_list(str_args)
	if args == null:
		return
	forward_kinematics.emit(args)

func parse_end_point(str_args:Array):
	if len(str_args) != 0:
		add_error('end_point does not take arguments')
	end_position.emit()

func parse_set_joints(str_args:Array):
	if len(str_args) == 0:
		add_error('Arguments expected')
		return
	var args = str_list_to_float_list(str_args)
	if args == null:
		return
	set_joints.emit(args)

func parse_ik(str_args:Array):
	if len(str_args) != 2:
		add_error('IK: Expected 2 arguments. %d gotten.' % len(str_args))
		return
	var args = str_list_to_vec(str_args)
	if args == null:
		return
	inverse_kinematics.emit(args)

func parse_eval(str_args:Array):
	# join everything back together
	var expr = ""
	for arg in str_args:
		expr += arg + " "
	var res_expression = Expression.new()
	var error = res_expression.parse(expr)
	if error != OK:
		add_error('Could not parse expression: ' + str(error))
		return
	var result = res_expression.execute()
	if not res_expression.has_execute_failed():
		add_success(str(result))
	else:
		add_error('Execution failed.')

func parse_cmd(text:String):
	text = text.lstrip(' ').rstrip(' ')
	var cmd_split = text.split(' ')
	var cmd = cmd_split[0]
	cmd_split.remove_at(0)
	
	# forward kinematics command
	if cmd in cmd_to_func.keys():
		cmd_to_func[cmd].call(cmd_split)
	else:
		add_error("unknown command")

func _on_visibility_changed() -> void:
	if self.visible:
		%cmd_input.grab_focus()
