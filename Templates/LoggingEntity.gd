extends Node2D
class_name LoggingEntity

signal log_message(msg:String)

func send_log(msg:String):
	log_message.emit(msg)
