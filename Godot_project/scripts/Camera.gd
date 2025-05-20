extends Spatial

var camrot_h = 0
var camrot_v = 0

var cam_v_max = 180
var cam_v_min = -180

var h_sensitivity = 0.3
var v_sensitivity = 0.3

var h_acceleration = 10
var v_acceleration = 10



func _ready():
	#Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	pass
	
func _input(event):
	if event is InputEventKey:
		if event.pressed and ( (event.scancode == KEY_Q and event.control == true) or event.scancode == KEY_ESCAPE ):
			get_tree().quit()
		
	if event is InputEventMouseMotion:
		camrot_h += - event.relative.x * h_sensitivity
		camrot_v += event.relative.y * v_sensitivity
		
func _physics_process(delta):
	camrot_v = clamp(camrot_v, cam_v_min, cam_v_max)
	$H.rotation_degrees.y = lerp($H.rotation_degrees.y, camrot_h, delta * h_acceleration) 
	$H/V.rotation_degrees.x = lerp($H.rotation_degrees.x, camrot_v, delta * v_acceleration)
