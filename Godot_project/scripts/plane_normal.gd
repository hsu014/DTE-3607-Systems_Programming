extends CollisionShape

# You can tweak this in the inspector if your plane is tilted (e.g., to lay flat)
export(Vector3) var rotation_offset_degrees = Vector3(0, 0, 0)

func _ready():
	if !(shape is PlaneShape):
		push_error("This script only works with PlaneShape!")
		set_physics_process(false)

func _physics_process(delta):
	var plane_shape = shape as PlaneShape

	# ✅ Global rotation of this node
	var basis = global_transform.basis

	# Convert degrees to radians for rotation offset
	var offset_euler = rotation_offset_degrees * deg2rad(1.0)

	# Convert to quaternion and then to basis
	var offset_quat = Quat(offset_euler)
	var offset_basis = Basis(offset_quat)

	# Apply rotation offset after global transform
	var final_basis = basis * offset_basis

	# Rotate the base normal (Vector3.UP)
	var rotated_normal = final_basis.xform(Vector3.UP).normalized()

	# Keep original plane distance from origin
	var distance = plane_shape.plane.d

	# Set the rotated plane
	plane_shape.plane = Plane(rotated_normal, distance)
