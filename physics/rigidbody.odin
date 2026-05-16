package physics

import "core:math"
import "core:math/linalg"
import rl "vendor:raylib"
import "core:fmt"

DEBUG_STUFF :: true

rb_add_force :: proc(rb: ^Rigidbody, force: Vec3) {
	rb.force_accum += force
	if DEBUG_STUFF {
		rl.DrawLine3D(rb.position, rb.position + force, rl.Color{255, 0, 0, 255})
	}
}

rb_add_acceleration :: proc(rb: ^Rigidbody, accel: Vec3) {
	if rb.inverse_mass == 0 {return}
	rb_add_force(rb, accel / rb.inverse_mass)
}

rb_add_torque :: proc(rb: ^Rigidbody, torque: Vec3) {
	rb.torque_accum += torque
}

rb_add_force_at_point :: proc(rb: ^Rigidbody, force: Vec3, point: Vec3) {
	rb.force_accum += force

	to_point := point - rb.position
	rb.torque_accum += linalg.cross(to_point, force)
}

rb_clear_accumulators :: proc(rb: ^Rigidbody) {
	rb.force_accum  = {}
	rb.torque_accum = {}
}

rb_integrate :: proc(rb: ^Rigidbody, dt: f32) {
	rb.acceleration_last_frame = rb.acceleration
	rb.acceleration = {} // NOt sure.
	rb.acceleration += rb.force_accum * rb.inverse_mass
	rb.force_accum = {}

	angular_acceleration := rb._inverse_inertia_tensor * rb.torque_accum
	rb.torque_accum = {}

	rb.velocity += rb.acceleration_last_frame * dt
	rb.angular_velocity += angular_acceleration * dt

	// Impose drag
	rb.velocity *= linalg.pow(rb.linear_damping, dt)
	rb.angular_velocity *= linalg.pow(rb.angular_damping, dt)

	rb.position += rb.velocity * dt
	rb.rotation = quat_rotate_by_axis(rb.rotation, rb.angular_velocity * dt)

	// Impose drag a second time
	rb.velocity *= linalg.pow(rb.linear_damping, dt)
	rb.angular_velocity *= linalg.pow(rb.angular_damping, dt)

	rb_recompute_derived(rb)
}

INERTIA_TENSOR_DEFAULT := linalg.inverse(inertia_tensor_box({1, 1, 1 }, 1))
INERTIA_TENSOR_INFINITE_MASS : Mat3 = 0
INERTIA_TENSOR_IDENTITY      : Mat3 = 1 // nice

rigidbody :: proc(
	position             := Vec3{0, 0, 0},
	rotation             := linalg.QUATERNIONF32_IDENTITY,
	inverse_mass         := f32(1),
	inverse_inertia_tensor_local := INERTIA_TENSOR_DEFAULT, // TODO: compute this
) -> Rigidbody {
	return {
		position             = position,
		rotation             = rotation,
		inverse_inertia_tensor_local = inverse_inertia_tensor_local,
		inverse_mass         = inverse_mass,
		linear_damping       = 1, //0.995,
		angular_damping      = 1, //0.995,
	}
}

// TODO: consider doing this lazily
rb_recompute_derived :: proc(rb : ^Rigidbody) {
	rot_mat := linalg.matrix3_from_quaternion(rb.rotation)
	rb._transform = linalg.matrix4_from_matrix3(rot_mat)
	rb._transform[3].xyz = rb.position

	rb._transform_inverse = linalg.matrix4_inverse(rb._transform)

	// The implementation in the book was optimized with a code generator.
	// I've done the change of basis explicitly. 
	// TODO: consider optimizing once we've gotten it working
	// TODO: get it working

	rot_mat_inverse := linalg.transpose(rot_mat)
	middle := rb.inverse_inertia_tensor_local * rot_mat_inverse
	rb._inverse_inertia_tensor = rot_mat * middle
}

rb_world_to_local_pos :: proc(rigidbody: ^Rigidbody, world: Vec3) -> Vec3 {
	return mat4_mul_vec3(rigidbody._transform_inverse, world)
}

rb_local_to_world_pos :: proc(rigidbody: ^Rigidbody, local: Vec3) -> Vec3 {
	return mat4_mul_vec3(rigidbody._transform, local)
}

velocity_as_quat :: proc(rb: ^Rigidbody) -> Quat {
	q := linalg.QUATERNIONF32_IDENTITY
	return quat_rotate_by_axis(q, rb.angular_velocity)
}

rb_get_velocity_at_point :: proc(rigidbody: ^Rigidbody, point: Vec3) -> Vec3 {
	// By crossing the relative position with the angular velocity, we
	// can actually get the linear velocity at a particular point on the rigidbody
	// caused by said angular velocity! Not obvious, but makes sense if you think about it
	// TODO: validate
	to_point := point - rigidbody.position
	velocity_from_rotation := linalg.cross(rigidbody.angular_velocity, to_point)

	return velocity_from_rotation + rigidbody.velocity
}
