// Contact resolution. The part where we iterate the contacts and apply the 
// required forces to seperate and bounce the objects

package physics

import "core:math"
import "core:math/linalg"


resolve_contact_velocities :: proc(contact: ^Contact) {
	// TODO: come back to part 14, i dont understand any the code here xdd
}

rb_add_force :: proc(rb: ^Rigidbody, force: Vec3) {
	rb.force_accum += force
}

rb_add_force_at_point :: proc(rb: ^Rigidbody, force: Vec3, point: Vec3) {
	// Very interesting that we add a force AND a torque, rather than
	// using mass+inertia tensor here to distribute between the two.
	// Maybe that is done in the resulution step. 

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
	rb.rotation = quaternion_rotate_by_axis(rb.rotation, rb.angular_velocity * dt)

	// Impose drag a second time
	rb.velocity *= linalg.pow(rb.linear_damping, dt)
	rb.angular_velocity *= linalg.pow(rb.angular_damping, dt)

	rb_recompute_derived(rb)
}
