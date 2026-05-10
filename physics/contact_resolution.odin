package physics

import "core:math"
import "core:math/linalg"

/*
Usage:
	// game update code can generate contacts
	{
		// add forces
		// collide certain colliders and objects as needed
	}

	// user to manually integrate each rigidbody. They can decide where they are stored.
	{
		for rb in rigidbody {
			physics.rb_integrate(&rb)
		}
	}

	physics.resolve_contacts(world)

The physics system then has a very immediate-mode feel to it, rather than
a create_collider() / destroy_collider() pair style API.
*/

resolve_contacts :: proc(world: ^World) {
	for i in 0..<world.contact_idx {
		resolve_contact(&world.contacts[i])
	}
}

resolve_contact :: proc(contact: ^Contact) {
	assert(contact.rigidbody != nil)

	// TODO: validate - shouldn't it be world->contact ??
	contact_to_world := make_orthonormal_basis(contact.normal)
	world_to_contact := linalg.transpose(contact_to_world)

	// resolving impulses
	{
		get_velocity_per_unit_impulse_contact :: proc(contact: ^Contact, rb: ^Rigidbody, world_to_contact: Mat3) -> f32 {
			velocity_per_unit_impulse_contact: f32

			// Rotational component
			{
				relative_contact_position := contact.position - rb.position
				torque_per_unit_impulse   := linalg.cross(relative_contact_position, contact.normal)
				rotation_per_unit_impulse := rb._inverse_inertia_tensor * torque_per_unit_impulse
				velocity_per_unit_impulse := linalg.cross(rotation_per_unit_impulse, relative_contact_position)

				// > It is better, in my opinion, to think in terms of the change of coordinates, because
				// > as we introduce friction in the next chapter, the simple scalar product trick can no
				// > longer be used.
				velocity_per_unit_impulse_contact := world_to_contact * rotation_per_unit_impulse
			}

			// linear component
			{
				// xd
				velocity_per_unit_impulse_contact += rb.inverse_mass
			}

			return velocity_per_unit_impulse_contact
		}


		delta_velocity := get_velocity_per_unit_impulse_contact(contact, contact.rigidbody, world_to_contact)
		if contact.other_rigidbody != nil {
			// TODO: validate. I think its wrong to add here, and we should be substracting.
			delta_velocity += get_velocity_per_unit_impulse_contact(contact, contact.other_rigidbody, world_to_contact)
		}

		get_required_impulse :: proc(
			contact: ^Contact,
			rb: ^Rigidbody,
			delta_velocity: f32,
			world_to_contact: Mat3,
			contact_to_world: Mat3,
		) -> (Vec3, Vec3) {
			relative_position := contact.position - rb.position
			velocity := linalg.cross(rb.angular_velocity, relative_position) +
				contact.rigidbody.velocity
				
			contact_velocity       := world_to_contact * velocity
			restitution            := f32(1) // TODO: make this settable. 'valid' values are between 0 and 1, possibly inclusive, but we'll allow anything
			desired_delta_velocity := -contact_velocity.x * (1 + restitution)

			contact_impulse   := Vec3{}
			contact_impulse.x = desired_delta_velocity / delta_velocity

			impulse          := contact_to_world * contact_impulse
			impulsive_torque := linalg.cross(impulse, relative_position)

			return impulse, impulsive_torque
		}

		impulse, impulsive_torque := get_required_impulse(contact, contact.rigidbody, delta_velocity, world_to_contact, contact_to_world)
		velocity_change         := contact.rigidbody.inverse_mass * impulse
		angular_velocity_change := contact.rigidbody._inverse_inertia_tensor * impulsive_torque
		// TODO: validate. no dt ??? HOW? 
		contact.rigidbody.velocity         += velocity_change
		contact.rigidbody.angular_velocity += angular_velocity_change

		if contact.other_rigidbody != nil {
			impulse, impulsive_torque := get_required_impulse(contact, contact.other_rigidbody, delta_velocity, world_to_contact, contact_to_world)
			velocity_change         := contact.other_rigidbody.inverse_mass * -impulse
			angular_velocity_change := contact.other_rigidbody._inverse_inertia_tensor * -impulsive_torque
			contact.other_rigidbody.velocity         += velocity_change
			contact.other_rigidbody.angular_velocity += angular_velocity_change
		}
	}

	// resolve interpenetration

	get_inertias :: proc(contact: ^Contact, rb: ^Rigidbody) -> (f32, f32) {
		relative_contact_position := contact.position - rb.position
		angular_inertia_world := linalg.cross(relative_contact_position, contact.normal)
		angular_inertia_world = rb._inverse_inertia_tensor * angular_inertia_world
		angular_inertia_world = linalg.cross(angular_inertia_world, relative_contact_position)
		angular_inertia := linalg.dot(angular_inertia_world, contact.normal)

		linear_inertia := rb.inverse_mass

		return linear_inertia, angular_inertia
	}

	apply_angular_move :: proc(contact: ^Contact, rb: ^Rigidbody, angular_inertia, angular_move: f32) {
		relative_contact_position := contact.position - rb.position
		impulsive_torque := linalg.cross(relative_contact_position, contact.normal)
		impulse_per_move := rb._inverse_inertia_tensor * impulsive_torque
		rotation_per_move := impulse_per_move / angular_inertia
		rotation := angular_move * rotation_per_move
		rb.rotation = quat_rotate_by_axis(rb.rotation, rotation)
	}

	linear_inertia, angular_inertia: f32
	linear_inertia_other, angular_inertia_other: f32

	linear_inertia, angular_inertia = get_inertias(contact, contact.rigidbody)
	total_inertia := linear_inertia + angular_inertia

	if contact.other_rigidbody != nil {
		linear_inertia_other, angular_inertia_other = get_inertias(contact, contact.rigidbody)
		total_inertia += linear_inertia_other + angular_inertia_other
	}

	get_linear_angular_moves :: proc(
		contact: ^Contact,
		linear_inertia, angular_inertia, total_inertia: f32
	) -> (linear_move, angular_move: f32) {
		// Dont let angular move get too large - we may over-rotate the object if this happens.
		// I was thinking hey why not just limit rotation such that the surface normal of the nearest surface
		// matches the contact normal, but sounds like a bunch of work so I haven't done that yet.
		// The book suggests to just use a threhsold that we can tweak as needed
		angular_limit_constraint := f32(0.5)

		linear_move  = contact.penetration * linear_inertia / total_inertia
		angular_move = contact.penetration * angular_inertia / total_inertia

		if abs(angular_move) > angular_limit_constraint {
			total_move := linear_move + angular_move

			if angular_move > 0 {
				angular_move = angular_limit_constraint
			} else {
				angular_move = -angular_limit_constraint
			}

			linear_move = total_move - angular_move
		}

		return
	}

	linear_move, angular_move := get_linear_angular_moves(contact, linear_inertia, angular_inertia, total_inertia)
	contact.rigidbody.position += linear_move * contact.normal
	apply_angular_move(contact, contact.rigidbody, angular_inertia, angular_move)

	if contact.other_rigidbody != nil {
		linear_move_other, angular_move_other := get_linear_angular_moves(contact, linear_inertia_other, angular_inertia_other, total_inertia)
		contact.other_rigidbody.position -= linear_move_other * contact.normal
		apply_angular_move(contact, contact.other_rigidbody, angular_inertia_other, angular_move_other)
	}
}

