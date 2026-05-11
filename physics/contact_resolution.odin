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

resolve_contacts :: proc(world: ^World, dt: f32) {
	// prepare the contacts
	{
		for i in 0..<world.contact_idx {
			contact := &world.contacts[i]

			if contact.bodies[i] == nil {
				contact.bodies[i], contact.bodies[i + 1] = contact.bodies[i + 1], contact.bodies[i]
				contact.normal = -contact.normal
			}
			assert(contact.bodies[0] != nil)

			contact._world_from_contact = make_orthonormal_basis(contact.normal)
			contact._contact_from_world = linalg.transpose(contact._world_from_contact)
			
			// Slightly different from the book - 
			// I (mis?)understood the contact point to be on the edge of the collider,
			// not some point directly in between the overlap, which is what is needed
			// for the book's formulation here to work. No matter - I'll just compute the
			// correct contact points from each collider's POV.
			// We may need to fix this if it's not stable though.
			contact._relative_contact_positions[0] = contact.position - contact.bodies[0].position
			if contact.bodies[1] != nil {
				contact._relative_contact_positions[1] = contact.position - contact.bodies[1].position
			}

			// This part only works because we have a single contact.position in between
			// the overlap of the two rigidbodies rather than 2 contact.positions
			// on the edges of the two colliders (which is what looks right in my head 
			// but it's not how it's done in practice)
			contact._relative_velocity = get_contact_relative_velocity(contact, 0)
			if contact.bodies[1] != nil {
				contact._relative_velocity -= get_contact_relative_velocity(contact, 1)
			}

			// TODO: compute desired delta or whtaever

			get_contact_relative_velocity :: proc(contact: ^Contact, body_idx: int) -> Vec3 {
				rb := contact.bodies[body_idx]

				velocity_linear := rb.velocity

				// TODO: i forgot which way around this was supposed to be. It Does matter
				velocity_angular := linalg.cross(
					rb.angular_velocity,
					contact._relative_contact_positions[body_idx], 
				)

				total_velocity := velocity_linear + velocity_angular
				contact_total_velocity := contact._contact_from_world * total_velocity
				return contact_total_velocity
			}
		}
	}

	// Resolve penetration
	{
		last_contact : ^Contact
		POSITION_ITERATIONS :: 1000
		for i in 0..<POSITION_ITERATIONS {
			worst_contact     : ^Contact
			worst_penetration : f32
			for &contact in world.contacts[0:world.contact_idx] {
				if contact.penetration > worst_penetration {
					worst_contact     = &contact
					worst_penetration = contact.penetration
				}
			}

			if worst_contact == nil {break}

			// TODO: Match the awake state at the contact. (What is this?)

			// Resolve the penetration
			{
				apply_position_change(worst_contact)

				// Update penetrations
				{
				}
			}
		}
	}

	// -------------- old stuff (we need it)

}

apply_impulse_change :: proc(contact: ^Contact) {
	get_velocity_per_unit_impulse_contact :: proc(contact: ^Contact, body_idx: int) -> f32 {
		rb := contact.bodies[body_idx]
		relative_contact_position := contact._relative_contact_positions[body_idx]

		contact_velocity_per_unit_impulse : f32

		// Rotational component
		{
			torque_per_unit_impulse   := linalg.cross(relative_contact_position, contact.normal)
			rotation_per_unit_impulse := rb._inverse_inertia_tensor * torque_per_unit_impulse
			velocity_per_unit_impulse := linalg.cross(rotation_per_unit_impulse, relative_contact_position)

			// > It is better, in my opinion, to think in terms of the change of coordinates, because
			// > as we introduce friction in the next chapter, the simple scalar product trick can no
			// > longer be used.
			contact_velocity_per_unit_impulse = (contact._contact_from_world * rotation_per_unit_impulse).x
		}

		// linear component
		{
			// xd
			contact_velocity_per_unit_impulse += rb.inverse_mass
		}

		return contact_velocity_per_unit_impulse
	}

	delta_velocity := get_velocity_per_unit_impulse_contact(contact, 0)
	if contact.bodies[1] != nil {
		// TODO: consider if it should really be -= (the book also had += btw so I'm keeping it, though it doesnt make sense)
		delta_velocity += get_velocity_per_unit_impulse_contact(contact, 1)
	}

	apply_required_impulse :: proc(contact: ^Contact, body_idx: int, delta_velocity: f32, sign: f32) {
		rb := contact.bodies[body_idx]
		relative_position := contact._relative_contact_positions[body_idx]

		velocity := linalg.cross(rb.angular_velocity, relative_position) + rb.velocity
			
		contact_velocity       := contact._contact_from_world * velocity
		restitution            := f32(1) // TODO: make this settable. 'valid' values are between 0 and 1, possibly inclusive, but we'll allow anything
		desired_delta_velocity := -contact_velocity.x * (1 + restitution)

		contact_impulse   := Vec3{}
		contact_impulse.x = desired_delta_velocity / delta_velocity

		impulse          := contact._world_from_contact * contact_impulse
		impulsive_torque := linalg.cross(impulse, relative_position)

		velocity_change         := rb.inverse_mass * impulse * sign
		angular_velocity_change := rb._inverse_inertia_tensor * impulsive_torque * sign

		// TODO: validate. no dt ??? HOW? 
		rb.velocity         += velocity_change
		rb.angular_velocity += angular_velocity_change
	}

	apply_required_impulse(contact, 0, delta_velocity, 1)
	if contact.bodies[1] != nil {
		apply_required_impulse(contact, 1, delta_velocity, -1)
	}
}

apply_position_change :: proc(contact: ^Contact) {
	get_inertias :: proc(contact: ^Contact, body_idx: int) -> (f32, f32) {
		rb := contact.bodies[body_idx]
		relative_contact_position := contact._relative_contact_positions[body_idx]

		// Angular
		angular_inertia_world := linalg.cross(relative_contact_position, contact.normal)
		angular_inertia_world = rb._inverse_inertia_tensor * angular_inertia_world
		angular_inertia_world = linalg.cross(angular_inertia_world, relative_contact_position)
		angular_inertia := linalg.dot(angular_inertia_world, contact.normal)

		// Linear
		linear_inertia := rb.inverse_mass

		return linear_inertia, angular_inertia
	}

	apply_angular_move :: proc(contact: ^Contact, body_idx: int, angular_inertia, angular_move: f32) {
		rb := contact.bodies[body_idx]
		relative_contact_position := contact._relative_contact_positions[body_idx]

		impulsive_torque := linalg.cross(relative_contact_position, contact.normal)
		impulse_per_move := rb._inverse_inertia_tensor * impulsive_torque
		rotation_per_move := impulse_per_move / angular_inertia
		rotation := angular_move * rotation_per_move

		rb.rotation = quat_rotate_by_axis(rb.rotation, rotation)
	}

	linear_inertias: [2]f32
	angular_inertias: [2]f32

	linear_inertias[0], angular_inertias[0] = get_inertias(contact, 0)
	total_inertia := linear_inertias[0] + angular_inertias[0]

	if contact.bodies[1] != nil {
		linear_inertias[1], angular_inertias[1] = get_inertias(contact, 1)
		total_inertia := linear_inertias[1] + angular_inertias[1]
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

	linear_move, angular_move := get_linear_angular_moves(contact, linear_inertias[0], angular_inertias[0], total_inertia)
	contact.bodies[0].position += linear_move * contact.normal
	apply_angular_move(contact, 0, angular_inertias[0], angular_move)

	if contact.bodies[1] != nil {
		linear_move, angular_move := get_linear_angular_moves(contact, linear_inertias[1], angular_inertias[1], total_inertia)
		contact.bodies[1].position += linear_move * contact.normal
		apply_angular_move(contact, 1, angular_inertias[1], angular_move)
	}
}
