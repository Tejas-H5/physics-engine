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

			calculate_desired_velocity(contact, dt)
		}
	}

	// Intermediate storage

	// TODO: resolve impulses at some point

	// TODO: need to spend some time understanding why this would even work

	// Resolve penetration
	{
		// TODO: How the heck is this populated for position algo ??
		linear_changes: [2]Vec3
		angular_changes: [2]Vec3

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

			apply_position_change(worst_contact, &linear_changes, &angular_changes)

			// May have changed the penetration of other bodies, so we update the contacts
			for &contact, i in world.contacts {
				for contact_body, contact_body_idx in contact.bodies {
					if contact_body == nil {continue}

					for &worst_contact_body, worst_contact_body_idx in worst_contact.bodies {
						if contact_body != worst_contact_body {continue}

						cp := linalg.cross(
							angular_changes[worst_contact_body_idx], 
							contact._relative_contact_positions[contact_body_idx]
						)

						cp += linear_changes[worst_contact_body_idx]

						// This is negative for the first body
						contact._relative_velocity += 
							(contact_body_idx == 0 ? -1 : 1) *
							contact._contact_from_world * cp

						calculate_desired_velocity(&contact, dt)
					}
				}
			}
		}
	}

	// TODO: Wait should this be happening first? and in the same loop??
	// Resolve velocity OMG it just never ends bro wtf
	{
		linear_changes: [2]Vec3
		angular_changes: [2]Vec3

		VELOCITY_ITERATIONS :: 1000
		for i in 0..<VELOCITY_ITERATIONS {
			fastest_contact : ^Contact
			fastest_speed2 : f32
			for &contact in world.contacts[0:world.contact_idx] {
				vel2 := linalg.length2(contact._relative_velocity)
				if vel2 > fastest_speed2 {
					fastest_speed2  = vel2
					fastest_contact = &contact
				}
			}

			if fastest_contact == nil {break}

			apply_impulse_change(fastest_contact, &linear_changes, &angular_changes, dt)

			// Other body velocities may have changed this time
			for &contact, i in world.contacts {
				for contact_body, contact_body_idx in contact.bodies {
					if contact_body == nil {continue}

					for &fastest_contact_body, fastest_contact_body_idx in fastest_contact.bodies {
						if contact_body != fastest_contact_body {continue}

						cp := linalg.cross(
							angular_changes[fastest_contact_body_idx], 
							contact._relative_contact_positions[contact_body_idx]
						)

						cp += linear_changes[fastest_contact_body_idx]

						// The second body negative.
						contact.penetration += 
							(contact_body_idx == 0 ? 1 : -1) *
							linalg.dot(cp, contact.normal)
					}
				}
			}
		}
	}
}

calculate_desired_velocity :: proc(contact: ^Contact, dt: f32) {
	velocity_from_acceleration := 
		linalg.dot(contact.bodies[0].acceleration_last_frame, dt * contact.normal)

	if contact.bodies[1] != nil {
		velocity_from_acceleration -=
			linalg.dot(contact.bodies[1].acceleration_last_frame, dt * contact.normal)
	}

	// If the velocity is very slow, limit the restitution
	velocity_limit := f32(0.0001)
	restitution := f32(0.1) // TODO: make this configurable. I think it's the 'bounce'
	if abs(contact._relative_velocity.x) < velocity_limit {
		// TODO: wait actually we dont have restitution yet. Stil not sure wtf it is
		restitution = 0
	}

	contact._desired_delta_velocity =
		-contact._relative_velocity.x +
		-restitution * (contact._relative_velocity.x - velocity_from_acceleration)
}

apply_impulse_change :: proc(
	contact: ^Contact,
	linear_changes: ^[2]Vec3,
	angular_changes: ^[2]Vec3,
	dt: f32,
) {
	delta_velocity := f32(0)

	for rb, body_idx in contact.bodies {
		if rb == nil {continue}

		relative_contact_position := contact._relative_contact_positions[body_idx]

		contact_velocity_per_unit_impulse : f32

		// Rotational component
		{
			torque_per_unit_impulse := 
				linalg.cross(relative_contact_position, contact.normal)
			rotation_per_unit_impulse := 
				rb._inverse_inertia_tensor * torque_per_unit_impulse
			velocity_per_unit_impulse := 
				linalg.cross(rotation_per_unit_impulse, relative_contact_position)

			// > It is better, in my opinion, to think in terms of the change of coordinates, because
			// > as we introduce friction in the next chapter, the simple scalar product trick can no
			// > longer be used.
			contact_velocity_per_unit_impulse = 
				(contact._contact_from_world * rotation_per_unit_impulse).x
		}

		// linear component
		{
			// xd
			contact_velocity_per_unit_impulse += rb.inverse_mass
		}

		delta_velocity += contact_velocity_per_unit_impulse
	}

	for rb, body_idx in contact.bodies {
		if rb == nil {continue}

		relative_position := contact._relative_contact_positions[body_idx]

		velocity := linalg.cross(rb.angular_velocity, relative_position) + rb.velocity
			
		contact._relative_velocity = contact._contact_from_world * velocity
		calculate_desired_velocity(contact, dt)

		contact_impulse   := Vec3{}
		contact_impulse.x = contact._desired_delta_velocity.x / delta_velocity

		impulse          := contact._world_from_contact * contact_impulse
		impulsive_torque := linalg.cross(impulse, relative_position)

		sign : f32 = body_idx == 0 ? 1 : -1
		velocity_change         := rb.inverse_mass * impulse * sign
		angular_velocity_change := rb._inverse_inertia_tensor * impulsive_torque * sign

		linear_changes[body_idx]  = velocity_change
		angular_changes[body_idx] = angular_velocity_change

		// TODO: validate. no dt ??? HOW? 
		rb.velocity         += velocity_change
		rb.angular_velocity += angular_velocity_change
	}
}

apply_position_change :: proc(
	contact: ^Contact,
	linear_changes, angular_changes: ^[2]Vec3
) {
	linear_inertias: [2]f32
	angular_inertias: [2]f32
	total_inertia: f32

	for rb, body_idx in contact.bodies {
		if rb == nil {continue}

		relative_contact_position := contact._relative_contact_positions[body_idx]

		// Angular
		angular_inertia_world := linalg.cross(relative_contact_position, contact.normal)
		angular_inertia_world = rb._inverse_inertia_tensor * angular_inertia_world
		angular_inertia_world = linalg.cross(angular_inertia_world, relative_contact_position)
		angular_inertia := linalg.dot(angular_inertia_world, contact.normal)
		angular_inertias[body_idx] = angular_inertia

		// Linear
		linear_inertia := rb.inverse_mass
		linear_inertias[body_idx] = linear_inertia

		total_inertia += linear_inertia + angular_inertia
	}

	// Need total inertia, so can't merge these two loops

	for rb, body_idx in contact.bodies {
		if rb == nil {continue}

		// Dont let angular move get too large - we may over-rotate the object if this happens.
		// I was thinking hey why not just limit rotation such that the surface normal of the nearest surface
		// matches the contact normal, but sounds like a bunch of work so I haven't done that yet.
		// The book suggests to just use a threhsold that we can tweak as needed
		angular_limit_constraint := f32(0.5)

		linear_move  := contact.penetration * linear_inertias[body_idx] / total_inertia
		angular_move := contact.penetration * angular_inertias[body_idx] / total_inertia

		if abs(angular_move) > angular_limit_constraint {
			total_move := linear_move + angular_move

			if angular_move > 0 {
				angular_move = angular_limit_constraint
			} else {
				angular_move = -angular_limit_constraint
			}

			linear_move = total_move - angular_move
		}

		relative_contact_position := contact._relative_contact_positions[body_idx]

		// Apply linear move
		linear_changes[body_idx] = linear_move * contact.normal
		contact.bodies[body_idx].position += linear_changes[body_idx]

		// NOTE: cyclone physics is far more complex here. 
		// The book must not have fully explained this bit yet.
		// Apply angular move
		impulsive_torque := linalg.cross(relative_contact_position, contact.normal)
		impulse_per_move := rb._inverse_inertia_tensor * impulsive_torque
		rotation_per_move := impulse_per_move / angular_inertias[body_idx]
		rotation := angular_move * rotation_per_move

		rb.rotation = quat_rotate_by_axis(rb.rotation, rotation)

		angular_changes[body_idx] = rotation
	}
}
