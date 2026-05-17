package physics

import "core:math"
import "core:math/linalg"

EPS :: 0.000001

RESTITUTION := 0.0 // TODO: make this configurable. I think it's the 'bounce'
FRICTION := 0.2

/*

This subsystem attempts to resolve physical collisions in a realistic manner. 
NOTE: for most arcadey games that I want to usually make, this is not what I want.
Instead, I just need an efficient way to find the contacts, so that I can implement 
resolution logic myself. However, it's always useful to have a real physical 
thing to fall back on. For example, in a racing game, I would implement the wheel_x_ground
physics myself, but it's nice to just fall back on a real physics engine for
the body x ground collisions.


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
	if world.contacts_idx == 0 {return}

	all_contacts := world.contacts[0:world.contacts_idx]

	// prepare the contacts
	{
		for &contact in all_contacts {
			if contact.bodies[0] == nil {
				contact.bodies[0], contact.bodies[1] = contact.bodies[1], contact.bodies[0]
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

			calculate_contact_relative_velocities(&contact)

			calculate_desired_velocity(&contact, dt)
		}
	}

	// Intermediate storage

	// TODO: resolve impulses at some point

	// TODO: need to spend some time understanding why this would even work

	// Resolve penetration
	{
		// TODO: How the heck is this populated for position algo ??
		linear_changes  : [2]Vec3
		angular_changes : [2]Vec3

		last_contact : ^Contact
		// TODO: 1000
		POSITION_ITERATIONS :: 50
		for i in 0..<POSITION_ITERATIONS {
			worst_contact_idx := -1
			worst_penetration : f32
			for &contact, i in all_contacts {
				if contact.penetration > worst_penetration {
					worst_contact_idx = i
					worst_penetration = contact.penetration
				}
			}

			if worst_contact_idx == -1 {break}
			worst_contact := &world.contacts[worst_contact_idx]

			// TODO: Match the awake state at the contact. (What is this?)

			// Resolve the penetration

			apply_position_change(worst_contact, &linear_changes, &angular_changes)

			// May have changed the penetration of other bodies, so we update the contacts
			for &contact, i in all_contacts {
				for contact_body, contact_body_idx in contact.bodies {
					if contact_body == nil {continue}

					for &worst_contact_body, worst_contact_body_idx in worst_contact.bodies {
						if contact_body != worst_contact_body {continue}

						angular_adjustment := linalg.cross(
							angular_changes[worst_contact_body_idx], 
							contact._relative_contact_positions[contact_body_idx]
						)

						linear_adjustment := linear_changes[worst_contact_body_idx]

						cp := linear_adjustment + angular_adjustment

						// This is negative for the first body - we reducing the penetration.
						contact.penetration += 
							(contact_body_idx == 0 ? -1 : 1) * 
							linalg.dot(contact.normal, cp)
					}
				}
			}
		}
	}

	// TODO: Wait should this be happening first? and in the same loop??
	// Resolve velocity OMG it just never ends bro wtf
	{
		linear_changes  : [2]Vec3
		angular_changes : [2]Vec3

		// TODO: 1000
		VELOCITY_ITERATIONS :: 50
		for i in 0..<VELOCITY_ITERATIONS {
			fastest_contact : ^Contact
			fastest_speed : f32 = 0 // negative means the contacts are seperating, we dont need to worry about those
			for &contact in all_contacts {
				closing_vel := -contact._relative_velocity.x
				if closing_vel > fastest_speed {
					fastest_speed  = closing_vel
					fastest_contact = &contact
				}
			}

			if fastest_contact == nil {break}

			apply_velocity_change(fastest_contact, &linear_changes, &angular_changes, dt)

			// Other body velocities may have changed this time
			for &contact, i in all_contacts {
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
						contact._desired_delta_velocity += 
							(contact_body_idx == 0 ? 1 : -1) *
							linalg.dot(cp, contact.normal)

						calculate_desired_velocity(&contact, dt)
					}
				}
			}
		}
	}
}

calculate_desired_velocity :: proc(contact: ^Contact, dt: f32) {
	velocity_from_acceleration := linalg.dot(contact.bodies[0].acceleration_last_frame, dt * contact.normal)

	if contact.bodies[1] != nil {
		velocity_from_acceleration -=
			linalg.dot(contact.bodies[1].acceleration_last_frame, dt * contact.normal)
	}

	// If the velocity is very slow, limit the restitution
	velocity_limit := f32(0.0001)
	restitution := f32(RESTITUTION)
	if abs(contact._relative_velocity.x) < velocity_limit {
		// TODO: wait actually we dont have restitution yet. Stil not sure wtf it is
		restitution = 0
	}

	contact._desired_delta_velocity = 
		-contact._relative_velocity.x +
		-restitution * (contact._relative_velocity.x - velocity_from_acceleration)
}

apply_velocity_change :: proc(
	contact: ^Contact,
	linear_changes: ^[2]Vec3,
	angular_changes: ^[2]Vec3,
	dt: f32,
) {
	impulse_to_torque := skew_symmetric(contact._relative_contact_positions[0])

	// TODO: try to understand how this works
	delta_vel_world := impulse_to_torque
	delta_vel_world *= contact.bodies[0]._inverse_inertia_tensor
	delta_vel_world *= impulse_to_torque
	delta_vel_world *= -1

	inverse_mass := contact.bodies[0].inverse_mass

	if contact.bodies[1] != nil {
		impulse_to_torque := skew_symmetric(contact._relative_contact_positions[1])
		delta_vel_world_2 := impulse_to_torque
		delta_vel_world_2 *= contact.bodies[1]._inverse_inertia_tensor
		delta_vel_world_2 *= impulse_to_torque
		delta_vel_world_2 *= -1

		delta_vel_world += delta_vel_world_2
		inverse_mass    += contact.bodies[1].inverse_mass
	}

	// Change of basis to convert into contact coordinates.
	delta_velocity := contact._contact_from_world
	delta_velocity *= delta_vel_world
	delta_velocity *= contact._world_from_contact

	// Also account for linear velocity. I'm surprised it works after the change of basis tbh
	delta_velocity[0, 0] += inverse_mass
	delta_velocity[1, 1] += inverse_mass
	delta_velocity[2, 2] += inverse_mass

	impulse_matrix := linalg.inverse(delta_velocity)

	// TODO: We are not handling static friction properly yet!!
	velocity_to_kill := Vec3{
		contact._desired_delta_velocity, 
		-contact._relative_velocity.y,
		-contact._relative_velocity.z,
	}

	contact_impulse := impulse_matrix * velocity_to_kill

	planar_impulse := math.sqrt(
		contact_impulse.y * contact_impulse.y +
		contact_impulse.z * contact_impulse.z
	)

	// TODO: lookup table of materials, probably. idk.
	friction := f32(FRICTION)

	use_dynamic_friction := planar_impulse > contact_impulse.x * friction
	if use_dynamic_friction {
		contact_impulse.y /= planar_impulse
		contact_impulse.z /= planar_impulse

		// TODO: validate we're pulling from the matrix correctly
		contact_impulse.x = 
			delta_velocity[0,0] +
			delta_velocity[0,1] + friction * contact_impulse.y +
			delta_velocity[0,2] + friction * contact_impulse.z
		contact_impulse.x = contact._desired_delta_velocity / contact_impulse.x

		// This is from the formula for dynamic friction.
		// dynamic_friction = -velocity * friction_coefficent * normal_force (i.e contact_impulse.x)
		contact_impulse.y *= friction * contact_impulse.x
		contact_impulse.z *= friction * contact_impulse.x
	}

	for rb, body_idx in contact.bodies {
		if rb == nil {continue}

		relative_position := contact._relative_contact_positions[body_idx]

		impulse          := contact._world_from_contact * contact_impulse
		impulsive_torque := linalg.cross(relative_position, impulse)

		sign : f32 = body_idx == 0 ? 1 : -1
		velocity_change         := rb.inverse_mass * impulse * sign
		angular_velocity_change := rb._inverse_inertia_tensor * impulsive_torque * sign

		linear_changes[body_idx]  = velocity_change
		angular_changes[body_idx] = angular_velocity_change

		// TODO: validate. no dt ??? HOW? 
		rb.velocity         += velocity_change
		rb.angular_velocity += angular_velocity_change
	}

	calculate_contact_relative_velocities(contact)
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

		linear_move, angular_move: f32
		if abs(total_inertia) > EPS {
			linear_move  = contact.penetration * linear_inertias[body_idx] / total_inertia
			angular_move = contact.penetration * angular_inertias[body_idx] / total_inertia
		}

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

		sign : f32 = body_idx == 0 ? 1 : -1

		// Apply linear move
		linear_changes[body_idx] = sign * linear_move * contact.normal

		assert(linear_changes[body_idx].x < 100000)
		assert(linear_changes[body_idx].x > -100000)

		rb.position += linear_changes[body_idx]

		// NOTE: cyclone physics is far more complex here. 
		// The book must not have fully explained this bit yet.
		// Apply angular move
		impulsive_torque := linalg.cross(relative_contact_position, contact.normal)
		impulse_per_move := rb._inverse_inertia_tensor * impulsive_torque
		rotation_per_move := abs(angular_inertias[body_idx]) < EPS ? 0 : impulse_per_move / angular_inertias[body_idx]
		rotation := sign * angular_move * rotation_per_move

		rb.rotation = quat_rotate_by_axis(rb.rotation, rotation)
		angular_changes[body_idx] = rotation

		rb_recompute_derived(rb)
	}
}

calculate_contact_relative_velocities :: proc(contact: ^Contact) {
	// This part only works because we have a single contact.position in between
	// the overlap of the two rigidbodies rather than 2 contact.positions
	// on the edges of the two colliders (which is what looks right in my head 
	// but it's not how it's done in practice)

	contact._relative_velocity = 0
	for rb, body_idx in contact.bodies {
		if rb == nil {continue}

		velocity_linear := rb.velocity

		// TODO: i forgot which way around this was supposed to be. It Does matter
		velocity_angular := linalg.cross(
			rb.angular_velocity,
			contact._relative_contact_positions[body_idx], 
		)

		total_velocity := velocity_linear + velocity_angular
		sign := f32(body_idx == 0 ? 1 : -1)
		transformed := contact._contact_from_world * total_velocity
		contact._relative_velocity += sign * transformed
	}
}
