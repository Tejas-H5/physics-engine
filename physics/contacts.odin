package physics

import "core:math"
import "core:math/linalg"

@(private)
get_next_contact :: proc(dst: ^World) -> ^Contact {
	assert(dst.contacts_idx < len(dst.contacts))
	slot := &dst.contacts[dst.contacts_idx]
	dst.contacts_idx += 1
	return slot
}

clear_contacts :: proc(w: ^World) {
	w.contacts_idx = 0
}

generate_contacts_for_colliders :: proc(a, b: ^Collider, dst: ^World) {
	assert(dst.contacts_idx < len(dst.contacts))

	switch a_shape in a.shape {
	case SphereShape:
		switch b_shape in b.shape {
		case SphereShape: generate_contacts_sphere_x_sphere(a, a_shape, b, b_shape, dst)
		case PlaneShape:  generate_contacts_plane_x_sphere(b, b_shape, a, a_shape, dst)
		case BoxShape:    generate_contacts_sphere_x_box(a, a_shape, b, b_shape, dst)
		}
	case PlaneShape:
		switch b_shape in b.shape {
		case PlaneShape:  // Uhnandled
		case SphereShape: generate_contacts_plane_x_sphere(a, a_shape, b, b_shape, dst) 
		case BoxShape:    generate_contacts_plane_x_box(a, a_shape, b, b_shape, dst)
		}
	case BoxShape:
		switch b_shape in b.shape {
		case SphereShape: generate_contacts_sphere_x_box(b, b_shape, a, a_shape, dst)
		case PlaneShape:  generate_contacts_plane_x_box(b, b_shape, a, a_shape, dst)
		case BoxShape:    generate_contacts_box_x_box(a, a_shape, b, b_shape, dst)
		}
	}

}


@(private)
generate_contacts_sphere_x_sphere :: proc(
	sphere_a_coll: ^Collider, sphere_a: SphereShape,
	sphere_b_coll: ^Collider, sphere_b: SphereShape,
	dst: ^World
) {
	a_pos := collider_position(sphere_a_coll)
	b_pos := collider_position(sphere_b_coll)

	b_to_a := a_pos - b_pos
	size    := linalg.length(b_to_a)
	if size <= 0.0 || size >= sphere_a.radius + sphere_b.radius {
		return
	}

	contact := get_next_contact(dst)

	contact.normal      = b_to_a / size
	contact.penetration = sphere_a.radius + sphere_b.radius - size

	contact.position  = b_pos + (sphere_b.radius - contact.penetration * 0.5) * contact.normal
	contact.bodies = { sphere_a_coll.rigidbody, sphere_b_coll.rigidbody }
}

// Technically - its a half space and not a plane :D - A 'real' plane handles collisions 
// coming from both sides. But I would never want that when I am using it.
@(private)
generate_contacts_plane_x_sphere :: proc(
	plane_coll: ^Collider, plane: PlaneShape,
	sphere_coll: ^Collider, sphere: SphereShape,
	dst: ^World
) {
	plane_pos  := collider_position(plane_coll)
	sphere_pos := collider_position(sphere_coll)

	to_sphere := sphere_pos - plane_pos
	distance  := linalg.dot(plane.normal, to_sphere)

	penetration := sphere.radius - distance
	if penetration < 0 {
		return
	}

	contact := get_next_contact(dst)
	contact.normal      = plane.normal
	contact.penetration = penetration
	contact.position    = sphere_pos + (-sphere.radius + penetration * 0.5) * contact.normal
	contact.bodies      = { sphere_coll.rigidbody, plane_coll.rigidbody }
}

@(private)
generate_contacts_plane_x_vertex :: proc(
	plane_coll: ^Collider, plane: PlaneShape,
	other_coll: ^Collider, vertex: Vec3, 
	plane_pos: Vec3,
	dst: ^World
) {
	to_vertex := vertex - plane_pos
	distance  := linalg.dot(plane.normal, to_vertex)

	if distance > 0 {
		return
	}

	contact := get_next_contact(dst)
	contact.normal       = plane.normal
	contact.penetration  = -distance
	contact.position     = vertex + 0.5 * contact.penetration * contact.normal
	contact.bodies    = { other_coll.rigidbody, plane_coll.rigidbody }
}

@(private)
generate_contacts_sphere_x_box :: proc(
	sphere_coll: ^Collider, sphere: SphereShape,
	box_coll: ^Collider, box: BoxShape,
	dst: ^World
) {
	sphere_pos := collider_position(sphere_coll)
	box_pos    := collider_position(box_coll)

	sphere_pos_relative := collider_relative_pos(box_coll, sphere_pos)

	if abs(sphere_pos_relative.x) - box.half_size.x > sphere.radius {return}
	if abs(sphere_pos_relative.y) - box.half_size.y > sphere.radius {return}
	if abs(sphere_pos_relative.z) - box.half_size.z > sphere.radius {return}

	// NOTE: Doesn't seem right. Closest point should be from the surface of the box, 
	// not from the center of the box to the sphere, right?
	// I think it does work though, because the point is in the box coordinates.
	closest_point_relative: Vec3
	closest_point_relative.x = math.clamp(sphere_pos_relative.x, -box.half_size.x, box.half_size.x)
	closest_point_relative.y = math.clamp(sphere_pos_relative.y, -box.half_size.y, box.half_size.y)
	closest_point_relative.z = math.clamp(sphere_pos_relative.z, -box.half_size.z, box.half_size.z)

	dist_squared := linalg.length2(closest_point_relative - sphere_pos_relative)
	if dist_squared > sphere.radius * sphere.radius {
		return
	}

	closest_point := collider_world_pos(box_coll, closest_point_relative)

	to_sphere := sphere_pos - closest_point
	normal    := linalg.normalize(to_sphere)

	contact := get_next_contact(dst)
	contact.normal       = normal 
	contact.penetration  = sphere.radius - math.sqrt(dist_squared)
	contact.position     = closest_point + 0.5 * contact.penetration * contact.normal
	contact.bodies       = { sphere_coll.rigidbody, box_coll.rigidbody }
}

// Almost, but not quite the same as sphere x box
get_contact_vertex_x_box :: proc(
	vertex_box_coll: ^Collider, vertex: Vec3, vertex_box_pos: Vec3,
	box_coll: ^Collider, box: BoxShape, box_pos: Vec3,
) -> (contact: Contact, ok: bool) {
	vertex_relative         := collider_relative_pos(box_coll, vertex)
	vertex_box_pos_relative := collider_relative_pos(box_coll, vertex_box_pos)

	// Opinion 1: The support function that computes the normal should look like this:
	//              ^
	//    \_        |         _/
	//  <-  \________________/   ->
	//     _/                \_
	//    /         |         \
	//              v
	//
	// In this way, the collision handling remains the same at the cornersr regardless of how the box was stretched.
	// That being said, we probably want to change the normal based on how the other cube was oriented,
	// in which case this method is not right, and we'll have to update it. I reckon it's a matter of taking this normal,
	// dotting it with all 3 axes of the incoming box and picking the one with the largest value, so this computation
	// would stil be needed in some sense.

	x_dist := box.half_size.x - abs(vertex_relative.x)
	y_dist := box.half_size.y - abs(vertex_relative.y)
	z_dist := box.half_size.z - abs(vertex_relative.z)

	if x_dist < 0 {return}
	if y_dist < 0 {return}
	if z_dist < 0 {return}

	closest_point_relative := vertex_relative
	if x_dist < y_dist && x_dist < z_dist {
		closest_point_relative.x = math.sign(vertex_relative.x) * box.half_size.x
	} else if y_dist < z_dist /* && y_dist < x_dist (inferred from prior checks) */ {
		closest_point_relative.y = math.sign(vertex_relative.y) * box.half_size.y
	} else /* if z_dist < x_dist && z_dist < y_dist (inferred from prior checks) */ {
		closest_point_relative.z = math.sign(vertex_relative.z) * box.half_size.z
	}

	closest_point := collider_world_pos(box_coll, closest_point_relative)
	to_closest_point := closest_point - vertex

	contact.penetration  = linalg.length(to_closest_point)
	contact.normal       = to_closest_point / contact.penetration
	contact.position     = vertex + 0.5 * contact.penetration * contact.normal
	contact.bodies    = { vertex_box_coll.rigidbody, box_coll.rigidbody }

	ok = true
	return 
}

// NOTE: though this may be the 'slower' approach, it will be useful for 
// if we ever decide to optimize this - we simply check if the optimized version is identical to this one.
// Apparently, all algorithms are either 
// a) simple and correctness can be easily verified by observation, but slow
// b) complicated and not obvious, but fast

// This is the final boss of collision detection tutorial level. If you can code this, then you are allowed to write a physic engine :D

@(private)
generate_contacts_box_x_box :: proc(
	box_a_coll: ^Collider, box_a: BoxShape,
	box_b_coll: ^Collider, box_b: BoxShape,
	dst: ^World
) {
	box_a_pos := collider_position(box_a_coll)
	box_b_pos := collider_position(box_b_coll)

	// Separating Axes.
	// If the boxes don't overlap on these 15 axes, then we know for sure that they don't touch

	has_no_overlap_on_any_seperating_axis :: proc(
		box_a_coll: ^Collider, box_a: BoxShape, box_a_pos: Vec3,
		box_b_coll: ^Collider, box_b: BoxShape, box_b_pos: Vec3,
	) -> bool {
		axis_1 := get_collider_axis(box_a_coll, 0)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_1) {return true}

		axis_2 := get_collider_axis(box_a_coll, 1)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_2) {return true}

		axis_3 := get_collider_axis(box_a_coll, 2)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_3) {return true}

		axis_4 := get_collider_axis(box_b_coll, 0)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_4) {return true}

		axis_5 := get_collider_axis(box_b_coll, 1)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_5) {return true}

		axis_6 := get_collider_axis(box_b_coll, 2)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_6) {return true}

		axis_7 := linalg.cross(axis_1, axis_4)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_7) {return true}

		axis_8 := linalg.cross(axis_1, axis_5)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_8) {return true}

		axis_9 := linalg.cross(axis_1, axis_6)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_9) {return true}

		axis_10 := linalg.cross(axis_2, axis_4)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_10) {return true}

		axis_11 := linalg.cross(axis_2, axis_5)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_11) {return true}

		axis_12 := linalg.cross(axis_2, axis_6)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_12) {return true}

		axis_13 := linalg.cross(axis_3, axis_4)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_13) {return true}

		axis_14 := linalg.cross(axis_3, axis_5)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_14) {return true}

		axis_15 := linalg.cross(axis_3, axis_6)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_15) {return true}

		return false
	}

	if has_no_overlap_on_any_seperating_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos) {
		return
	}

	// NOTE: I didn't really understand the caching algorithm or whatever that was mentioned in the book.
	// Instead, I've assumed via intuition that we can have at max 4 valid contacts at a time.
	// Instead of caching, I'll just keep track of the 4 contacts with the largest penetration and push those. 
	// Or maybe it's 8, or something. whatever.

	ContactAccumulator :: struct{
		vertex_contacts: [4]Contact, // maximize depth
		vertex_min_idx : int,
		edge_contacts: [4]Contact,   // minimize depth
		edge_min_idx : int,
	}

	acc : ContactAccumulator
	for &contact in acc.edge_contacts {
		contact.penetration = math.INF_F32
	}

	push_vertex_contact :: proc(acc: ^ContactAccumulator, new_contact: Contact) {
		if new_contact.penetration < acc.vertex_contacts[acc.vertex_min_idx].penetration {return}
		if new_contact.normal == 0 {return}

		acc.vertex_contacts[acc.vertex_min_idx] = new_contact

		for &contact, i in acc.vertex_contacts {
			if contact.penetration < acc.vertex_contacts[acc.vertex_min_idx].penetration {
				acc.vertex_min_idx = i
			}
		}
	}

	push_edge_contact :: proc(acc: ^ContactAccumulator, new_contact: Contact) {
		for &contact, i in acc.edge_contacts {
			if new_contact.penetration < contact.penetration {
				acc.edge_contacts[i] = new_contact
				break
			}
		}
		for &contact, i in acc.edge_contacts {
			if contact.penetration < acc.edge_contacts[acc.edge_min_idx].penetration {
				acc.edge_min_idx = i
			}
		}
	}

	// Point x Face
	{
		// TODO: we may want a more specific function here.
		// enumerate the points of one cube and the other cube.

		find_max_penetrating_vertex_of_a_into_b :: proc(
			box_a_coll: ^Collider, box_a: BoxShape, box_a_pos: Vec3,
			box_b_coll: ^Collider, box_b: BoxShape, box_b_pos: Vec3,
			acc: ^ContactAccumulator,
		) {
			for corner in BOX_CORNERS {
				vertex := get_vertex_on_cube(box_a_coll, box_a, corner)

				contact, ok := get_contact_vertex_x_box(
					box_a_coll, vertex, box_a_pos, 
					box_b_coll, box_b, box_b_pos,
				)
				if ok {
					push_vertex_contact(acc, contact)
				}
			}

			return
		}

		find_max_penetrating_vertex_of_a_into_b(
			box_a_coll, box_a, box_a_pos,
			box_b_coll, box_b, box_b_pos,
			&acc,
		)
			
		find_max_penetrating_vertex_of_a_into_b(
			box_b_coll, box_b, box_b_pos,
			box_a_coll, box_a, box_a_pos,
			&acc,
		)
	}

	// Edge x Edge
	{
		// enumerate the edges of one cube and the other cube.
		// Figure out which pair of edges are contacting

		box_a_axes := get_box_axes(box_a_coll, box_a)
		box_a_half_size_oriented := box_a.half_size * box_a_axes
		box_b_axes := get_box_axes(box_b_coll, box_b)
		box_b_half_size_oriented := box_b.half_size * box_b_axes

		// aw hell naww
		for edge_a in BOX_EDGES {
			for edge_b in BOX_EDGES {
				a_v1 := box_a_pos + box_a_half_size_oriented * edge_a[0]
				a_v2 := box_a_pos + box_a_half_size_oriented * edge_a[1]

				b_v1 := box_b_pos + box_b_half_size_oriented * edge_b[0]
				b_v2 := box_b_pos + box_b_half_size_oriented * edge_b[1]

				// NOTE: the problem with the current approach is that when two boxes slightly intersect,
				// the most powerful axis will be a shear axis, which makes no sense. 
				// So we'll actually need to find the shallowest edge_x_edge collision instead of the deepest.

				a, ta, b, tb, are_parallel := closest_points_between_lines(a_v1, a_v2, b_v1, b_v2)
				if !are_parallel && 0 <= ta && ta <= 1 && 0 <= tb && tb <= 1 {
					// Check point b is closer to box a's center than point a, and vice versa
					if linalg.length2(b - box_a_pos) < linalg.length2(a - box_a_pos) {
						if linalg.length2(a - box_b_pos) < linalg.length2(b - box_b_pos) {
							// TODO: - check if I need to generate 2 contacts actually.
							// Or maybe the physics engine should know to apply 2 forces?

							// The position of the contact in world coordinates.
							// When both bodies are specified, it is only mid-way between the inter-penetrating points

							a_to_b := b - a
							penetration := linalg.length(a_to_b)
							normal      := a_to_b / penetration
							push_edge_contact(&acc, Contact{
								penetration = penetration,
								normal      = normal,
								position    = a + 0.5 * penetration * normal,
								bodies   = { box_a_coll.rigidbody, box_b_coll.rigidbody, },
							})
						}
					}
				}
			}
		}
	}

	for contact in acc.vertex_contacts {
		if dst.contacts_idx >= len(dst.contacts) {break}
		if contact.penetration == 0 {continue}
		next_contact := get_next_contact(dst)
		next_contact^ = contact
	}
	min_edge_pen := acc.edge_contacts[acc.edge_min_idx].penetration
	for contact in acc.edge_contacts {
		if dst.contacts_idx >= len(dst.contacts) {break}

		if contact.penetration == 0 {continue}
		if contact.penetration == math.INF_F32 {continue}
		if contact.penetration > min_edge_pen {continue}


		next_contact := get_next_contact(dst)
		next_contact^ = contact
	}
}


get_vertex_on_cube :: proc(box_coll: ^Collider, box: BoxShape, corner: Vec3) -> Vec3 {
	// TODO: we can probably make this a lot faster
	orientation := linalg.matrix3_from_matrix4(box_coll._transform)
	vertex      := box_coll.rigidbody.position + orientation * (box.half_size * corner)
	return vertex
}

@(private)
generate_contacts_plane_x_box :: proc(
	plane_coll: ^Collider, plane: PlaneShape,
	box_coll: ^Collider, box: BoxShape,
	dst: ^World
) {
	plane_pos := collider_position(plane_coll)
	box_pos := collider_position(box_coll)

	for corner in BOX_CORNERS {
		if dst.contacts_idx >= len(dst.contacts) {break}

		vertex := get_vertex_on_cube(box_coll, box, corner)
		generate_contacts_plane_x_vertex(plane_coll, plane, box_coll, vertex, plane_pos, dst)
	}
}

@(private)
overlap_on_axis :: proc(
	box_a_coll: ^Collider, box_a: BoxShape, box_a_pos: Vec3,
	box_b_coll: ^Collider, box_b: BoxShape, box_b_pos: Vec3,
	axis: Vec3,
) -> bool {
	box_a_radius := transform_to_axis(box_a_coll, box_a, box_a_pos, axis)
	box_b_radius := transform_to_axis(box_b_coll, box_b, box_b_pos, axis)

	a_to_b := box_b_pos - box_a_pos
	distance := abs(linalg.dot(axis, a_to_b))

	return distance <= box_a_radius + box_b_radius
}

@(private)
transform_to_axis :: proc(
	coll: ^Collider, box: BoxShape, box_pos: Vec3,
	axis: Vec3
) -> f32 {
	// Gets the 'radius' of the box in a particular axis.
	// Because half_size is a vector from the center to the diagonal, and based on how the box is rotatied, we can't
	// just project half_size onto the axis. We need to take the absolute value, which is effectively like choosing
	// the side that increases the radius every time. 
	//
	//         +----------- __+ A  (When rotated anticlockwise by a bit, projecting A down onto the X axis will be 
	//         |         __-  |     smaller than projecting B. This is what abs avoids. Super not obvious at all lol, and bro didn't explain it in the book)
	//         |        +     |     
	//         |         --_  |
	//         _----------- --+ B

	x_amount := abs(linalg.dot(axis, get_collider_axis(coll, 0)))
	y_amount := abs(linalg.dot(axis, get_collider_axis(coll, 1)))
	z_amount := abs(linalg.dot(axis, get_collider_axis(coll, 2)))

	return (
		box.half_size.x * x_amount +
		box.half_size.y * y_amount +
		box.half_size.z * z_amount
	)
}


