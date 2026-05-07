package physics

import "core:math"
import "core:math/linalg"
import rl "vendor:raylib"

Vec3  :: linalg.Vector3f32
Vec4  :: linalg.Vector4f32
Quat  :: linalg.Quaternionf32
Mat4  :: linalg.Matrix4f32
Mat3  :: linalg.Matrix3f32
Color :: rl.Color


// Created while reading Ian Millington - Game Physics-Engine Development. Actually amazing book.
// However I have opted to use inbuilt odin stuff where possible, and a procedural style instead of OOP.
// We may actually be slower! TODO: elaborate

World :: struct {
	contacts    : []Contact,
	contact_idx : int,
}

make_world :: proc(max_num_contacts: int) -> World {
	return World {
		contacts = make([]Contact, max_num_contacts),
	}
}

delete_world :: proc(world: ^World) {
	delete(world.contacts)
}

Rigidbody :: struct {
	position : Vec3,
	rotation : Quat,
	inertia_tensor : Mat3,

	// Derived data

	_transform, _transform_inverse : Mat4,
}

rigidbody :: proc(position := Vec3{0, 0, 0}, rotation := linalg.QUATERNIONF32_IDENTITY) -> Rigidbody {
	return {
		position = position,
		rotation = rotation,
	}
}

// TODO: consider not using rodata
NILL_RIGIDBODY := Rigidbody{}

Collider :: struct {
	rigidbody : ^Rigidbody,
	shape     : ColliderShape,
	local_offset    : Mat4,

	_transform, _transform_inverse : Mat4, 
}

ColliderShape :: union {
	SphereShape, 
	PlaneShape,
	BoxShape,
}

collider :: proc(rb: ^Rigidbody, shape: ColliderShape, offset := linalg.MATRIX4F32_IDENTITY) -> Collider {
	return Collider {
		rigidbody    = rb,
		local_offset = offset,
		shape        = shape,
	}
}


// A real sphere would have a position. This is merely a shape.
SphereShape :: struct {
	radius: f32,
}

PlaneShape :: struct {
	normal: Vec3,
}

BoxShape :: struct {
	half_size : Vec3,
}

Contact :: struct {
	// The position of the contact in world coordinates.
	// When both bodies are specified, it is only mid-way between the inter-penetrating points
	position    : Vec3,
	// The normal of the contact in world coordinates
	normal      : Vec3,
	// The depth of penetration
	penetration : f32,

	// If you're resolving contacts properly, it shouldn't matter what the order is here.
	colliders   : [2]^Collider 
}

begin_world :: proc(world: ^World) {
	world.contact_idx = 0
}

// TODO: consider doing this lazily
rb_recompute_transform :: proc(rigidbody : ^Rigidbody) {
	rigidbody._transform = linalg.matrix4_from_quaternion(rigidbody.rotation)

	rigidbody._transform[0, 3] = rigidbody.position[0]
	rigidbody._transform[1, 3] = rigidbody.position[1]
	rigidbody._transform[2, 3] = rigidbody.position[2]

	rigidbody._transform_inverse = linalg.matrix4_inverse(rigidbody._transform)
}

coll_recompute_transform :: proc(coll: ^Collider) {
	if coll.rigidbody == nil {
		coll._transform = coll.local_offset
	} else {
		coll._transform = coll.rigidbody._transform * coll.local_offset
	}

	coll._transform_inverse = linalg.matrix4_inverse(coll._transform)
}

generate_contacts_for_colliders :: proc(a, b: ^Collider, dst: ^World) {
	assert(dst.contact_idx < len(dst.contacts))

	if a.rigidbody == nil {
		a.rigidbody = &NILL_RIGIDBODY
		NILL_RIGIDBODY = Rigidbody{}
	}
	if b.rigidbody == nil {
		b.rigidbody = &NILL_RIGIDBODY
		NILL_RIGIDBODY = Rigidbody{}
	}

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
get_next_contact :: proc(dst: ^World) -> ^Contact {
	assert(dst.contact_idx < len(dst.contacts))
	slot := &dst.contacts[dst.contact_idx]
	dst.contact_idx += 1
	return slot
}

// Homegenous coordinates to position, assuming h.w == 1
vec4_to_vec3 :: proc(h: Vec4) -> Vec3 {
	return Vec3{h.x, h.y, h.z}
}

// Position to homegenous coordinates, h.w == 1
vec3_to_vec4 :: proc(p: Vec3) -> Vec4 {
	return Vec4{p.x, p.y, p.z, 1}
}

get_collider_pos :: proc(c: ^Collider) -> Vec3 {
	return get_axis(c._transform, 3)
}

@(private)
generate_contacts_sphere_x_sphere :: proc(
	sphere_a_coll: ^Collider, sphere_a: SphereShape,
	sphere_b_coll: ^Collider, sphere_b: SphereShape,
	dst: ^World
) {
	a_pos := get_collider_pos(sphere_a_coll)
	b_pos := get_collider_pos(sphere_b_coll)

	midline := b_pos - a_pos
	size    := linalg.length(midline)
	if size <= 0.0 || size >= sphere_a.radius + sphere_b.radius {
		return
	}

	contact := get_next_contact(dst)

	contact.normal      = midline / size
	contact.penetration = sphere_a.radius + sphere_b.radius - size
	contact.position    = a_pos + (sphere_a.radius - contact.penetration / 0.5) * contact.normal
	contact.colliders[0] = sphere_a_coll
	contact.colliders[1] = sphere_b_coll
}

// Technically - its a half space and not a plane :D - A 'real' plane handles collisions 
// coming from both sides. But I would never want that when I am using it.
@(private)
generate_contacts_plane_x_sphere :: proc(
	plane_coll: ^Collider, plane: PlaneShape,
	sphere_coll: ^Collider, sphere: SphereShape,
	dst: ^World
) {
	plane_pos  := get_collider_pos(plane_coll)
	sphere_pos := get_collider_pos(sphere_coll)

	to_sphere := sphere_pos - plane_pos
	distance  := linalg.dot(plane.normal, to_sphere)

	penetration := sphere.radius - distance
	if penetration < 0 {
		return
	}

	contact := get_next_contact(dst)
	contact.normal       = plane.normal
	contact.penetration  = penetration
	contact.position     = sphere_pos + (-sphere.radius) * contact.normal
	contact.colliders[0] = plane_coll
	contact.colliders[1] = sphere_coll
}

BOX_CORNERS :: []Vec3{
	{1,1,1},
	{1,1,-1},
	{1,-1,1},
	{1,-1,-1},
	{-1,1,1},
	{-1,1,-1},
	{-1,-1,1},
	{-1,-1,-1},
}

BOX_EDGES :: [][2]Vec3 {
	{ {1, 1, 1}, {-1, 1, 1} },
	{ {1, 1, 1}, {1, -1, 1} },
	{ {1, 1, 1}, {1, 1, -1} },
	{ {1, 1, -1}, {-1, 1, -1} },
	{ {1, 1, -1}, {1, -1, -1} },
	{ {1, -1, 1}, {-1, -1, 1} },
	{ {1, -1, 1}, {1, -1, -1} },
	{ {1, -1, -1}, {-1, -1, -1} },
	{ {-1, 1, 1}, {-1, -1, 1} },
	{ {-1, 1, 1}, {-1, 1, -1} },
	{ {-1, 1, -1}, {-1, -1, -1} },
	{ {-1, -1, 1}, {-1, -1, -1} },
}

get_box_half_size_oriented :: proc(box_coll: ^Collider, box: BoxShape) -> Vec3 {
	return box.half_size * get_box_axes(box_coll, box)
}

get_box_axes :: proc(box_coll: ^Collider, box: BoxShape) -> Vec3 {
	return (
		get_collider_axis(box_coll, 0) + 
		get_collider_axis(box_coll, 1) + 
		get_collider_axis(box_coll, 2)
	)
}

@(private)
generate_contacts_plane_x_box :: proc(
	plane_coll: ^Collider, plane: PlaneShape,
	box_coll: ^Collider, box: BoxShape,
	dst: ^World
) {
	plane_pos := get_collider_pos(plane_coll)
	box_pos := get_collider_pos(box_coll)

	box_half_size_oriented := get_box_half_size_oriented(box_coll, box)

	for &corner in BOX_CORNERS {
		if dst.contact_idx >= len(dst.contacts) {break}

		// TODO: Is it faster to unroll?
		vertex := box_pos + (box_half_size_oriented * corner)
		generate_contacts_plane_x_vertex(plane_coll, plane, box_coll, vertex, plane_pos, dst)
	}
}

/*
Getting column 1, 2, or 3 of a matrix should be equivelant to doing
M*Vec3{1, 0, 0}, M*Vec3{0, 1, 0}, M*Vec3{0, 0, 1}, respectively.
Getting column 4 is equivelant to getting the translation component.
*/
get_axis :: proc(mat: Mat4, axis: int) -> Vec3 #no_bounds_check {
	return Vec3{
		mat[0, axis],
		mat[1, axis],
		mat[2, axis],
	}
}

// NOTE: Assumes collider itself was not also transformed.
// TODO: Each collider might need it's own matrix pairs.
get_collider_axis :: proc(coll: ^Collider, axis: int) -> Vec3 {
	return get_axis(coll._transform, axis)
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
	// contact.position     = vertex + contact.penetration * contact.normal
	contact.position     = vertex
	contact.colliders[0] = other_coll
	contact.colliders[1] = plane_coll
}

// NOTE: possibly slow. We may want to cache the inverse too?
coll_relative_pos :: proc(coll: ^Collider, world: Vec3) -> Vec3 {
	return mat4_mul_vec3(coll._transform_inverse, world)
}

coll_world_pos :: proc(coll: ^Collider, relative: Vec3) -> Vec3 {
	return mat4_mul_vec3(coll._transform, relative)
}

rb_relative_pos :: proc(rigidbody: ^Rigidbody, world: Vec3) -> Vec3 {
	return mat4_mul_vec3(rigidbody._transform_inverse, world)
}

rb_world_pos :: proc(rigidbody: ^Rigidbody, relative: Vec3) -> Vec3 {
	return mat4_mul_vec3(rigidbody._transform, relative)
}

mat4_mul_vec3 :: proc(mat: Mat4, vec: Vec3) -> Vec3 {
	result := mat * vec3_to_vec4(vec)
	return vec4_to_vec3(result)
}

@(private)
generate_contacts_sphere_x_box :: proc(
	sphere_coll: ^Collider, sphere: SphereShape,
	box_coll: ^Collider, box: BoxShape,
	dst: ^World
) {
	sphere_pos := get_collider_pos(sphere_coll)
	box_pos    := get_collider_pos(box_coll)

	contact, ok := get_contact_sphere_x_box(
		sphere_coll, sphere, sphere_pos,
		box_coll, box, box_pos,
	)

	if ok {
		contact_ptr := get_next_contact(dst)
		contact_ptr^ = contact
	}
}

// NOTE: sphere_coll may or may not be null here.
get_contact_sphere_x_box :: proc(
	sphere_coll: ^Collider, sphere: SphereShape, sphere_pos: Vec3,
	box_coll: ^Collider, box: BoxShape, box_pos: Vec3,
) -> (contact: Contact, ok: bool) {
	sphere_pos_relative := coll_relative_pos(box_coll, sphere_pos)

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

	closest_point := coll_world_pos(box_coll, closest_point_relative)

	to_sphere := sphere_pos - closest_point
	normal    := linalg.normalize(to_sphere)

	contact.normal       = normal 
	contact.penetration  = sphere.radius - math.sqrt(dist_squared)
	contact.position     = closest_point
	contact.colliders[0] = sphere_coll
	contact.colliders[1] = box_coll

	ok = true
	return 
}

// Almost, but not quite the same as sphere x box
get_contact_vertex_x_box :: proc(
	vertex_box_coll: ^Collider, vertex: Vec3, vertex_box_pos: Vec3,
	box_coll: ^Collider, box: BoxShape, box_pos: Vec3,
) -> (contact: Contact, ok: bool) {
	vertex_relative         := coll_relative_pos(box_coll, vertex)
	vertex_box_pos_relative := coll_relative_pos(box_coll, vertex_box_pos)

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

	closest_point := coll_world_pos(box_coll, closest_point_relative)
	to_closest_point := closest_point - vertex

	contact.penetration  = linalg.length(to_closest_point)
	contact.normal       = to_closest_point / contact.penetration
	contact.position     = vertex
	contact.colliders[0] = vertex_box_coll
	contact.colliders[1] = box_coll

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
	box_a_pos := get_collider_pos(box_a_coll)
	box_b_pos := get_collider_pos(box_b_coll)

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

	ContactAccumulator :: struct{
		max_contacts: [4]Contact,
		min_idx : int,
	}

	acc : ContactAccumulator

	push_contact :: proc(acc: ^ContactAccumulator, new_contact: Contact) {
		if new_contact.penetration < acc.max_contacts[acc.min_idx].penetration {return}
		if new_contact.normal == 0 {return}

		acc.max_contacts[acc.min_idx] = new_contact

		for &contact, i in acc.max_contacts {
			if contact.penetration < acc.max_contacts[acc.min_idx].penetration {
				acc.min_idx = i
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
			box_half_size_oriented := get_box_half_size_oriented(box_a_coll, box_a)

			for &corner in BOX_CORNERS {
				vertex := box_a_pos + (box_half_size_oriented * corner)

				contact, ok := get_contact_vertex_x_box(
					box_a_coll, vertex, box_a_pos, 
					box_b_coll, box_b, box_b_pos,
				)
				if ok {
					push_contact(acc, contact)
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
		for edge in BOX_EDGES {
			a_v1 := box_a_pos + box_a_half_size_oriented * edge[0]
			a_v2 := box_a_pos + box_a_half_size_oriented * edge[1]

			for edge in BOX_EDGES {
				b_v1 := box_b_pos + box_b_half_size_oriented * edge[0]
				b_v2 := box_b_pos + box_b_half_size_oriented * edge[1]

				a, b := closest_points_between_lines(a_v1, a_v2, b_v1, b_v2)
				a_to_b := b - a

				// Check point b is closer to box a's center than point a
				if linalg.length2(b - box_a_pos) < linalg.length2(a - box_a_pos) {
					penetration := linalg.length(a_to_b)

					// TODO: - check if I need to generate 2 contacts actually.
					// Or maybe the physics engine should know to apply 2 forces?

					// The position of the contact in world coordinates.
					// When both bodies are specified, it is only mid-way between the inter-penetrating points

					push_contact(&acc, Contact{
						position    = a,
						penetration = penetration,
						normal      = a_to_b / penetration,
						colliders   = {box_a_coll, box_b_coll},
					})
				}
			}
		}
	}

	for contact in acc.max_contacts {
		if contact.penetration == 0 {break}
		if dst.contact_idx >= len(dst.contacts) {break}
		next_contact := get_next_contact(dst)
		next_contact^ = contact
	}
}

// Finds the closest points between two infinite 3D lines.
// L1 = P1 + t1*V1
// L2 = P2 + t2*V2
// NOTE: this was a python snippet I copy pasted. It was surprisingly easy to convert
closest_points_between_lines :: proc(P1, P12, P2, P22: Vec3) -> (Vec3, Vec3) {
	V1 := P12 - P1
	V2 := P22 - P2

    // Vector between line starting points
	w0 := P1 - P2

    a := linalg.dot(V1, V1)
    b := linalg.dot(V1, V2)
    c := linalg.dot(V2, V2)
    d := linalg.dot(V1, w0)
    e := linalg.dot(V2, w0)

    denom := a * c - b * b

    // Check if lines are parallel (denominator close to zero)
	t1, t2: f32
    if denom < 1e-6 {
		t1 = 0
        t2 = d / b if b != 0 else 0
	} else {
		t1 = (b * e - c * d) / denom
        t2 = (a * e - b * d) / denom
	}

	closest_point_on_L1 := P1 + t1 * V1
    closest_point_on_L2 := P2 + t2 * V2

    return closest_point_on_L1, closest_point_on_L2
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
