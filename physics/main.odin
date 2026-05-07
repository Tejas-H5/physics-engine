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

	_transform : Mat4,
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
	offset    : Mat4,
	shape     : ColliderShape,
}

ColliderShape :: union {
	SphereShape, 
	PlaneShape,
	BoxShape,
}

collider :: proc(rb: ^Rigidbody, shape: ColliderShape, offset := linalg.MATRIX4F32_IDENTITY) -> Collider {
	return Collider {
		rigidbody = rb,
		offset    = offset,
		shape     = shape,
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

rb_recompute_transform :: proc(rigidbody : ^Rigidbody) {
	rigidbody._transform = linalg.matrix4_from_quaternion(rigidbody.rotation)

	rigidbody._transform[0, 3] = rigidbody.position[0]
	rigidbody._transform[1, 3] = rigidbody.position[1]
	rigidbody._transform[2, 3] = rigidbody.position[2]
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
	return Vec3{h.x, h.y, h.x}
}

// Position to homegenous coordinates, h.w == 1
vec3_to_vec4 :: proc(p: Vec3) -> Vec4 {
	return Vec4{p.x, p.y, p.x, 1}
}

get_collider_pos :: proc(c: ^Collider) -> Vec3 {
	// c_pos := linalg.matrix_mul_vector(c.offset, Vec4{0, 0, 0, 0})
	// c_pos = linalg.matrix_mul_vector(c.body._transform, c_pos)
	// NOTE: we could be directly reading off the matrix instead.

	return Vec3{
		c.rigidbody._transform[0, 3],
		c.rigidbody._transform[1, 3],
		c.rigidbody._transform[2, 3],
	}
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
		get_axis(box_coll.rigidbody._transform, 0) + 
		get_axis(box_coll.rigidbody._transform, 1) + 
		get_axis(box_coll.rigidbody._transform, 2)
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

get_collider_axis :: proc(coll: ^Collider, axis: int) -> Vec3 {
	return get_axis(coll.rigidbody._transform, axis)
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
to_relative_pos :: proc(transform: Mat4, relative: Vec3) -> Vec3 {
	inverse := linalg.inverse(transform)
	inverted := inverse * vec3_to_vec4(relative)
	return vec4_to_vec3(inverted)
}

to_world_pos :: proc(transform: Mat4, world: Vec3) -> Vec3 {
	h := transform * vec3_to_vec4(world)
	return vec4_to_vec3(h)
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
	sphere_pos_relative := to_relative_pos(box_coll.rigidbody._transform, sphere_pos)

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

	closest_point := to_world_pos(box_coll.rigidbody._transform, closest_point_relative)

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

	{
		axis_1 := get_collider_axis(box_a_coll, 0)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_1) {return}

		axis_2 := get_collider_axis(box_a_coll, 1)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_2) {return}

		axis_3 := get_collider_axis(box_a_coll, 2)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_3) {return}

		axis_4 := get_collider_axis(box_b_coll, 0)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_4) {return}

		axis_5 := get_collider_axis(box_b_coll, 1)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_5) {return}

		axis_6 := get_collider_axis(box_b_coll, 2)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_6) {return}

		axis_7 := linalg.cross(axis_1, axis_4)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_7) {return}

		axis_8 := linalg.cross(axis_1, axis_5)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_8) {return}

		axis_9 := linalg.cross(axis_1, axis_6)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_9) {return}

		axis_10 := linalg.cross(axis_2, axis_4)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_10) {return}

		axis_11 := linalg.cross(axis_2, axis_5)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_11) {return}

		axis_12 := linalg.cross(axis_2, axis_6)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_12) {return}

		axis_13 := linalg.cross(axis_3, axis_4)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_13) {return}

		axis_14 := linalg.cross(axis_3, axis_5)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_14) {return}

		axis_15 := linalg.cross(axis_3, axis_6)
		if !overlap_on_axis(box_a_coll, box_a, box_a_pos, box_b_coll, box_b, box_b_pos, axis_15) {return}
	}

	// NOTE: I didn't really understand the caching algorithm or whatever that was mentioned in the book.
	// Instead, I've assumed via intuition that we can have at max 4 valid contacts at a time.
	// Instead of caching, I'll just keep track of the 4 contacts with the largest penetration and push those. 

	ContactAccumulator :: struct{
		max_contacts: [4]Contact,
		min_depth : f32,
	}

	acc : ContactAccumulator
	acc.min_depth = math.INF_F32

	push_contact :: proc(acc: ^ContactAccumulator, new_contact: Contact) {
		if new_contact.penetration < acc.min_depth {return}

		acc.min_depth = linalg.min(acc.min_depth, new_contact.penetration)

		for &contact, i in acc.max_contacts {
			if contact.penetration >= new_contact.penetration {continue}
			acc.max_contacts[i] = new_contact
			break
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

				contact, ok := get_contact_sphere_x_box(
					box_a_coll, SphereShape{0}, vertex, 
					box_b_coll, box_b, box_b_pos,
				)
				if ok{
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
	distance := linalg.length(a_to_b)

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
	return (
		box.half_size.x * abs(linalg.dot(axis, get_collider_axis(coll, 1))) +
		box.half_size.y * abs(linalg.dot(axis, get_collider_axis(coll, 2))) +
		box.half_size.z * abs(linalg.dot(axis, get_collider_axis(coll, 3)))
	)
}
