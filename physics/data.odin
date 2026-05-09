// The data model lives here

package physics

import "core:math"
import "core:math/linalg"

// Created while reading Ian Millington - Game Physics-Engine Development. Actually amazing book.
// However I have opted to use inbuilt odin stuff where possible, and a procedural style instead of OOP.
// We may actually be slower! There are a lot of times in the book where we can take 
// shortcuts or do things ourselves because we understand how the maths works.
// I.e rather than Matrix * { 1, 0, 0 } to get an axis, we can just read the first colum
// out of the matrix.
// Or, instead of storing 4x4 matrices everywhere, we can just store a 3x4 matrix, which 
// saves 4 components and effectively represents the same data. Instead of passing Vec4s around
// Everywhere, can just always assume z=1, and convert up and down as needed, etc.
// The code will be more heavily commented than usual, so that I can continue to understand it.

Vec3  :: linalg.Vector3f32
Vec4  :: linalg.Vector4f32
Quat  :: linalg.Quaternionf32
Mat4  :: linalg.Matrix4f32
Mat3  :: linalg.Matrix3f32
Color :: [4]f32

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
	rotation : Quat, // aka orientation
	local_inertia_tensor : Mat3,

	// Derived data

	_transform, _transform_inverse : Mat4,
	_inverse_inertia_tensor: Mat3,
}

// Useful for objects that dont already have one ig.
// I would consider removing though
NILL_RIGIDBODY := Rigidbody{}

rigidbody :: proc(position := Vec3{0, 0, 0}, rotation := linalg.QUATERNIONF32_IDENTITY) -> Rigidbody {
	return {
		position = position,
		rotation = rotation,
	}
}

// TODO: consider doing this lazily
rb_recompute_derived :: proc(rigidbody : ^Rigidbody) {
	rigidbody._transform = linalg.matrix4_from_quaternion(rigidbody.rotation)
	rigidbody._transform[0, 3] = rigidbody.position[0]
	rigidbody._transform[1, 3] = rigidbody.position[1]
	rigidbody._transform[2, 3] = rigidbody.position[2]

	rigidbody._transform_inverse = linalg.matrix4_inverse(rigidbody._transform)

	// The implementation in the book was optimized with a code generator.
	world_rotation       := linalg.matrix3_from_matrix4(rigidbody._transform)
	inertia_tensor_world := world_rotation * rigidbody.local_inertia_tensor
	rigidbody._inverse_inertia_tensor = linalg.inverse(inertia_tensor_world)
}

rb_relative_pos :: proc(rigidbody: ^Rigidbody, world: Vec3) -> Vec3 {
	return mat4_mul_vec3(rigidbody._transform_inverse, world)
}

rb_world_pos :: proc(rigidbody: ^Rigidbody, relative: Vec3) -> Vec3 {
	return mat4_mul_vec3(rigidbody._transform, relative)
}

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

collider_position :: proc(c: ^Collider) -> Vec3 {
	return get_axis(c._transform, 3)
}

collider :: proc(rb: ^Rigidbody, shape: ColliderShape, offset := linalg.MATRIX4F32_IDENTITY) -> Collider {
	return Collider {
		rigidbody    = rb,
		local_offset = offset,
		shape        = shape,
	}
}

collider_recompute_transform :: proc(coll: ^Collider) {
	if coll.rigidbody == nil {
		coll._transform = coll.local_offset
	} else {
		coll._transform = coll.rigidbody._transform * coll.local_offset
	}

	coll._transform_inverse = linalg.matrix4_inverse(coll._transform)
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

// NOTE: Assumes collider itself was not also transformed.
// TODO: Each collider might need it's own matrix pairs.
get_collider_axis :: proc(coll: ^Collider, axis: int) -> Vec3 {
	return get_axis(coll._transform, axis)
}

collider_relative_pos :: proc(coll: ^Collider, world: Vec3) -> Vec3 {
	return mat4_mul_vec3(coll._transform_inverse, world)
}

collider_world_pos :: proc(coll: ^Collider, relative: Vec3) -> Vec3 {
	return mat4_mul_vec3(coll._transform, relative)
}

// TODO: Mesh collider. 
// With this done, we can make literally anything.

Contact :: struct {
	// The position of the contact in world coordinates, where other_coll should push back on coll.
	position    : Vec3,
	// The direction in which other_coll should push back on coll
	normal      : Vec3,
	// The amount which we should move coll in the direction of normal such that they are just touching.
	penetration : f32,

	// The collider doing the colliding
	collider: ^Collider,
	// The collider resisting the collision
	// NOTE: we only generate one pair of contacts, and assume your resolution pass will 
	// also handle other_collider -> collider as well for each contact we generate
	other_collider: ^Collider,
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
