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

QUAT_IDENTITY :: linalg.QUATERNIONF32_IDENTITY

World :: struct {
	contacts    : []Contact,
	contacts_idx : int,
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
	position, velocity, acceleration : Vec3,
	acceleration_last_frame : Vec3,
	inverse_mass: f32,
	force_accum: Vec3,

	rotation : Quat,
	angular_velocity, angular_acceleration : Vec3,
	inverse_inertia_tensor_local : Mat3,
	torque_accum: Vec3,

	// Damping is required to remove energy added
	// through numerical instability in the integrator
	linear_damping, angular_damping : f32,


	// Derived data

	_transform, _transform_inverse : Mat4,
	_inverse_inertia_tensor: Mat3,
}

Collider :: struct {
	rigidbody    : ^Rigidbody,
	shape        : ColliderShape,
	local_offset : Mat4,

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

// TODO: Mesh collider. 
// With this done, we can make literally anything.

Contact :: struct {
	// The position of the contact in world coordinates, where other_coll should push back on coll.
	// It's directly in-between the overlap. It needs to be - a lot of the contact resolution stuff
	// assumes we can reuse this position for both bodies when resolving the contact, so 
	// it can't be on the edge of either collider (which is how I had it before)
	position    : Vec3,
	// The direction in which other_coll should push back on coll
	normal      : Vec3,
	// The amount which we should move coll in the direction of normal such that they are just touching.
	penetration : f32,

	// The bodies colliding with each other. The position and normal are from the POV of the first body.
	// It is a deliberate deicision to not include the actual collider.
	// A collider is just one mechanism of creating a contact.
	// Any user code could also create contacts, and
	// have them resolve the same way as internal engine code!
	bodies: [2]^Rigidbody,

	// Contact resolution step - computed values
	// TODO: all our matricies should be like Target_from_Source. 
	// The names will touch each other in the code, i.e contact_pos = contact_from_world * world_pos
	_world_from_contact         : Mat3, 
	_contact_from_world         : Mat3,
	_relative_velocity          : Vec3,
	_desired_delta_velocity     : f32,
	_relative_contact_positions : [2]Vec3,
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
