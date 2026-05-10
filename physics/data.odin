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
	position, velocity, acceleration : Vec3,
	acceleration_last_frame : Vec3,
	inverse_mass: f32,
	force_accum: Vec3,

	rotation : Quat,
	angular_velocity, angular_acceleration : Vec3,
	inertia_tensor_local : Mat3,
	torque_accum: Vec3,

	// Damping is required to remove energy added
	// through numerical instability in the integrator
	linear_damping, angular_damping : f32,


	// Derived data

	_transform, _transform_inverse : Mat4,
	_inverse_inertia_tensor: Mat3,
}

// Useful for objects that dont already have one ig.
// I would consider removing though
NILL_RIGIDBODY := Rigidbody{}

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
	position          : Vec3,
	// The direction in which other_coll should push back on coll
	normal      : Vec3,
	// The amount which we should move coll in the direction of normal such that they are just touching.
	penetration : f32,

	// The body doing the colliding, and the body resisting the collision.
	// It is a deliberate deicision to not include the actual collider.
	// A collider is just one mechanism of creating a contact.
	// Any user code could also create contacts, and
	// have them resolve the same way as internal engine code!
	rigidbody, other_rigidbody: ^Rigidbody,
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
