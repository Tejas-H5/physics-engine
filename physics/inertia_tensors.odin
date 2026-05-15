package physics

import "core:math"
import "core:math/linalg"

// The 'intertia tensor' (OMG) for our usecase is just a 3x3 matrix. 
// It represents how easy it is to cause an object to rotate along a particular exist.
// The moment of inertia along a particular axis (through the objects center of mass) 'a' can be computed by 
// summing the squared distance from the particle to the axis of rotation.
// Ia = sum(i=1 -> n, dist(axis, particle[i].position)^2 * particle[i].mass)
// The physics engine has no atoms or whatever, this is just an analogy.
// I will use the relative bounding box of an object to estimate the inertia tensor.
// Because I cannot simply write into the matrix like Ian Millington xd
// NOTE: if you believe your object is very aerodynamic (or not) along a particular axis,
// you'll need to emulate this be passing in, for example, a wider width along X to model the inertia tensor of a plane for example.
// (Or maybe you would use your own drag tensor ?? I'm not sure yet)
// Not really sure how the products of inertia work just yet ..
// You compute them as
// Ixy = sum(
//     i=1 -> n, 
//     dot(x_axis, particle[i].position) * dot(y_axis, particle[i].position) * 
//     particle[i].mass
// )
// But I cannot describe why, and how they work just yet.
// > It is entirely possible to have a non-positive total
// > product of inertia. Zero values are particularly common for many different shaped
// > objects
// so maybe you should really be making it from scratch? for now I estimate it with size.x * size.y
// At least for boxes they're zero. Will use the more and then update this wall of text


inertia_tensor_box :: proc "contextless" (half_size: Vec3, mass: f32) -> Mat3 {
	sx := half_size.x * 2
	sy := half_size.y * 2
	sz := half_size.z * 2

	i_x := (1.0 / 12.0) * mass * (sy * sy + sz * sz)
	i_y := (1.0 / 12.0) * mass * (sx * sx + sz * sz)
	i_z := (1.0 / 12.0) * mass * (sx * sx + sy * sy)

	return Mat3{
		i_x, 0, 0,
		0, i_y, 0,
		0, 0, i_z,
	}
}

inertia_tensor_sphere :: proc(radius: f32, mass: f32) -> Mat3 {
	val := (2.0 / 5.0) * mass * radius * radius

	return Mat3{
		val, 0, 0,
		0, val, 0,
		0, 0, val,
	}
}


