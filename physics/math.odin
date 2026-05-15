// Math helpers

package physics

import "core:math"
import "core:math/linalg"

mat4_mul_vec3 :: proc(mat: Mat4, vec: Vec3) -> Vec3 {
	result := mat * vec3_to_vec4(vec)
	return vec4_to_vec3(result)
}

// Homegenous coordinates to position, assuming h.w == 1
vec4_to_vec3 :: proc(h: Vec4) -> Vec3 {
	return Vec3{h.x, h.y, h.z}
}

// Position to homegenous coordinates, h.w == 1
vec3_to_vec4 :: proc(p: Vec3) -> Vec4 {
	return Vec4{p.x, p.y, p.z, 1}
}

// NOTE: This matrix effectively represents a change in rotation, with no scaling or skewing.
// This means you can multiply by it's transpose (just use a different proc, no need to compute the transpose or anything) to get the inverse transformation.
make_orthonormal_basis :: proc(normal: Vec3) -> Mat3 {
	tangent2, tangent1: Vec3

	// Use projecting to a plane and -y/x trick to get a perpendicular vector.
	if abs(normal.x) > abs(normal.y) {
		// Were' nearer the X axis, so use the Y axis

		scale_factor := 1 / linalg.sqrt(normal.z * normal.z + normal.x * normal.x)
		tangent1 = {normal.z * scale_factor, 0, -normal.x * scale_factor}

		// The new Y axis is at right angles to the new X and Z axes
		// NOTE: This is just a cross product, but z.y terms are 0
		tangent2 = {
			normal.y * tangent1.x,
			normal.z * tangent1.x - normal.x * tangent1.z,
			-normal.y * tangent1.x
		}
	} else {
		// We're nearer the Y axis, so use the X axis

		scale_factor := 1 / linalg.sqrt(normal.z * normal.z + normal.y * normal.y)
		tangent1 = {0, -normal.z * scale_factor, normal.y * scale_factor }

		// The new Y axis is at right angles to the new X and Z axes
		// NOTE: This is just a cross product, but z.x terms are 0
		tangent2 = {
			normal.y * tangent1.z - normal.z * tangent1.y,
			-normal.x * tangent1.z,
			normal.x * tangent1.y,
		}
	}

	mat : Mat3
	mat[0] = normal
	mat[1] = tangent1
	mat[2] = tangent2
	return mat
}

/*
Getting column 1, 2, or 3 of a matrix should be equivelant to doing
M*Vec3{1, 0, 0}, M*Vec3{0, 1, 0}, M*Vec3{0, 0, 1}, respectively.
Getting column 4 is equivelant to getting the translation component.
*/
get_axis :: proc(mat: Mat4, axis: int) -> Vec3 #no_bounds_check {
	// TODO: just inline - I didn't know odin had this at the time lol.
	return mat[axis].xyz
}

// Finds the closest points between two infinite 3D lines.
// It's a copypaste of https://paulbourke.net/geometry/pointlineplane/lineline.c
closest_points_between_lines :: proc(p1, p2, p3, p4: Vec3) -> (pa: Vec3, ta: f32, pb: Vec3, tb: f32, are_parallel: bool) {
	p13 := p1 - p3
	p43 := p4 - p3

	EPS :: 0.000001

	if abs(p43.x) > EPS || abs(p43.y) > EPS || abs(p43.z) > EPS {
		p21 := p2 - p1
		if abs(p21.x) > EPS || abs(p21.y) > EPS || abs(p21.z) > EPS {
			d1343 := p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
			d4321 := p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
			d1321 := p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
			d4343 := p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
			d2121 := p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

			denom := d2121 * d4343 - d4321 * d4321;
			if (abs(denom) > EPS) {
				numer := d1343 * d4321 - d1321 * d4343;

				ta = numer / denom;
				tb = (d1343 + d4321 * (ta)) / d4343;

				pa = p1 + ta * p21
				pb = p3 + tb * p43

				are_parallel = false
			}
		}
	}

	return
}

// 'axis' merges axis/angle representation by making the length of the vector the angle
// I saw it in https://github.com/idmillington/cyclone-physics/blob/d75c8d9edeebfdc0deebe203fe862299084b1e30/include/cyclone/core.h#L512
// (addScaledVector) so it must be right. Right? 
quat_rotate_by_axis :: proc(quat: Quat, axis: Vec3) -> Quat {
	angle := linalg.length(axis)
	axis := abs(angle) < 0.000001 ? Vec3{0, 1, 0} : axis / angle
	axis_quat := linalg.quaternion_angle_axis(angle, axis)

	return axis_quat * quat

	// Why doesn't this work xDDD
	// result := quat
	//
	// q := Quat{}
	// q.x = axis.x
	// q.y = axis.y
	// q.z = axis.z
	// q.w = 0
	//
	// return linalg.mul(quat, q)
}
