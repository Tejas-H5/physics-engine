package main

import "core:math"
import "core:math/linalg"

rotate_dir_along_axis :: proc(dir: Vec3, axis: Vec3, angle: f32) -> Vec3 {
	rotation := linalg.quaternion_angle_axis(angle, axis)
	return linalg.quaternion_mul_vector3(rotation, dir)
}

get_axis :: proc(negative, positive: bool) -> f32 {
	switch {
	case positive: return 1;
	case negative: return -1;
	case:          return 0
	}
}

vector_is_nan :: proc "contextless" (v: Vec3) -> bool {
	return math.is_nan(v.x) || math.is_nan(v.y) || math.is_nan(v.z)
}

vector_slerp_safe :: proc "contextless" (x, y: Vec3, t: f32) -> Vec3 {
	res := linalg.vector_slerp(x, y, t)
	if !vector_is_nan(res) {
		return res
	}
	return y
}
