package physics

import "core:math"
import "core:math/linalg"

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

