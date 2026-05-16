package main

import "core:c"
import "core:fmt"
import "core:math"
import "core:math/linalg"
import "physics"
import "ui"
import rl "vendor:raylib"
import rlgl "vendor:raylib/rlgl"

Vec2 :: rl.Vector2
Vec3 :: physics.Vec3
Rot :: linalg.Quaternionf32
Color :: rl.Color

TIMESTEP :: 1.0 / 120.0

GameState :: struct {
	requested_quit:           bool,
	size:                     Vec2,
	t, dt:                    f32,
	last_monitor:             c.int,
	camera:                   rl.Camera,
	camera_offset, camera_up: Vec3,
	items:                    []Item,
	player_idx:               int,
	world:                    physics.World,
	disable_physics:    bool,
	slow: bool,
}

Item :: struct {
	velocity:  Vec3,
	size:      Vec3,
	color:     Color,
	static:    bool,
	rigidbody: physics.Rigidbody,
	coll:      physics.Collider,
	model:     rl.Model,
}

load_game_state :: proc() -> ^GameState {
	state := new(GameState)

	state.camera_offset = {0, 0, -1}
	state.camera_up = {0, 1, 0}
	state.last_monitor = -1

	cube_mesh := rl.GenMeshCube(1, 1, 1)
	sphere_mesh := rl.GenMeshSphere(0.5, 64, 64)

	state.items = make([]Item, 2)

	// Cubes
	item := &state.items[0]
	item^ = Item {
		size      = {1, 1, 1},
		color     = {255, 0, 0, 50},
		rigidbody = physics.rigidbody(
			position = {0, 3, 0},
			inverse_inertia_tensor_local = physics.INERTIA_TENSOR_IDENTITY,
		),
		model     = rl.LoadModelFromMesh(cube_mesh),
		coll      = physics.collider(
			&item.rigidbody,
			physics.BoxShape{half_size = Vec3{0.5, 0.5, 0.5}},
		),
	}

	// item = &state.items[1]
	// item^ = Item{
	// 	size      = {1,1,1},
	// 	color     = {255, 0, 0, 50},
	// 	rigidbody = physics.rigidbody(position={2, 1, 0}),
	// 	model     = rl.LoadModelFromMesh(cube_mesh),
	// 	coll      = physics.collider(&item.rigidbody, physics.BoxShape{ half_size = Vec3{1, 1, 1} / 2 }),
	// }

	// Sphere

	// item := &state.items[0]
	// item^ = Item{
	// 	size      = {1,1,1},
	// 	color     = {255, 0, 0, 50},
	// 	rigidbody = physics.rigidbody(position={0, 3, 0}),
	// 	model     = rl.LoadModelFromMesh(sphere_mesh),
	// 	coll      = physics.collider(&item.rigidbody, physics.SphereShape{ radius = 0.5 }),
	// }

	item = &state.items[1]
	item^ = Item {
		size      = {1, 1, 1},
		color     = {255, 0, 0, 50},
		rigidbody = physics.rigidbody(
			position = {0.2, 0, 0.2},
			// The ground!
			inverse_mass = 0,
			inverse_inertia_tensor_local = physics.INERTIA_TENSOR_INFINITE_MASS,
		),
		model     = rl.LoadModelFromMesh(cube_mesh),
		coll      = physics.collider(
			&item.rigidbody,
			// physics.BoxShape{half_size = Vec3{0.5, 0.5, 0.5}},
			physics.PlaneShape{normal={0, 1, 0}},
		),
	}

	state.world = physics.make_world(max_num_contacts = 1024)

	return state
}

free_game_state :: proc(state: ^GameState) {
	physics.delete_world(&state.world)
}

main :: proc() {
	rl.SetConfigFlags({.VSYNC_HINT})
	rl.InitWindow(800, 600, "Infinite Horizon")
	rl.SetWindowState({.WINDOW_MAXIMIZED, .WINDOW_RESIZABLE})
	rl.SetExitKey(.KEY_NULL)

	set_logging_type(.Fmt)

	defer rl.CloseWindow()

	state := load_game_state()

	for !rl.WindowShouldClose() && !state.requested_quit {
		monitor := rl.GetCurrentMonitor()
		if state.last_monitor != monitor {
			state.last_monitor = monitor
			rl.SetTargetFPS(rl.GetMonitorRefreshRate(monitor))
		}

		state.size.x = f32(rl.GetScreenWidth())
		state.size.y = f32(rl.GetScreenHeight())
		state.dt = rl.GetFrameTime()

		run_game(state)

		free_all(context.temp_allocator)
	}
}

x_angle := f32(0)
y_angle := f32(0)
zoom := f32(1.5)

run_game :: proc(state: ^GameState) {
	rl.BeginDrawing()
	defer rl.EndDrawing()

	state.t += rl.GetFrameTime()
	dt := f32(TIMESTEP)

	movement: Vec3
	x_rot, y_rot: f32

	// Input
	if rl.IsKeyPressed(.ESCAPE) {
		state.requested_quit = true
	}
	if rl.IsKeyDown(.W) {movement.z = 10}
	if rl.IsKeyDown(.S) {movement.z -= 10}
	if rl.IsKeyDown(.A) {movement.x -= 10}
	if rl.IsKeyDown(.D) {movement.x = 10}
	if rl.IsKeyDown(.Q) {movement.y -= 10}
	if rl.IsKeyDown(.E) {movement.y = 10}
	if rl.IsKeyPressed(.TAB) {
		state.player_idx = (state.player_idx + 1) % len(state.items)
	}
	if rl.IsKeyDown(.UP) {x_rot += 1}
	if rl.IsKeyDown(.DOWN) {x_rot -= 1}
	if rl.IsKeyDown(.LEFT) {y_rot += 1}
	if rl.IsKeyDown(.RIGHT) {y_rot -= 1}

	if rl.IsKeyPressed(.SPACE) {
		state.disable_physics = !state.disable_physics
	}

	x_angle += x_rot * dt * 4
	y_angle += y_rot * dt * 4

	player := &state.items[state.player_idx]
	player.rigidbody.position += movement * dt

	if rl.IsKeyDown(.ONE) {
		player.rigidbody.rotation = physics.quat_rotate_by_axis(player.rigidbody.rotation, Vec3{4 * dt, 0, 0})
	} else if rl.IsKeyDown(.TWO) {
		player.rigidbody.rotation = physics.quat_rotate_by_axis(player.rigidbody.rotation, Vec3{ -4 * dt, 0, 0})
	}

	if rl.IsKeyDown(.THREE) {
		player.rigidbody.rotation = physics.quat_rotate_by_axis(player.rigidbody.rotation, Vec3{0, 4 * dt, 0})
	} else if rl.IsKeyDown(.FOUR) {
		player.rigidbody.rotation = physics.quat_rotate_by_axis(player.rigidbody.rotation, Vec3{0, -4 * dt, 0})
	}

	if rl.IsKeyDown(.R) {
		// Reset stuff
		player.rigidbody.velocity = 0
		player.rigidbody.acceleration = 0
		player.rigidbody.acceleration_last_frame = 0
		player.rigidbody.angular_velocity = 0
	}
	if rl.IsKeyDown(.P) {
		player.rigidbody.position = {0, 4, 0}
	}
	if rl.IsKeyDown(.T) { // .R was taken
		player.rigidbody.rotation = physics.QUAT_IDENTITY
	}
	if rl.IsKeyPressed(.S) {
		state.slow = !state.slow
	}

	if !state.disable_physics {
		// Update
		// for state.t  > 0 {
		// 	state.t -= dt

		dt := dt
		if state.slow {
			dt = dt / 10
		}

		update_physics(state, dt)
		// }
	}

	// Render
	{
		rl.ClearBackground({0, 174, 255, 255})

		rot :=
			linalg.quaternion_from_euler_angle_y(y_angle) *
			linalg.quaternion_from_euler_angle_x(x_angle)
		offset := linalg.quaternion_mul_vector3(rot, Vec3{0, 0, 1.5 * zoom})

		state.camera = rl.Camera {
			position   = player.rigidbody.position + offset,
			target     = player.rigidbody.position,
			up         = {0, 1, 0},
			fovy       = 90,
			projection = .PERSPECTIVE,
		}

		// 3d stuff
		{
			rl.BeginMode3D(state.camera)
			rlgl.EnableDepthTest()

			defer rl.EndMode3D()

			rl.DrawGrid(64, 64)

			for &item in state.items {
				angle, axis := linalg.angle_axis_from_quaternion(item.rigidbody.rotation)
				rl.DrawModelEx(
					item.model,
					item.rigidbody.position,
					axis,
					math.DEG_PER_RAD * angle,
					1,
					item.color,
				)
			}

			rlgl.DisableDepthTest()

			for idx in 0 ..< state.world.contacts_idx {
				contact := &state.world.contacts[idx]


				rl.DrawLine3D(
					contact.position,
					contact.position + 1 * contact.normal,
					Color{0, 255, 0, 255},
				)
				rl.DrawLine3D(
					contact.position,
					contact.position + contact.penetration * contact.normal,
					Color{255, 0, 0, 255},
				)

				for body in contact.bodies {
					relative_position := contact.position - body.position

					// axis
					rl.DrawLine3D(body.position, body.position + linalg.quaternion_mul_vector3(body.rotation, Vec3{1, 0, 0}), {255, 255, 255, 255})
					rl.DrawLine3D(body.position, body.position + linalg.quaternion_mul_vector3(body.rotation, Vec3{0, 1, 0}), {255, 255, 255, 255})
					rl.DrawLine3D(body.position, body.position + linalg.quaternion_mul_vector3(body.rotation, Vec3{0, 0, 1}), {255, 255, 255, 255})

					// Relative pos
					rl.DrawLine3D(body.position, contact.position, Color{255, 255, 0, 255})



					// #0000FF This is what the angular velocity SHOULD be
					torque_axis := linalg.cross(relative_position, contact.normal)
					rl.DrawLine3D(body.position, body.position + torque_axis, Color{0, 0, 255, 255})
					// #008888
					rotation_per_unit_impulse := body._inverse_inertia_tensor * torque_axis
					rl.DrawLine3D(body.position, body.position + rotation_per_unit_impulse, Color{0, 128, 128, 255})
					// #FF8800 Torque-induced-velocity at point
					// velocity_from_torque := linalg.cross(torque_axis, relative_position)
					// rl.DrawLine3D(contact.position, contact.position + velocity_from_torque, Color{255, 125, 0, 255})


					// #FF00FF - linear velocity
					// rl.DrawLine3D(body.position, body.position + body.velocity, Color{255, 0, 255, 255})
					// #00FFFF - angular velocity(actual)
					// rl.DrawLine3D(body.position, body.position + body.angular_velocity, Color{0, 255, 255, 255})

				}

				rl.DrawSphere(contact.position, 0.05, Color{255, 0, 0, 255})
			}
		}

		// UI
		{
			rect := ui.rect_from_size(state.size.x, state.size.y)

			ui.inset(&rect, 10)

			font_size := f32(40)

			row := ui.cut_top(&rect, font_size); {
				draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "disabled: %v, slow: %v", state.disable_physics, state.slow)
			}

			row = ui.cut_top(&rect, font_size); {
				draw_text(
					row.x,
					row.y,
					{0, 0, 0, 255},
					font_size,
					"contacts: %v",
					state.world.contacts_idx,
				)
			}

			row = ui.cut_top(&rect, font_size); {
				draw_text(
					row.x,
					row.y,
					{0, 0, 0, 255},
					font_size,
					"pos: %v",
					player.rigidbody.position,
				)
			}

			row = ui.cut_top(&rect, font_size); {
				draw_text(
					row.x,
					row.y,
					{0, 0, 0, 255},
					font_size,
					"acc: %v",
					player.rigidbody.acceleration_last_frame,
				)
			}

			row = ui.cut_top(&rect, font_size); {
				draw_text(
					row.x,
					row.y,
					{0, 0, 0, 255},
					font_size,
					"vel: %v",
					player.rigidbody.velocity,
				)
			}

			row = ui.cut_top(&rect, font_size); {
				draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "size: %v", player.size)
			}

			row = ui.cut_top(&rect, font_size); {
				scale := Vec3 {
					linalg.length(player.rigidbody._transform[0]),
					linalg.length(player.rigidbody._transform[1]),
					linalg.length(player.rigidbody._transform[2]),
				}

				draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "scale: %v", scale)
			}

			row = ui.cut_top(&rect, font_size); {
				scale := Vec3 {
					linalg.length(player.coll._transform[0]),
					linalg.length(player.coll._transform[1]),
					linalg.length(player.coll._transform[2]),
				}

				draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "scale2: %v", scale)
			}

			axis, angle := linalg.angle_axis_from_quaternion(player.rigidbody.rotation)
			row = ui.cut_top(&rect, font_size); {
				draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "axis: %v", axis)
			}
			row = ui.cut_top(&rect, font_size); {
				draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "angle: %v", angle)
			}
		}
	}
}

draw_text :: proc(x, y: f32, color: Color, font_size: f32, format: string, args: ..any) {
	str := fmt.ctprintf(format, ..args)
	rl.DrawText(str, c.int(x), c.int(y), c.int(font_size), color)
}

update_physics :: proc(state: ^GameState, dt: f32) {
	dt := dt
	if dt > 0.1 {dt = 0.1}

	for &item in state.items {
		physics.rb_add_acceleration(&item.rigidbody, {0, -10, 0})
	}

	for &item in state.items {
		physics.rb_recompute_derived(&item.rigidbody)
		physics.collider_recompute_transform(&item.coll)
	}
	// Collide everything with everything else
	physics.clear_contacts(&state.world) // TODO: consider just putting this into resolve_contacts
	for i in 0 ..< len(state.items) {
		for j in i + 1 ..< len(state.items) {
			item1 := &state.items[i]
			item2 := &state.items[j]
			physics.generate_contacts_for_colliders(&item1.coll, &item2.coll, &state.world)
		}
	}

	for &item in state.items {
		physics.rb_integrate(&item.rigidbody, dt)
	}
	physics.resolve_contacts(&state.world, dt)

	for &item in state.items {
		physics.rb_recompute_derived(&item.rigidbody)
		physics.collider_recompute_transform(&item.coll)
	}
}
