package main

import "physics"
import "ui"
import rl "vendor:raylib"
import rlgl "vendor:raylib/rlgl"
import "core:c"
import "core:math/linalg"
import "core:fmt"
import "core:math"

Vec2  :: rl.Vector2
Vec3  :: rl.Vector3
Rot   :: linalg.Quaternionf32
Color :: rl.Color

TIMESTEP    :: 1.0 / 120.0

GameState :: struct {
	requested_quit : bool,
	size           : Vec2,
	t, dt          : f32,
	last_monitor: c.int,

	camera      : rl.Camera,
	camera_offset, camera_up : Vec3,

	items: []Item,
	player_idx : int,

	world: physics.World,
}

Item :: struct {
	velocity : Vec3,
	size     : Vec3,
	color    : Color,

	static : bool,

	rigidbody : physics.Rigidbody,
	coll      : physics.Collider,
	model     : rl.Model,
}

load_game_state :: proc() -> ^GameState {
	state := new(GameState)

	state.camera_offset = {0, 0, -1}
	state.camera_up     = {0, 1, 0}
	state.last_monitor  = -1

	cube_mesh := rl.GenMeshCube(1, 1, 1)
	sphere_mesh := rl.GenMeshSphere(0.5, 64, 64)

	state.items = make([]Item, 4)

	// Cubes
	item := &state.items[0]
	item^ = Item{
		size      = {1,1,1},
		color     = {255, 0, 0, 50},
		rigidbody = physics.rigidbody(position={1.5, 0.5, 0.5}),
		model     = rl.LoadModelFromMesh(cube_mesh),
		coll      = physics.collider(&item.rigidbody, physics.BoxShape{ half_size = Vec3{1, 1, 1} / 2 }),
	}

	item = &state.items[1]
	item^ = Item{
		size      = {1,1,1},
		color     = {255, 0, 0, 50},
		rigidbody = physics.rigidbody(position={2, 0, 0}),
		model     = rl.LoadModelFromMesh(cube_mesh),
		coll      = physics.collider(&item.rigidbody, physics.BoxShape{ half_size = Vec3{1, 1, 1} / 2 }),
	}

	// Sphere

	item = &state.items[2]
	item^ = Item{
		size      = {1,1,1},
		color     = {255, 0, 0, 50},
		rigidbody = physics.rigidbody(position={4, 0, 0}),
		model     = rl.LoadModelFromMesh(sphere_mesh),
		coll      = physics.collider(&item.rigidbody, physics.SphereShape{ radius = 0.5 }),
	}

	item = &state.items[3]
	item^ = Item{
		size      = {1,1,1},
		color     = {255, 0, 0, 50},
		rigidbody = physics.rigidbody(position={6, 0, 0}),
		model     = rl.LoadModelFromMesh(sphere_mesh),
		coll      = physics.collider(&item.rigidbody, physics.SphereShape{ radius = 0.5 }),
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
		state.dt     = rl.GetFrameTime();

		run_game(state)

		free_all(context.temp_allocator)
	}
}

x_angle := f32(0)
y_angle := f32(0)
zoom := f32(1.5)

run_game :: proc(state: ^GameState) {
	rl.BeginDrawing(); 
	defer rl.EndDrawing()

	state.t += rl.GetFrameTime()
	dt := f32(TIMESTEP)

	movement : Vec3
	x_rot, y_rot : f32

	// Input
	if rl.IsKeyPressed(.ESCAPE) {
		state.requested_quit = true 
	}
	if rl.IsKeyDown(.W) { movement.z = 10;  }
	if rl.IsKeyDown(.S) { movement.z -= 10; }
	if rl.IsKeyDown(.A) { movement.x -= 10; }
	if rl.IsKeyDown(.D) { movement.x = 10; }
	if rl.IsKeyDown(.Q) { movement.y -= 10; }
	if rl.IsKeyDown(.E) { movement.y = 10;  }
	if rl.IsKeyPressed(.TAB) {  
		state.player_idx = (state.player_idx + 1) % len(state.items)
	}
	if rl.IsKeyDown(.UP) {    x_rot += 1;  }
	if rl.IsKeyDown(.DOWN) {  x_rot -= 1;  }
	if rl.IsKeyDown(.LEFT) {  y_rot += 1;  }
	if rl.IsKeyDown(.RIGHT) { y_rot -= 1;  }

	x_angle += x_rot * dt * 4
	y_angle += y_rot * dt * 4

	player := &state.items[state.player_idx]
	player.rigidbody.position += movement * dt

	// Update
	for state.t  > 0 {
		state.t -= dt

		update_physics(state, dt)
	}

	// Render
	{
		rl.ClearBackground({0, 174, 255, 255})

		rot := linalg.quaternion_from_euler_angle_y(y_angle) *
			linalg.quaternion_from_euler_angle_x(x_angle)
		offset := linalg.quaternion_mul_vector3(rot, Vec3{0, 0, 1.5 * zoom})

		state.camera = rl.Camera{
			position   = player.rigidbody.position + offset,
			target     = player.rigidbody.position,
			up         = {0, 1, 0},
			fovy       = 90,
			projection = .PERSPECTIVE,
		}

		// 3d stuff
		{
			rl.BeginMode3D(state.camera);
			rlgl.EnableDepthTest()

			defer rl.EndMode3D();

			rl.DrawGrid(64, 64)

			for &item in state.items {
				axis, angle := linalg.angle_axis_from_quaternion(item.rigidbody.rotation)
				rl.DrawModelEx(
					item.model,
					item.rigidbody.position,
					angle, axis,
					1,
					item.color
				)
			}

			rlgl.DisableDepthTest()

			for idx in 0..<state.world.contact_idx {
				contact := &state.world.contacts[idx]
				rl.DrawLine3D(contact.position, contact.position + contact.penetration * contact.normal, Color{255, 0, 0, 255})
				rl.DrawSphere(contact.position, 0.05, Color{255, 0, 0, 255})
			}
		}

		// UI 
		{
			rect := ui.rect_from_size(state.size.x, state.size.y)

			ui.inset(&rect, 10)

			font_size := f32(100)

			row := ui.cut_top(&rect, font_size)
			draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "contacts: %v", state.world.contact_idx)

			row = ui.cut_top(&rect, font_size)
			draw_text(row.x, row.y, {0, 0, 0, 255}, font_size, "pos: %v", player.rigidbody.position)
		}
	}
}

draw_text :: proc(x, y: f32, color: Color, font_size: f32, format: string, args: ..any) {
	str := fmt.ctprintf(format, ..args)
	rl.DrawText(str, c.int(x), c.int(y), c.int(font_size), color);
}

update_physics :: proc(state: ^GameState, dt: f32) {
	ground_plane := physics.Collider{
		local_offset = linalg.matrix4_translate_f32({0, -0.25, 0}),
		shape        = physics.PlaneShape{ normal = {0, 1, 0 } }
	}
	physics.coll_recompute_transform(&ground_plane)

	physics.begin_world(&state.world)
	for &item in state.items {
		physics.rb_recompute_transform(&item.rigidbody)
		physics.coll_recompute_transform(&item.coll)
	}

	// Collide everything with the ground
	for &item in state.items {
		physics.generate_contacts_for_colliders(&ground_plane, &item.coll, &state.world)
	}

	// Collide everything with everything else
	for i in 0..<len(state.items) {
		for j in i+1..<len(state.items) {
			item1 := &state.items[i]
			item2 := &state.items[j]
			physics.generate_contacts_for_colliders(&item1.coll, &item2.coll, &state.world)
		}
	}

	for idx in 0..<state.world.contact_idx {
		contact := &state.world.contacts[idx]
		if contact.colliders[0].rigidbody != nil {
			contact.colliders[0].rigidbody.position += contact.normal * contact.penetration * dt * 0.5
		}
	}
}
