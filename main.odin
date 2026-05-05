package main

import "physics"
import rl "vendor:raylib"
import "core:c"
import "core:math/linalg"
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

	fireworks: [dynamic; 10]Firework,
}

Cube :: struct {
	velocity : Vec3,
	position : Vec3,
	size     : Vec3,
	color    : Color,

	static : bool,
}

Firework :: struct {
	using particle: physics.Particle,
	val: f32,
}

update :: proc(val: ^Firework) {
	particle := cast(^physics.Particle)val
}

load_game_state :: proc() -> ^GameState {
	state := new(GameState)

	state.camera_offset = {0, 0, -1}
	state.camera_up     = {0, 1, 0}
	state.last_monitor  = -1

	reset_particles(state)

	return state
}

reset_particles :: proc(state: ^GameState) {
	clear(&state.fireworks)

	// bouncy ball ahhh
	append(&state.fireworks, Firework{
		force={-1, 0, 0},
		damping=0.995,
		_inverse_mass=1.0/0.01,
		color={255, 0, 0, 255},
		gravity=physics.G,
	})

	append(&state.fireworks, Firework{
		force={-100, 0, 0},
		damping=0.995,
		_inverse_mass=1.0/0.1,
		color={255, 0, 0, 255},
		gravity=physics.G / 2,
	})

	// append(&state.particles, physics.Particle{})
	// append(&state.particles, physics.Particle{})
	// append(&state.particles, physics.Particle{})
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
	}
}


run_game :: proc(state: ^GameState) {
	rl.BeginDrawing(); 
	defer rl.EndDrawing()

	switch {
	case rl.IsKeyPressed(.R):
		reset_particles(state)
	case rl.IsKeyPressed(.ESCAPE):
		state.requested_quit = true
	}

	state.t += rl.GetFrameTime()
	dt := f32(TIMESTEP)

	// Update pass
	for state.t  > 0 {
		state.t -= dt

		update_physics(state, dt)
	}

	// Render pass
	{
		rl.ClearBackground({0, 174, 255, 255})

		state.camera = rl.Camera{
			position   = {-40, -30, -40},
			target     = {},
			up         = {0, 1, 0},
			fovy       = 90,
			projection = .PERSPECTIVE,
		}
		state.camera.target = state.camera.position + {0, 0, 1}

		rl.BeginMode3D(state.camera);
		defer rl.EndMode3D();

		rl.DrawGrid(64, 64)

		for &particle in state.particles {
			rl.DrawSphere(
				particle.position,
				1,
				particle.color,
			)
		}
	}
}

update_physics :: proc(state: ^GameState, dt: f32) {
	for &particle in state.particles {
		physics.particle_integrate(&particle, dt)
	}
}
