pub use chaos_framework::{*, Cuboid};
use rand::distributions::{uniform::SampleUniform, Uniform};
use rapier3d::{na::{self, vector}, prelude::{DefaultBroadPhase, RigidBodyHandle}};

use crate::physics::RapierPhysicsWorld;
use rapier3d::prelude::*;

const BOX_LENGTH: f32 = 1.25;
const BOX_HEIGHT: f32 = 2.75;
const BOX_WIDTH: f32 = 0.05;

pub fn run() {
    println!("GAME STARTING HELP ME PLEASE HELP ME HES KEPT ME HOSTAGE");
    
    let mut el = EventLoop::new(800, 800);
    unsafe {
        Enable(DEPTH_TEST);
    }
    el.window.set_title("PTetris");
    let mut renderer = Renderer::new();
    // renderer.camera.set_projection(ProjectionType::Orthographic);
    
    /* initialize the physics world with the singular line of boiler plate */
    let mut broad_phase = DefaultBroadPhase::new();
    let mut physics_world = RapierPhysicsWorld::new(&mut broad_phase);

    renderer.camera.pos = vec3(0.025, 0.75, 2.5);
    renderer.camera.yaw = -90.0;
    renderer.camera.pitch = -14.0;

    let box_size = vec2(BOX_LENGTH, BOX_HEIGHT);
    let box_color = vec4(0.1, 0.2, 0.3, 1.);

    let pieces_box = vec![
        renderer.add_mesh(Cuboid::new(vec3(box_size[0], BOX_WIDTH, box_size[0]), box_color).mesh()).unwrap(),
        renderer.add_mesh(Cuboid::new(vec3(BOX_WIDTH, box_size[1], box_size[0]), box_color).mesh()).unwrap(),
        renderer.add_mesh(Cuboid::new(vec3(box_size[0], BOX_WIDTH, box_size[0]), box_color).mesh()).unwrap(),
        renderer.add_mesh(Cuboid::new(vec3(BOX_WIDTH, box_size[1], box_size[0]), box_color).mesh()).unwrap(),
        renderer.add_mesh(Cuboid::new(vec3(box_size[0], box_size[1], BOX_WIDTH), box_color).mesh()).unwrap(),
    ];

    for counter in 0..pieces_box.len()+1 {
       match counter{ 
        0 => {
            renderer.get_mesh_mut(pieces_box[counter]).unwrap().set_position(vec3(0., box_size[1]/2., 0.));
            physics_world.add_static_cube_rigidbody(vec3(0., box_size[1]/2., 0.), vec3(box_size[0], BOX_WIDTH, box_size[0]));
        }

        1 => {
            renderer.get_mesh_mut(pieces_box[counter]).unwrap().add_position(vec3(box_size[0]/2., 0., 0.));
            physics_world.add_static_cube_rigidbody(vec3(box_size[0]/2., 0., 0.), vec3(BOX_WIDTH, box_size[1], box_size[0]));
        }
        
        2 => {
            renderer.get_mesh_mut(pieces_box[counter]).unwrap().add_position(vec3(0., -box_size[1]/2., 0.));
            physics_world.add_static_cube_rigidbody(vec3(0., -box_size[1]/2., 0.), vec3(box_size[0], BOX_WIDTH, box_size[0]));
        }

        3 => {
            renderer.get_mesh_mut(pieces_box[counter]).unwrap().add_position(vec3(-box_size[0]/2., 0., 0.));
            physics_world.add_static_cube_rigidbody(vec3(-box_size[0]/2., 0., 0.), vec3(BOX_WIDTH, box_size[1], box_size[0]));
        }

        4 => {
            renderer.get_mesh_mut(pieces_box[counter]).unwrap().add_position(vec3(0., 0., -box_size[0]/2.));
            physics_world.add_static_cube_rigidbody(vec3(0., 0., -box_size[0]/2.), vec3(box_size[0], box_size[1], BOX_WIDTH));
        }

        5 => {
            physics_world.add_static_cube_rigidbody(vec3(0., 0., box_size[0]/2.), vec3(box_size[0], box_size[1], BOX_WIDTH));
        }

        _ => { print!("bogus amogus number") } // every other number
       }
    }
    
    renderer.add_light(Light { position: vec3(0., box_size[1]/2., 0.), color: Vec3::ONE/3. });
    renderer.add_light(Light { position: vec3(0., -box_size[1]/2., 0.), color: Vec3::ONE/3. });
    let cam_light = renderer.add_light(Light { position: vec3(0., 0.0, 2.5), color: Vec3::ONE }).unwrap();

    let sky_texture = renderer.add_texture("src/assets/textures/sky.jpeg").unwrap();
    let grass_texture = renderer.add_texture("src/assets/textures/grass.png").unwrap();
    
    //let grass_texture = renderer.add_texture("src/assets/textures/bad_grass.jpg").unwrap();

    let mut sphere = Sphere::new(64, 20., Vec4::ONE).mesh();
    for vertex in sphere.vertices.iter_mut() {
        let normal = vertex.normal;
        vertex.normal = -normal;
    }
    sphere.set_texture(sky_texture, &renderer);
    let sky_ball = renderer.add_mesh(sphere).unwrap();
    renderer.get_mesh_mut(sky_ball).unwrap().set_position(vec3(0.0, -BOX_HEIGHT/2.0-0.01, 0.0));
    
    let mut quad = Quad::new(vec3(40., 40., f32::INFINITY), Vec4::ONE).mesh();
    quad.set_texture(grass_texture, &renderer);
    let floor = renderer.add_mesh(quad).unwrap();
    let floor = renderer.get_mesh_mut(floor).unwrap();
    floor.set_rotation(Quat::from_euler(EulerRot::XYZ, -90.0_f32.to_radians(), 0.0, 0.0));
    floor.set_position(vec3(-20.0, -BOX_HEIGHT/2.0-0.01, 20.0));

    let mut game = Game::new();
    
    while !el.window.should_close() {
        el.update();
        renderer.update();
        physics_world.set_dt(el.dt);
        physics_world.step(game.gravity);

        renderer.lights[cam_light].position = renderer.camera.pos;

        renderer.camera.update(renderer.camera.pos, &el);
        
        if el.is_key_down(glfw::Key::LeftAlt) {
            el.window.set_cursor_mode(CursorMode::Disabled);
            renderer.camera.mouse_callback(el.event_handler.mouse_pos, &el.window);
            renderer.camera.input(&el);
        } else {
            renderer.camera.mouse_callback(Vec2::ZERO, &el.window);
            renderer.camera.pos = vec3(0.025, 0.75, 2.5);
            renderer.camera.yaw = -90.0;
            renderer.camera.pitch = -14.0;
            el.window.set_cursor_mode(CursorMode::Normal);
        }

        el.window.set_cursor_mode(CursorMode::Disabled);

        game.update(&mut renderer, &mut physics_world, &el);

        unsafe {
            ClearColor(0.05, 0.075, 0.1, 1.0);
            Clear(COLOR_BUFFER_BIT | DEPTH_BUFFER_BIT);
            
            renderer.draw();
        }
    }
}

struct Piece {
    handle_renderer: MeshHandle,
    handle_physics: RigidBodyHandle,
    color: Vec3,
}

impl Piece {
    pub fn new(renderer: &mut Renderer, physics_world: &mut RapierPhysicsWorld) -> Self {
        let colors = [Vec3::ZERO, vec3(1., 0., 0.), vec3(0., 1., 0.), vec3(0., 0., 1.), Vec3::ONE]; // array of all possible colors

        let color = colors[rand_betw(0, colors.len())]; // sort color

        let mesh_handle = renderer.add_mesh(Cuboid::new(Vec3::ONE/5., Vec4::ONE).mesh()).unwrap();

        let mut mesh = &mut renderer.meshes[mesh_handle];
        mesh.color = color;

        let physics_handle = physics_world.add_cube_rigidbody(vec3(0.0, (BOX_HEIGHT/2.) - (BOX_WIDTH/2.) - 1.0/10., 0.0), Vec3::ONE * 0.2);

        Self {
            handle_renderer: mesh_handle,
            handle_physics: physics_handle,
            color,
        }
    }
    
    pub fn update(&mut self, renderer: &mut Renderer, physics_world: &mut RapierPhysicsWorld, el: &EventLoop) {
        let mesh = renderer.get_mesh_mut(self.handle_renderer).unwrap();
        let object = &physics_world.rigid_body_set[self.handle_physics];
        let t = object.translation();
        let r = object.rotation();

        mesh.position = vec3(t.x, t.y, t.z);
        mesh.rotation = quat(r.i, r.j, r.k, r.w);
        //g * 2
    }

    pub fn rotate(&mut self, r: Vec3, physics_world: &mut RapierPhysicsWorld) {
        let object = &mut physics_world.rigid_body_set[self.handle_physics];
        object.set_angvel(vector![r.x, r.y, r.z], true);
    } 

    pub fn translate(&mut self, t: Vec3, physics_world: &mut RapierPhysicsWorld) {
        let object = &mut physics_world.rigid_body_set[self.handle_physics];
        let curr_vel = object.linvel();
        let gravity = curr_vel.y;
        object.set_linvel(vector![t.x, gravity, t.z], true);
    } 
}

struct Game {
    current_time: std::time::Instant,
    spawn_delay: f32,
    pieces: Vec<Piece>,
    gravity: Vec3,
}


impl Game {
    pub fn new() -> Self {
        Self {
            current_time: std::time::Instant::now(),
            spawn_delay: 0.0,
            pieces: vec![],
            gravity: vec3(0.0, -1.0, 0.0)
        }
    }

    pub fn update(&mut self, renderer: &mut Renderer, physics_world: &mut RapierPhysicsWorld, el: &EventLoop) {
        if self.current_time.elapsed().as_secs_f32() >= self.spawn_delay {
            // if time elapsed since last piece spawn >= spawn delay, spawn a new piece

            self.pieces.push(Piece::new(renderer, physics_world));

            if self.gravity.y > -10.0 {
                self.gravity -= vec3(0.0, 0.5, 0.0);
            }

            if self.spawn_delay > 0.5 {
                self.spawn_delay -= 0.1;
            } 

            self.current_time = std::time::Instant::now() // reset timer
        }

        if el.is_key_down(glfw::Key::Q) {
            if let Some(last) = self.pieces.last_mut() {
                last.rotate(vec3(0.0, 0.0, 5.0), physics_world);
            }
        }
        if el.is_key_down(glfw::Key::E) {
            if let Some(last) = self.pieces.last_mut() {
                last.rotate(vec3(0.0, 0.0, -5.0), physics_world);
            }
        }

        let mut move_vec = Vec3::ZERO;
        let speed = 1.;

        if el.is_key_down(glfw::Key::A) {
            move_vec.x -= speed;
        }
        if el.is_key_down(glfw::Key::D) {
            move_vec.x += speed;
        }
        if el.is_key_down(glfw::Key::W) {
            move_vec.z -= speed;
        }
        if el.is_key_down(glfw::Key::S) {
            move_vec.z += speed;
        }

        if let Some(last) = self.pieces.last_mut() {
            last.translate(move_vec.normalize_or_zero(), physics_world);
        }

        for piece in self.pieces.iter_mut() {
            piece.update(renderer, physics_world, el);
        }
    }
}
