use std::mem;
use std::path::Path;
use std::rc::Rc;
use std::sync::{Arc, RwLock};

use crate::crucible::{make_world, Crucible};
use crate::engine::GraphicsManager;
use crate::robot::Robot;
use crate::robot::fourdof::Fourdof;
use crate::robot::eightdof::Eightdof;
use crate::world_owner::WorldOwner;
use kiss3d::camera::Camera;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3};
use nphysics3d::object::{BodyHandle, ColliderHandle};
use nphysics3d::world::World;

#[derive(PartialEq)]
enum RunMode {
    Running,
    Stop,
    Step,
}

pub struct Testbed {
    world: Box<WorldOwner>,
    window: Option<Box<Window>>,
    graphics: GraphicsManager,
    nsteps: usize,
    callbacks: Callbacks,
    time: f32,

    font: Rc<Font>,
    running: RunMode,

    robot: Option<Robot>,
    robot_colors: Vec<Point3<f32>>,
    robot_color: usize,

    crucible: Crucible,
    showing_gen: usize,
    reset: bool,
}

type Callbacks = Vec<Box<Fn(&mut WorldOwner, &mut GraphicsManager, f32)>>;

impl Testbed {
    pub fn new_empty() -> Testbed {
        let mut graphics = GraphicsManager::new();
        let world = World::new();

        let mut window = Box::new(Window::new("nphysics: 3d demo"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));
        window.set_light(Light::Absolute(Point3::new(30.0, 30.0, 30.0)));
        window.set_ambient_light(Point3::new(0.2, 0.2, 0.2));
        window.add_texture_from_memory(include_bytes!("../assets/floor.png"), "floor");
        window.add_texture_from_memory(include_bytes!("../assets/metal.png"), "metal");

        let camera = graphics.camera_mut();
        camera.set_max_pitch(std::f32::consts::PI / 2.0);        
        camera.set_min_dist(10.0);
        camera.set_max_dist(400.0);

        let robot_colors = vec![
            Point3::new(0.557, 0.922, 0.000),
            Point3::new(1.000, 0.286, 0.000),
            Point3::new(0.557, 0.922, 0.000),
            Point3::new(0.565, 0.016, 0.659),
            Point3::new(0.047, 0.353, 0.651),
        ];

        Testbed {
            world: Box::new(Arc::new(RwLock::new(world))),
            callbacks: Vec::new(),
            window: Some(window),
            graphics,
            nsteps: 100,
            time: 0.0,
            font: Font::from_bytes(include_bytes!("../assets/UbuntuMono-Regular.ttf")).unwrap(),
            running: RunMode::Step,
            crucible: Crucible::new(|individual, world| { let mut robot = Robot::Eightdof(Eightdof::new()); robot.spawn_individual(individual, world); robot }),
            robot: None,
            robot_colors: robot_colors,
            robot_color: 0,
            reset: true,
            showing_gen: 1,
        }
    }

    pub fn new(world: World<f32>) -> Self {
        Self::new_with_world_owner(Box::new(world))
    }

    pub fn new_with_world_owner(world: Box<WorldOwner>) -> Self {
        let mut res = Testbed::new_empty();

        res.set_world_owner(world);
        res
    }

    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.nsteps = nsteps
    }

    pub fn running_replace_world(&mut self, world: World<f32>, window: &mut Window) {
        self.world = Box::new(world);

        let mut world = self.world.get_mut();

        world.enable_performance_counters();
        self.graphics.clear(window);

        for co in world.colliders() {
            self.graphics.add(window, co.handle(), &world);
        }
    }

    pub fn set_world(&mut self, world: World<f32>) {
        self.set_world_owner(Box::new(world))
    }

    pub fn set_world_owner(&mut self, world: Box<WorldOwner>) {
        self.world = world;
        let mut world = self.world.get_mut();
        world.enable_performance_counters();

        self.graphics.clear(self.window.as_mut().unwrap());

        for co in world.colliders() {
            self.graphics
                .add(self.window.as_mut().unwrap(), co.handle(), &world);
        }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.graphics.camera_mut().look_at(eye, at);
    }

    pub fn set_body_color(&mut self, body: BodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(body, color);
    }
    
    pub fn set_body_texture(&mut self, body: BodyHandle, texture: String) {
        self.graphics.set_body_texture(body, texture);
    }

    pub fn set_collider_color(&mut self, collider: ColliderHandle, color: Point3<f32>) {
        self.graphics.set_collider_color(collider, color);
    }

    pub fn world(&self) -> &Box<WorldOwner> {
        &self.world
    }

    pub fn graphics_mut(&mut self) -> &mut GraphicsManager {
        &mut self.graphics
    }

    pub fn load_obj(path: &str) -> Vec<(Vec<Point3<f32>>, Vec<usize>)> {
        let path = Path::new(path);
        let empty = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.into_iter() {
            let vertices = m.coords().read().unwrap().to_owned().unwrap();
            let indices = m.faces().read().unwrap().to_owned().unwrap();

            let mut flat_indices = Vec::new();

            for i in indices.into_iter() {
                flat_indices.push(i.x as usize);
                flat_indices.push(i.y as usize);
                flat_indices.push(i.z as usize);
            }

            let m = (vertices, flat_indices);

            res.push(m);
        }

        res
    }

    pub fn add_callback<F: Fn(&mut WorldOwner, &mut GraphicsManager, f32) + 'static>(
        &mut self,
        callback: F,
    ) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn run(mut self) {
        let window = mem::replace(&mut self.window, None).unwrap();
        window.render_loop(self);
    }
}

type CameraEffects<'a> = (
    Option<&'a mut Camera>,
    Option<&'a mut PlanarCamera>,
    Option<&'a mut PostProcessingEffect>,
);

impl State for Testbed {
    fn cameras_and_effect(&mut self) -> CameraEffects<'_> {
        (Some(self.graphics.camera_mut()), None, None)
    }

    fn step(&mut self, window: &mut Window) {
        if self.reset {
            let mut world = make_world();
            let mut robot = Robot::Eightdof(Eightdof::new());
            robot.spawn_individual(self.crucible.population.best(), &mut world);
            self.set_body_color(robot.body_handle().unwrap(), self.robot_colors[self.robot_color]);
            self.set_body_texture(robot.body_handle().unwrap(), "metal".to_string());
            self.set_body_color(BodyHandle::ground(), Point3::new(0.5, 0.5, 0.5));
            self.set_body_texture(BodyHandle::ground(), "floor".to_string());
            self.robot_color += 1;
            if self.robot_color >= self.robot_colors.len() {
                self.robot_color = 0
            }
            self.robot = Some(robot);

            self.running_replace_world(world, window);
            self.time = 0.0;
            self.reset = false;
        }

        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::T, Action::Release, _) => {
                    if self.running == RunMode::Stop {
                        self.running = RunMode::Running;
                    } else {
                        self.running = RunMode::Stop;
                    }
                }
                WindowEvent::Key(Key::S, Action::Release, _) => self.running = RunMode::Step,
                _ => {}
            }
        }

        if self.running != RunMode::Stop {
            // let before = time::precise_time_s();
            for _ in 0..self.nsteps {
                self.crucible.step();
            }

            {
                for f in &self.callbacks {
                    f(&mut *self.world, &mut self.graphics, self.time)
                }

                let mut world = self.world.get_mut();
                let timestep = world.timestep();

                if let Some(robot) = &mut self.robot {
                    robot.step(&mut world, timestep);
                }
                world.step();

                self.time += timestep;

                if self.showing_gen != self.crucible.generation {
                    self.reset = true;
                    self.showing_gen = self.crucible.generation;
                }
            }

            self.graphics.draw(&self.world.get(), window);
        }

        if self.running == RunMode::Step {
            self.running = RunMode::Stop;
        }

        let color = Point3::new(0.0, 0.0, 0.0);

        window.draw_text(
            &format!(
                "{:.0}%",
                self.crucible.individual as f32 / self.crucible.population.num as f32 * 100.0,
            )[..],
            &Point2::new(5.0, 5.0),
            40.0,
            &self.font,
            &color,
        );

        if self.running != RunMode::Running {
            window.draw_text(
                "(Paused)",
                &Point2::new(100.0, 5.0),
                40.0,
                &self.font,
                &color,
            );
        }

        // TODO can I do this without the intermediate vec? without String?
        let mut lines: Vec<String> = self
            .crucible
            .stats
            .iter()
            .rev()
            .take(5)
            .map(|x| {
                format!(
                    "{:<4} {:>6.2} {:>6.2} {:>6.2}",
                    x.generation, x.min_fitness, x.avg_fitness, x.max_fitness
                )
            })
            .collect();
        lines.insert(0, "Gen    Min    Avg    Max".to_string());
        let status = lines.join("\n");

        window.draw_text(
            status.as_str(),
            &Point2::new(5.0, 85.0),
            40.0,
            &self.font,
            &color,
        );

        window.draw_text(CONTROLS, &Point2::new(5.0, 365.0), 40.0, &self.font, &color);
    }
}

const CONTROLS: &str = "Left click + drag: rotate the camera.
Right click + drag: pan the camera.
Mouse wheel: zoom in/zoom out.
T: pause/resume simulation.
S: step simulation.";
