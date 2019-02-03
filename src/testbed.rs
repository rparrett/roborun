use num::Bounded;
use std::collections::HashMap;
use std::env;
use std::mem;
use std::path::Path;
use std::rc::Rc;
use std::sync::{Arc, RwLock};

use crate::engine::GraphicsManager;
use crate::world_owner::WorldOwner;
use kiss3d::camera::Camera;
use kiss3d::event::{Action, Key, Modifiers, WindowEvent};
use kiss3d::light::Light;
use kiss3d::loader::obj;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use na::{self, Point2, Point3, Vector3};
use ncollide3d::query::{self, Ray};
use ncollide3d::utils::GenerationalId;
use ncollide3d::world::CollisionGroups;
use nphysics3d::joint::{ConstraintHandle, MouseConstraint};
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
    hide_overlay: bool,

    font: Rc<Font>,
    running: RunMode,
}

type Callbacks = Vec<Box<Fn(&mut WorldOwner, &mut GraphicsManager, f32)>>;

impl Testbed {
    pub fn new_empty() -> Testbed {
        let graphics = GraphicsManager::new();
        let world = World::new();

        let mut window = Box::new(Window::new("roborun"));
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        Testbed {
            world: Box::new(Arc::new(RwLock::new(world))),
            callbacks: Vec::new(),
            window: Some(window),
            graphics,
            nsteps: 1,
            time: 0.0,
            hide_overlay: false,
            font: Font::default(),
            running: RunMode::Step,
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

    pub fn hide_overlay(&mut self) {
        self.hide_overlay = true;
    }

    pub fn show_overlay(&mut self) {
        self.hide_overlay = false;
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
        self.graphics.look_at(eye, at);
    }

    pub fn set_body_color(&mut self, world: &World<f32>, body: BodyHandle, color: Point3<f32>) {
        self.graphics.set_body_color(world, body, color);
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
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::P, Action::Release, _) => {
                    if self.running == RunMode::Stop {
                        self.running = RunMode::Running;
                    } else {
                        self.running = RunMode::Stop;
                    }
                }
                WindowEvent::Key(Key::S, Action::Release, _) => self.running = RunMode::Step,
                WindowEvent::Key(Key::O, Action::Release, _) => {
                    self.hide_overlay = !self.hide_overlay
                }
                _ => {}
            }
        }

        if self.running != RunMode::Stop {
            // let before = time::precise_time_s();
            for _ in 0..self.nsteps {
                for f in &self.callbacks {
                    f(&mut *self.world, &mut self.graphics, self.time)
                }

                let mut world = self.world.get_mut();
                world.step();
                self.time += world.timestep();
            }

            self.graphics.draw(&self.world.get());
        }

        if self.running == RunMode::Step {
            self.running = RunMode::Stop;
        }

        let color = Point3::new(0.0, 0.0, 0.0);

        if !self.hide_overlay {
            window.draw_text(
                &format!(
                    "Physics: {:.*} fps",
                    0,
                    1.0 / self.world.get().performance_counters().step_time()
                )[..],
                &Point2::origin(),
                40.0,
                &self.font,
                &color,
            );

            window.draw_text(CONTROLS, &Point2::new(0.0, 75.0), 40.0, &self.font, &color);

            if self.running == RunMode::Stop {
                window.draw_text("Paused", &Point2::new(0.0, 400.0), 40.0, &self.font, &color);
            }
        }
    }
}

const CONTROLS: &str = "Controls:
    Left click + drag: rotate the camera.
    Mouse wheel: zoom in/zoom out.
    P: pause/resume simulation.
    S: step simulation.
    O: hide this overlay
";
