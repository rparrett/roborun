#[macro_use]
extern crate stdweb;

extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate num_traits as num;
extern crate rand;
extern crate time;

mod actuator;
mod engine;
mod individual;
mod objects;
mod robot;
mod testbed;
mod world_owner;

use crate::engine::GraphicsManager;
use crate::individual::Individual;
use crate::testbed::Testbed;
use crate::world_owner::WorldOwner;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use robot::Robot;
use std::cell::RefCell;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Create a ground.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(50.0, 50.0, 50.0)));
    let ground_pos = Isometry3::new(Vector3::y() * -61.5, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::new(0.0, 0.9), // ground is a bit sticky
    );

    let individual = Individual::random(3, 3 * 12);
    let mut robot = Robot::from_individual(individual, &mut world);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new_empty();
    testbed.set_body_color(&world, robot.body, Point3::new(0.0, 1.0, 0.0));
    testbed.set_world(world);

    let last_tick = 0.0;
    let last_tick = RefCell::new(last_tick);

    let robot = RefCell::new(robot);

    testbed.add_callback(move |world_owner, _, time| {
        let mut last_tick = last_tick.borrow_mut();
        let elapsed = time - *last_tick;
        *last_tick = time.clone();

        let mut w = world_owner.get_mut();

        let mut robot = robot.borrow_mut();

        robot.step(&mut w, elapsed);
    });

    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
