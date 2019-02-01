#[macro_use]
extern crate stdweb;

extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate num_traits as num;
extern crate kiss3d;
extern crate rand;
extern crate time;

mod testbed;
mod world_owner;
mod engine;
mod objects;
mod robot;
mod actuator;

use na::{Isometry3, Point3, Real, Translation3, Unit, Vector2, Vector3};
use ncollide3d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::joint::{
    BallJoint, CartesianJoint, CylindricalJoint, FixedJoint, FreeJoint, HelicalJoint, Joint,
    PinSlotJoint, PlanarJoint, PrismaticJoint, RectangularJoint, RevoluteJoint, UniversalJoint,
};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use crate::testbed::Testbed;
use crate::world_owner::WorldOwner;
use crate::engine::GraphicsManager;
use std::cell::RefCell;
use std::f32::consts::PI;
use robot::Robot;

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
    let ground_pos = Isometry3::new(Vector3::y() * -60.5, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let mut robot = Robot::spawn(&mut world);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new_empty();
    testbed.set_body_color(&world, robot.body, Point3::new(0.0, 1.0, 0.0));
    testbed.set_world(world);

    let last_tick = 0.0;
    let last_tick = RefCell::new(last_tick);

    let on = false;
    let on = RefCell::new(on);

    let first_tick = true;
    let first_tick = RefCell::new(first_tick);

    let robot = RefCell::new(robot);

    testbed.add_callback(move |world_owner, _, time| {
        let mut w = world_owner.get_mut();

        let mut last_tick = last_tick.borrow_mut();
        let mut on = on.borrow_mut();
        let mut first_tick = first_tick.borrow_mut();

        let mut robot = robot.borrow_mut();
        
        if *first_tick || time > *last_tick + 2.0 {
            for a in robot.actuators.iter_mut() {
                if *on {
                    a.set_position(0.1);
                } else {
                    a.set_position(-1.0);
                }
            }

            *on = !*on;
            *last_tick = time;
            *first_tick = false;
        }
        
        for a in robot.actuators.iter_mut() {
            a.step(&mut w);
        }
    });

    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
