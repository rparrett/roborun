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
    let max_leg_torque = 160.0;

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

    let robot = Robot::new(&mut world);

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

    testbed.add_callback(move |world_owner, _, time| {
        let mut world = world_owner.get_mut();

        let mut t = last_tick.borrow_mut();
        let mut o = on.borrow_mut();

        if time < *t + 1.0 {
            return;
        }

        if let Some(mut j) = world.multibody_link_mut(robot.actuators[0]) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(1.0);
            } else {
                dof.set_desired_angular_motor_velocity(-1.0);
            }

            dof.enable_angular_motor();
        }

        if let Some(mut j) = world.multibody_link_mut(robot.actuators[1]) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(-1.0);
            } else {
                dof.set_desired_angular_motor_velocity(1.0);
            }

            dof.enable_angular_motor();
        }

        if let Some(mut j) = world.multibody_link_mut(robot.actuators[2]) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(1.0);
            } else {
                dof.set_desired_angular_motor_velocity(-1.0);
            }

            dof.enable_angular_motor();
        }

        if let Some(mut j) = world.multibody_link_mut(robot.actuators[3]) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(-0.5);
            } else {
                dof.set_desired_angular_motor_velocity(0.5);
            }

            dof.enable_angular_motor();
        }

        *o = !*o;
        *t = time;
    });

    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
