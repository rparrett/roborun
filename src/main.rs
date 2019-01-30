#[macro_use]
extern crate stdweb;

extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Real, Translation3, Unit, Vector2, Vector3};
use ncollide3d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::joint::{
    BallJoint, CartesianJoint, CylindricalJoint, FixedJoint, FreeJoint, HelicalJoint, Joint,
    PinSlotJoint, PlanarJoint, PrismaticJoint, RectangularJoint, RevoluteJoint, UniversalJoint,
};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;
use std::f32::consts::PI;
use std::cell::RefCell;

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
    let ground_pos = Isometry3::new(Vector3::y() * -57.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let body_shape = ShapeHandle::new(Cuboid::new(Vector3::new(4.0, 1.0, 4.0)));
    let body_pos = Isometry3::new(Vector3::y() * -1.0, na::zero());
    let body_com = body_shape.center_of_mass();
    let body_inertia = body_shape.inertia(1.0);
    let body_joint = FreeJoint::new(body_pos);
    let body_handle = world.add_multibody_link(
        BodyHandle::ground(),
        body_joint,
        na::zero(),
        na::zero(),
        body_inertia,
        body_com,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        body_shape,
        body_handle,
        Isometry3::identity(),
        Material::default(),
    );

    let leg_a_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
    let leg_a_com = leg_a_shape.center_of_mass();
    let leg_a_inertia = leg_a_shape.inertia(1.0);
    let mut leg_a_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
    leg_a_joint.enable_max_angle(1.0);
    leg_a_joint.enable_min_angle(-1.0);
    leg_a_joint.set_max_angular_motor_torque(max_leg_torque);
    let leg_a_handle = world.add_multibody_link(
        body_handle,
        leg_a_joint,
        Vector3::new(-4.0, -1.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
        leg_a_inertia,
        leg_a_com,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        leg_a_shape,
        leg_a_handle,
        Isometry3::identity(),
        Material::default(),
    );

    let leg_b_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
    let leg_b_com = leg_b_shape.center_of_mass();
    let leg_b_inertia = leg_b_shape.inertia(1.0);
    let mut leg_b_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
    leg_b_joint.enable_max_angle(1.0);
    leg_b_joint.enable_min_angle(-1.0);
    leg_b_joint.set_max_angular_motor_torque(max_leg_torque);
    let leg_b_handle = world.add_multibody_link(
        body_handle,
        leg_b_joint,
        Vector3::new(4.0, -1.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
        leg_b_inertia,
        leg_b_com,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        leg_b_shape,
        leg_b_handle,
        Isometry3::identity(),
        Material::default(),
    );

    let leg_c_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
    let leg_c_com = leg_c_shape.center_of_mass();
    let leg_c_inertia = leg_c_shape.inertia(1.0);
    let mut leg_c_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    leg_c_joint.enable_max_angle(1.0);
    leg_c_joint.enable_min_angle(-1.0);
    leg_c_joint.set_max_angular_motor_torque(max_leg_torque);
    let leg_c_handle = world.add_multibody_link(
        body_handle,
        leg_c_joint,
        Vector3::new(0.0, -1.0, 4.0),
        Vector3::new(0.0, 4.0, 0.0),
        leg_c_inertia,
        leg_c_com,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        leg_c_shape,
        leg_c_handle,
        Isometry3::identity(),
        Material::default(),
    );

    let leg_d_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
    let leg_d_com = leg_d_shape.center_of_mass();
    let leg_d_inertia = leg_d_shape.inertia(1.0);
    let mut leg_d_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    leg_d_joint.enable_max_angle(1.0);
    leg_d_joint.enable_min_angle(-1.0);
    leg_d_joint.set_max_angular_motor_torque(max_leg_torque);
    let leg_d_handle = world.add_multibody_link(
        body_handle,
        leg_d_joint,
        Vector3::new(0.0, -1.0, -4.0),
        Vector3::new(0.0, 4.0, 0.0),
        leg_d_inertia,
        leg_d_com,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        leg_d_shape,
        leg_d_handle,
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new_empty();
    testbed.set_body_color(&world, body_handle, Point3::new(0.0, 1.0, 0.0));
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

        if let Some(mut j) = world.multibody_link_mut(leg_a_handle) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
                
            if *o {
                dof.set_desired_angular_motor_velocity(1.0);
            } else {
                dof.set_desired_angular_motor_velocity(-1.0);
            }
            
            dof.enable_angular_motor();
        }
        
        if let Some(mut j) = world.multibody_link_mut(leg_b_handle) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(-1.0);
            } else {
                dof.set_desired_angular_motor_velocity(1.0);
            }
            
            dof.enable_angular_motor();
        }
        
        if let Some(mut j) = world.multibody_link_mut(leg_c_handle) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(1.0);
            } else {
                dof.set_desired_angular_motor_velocity(-1.0);
            }
            
            dof.enable_angular_motor();
        }
        
        if let Some(mut j) = world.multibody_link_mut(leg_d_handle) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            if *o {
                dof.set_desired_angular_motor_velocity(-1.0);
            } else {
                dof.set_desired_angular_motor_velocity(1.0);
            }
            
            dof.enable_angular_motor();
        }

        *o = !*o;
        *t = time;
    });

    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
