use na::{Isometry3, Point3, Vector3, Unit};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{RevoluteJoint, FreeJoint};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use super::COLLIDER_MARGIN;
use crate::actuator::Actuator;

pub struct Robot {
    pub body: BodyHandle,
    pub actuators: Vec<Actuator>,
}

impl Robot {
    pub fn spawn(world: &mut World<f32>) -> Robot {
        let max_leg_torque = 160.0;

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
        let leg_a_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
        let leg_a_handle = world.add_multibody_link(
            body_handle,
            leg_a_joint,
            Vector3::new(-4.0, -2.0, 0.0),
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
        let mut actuator_a = Actuator::new(leg_a_handle);
        actuator_a.set_name("a");

        let leg_b_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
        let leg_b_com = leg_b_shape.center_of_mass();
        let leg_b_inertia = leg_b_shape.inertia(1.0);
        let leg_b_joint = RevoluteJoint::new(Unit::new_normalize(Vector3::new(0.0, 0.0, -1.0)), 0.0);
        let leg_b_handle = world.add_multibody_link(
            body_handle,
            leg_b_joint,
            Vector3::new(4.0, -2.0, 0.0),
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
        let mut actuator_b = Actuator::new(leg_b_handle);
        actuator_b.set_name("b");

        let leg_c_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
        let leg_c_com = leg_c_shape.center_of_mass();
        let leg_c_inertia = leg_c_shape.inertia(1.0);
        let leg_c_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        let leg_c_handle = world.add_multibody_link(
            body_handle,
            leg_c_joint,
            Vector3::new(0.0, -2.0, 4.0),
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
        let mut actuator_c = Actuator::new(leg_c_handle);
        actuator_c.set_name("c");

        let leg_d_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 4.0, 1.0)));
        let leg_d_com = leg_d_shape.center_of_mass();
        let leg_d_inertia = leg_d_shape.inertia(1.0);
        let leg_d_joint = RevoluteJoint::new(Unit::new_normalize(Vector3::new(-1.0, 0.0, 0.0)), 0.0);
        let leg_d_handle = world.add_multibody_link(
            body_handle,
            leg_d_joint,
            Vector3::new(0.0, -2.0, -4.0),
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
        let mut actuator_d = Actuator::new(leg_d_handle);
        actuator_d.set_name("d");

        let mut actuators =  vec![actuator_a, actuator_b, actuator_c, actuator_d];

        for a in actuators.iter_mut() {
            a.set_max_angle(0.1);
            a.set_min_angle(-1.0);
            a.set_max_torque(160.0);
            a.set_max_velocity(1.0);
            a.setup(world);
        }

        Robot {
            body: body_handle,
            actuators: actuators 
        }
    }
}
