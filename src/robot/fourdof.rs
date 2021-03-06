use crate::actuator::Actuator;
use crate::individual::Individual;
use crate::robot::brain::DumbBrain;
use itertools::Itertools;
use na::{Isometry3, Point3, Unit, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{FreeJoint, RevoluteJoint};
use nphysics3d::object::{Body, BodyHandle, BodyPart, ColliderDesc, MultibodyDesc};
use nphysics3d::world::World;

pub struct Fourdof {
    pub body: Option<BodyHandle>,
    brain: DumbBrain,
}

impl Fourdof {
    pub fn new() -> Fourdof {
        Fourdof {
            body: None,
            brain: DumbBrain::new(),
        }
    }

    pub fn body_handle(&self) -> Option<BodyHandle> {
        self.body
    }

    pub fn spawn(&mut self, world: &mut World<f32>) {
        let mut id: usize = 0;

        let body_shape = ShapeHandle::new(Cuboid::new(Vector3::new(4.0, 1.0, 4.0)));
        let body_pos = Isometry3::new(Vector3::y() * 11.0, na::zero());
        let body_joint = FreeJoint::new(body_pos);
        let body_collider = ColliderDesc::new(body_shape).density(0.5);
        let multibody = MultibodyDesc::new(body_joint).collider(&body_collider);
        let body = multibody.build(world);
        let body_handle = body.handle();
        let body_part_handle = body.root().part_handle();
        id += 1;

        let leg_a_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_a_collider = ColliderDesc::new(leg_a_shape).density(0.5);
        let leg_a_joint = RevoluteJoint::new(Vector3::z_axis(), 0.0);
        MultibodyDesc::new(leg_a_joint)
            .collider(&leg_a_collider)
            .body_shift(Vector3::new(0.0, 4.0, 0.0))
            .parent_shift(Vector3::new(4.0, -2.0, 0.0))
            .build_with_parent(body_part_handle, world)
            .unwrap()
            .set_name("leg_a".to_string());
        let mut actuator_a = Actuator::new(body_handle, id);
        actuator_a.set_name("leg_a");
        id += 1;

        let leg_b_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_b_collider = ColliderDesc::new(leg_b_shape).density(0.5);
        let leg_b_joint =
            RevoluteJoint::new(Unit::new_normalize(Vector3::new(0.0, 0.0, -1.0)), 0.0);
        MultibodyDesc::new(leg_b_joint)
            .collider(&leg_b_collider)
            .body_shift(Vector3::new(0.0, 4.0, 0.0))
            .parent_shift(Vector3::new(-4.0, -2.0, 0.0))
            .build_with_parent(body_part_handle, world)
            .unwrap()
            .set_name("leg_b".to_string());
        let mut actuator_b = Actuator::new(body_handle, id);
        actuator_b.set_name("leg_b");
        id += 1;

        let leg_c_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_c_collider = ColliderDesc::new(leg_c_shape).density(0.5);
        let leg_c_joint =
            RevoluteJoint::new(Unit::new_normalize(Vector3::new(-1.0, 0.0, 0.0)), 0.0);
        MultibodyDesc::new(leg_c_joint)
            .collider(&leg_c_collider)
            .body_shift(Vector3::new(0.0, 4.0, 0.0))
            .parent_shift(Vector3::new(0.0, -2.0, 4.0))
            .build_with_parent(body_part_handle, world)
            .unwrap()
            .set_name("leg_c".to_string());
        let mut actuator_c = Actuator::new(body_handle, id);
        actuator_c.set_name("leg_c");
        id += 1;

        let leg_d_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_d_collider = ColliderDesc::new(leg_d_shape).density(0.5);
        let leg_d_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        MultibodyDesc::new(leg_d_joint)
            .collider(&leg_d_collider)
            .body_shift(Vector3::new(0.0, 4.0, 0.0))
            .parent_shift(Vector3::new(0.0, -2.0, -4.0))
            .build_with_parent(body_part_handle, world)
            .unwrap()
            .set_name("leg_d".to_string());
        let mut actuator_d = Actuator::new(body_handle, id);
        actuator_d.set_name("leg_d");

        for a in [
            &mut actuator_a,
            &mut actuator_b,
            &mut actuator_c,
            &mut actuator_d,
        ]
        .iter_mut()
        {
            a.set_max_angle(1.5);
            a.set_min_angle(-0.2);
            a.set_max_torque(160.0);
            a.set_max_velocity(1.0);
            a.setup(world);
        }

        self.brain.push_actuator(actuator_a);
        self.brain.push_actuator(actuator_b);
        self.brain.push_actuator(actuator_c);
        self.brain.push_actuator(actuator_d);

        self.body = Some(body_handle);

        // TODO: These default actuations for testing should probably
        // be replaced by a default individual with the appropriate
        // genes.

        self.brain.push_actuation(0, 1.0, -0.1);
        self.brain.push_actuation(1, 1.0, -0.1);
        self.brain.push_actuation(2, 1.0, -0.1);
        self.brain.push_actuation(3, 1.0, -0.1);
        self.brain.push_actuation(0, 2.5, 1.0);
        self.brain.push_actuation(1, 2.5, 1.0);
        self.brain.push_actuation(2, 2.5, 1.0);
        self.brain.push_actuation(3, 2.5, 1.0);
    }

    pub fn spawn_individual(&mut self, individual: &Individual, world: &mut World<f32>) {
        self.spawn(world);

        self.brain.reset();

        for (a, b, c) in individual.genes.iter().tuples::<(_, _, _)>() {
            self.brain.push_actuation(
                (a * self.brain.len_actuators() as f32) as usize,
                b * 5.0,
                c * 1.7 - 0.2,
            );
        }

        self.brain.setup();
    }

    pub fn step(&mut self, world: &mut World<f32>, elapsed: f32) {
        self.brain.step(world, elapsed);
    }

    pub fn fitness(&self, world: &World<f32>) -> f32 {
        // x/y distance from the origin, unless we've tipped onto our side or gone upside down.
        //
        // evaluating that second condition at this time means that a robot is allowed to
        // somersault, as long as it's right-side-up when the fitness is evaluated. that's
        // probably not ideal.
        //
        // also, it's possible that a very fit robot that has discovered the edge of the earth
        // will mistakenly be judged unfit as it tumbles. can we check its current collision
        // state or something?
        //
        // maybe it would be good to break ties with genome size or something

        let body = match self.body {
            Some(body) => body,
            None => return 0.0,
        };

        let link = match world.multibody(body).and_then(|mb| mb.link(0)) {
            Some(link) => link,
            None => return 0.0,
        };

        let pos = link.position();

        let up = pos.rotation * Vector3::new(0.0, 1.0, 0.0);
        if up[1] < 0.0 {
            return 0.0;
        }

        let translation = pos.translation;

        nalgebra::distance(
            &Point3::new(translation.vector[0], 0.0, translation.vector[2]),
            &Point3::new(0.0, 0.0, 0.0),
        )
    }
}
