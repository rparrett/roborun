use crate::actuator::Actuator;
use crate::individual::Individual;
use itertools::Itertools;
use na::{Isometry3, Point3, Unit, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{FreeJoint, RevoluteJoint};
use nphysics3d::object::{Body, BodyHandle, BodyPart, ColliderDesc, MultibodyDesc};
use nphysics3d::world::World;

#[derive(Debug)]
pub struct Actuation {
    actuator: usize,
    time: f32,
    position: f32,
}

impl Actuation {
    pub fn new(actuator: usize, time: f32, position: f32) -> Actuation {
        Actuation {
            actuator: actuator,
            time: time,
            position: position,
        }
    }
}

pub struct Eightdof {
    pub body: Option<BodyHandle>,
    actuators: Vec<Actuator>,
    time: f32,
    actuations: Vec<Actuation>,
    current_actuation: usize,
}

impl Eightdof {
    pub fn new() -> Eightdof {
        Eightdof {
            body: None,
            actuators: Vec::new(),
            time: 0.0,
            actuations: Vec::new(),
            current_actuation: 0,
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

        self.actuators.push(actuator_a);
        self.actuators.push(actuator_b);
        self.actuators.push(actuator_c);
        self.actuators.push(actuator_d);

        for a in self.actuators.iter_mut() {
            a.set_max_angle(1.5);
            a.set_min_angle(-0.2);
            a.set_max_torque(160.0);
            a.set_max_velocity(1.0);
            a.setup(world);
        }

        self.body = Some(body_handle);

        // TODO: These default actuations for testing should probably
        // be replaced by a default individual with the appropriate
        // genes.

        self.actuations.push(Actuation::new(0, 1.0, -0.1));
        self.actuations.push(Actuation::new(1, 1.0, -0.1));
        self.actuations.push(Actuation::new(2, 1.0, -0.1));
        self.actuations.push(Actuation::new(3, 1.0, -0.1));
        self.actuations.push(Actuation::new(0, 2.5, 1.0));
        self.actuations.push(Actuation::new(1, 2.5, 1.0));
        self.actuations.push(Actuation::new(2, 2.5, 1.0));
        self.actuations.push(Actuation::new(3, 2.5, 1.0));
    }

    pub fn spawn_individual(&mut self, individual: &Individual, world: &mut World<f32>) {
        self.spawn(world);

        self.actuations.clear();

        for (a, b, c) in individual.genes.iter().tuples::<(_, _, _)>() {
            self.actuations.push(Actuation::new(
                (a * self.actuators.len() as f32) as usize,
                b * 5.0,
                c * 1.7 - 0.2,
            ));
        }
    }

    pub fn step(&mut self, world: &mut World<f32>, elapsed: f32) {
        self.time += elapsed;

        // TODO this seems horrendously ugly. Re-evaluate when you are less
        // bad at rust.

        loop {
            if let Some(a) = self.actuations.get(self.current_actuation) {
                if self.time < a.time {
                    break;
                }

                if let Some(ab) = self.actuators.get_mut(a.actuator) {
                    ab.set_position(a.position);
                } else {
                    break;
                }
            } else {
                break;
            }

            self.current_actuation += 1;
            if self.current_actuation > self.actuations.len() - 1 {
                self.current_actuation = 0;
                self.time = 0.0;
                break;
            }
        }

        for a in self.actuators.iter_mut() {
            a.step(world);
        }
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

        if let Some(b) = world.multibody(body).and_then(|mb| mb.link(0)) {
            let p = b.position();
            let t = p.translation;

            let d = nalgebra::distance(
                &Point3::new(t.vector[0], 0.0, t.vector[2]),
                &Point3::new(0.0, 0.0, 0.0),
            );

            let up = p.rotation * Vector3::new(0.0, 1.0, 0.0);
            if up[1] < 0.0 {
                return 0.0;
            }

            return d;
        }

        0.0
    }
}