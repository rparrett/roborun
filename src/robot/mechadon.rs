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

/// Bot inspired by the "battle bot" mechadon.
///
/// The real mechadon was 14dof, and this version seems pretty lame.
/// See https://www.youtube.com/watch?v=pYSK0h8sR40 @ 54:50 or so.
/// We would need to support multiple angular motors per joint to
/// accomplish this.
pub struct Mechadon {
    pub body: Option<BodyHandle>,
    actuators: Vec<Actuator>,
    time: f32,
    actuations: Vec<Actuation>,
    current_actuation: usize,
}

impl Mechadon {
    pub fn new() -> Mechadon {
        Mechadon {
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
        let leg_max_angle = 0.78;
        let leg_min_angle = -0.4;
        let body_max_angle = 1.0;
        let body_min_angle = -1.0;
        let density = 1.0;

        let leg_offset_z = 6.0;
        let leg_offset_y = 2.0;
        let leg_joint_offset_y = 3.0;

        let mut id: usize = 0;

        let body_shape = ShapeHandle::new(Cuboid::new(Vector3::new(2.0, 1.0, 4.0)));
        let body_pos = Isometry3::new(Vector3::y() * 11.0, na::zero());
        let body_joint = FreeJoint::new(body_pos);
        let body_collider = ColliderDesc::new(body_shape).density(density);
        let multibody = MultibodyDesc::new(body_joint).collider(&body_collider);
        let body = multibody.build(world);
        let body_handle = body.handle();
        let body_part_handle = body.root().part_handle();
        id += 1;

        let sub_body_a_shape = ShapeHandle::new(Cuboid::new(Vector3::new(2.0, 1.0, 4.0)));
        let sub_body_a_collider = ColliderDesc::new(sub_body_a_shape).density(density);
        let sub_body_a_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        let sub_body_a = MultibodyDesc::new(sub_body_a_joint)
            .collider(&sub_body_a_collider)
            .body_shift(Vector3::new(0.0, 0.0, 0.0))
            .parent_shift(Vector3::new(5.0, 0.0, 0.0))
            .build_with_parent(body_part_handle, world)
            .unwrap();

        sub_body_a.set_name("sub_body_a".to_string());
        let sub_body_a_handle = sub_body_a.part_handle();
        let mut actuator_b_a = Actuator::new(body_handle, id);
        actuator_b_a.set_name("sub_body_a");
        actuator_b_a.set_max_angle(body_max_angle);
        actuator_b_a.set_min_angle(body_min_angle);
        id += 1;

        let sub_body_b_shape = ShapeHandle::new(Cuboid::new(Vector3::new(2.0, 1.0, 4.0)));
        let sub_body_b_collider = ColliderDesc::new(sub_body_b_shape).density(density);
        let sub_body_b_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        let sub_body_b = MultibodyDesc::new(sub_body_b_joint)
            .collider(&sub_body_b_collider)
            .body_shift(Vector3::new(0.0, 0.0, 0.0))
            .parent_shift(Vector3::new(-5.0, 0.0, 0.0))
            .build_with_parent(body_part_handle, world)
            .unwrap();
        sub_body_b.set_name("sub_body_b".to_string());
        let sub_body_b_handle = sub_body_b.part_handle();
        let mut actuator_b_b = Actuator::new(body_handle, id);
        actuator_b_b.set_name("sub_body_b");
        actuator_b_b.set_max_angle(body_max_angle);
        actuator_b_b.set_min_angle(body_min_angle);
        id += 1;

        // middle legs

        let leg_a_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_a_collider = ColliderDesc::new(leg_a_shape).density(density);
        let leg_a_joint =
            RevoluteJoint::new(Unit::new_normalize(Vector3::new(-1.0, 0.0, 0.0)), 0.0);
        MultibodyDesc::new(leg_a_joint)
            .collider(&leg_a_collider)
            .body_shift(Vector3::new(0.0, leg_joint_offset_y, 0.0))
            .parent_shift(Vector3::new(0.0, leg_offset_y, leg_offset_z))
            .build_with_parent(body_part_handle, world)
            .unwrap()
            .set_name("leg_a".to_string());
        let mut actuator_a = Actuator::new(body_handle, id);
        actuator_a.set_name("leg_a");
        actuator_a.set_max_angle(leg_max_angle);
        actuator_a.set_min_angle(leg_min_angle);
        id += 1;

        let leg_b_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_b_collider = ColliderDesc::new(leg_b_shape).density(density);
        let leg_b_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        MultibodyDesc::new(leg_b_joint)
            .collider(&leg_b_collider)
            .body_shift(Vector3::new(0.0, leg_joint_offset_y, 0.0))
            .parent_shift(Vector3::new(0.0, leg_offset_y, -1.0 * leg_offset_z))
            .build_with_parent(body_part_handle, world)
            .unwrap()
            .set_name("leg_b".to_string());
        let mut actuator_b = Actuator::new(body_handle, id);
        actuator_b.set_name("leg_b");
        actuator_b.set_max_angle(leg_max_angle);
        actuator_b.set_min_angle(leg_min_angle);
        id += 1;

        // outer legs

        let leg_c_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_c_collider = ColliderDesc::new(leg_c_shape).density(density);
        let leg_c_joint =
            RevoluteJoint::new(Unit::new_normalize(Vector3::new(-1.0, 0.0, 0.0)), 0.0);
        MultibodyDesc::new(leg_c_joint)
            .collider(&leg_c_collider)
            .body_shift(Vector3::new(0.0, leg_joint_offset_y, 0.0))
            .parent_shift(Vector3::new(0.0, leg_offset_y, leg_offset_z))
            .build_with_parent(sub_body_a_handle, world)
            .unwrap()
            .set_name("leg_c".to_string());
        let mut actuator_c = Actuator::new(body_handle, id);
        actuator_c.set_name("leg_c");
        actuator_c.set_max_angle(leg_max_angle);
        actuator_c.set_min_angle(leg_min_angle);
        id += 1;

        let leg_d_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_d_collider = ColliderDesc::new(leg_d_shape).density(density);
        let leg_d_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        MultibodyDesc::new(leg_d_joint)
            .collider(&leg_d_collider)
            .body_shift(Vector3::new(0.0, leg_joint_offset_y, 0.0))
            .parent_shift(Vector3::new(0.0, leg_offset_y, -1.0 * leg_offset_z))
            .build_with_parent(sub_body_a_handle, world)
            .unwrap()
            .set_name("leg_d".to_string());
        let mut actuator_d = Actuator::new(body_handle, id);
        actuator_d.set_name("leg_d");
        actuator_d.set_max_angle(leg_max_angle);
        actuator_d.set_min_angle(leg_min_angle);
        id += 1;

        let leg_e_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_e_eollider = ColliderDesc::new(leg_e_shape).density(density);
        let leg_e_joint =
            RevoluteJoint::new(Unit::new_normalize(Vector3::new(-1.0, 0.0, 0.0)), 0.0);
        MultibodyDesc::new(leg_e_joint)
            .collider(&leg_e_eollider)
            .body_shift(Vector3::new(0.0, leg_joint_offset_y, 0.0))
            .parent_shift(Vector3::new(0.0, leg_offset_y, leg_offset_z))
            .build_with_parent(sub_body_b_handle, world)
            .unwrap()
            .set_name("leg_e".to_string());
        let mut actuator_e = Actuator::new(body_handle, id);
        actuator_e.set_name("leg_e");
        actuator_e.set_max_angle(leg_max_angle);
        actuator_e.set_min_angle(leg_min_angle);
        id += 1;

        let leg_f_shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 4.0, 0.5)));
        let leg_f_eollider = ColliderDesc::new(leg_f_shape).density(density);
        let leg_f_joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
        MultibodyDesc::new(leg_f_joint)
            .collider(&leg_f_eollider)
            .body_shift(Vector3::new(0.0, leg_joint_offset_y, 0.0))
            .parent_shift(Vector3::new(0.0, leg_offset_y, -1.0 * leg_offset_z))
            .build_with_parent(sub_body_b_handle, world)
            .unwrap()
            .set_name("leg_f".to_string());
        let mut actuator_f = Actuator::new(body_handle, id);
        actuator_f.set_name("leg_f");
        actuator_f.set_max_angle(leg_max_angle);
        actuator_f.set_min_angle(leg_min_angle);

        self.actuators.push(actuator_b_a);
        self.actuators.push(actuator_b_b);

        self.actuators.push(actuator_a);
        self.actuators.push(actuator_b);
        self.actuators.push(actuator_c);
        self.actuators.push(actuator_d);
        self.actuators.push(actuator_e);
        self.actuators.push(actuator_f);

        for a in self.actuators.iter_mut() {
            a.set_max_torque(120.0);
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
        self.actuations.push(Actuation::new(4, 1.0, -0.1));
        self.actuations.push(Actuation::new(0, 2.5, 1.0));
        self.actuations.push(Actuation::new(1, 2.5, 1.0));
        self.actuations.push(Actuation::new(2, 2.5, 1.0));
        self.actuations.push(Actuation::new(3, 2.5, 1.0));
        self.actuations.push(Actuation::new(4, 2.5, 1.0));
    }

    pub fn spawn_individual(&mut self, individual: &Individual, world: &mut World<f32>) {
        self.spawn(world);

        self.actuations.clear();

        for (a, b, c) in individual.genes.iter().tuples::<(_, _, _)>() {
            let actuator = (a * self.actuators.len() as f32) as usize;
            let range = self.actuators[actuator].max_angle - self.actuators[actuator].min_angle;

            self.actuations.push(Actuation::new(
                actuator,
                b * 5.0,
                c * range + self.actuators[actuator].min_angle,
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

        let dist = nalgebra::distance(
            &Point3::new(translation.vector[0], 0.0, translation.vector[2]),
            &Point3::new(0.0, 0.0, 0.0),
        );

        dist
    }
}
