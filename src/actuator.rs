use nphysics3d::joint::{RevoluteJoint};
use nphysics3d::object::BodyHandle;
use nphysics3d::world::World;

pub struct Actuator {
    pub max_torque: f32,
    pub min_angle: f32,
    pub max_angle: f32,
    pub max_velocity: f32,
    pub desired_position: f32,
    pub deadzone: f32,
    pub handle: BodyHandle,
    pub name: &'static str,
}

impl Actuator {
    pub fn new(handle: BodyHandle) -> Actuator {
        Actuator {
            max_torque: 160.0,
            min_angle: -1.0,
            max_angle: 1.0,
            max_velocity: 1.0,
            desired_position: 0.0,
            deadzone: 0.02,
            handle: handle,
            name: "?",
        }
    }

    pub fn set_name(&mut self, name: &'static str) {
        self.name = name;
    }

    pub fn set_max_torque(&mut self, max_torque: f32) {
        self.max_torque = max_torque;
    }

    pub fn set_max_angle(&mut self, max_angle: f32) {
        self.max_angle = max_angle;
    }

    pub fn set_min_angle(&mut self, min_angle: f32) {
        self.min_angle = min_angle;
    }

    pub fn set_max_velocity(&mut self, max_velocity: f32) {
        self.max_velocity = max_velocity;
    }

    pub fn set_position(&mut self, pos: f32) {
        self.desired_position = pos;
    }

    pub fn setup(&self, world: &mut World<f32>) {
        if let Some(mut j) = world.multibody_link_mut(self.handle) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            dof.enable_max_angle(self.max_angle);
            dof.enable_min_angle(self.min_angle);
            dof.set_max_angular_motor_torque(self.max_torque);

            dof.set_desired_angular_motor_velocity(0.0);
            dof.enable_angular_motor();
        }
    }

    pub fn step(&mut self, world: &mut World<f32>) {
        if let Some(mut j) = world.multibody_link_mut(self.handle) {
            let dof = j.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();

            let v = if dof.angle() < self.desired_position - self.deadzone {
                self.max_velocity
            } else if dof.angle() > self.desired_position + self.deadzone {
                self.max_velocity * -1.0
            } else {
                0.0
            };

            // console!(log, format!("[{}] {} < {}->{} < {}: {}", self.name, self.min_angle, dof.angle(), self.desired_position, self.max_angle, v));

            dof.set_desired_angular_motor_velocity(v);
        }
    }
}
