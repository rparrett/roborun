use crate::individual::Individual;
use nphysics3d::object::BodyHandle;
use nphysics3d::world::World;

pub mod mechadon;
pub mod fourdof;

use crate::robot::mechadon::Mechadon;
use crate::robot::fourdof::Fourdof;

pub enum Robot {
    Fourdof(Fourdof),
    Mechadon(Mechadon),
}

impl Robot {
    pub fn spawn(&mut self, world: &mut World<f32>) {
        match *self {
            Robot::Fourdof(ref mut robot) => robot.spawn(world),
            Robot::Mechadon(ref mut robot) => robot.spawn(world),
        }
    }

    pub fn spawn_individual(&mut self, individual: &Individual, world: &mut World<f32>) {
        match *self {
            Robot::Fourdof(ref mut robot) => robot.spawn_individual(individual, world),
            Robot::Mechadon(ref mut robot) => robot.spawn_individual(individual, world),
        }
    }

    pub fn step(&mut self, world: &mut World<f32>, elapsed: f32) {
        match *self {
            Robot::Fourdof(ref mut robot) => robot.step(world, elapsed),
            Robot::Mechadon(ref mut robot) => robot.step(world, elapsed),
        }
    }

    pub fn fitness(&self, world: &World<f32>) -> f32 {
        match *self {
            Robot::Fourdof(ref robot) => robot.fitness(world),
            Robot::Mechadon(ref robot) => robot.fitness(world),
        }
    }

    pub fn body_handle(&self) -> Option<BodyHandle> {
        match *self {
            Robot::Fourdof(ref robot) => robot.body_handle(),
            Robot::Mechadon(ref robot) => robot.body_handle(),
        }
    }
}
