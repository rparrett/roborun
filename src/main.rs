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
mod population;
mod robot;
mod testbed;
mod world_owner;

use crate::engine::GraphicsManager;
use crate::individual::Individual;
use crate::population::Population;
use crate::testbed::Testbed;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, Cylinder, ShapeHandle};
use nphysics3d::object::{BodyHandle, ColliderDesc};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use robot::Robot;
use std::cell::RefCell;

fn evaluate_population(population: &mut Population) {
    for individual in population.individuals.iter_mut() {
        let mut world = make_world();
        let mut robot = Robot::from_individual(individual, &mut world);

        let mut elapsed = 0.0;

        for _ in 0..1000 {
            robot.step(&mut world, elapsed);
            world.step();
            elapsed = world.timestep();
        }

        individual.fitness = robot.fitness(&mut world);
    }
}

fn make_world() -> World<f32> {
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(100.0, 10.0, 100.0)));
    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -10.0)
        .build(&mut world);

    return world;
}

fn main() {
    let mut population = Population::random(50);
    evaluate_population(&mut population);
    console!(log, format!("gen {}: {}", 1, population.best().fitness));

    for gen in 0..100 {
        population.cull();
        evaluate_population(&mut population);
        console!(
            log,
            format!("gen {}: {}", gen + 2, population.best().fitness)
        );
    }

    let mut world = make_world();

    let mut robot = Robot::from_individual(population.best(), &mut world);

    console!(log, format!("showing you: {:?}", population.best()));
    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new_empty();
    testbed.set_world(world);
    testbed.set_body_color(robot.body, Point3::new(0.0, 1.0, 0.0));

    let robot = RefCell::new(robot);

    testbed.add_callback(move |world_owner, _, time| {
        let mut w = world_owner.get_mut();

        let mut robot = robot.borrow_mut();

        let t = w.timestep();

        robot.step(&mut w, t);
    });

    testbed.look_at(Point3::new(60.0, 40.0, 30.0), Point3::new(0.0, 2.0, 0.0));
    testbed.run();
}
