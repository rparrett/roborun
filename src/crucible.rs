use crate::individual::Individual;
use crate::population::Population;
use crate::robot::Robot;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, Cylinder, ShapeHandle};
use nphysics3d::object::{BodyHandle, ColliderDesc};
use nphysics3d::world::World;

pub struct Crucible {
    pub population: Population,
    pub generation: usize,
    pub individual: usize,
    pub step: usize,
    world: World<f32>,
    robot: Robot,
    elapsed: f32,
    best: Option<Individual>,
}

impl Crucible {
    pub fn new() -> Crucible {
        let population = Population::random(100);
        let mut world = make_world();
        let robot = Robot::from_individual(&population.individuals[0], &mut world);

        Crucible {
            population: population,
            generation: 1,
            individual: 0,
            step: 0,
            world: world,
            robot: robot,
            elapsed: 0.0,
            best: None,
        }
    }

    pub fn step(&mut self) {
        if self.step == 0 {
            self.world = make_world();
            self.robot = Robot::from_individual(
                &self.population.individuals[self.individual],
                &mut self.world,
            );
        }

        self.robot.step(&mut self.world, self.elapsed);
        self.world.step();
        self.step += 1;
        self.elapsed = self.world.timestep();

        if self.step == 1000 {
            self.step = 0;
            self.elapsed = 0.0;

            self.population.individuals[self.individual].fitness =
                self.robot.fitness(&mut self.world);

            self.individual += 1;

            if self.individual == self.population.num {
                console!(
                    log,
                    format!(
                        "gen {}: {}",
                        self.generation,
                        self.population.best().fitness
                    )
                );

                self.best = Some(self.population.best().clone());
                self.generation += 1;
                self.population.cull();
                self.individual = 0;
            }
        }
    }
}

pub fn make_world() -> World<f32> {
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(100.0, 10.0, 100.0)));
    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -10.0)
        .build(&mut world);

    return world;
}
