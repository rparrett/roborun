use crate::individual::Individual;
use crate::population::Population;
use crate::robot::Robot;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, Cylinder, ShapeHandle};
use nphysics3d::object::{BodyHandle, ColliderDesc};
use nphysics3d::world::World;

pub struct GenerationStats {
    pub generation: usize,
    pub avg_fitness: f32,
    pub min_fitness: f32,
    pub max_fitness: f32,
}

pub struct Crucible {
    pub population: Population,
    pub generation: usize,
    pub individual: usize,
    pub step: usize,
    pub max_step: usize,
    world: World<f32>,
    robot: Robot,
    elapsed: f32,
    pub last_best: Individual,
    pub stats: Vec<GenerationStats>,
}

impl Crucible {
    pub fn new() -> Crucible {
        let population = Population::random(100);
        let mut world = make_world();
        let robot = Robot::from_individual(&population.individuals[0], &mut world);
        let last_best = population.individuals[0].clone();

        Crucible {
            population: population,
            generation: 1,
            individual: 0,
            step: 0,
            max_step: 1500,
            world: world,
            robot: robot,
            elapsed: 0.0,
            last_best: last_best,
            stats: Vec::new(),
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

        if self.step == self.max_step {
            self.step = 0;
            self.elapsed = 0.0;

            self.population.individuals[self.individual].fitness =
                self.robot.fitness(&mut self.world);

            self.individual += 1;

            if self.individual == self.population.num {
                self.last_best = self.population.best().clone();

                let stats = self.population.stats();

                self.stats.push(GenerationStats {
                    generation: self.generation,
                    min_fitness: stats.0,
                    max_fitness: stats.1,
                    avg_fitness: stats.2,
                });

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
