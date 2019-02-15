use crate::individual::Individual;
use crate::population::Population;
use crate::robot::Robot;
use na::Vector3;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::object::ColliderDesc;
use nphysics3d::world::World;
use serde::Serialize;

#[derive(Clone, Serialize)]
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
    build_robot: Callback,
}

type Callback = Box<Fn(&Individual, &mut World<f32>) -> Robot>;

impl Crucible {
    pub fn new<F: Fn(&Individual, &mut World<f32>) -> Robot + 'static>(
        population: Population,
        max_step: usize,
        build_robot: F,
    ) -> Crucible {
        let mut world = make_world();
        let robot = build_robot(&population.individuals[0], &mut world);
        let last_best = population.individuals[0].clone();

        Crucible {
            population,
            max_step,
            world,
            robot,
            last_best,
            generation: 1,
            individual: 0,
            step: 0,
            elapsed: 0.0,
            stats: Vec::new(),
            build_robot: Box::new(build_robot),
        }
    }

    pub fn step(&mut self) {
        if self.step == 0 {
            self.world = make_world();
            self.robot = (self.build_robot)(
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
    let ground_material = BasicMaterial::new(0.3, 0.9); // a somewhat squishy high friction ground
    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -10.0)
        .material(MaterialHandle::new(ground_material))
        .build(&mut world);

    return world;
}
