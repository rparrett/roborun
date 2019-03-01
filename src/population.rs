use crate::individual::Individual;
use rand::rngs::OsRng;
use rand::seq::IteratorRandom;
use rand::Rng;

pub struct PopulationBuilder {
    num: usize,
    crossover_rate: f32,
    mutation_rate: f32,
    elitist: bool,
}

impl PopulationBuilder {
    pub fn new() -> PopulationBuilder {
        PopulationBuilder {
            num: 100,
            crossover_rate: 0.6,
            mutation_rate: 0.07,
            elitist: false,
        }
    }

    pub fn size(mut self, v: usize) -> PopulationBuilder {
        self.num = v;
        self
    }

    pub fn crossover_rate(mut self, v: f32) -> PopulationBuilder {
        self.crossover_rate = v;
        self
    }

    pub fn mutation_rate(mut self, v: f32) -> PopulationBuilder {
        self.mutation_rate = v;
        self
    }

    pub fn elitist(mut self, v: bool) -> PopulationBuilder {
        self.elitist = v;
        self
    }

    pub fn build(self) -> Population {
        let mut individuals: Vec<Individual> = Vec::with_capacity(self.num);

        for _ in 0..self.num {
            individuals.push(Individual::random(3, 3 * 16, 3)); // TODO magic
        }

        Population {
            individuals,
            num: self.num,
            crossover_rate: self.crossover_rate,
            mutation_rate: self.mutation_rate,
            elitist: self.elitist,
        }
    }
}

pub struct Population {
    pub individuals: Vec<Individual>,
    pub num: usize,
    crossover_rate: f32,
    mutation_rate: f32,
    elitist: bool,
}

impl Population {
    pub fn best(&self) -> &Individual {
        self.individuals
            .iter()
            .max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap())
            .unwrap()
    }

    pub fn stats(&self) -> (f32, f32, f32) {
        let mut min = std::f32::MAX;
        let mut max = 0.0;
        let mut sum = 0.0;

        for i in self.individuals.iter() {
            if i.fitness < min {
                min = i.fitness;
            }
            if i.fitness > max {
                max = i.fitness;
            }
            sum += i.fitness;
        }

        let avg = sum / self.individuals.len() as f32;

        (min, max, avg)
    }

    pub fn cull(&mut self) {
        let mut rng = OsRng::new().unwrap();

        let mut gen: Vec<Individual> = Vec::with_capacity(self.num);

        // crossover (or clone)
        while gen.len() < self.num {
            // TODO magic
            let indices = self.tournament_select_two(&mut rng, 4);
            let parents = (&self.individuals[indices.0], &self.individuals[indices.1]);

            let mut children = if rng.gen_range(0.0, 1.0) > self.crossover_rate {
                Individual::one_gap_one_point(parents.0, parents.1)
            } else {
                ((*parents.0).clone(), (*parents.1).clone())
            };

            children.0.mutate(self.mutation_rate);
            children.1.mutate(self.mutation_rate);
            gen.push(children.0);
            gen.push(children.1);
        }

        if self.elitist {
            let best = self
                .individuals
                .iter()
                .cloned()
                .max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap())
                .unwrap();

            gen[0] = best;
        }

        self.individuals = gen;
    }

    pub fn tournament_select_one<R: Rng>(&self, rng: &mut R, size: usize) -> usize {
        (0..self.num)
            .choose_multiple(rng, size)
            .iter()
            .cloned()
            .max_by(|a, b| {
                self.individuals[*a]
                    .fitness
                    .partial_cmp(&self.individuals[*b].fitness)
                    .unwrap()
            })
            .unwrap()
    }

    pub fn tournament_select_two<R: Rng>(&self, rng: &mut R, size: usize) -> (usize, usize) {
        let a = self.tournament_select_one(rng, size);

        // inf if population size is 1
        let b = loop {
            let next = self.tournament_select_one(rng, size);
            if next != a {
                break next;
            }
        };

        (a, b)
    }
}
