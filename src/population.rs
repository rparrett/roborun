use crate::individual::Individual;
use rand::rngs::OsRng;
use rand::seq::IteratorRandom;
use rand::Rng;

pub struct PopulationBuilder {
    num: usize,
    selection_ratio: f32,
    crossover_ratio: f32,
    crossover_clone_ratio: f32,
    mutation_rate: f32,
}

impl PopulationBuilder {
    pub fn new() -> PopulationBuilder {
        PopulationBuilder {
            num: 100,
            selection_ratio: 0.2,
            crossover_ratio: 0.6,
            crossover_clone_ratio: 0.5,
            mutation_rate: 0.07,
        }
    }

    pub fn size(mut self, v: usize) -> PopulationBuilder {
        self.num = v;
        self
    }

    pub fn selection_ratio(mut self, v: f32) -> PopulationBuilder {
        self.selection_ratio = v;
        self
    }

    pub fn crossover_ratio(mut self, v: f32) -> PopulationBuilder {
        self.selection_ratio = v;
        self
    }

    pub fn crossover_clone_ratio(mut self, v: f32) -> PopulationBuilder {
        self.crossover_clone_ratio = v;
        self
    }

    pub fn mutation_rate(mut self, v: f32) -> PopulationBuilder {
        self.mutation_rate = v;
        self
    }

    pub fn build(self) -> Population {
        let mut individuals: Vec<Individual> = Vec::new();
        for _ in 0..self.num {
            individuals.push(Individual::random(3, 3 * 16, 3)); // TODO magic
        }

        Population {
            individuals,
            num: self.num,
            selection_ratio: self.selection_ratio,
            crossover_ratio: self.crossover_ratio,
            crossover_clone_ratio: self.crossover_clone_ratio,
            mutation_rate: self.mutation_rate,
        }
    }
}

pub struct Population {
    pub individuals: Vec<Individual>,
    pub num: usize,
    selection_ratio: f32,
    crossover_ratio: f32,
    crossover_clone_ratio: f32,
    mutation_rate: f32,
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

        let keep = (self.selection_ratio * self.num as f32) as usize;
        let breed = (self.crossover_ratio * self.num as f32) as usize;
        let fill = if breed + keep > self.num {
            0
        } else {
            self.num - breed - keep
        };

        self.individuals
            .sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap());
        self.individuals.truncate(keep);

        for _ in 0..breed {
            // crossover or just clone and mutate

            if rng.gen_range(0.0, 1.0) > self.crossover_clone_ratio {
                let parents = self
                    .individuals
                    .iter()
                    .by_ref()
                    .take(keep as usize)
                    .choose_multiple(&mut rng, 2);
                let mut child = Individual::one_gap_one_point(parents[0], parents[1]);
                child.mutate(self.mutation_rate);
                self.individuals.push(child);
            } else {
                let mut child = self
                    .individuals
                    .iter()
                    .take(keep as usize)
                    .choose(&mut rng)
                    .unwrap()
                    .clone();
                child.mutate(self.mutation_rate);
                self.individuals.push(child);
            }
        }

        for _ in 0..fill {
            self.individuals.push(Individual::random(3, 3 * 16, 3)); // TODO magic
        }

        for i in 0..keep {
            self.individuals[i].mutate(self.mutation_rate);
        }
    }
}
