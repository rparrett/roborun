use crate::individual::Individual;
use rand::rngs::OsRng;
use rand::seq::IteratorRandom;
use rand::Rng;

pub struct PopulationBuilder {
    num: usize,
    selection_rate: f32,
    crossover_rate: f32,
    mutation_rate: f32,
}

impl PopulationBuilder {
    pub fn new() -> PopulationBuilder {
        PopulationBuilder {
            num: 100,
            selection_rate: 0.2,
            crossover_rate: 0.6,
            mutation_rate: 0.07,
        }
    }

    pub fn size(mut self, v: usize) -> PopulationBuilder {
        self.num = v;
        self
    }

    pub fn selection_rate(mut self, v: f32) -> PopulationBuilder {
        self.selection_rate = v;
        self
    }

    pub fn crossover_rate(mut self, v: f32) -> PopulationBuilder {
        self.selection_rate = v;
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
            selection_rate: self.selection_rate,
            crossover_rate: self.crossover_rate,
            mutation_rate: self.mutation_rate,
        }
    }
}

pub struct Population {
    pub individuals: Vec<Individual>,
    pub num: usize,
    selection_rate: f32,
    crossover_rate: f32,
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

        // dumb truncation selection

        let keep = (self.selection_rate * self.num as f32) as usize;

        self.individuals
            .sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap());
        self.individuals.truncate(keep);

        // crossover (or clone)
        while self.individuals.len() < self.num {
            let parents = self
                .individuals
                .iter()
                .by_ref()
                .take(keep as usize)
                .choose_multiple(&mut rng, 2);

            let mut children = if rng.gen_range(0.0, 1.0) > self.crossover_rate {
                Individual::one_gap_one_point(parents[0], parents[1])
            } else {
                (parents[0].clone(), parents[1].clone())
            };

            children.0.mutate(self.mutation_rate);
            children.1.mutate(self.mutation_rate);
            self.individuals.push(children.0);
            self.individuals.push(children.1);
        }
    }
}
