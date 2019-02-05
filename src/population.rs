use crate::individual::Individual;
use rand::rngs::OsRng;
use rand::seq::IteratorRandom;
use rand::Rng;

pub struct Population {
    pub individuals: Vec<Individual>,
    pub num: usize,
}

impl Population {
    pub fn random(num: usize) -> Population {
        let mut individuals: Vec<Individual> = Vec::new();
        for _ in 0..num {
            individuals.push(Individual::random(3, 3 * 16)); // TODO magic
        }

        Population {
            individuals: individuals,
            num: num,
        }
    }

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

        // TODO magic
        let selection_ratio = 0.2;
        let crossover_ratio = 0.6;
        let crossover_clone_ratio = 0.5;
        let mutation_rate = 0.05;

        let keep = (selection_ratio * self.num as f32) as usize;
        let breed = (crossover_ratio * self.num as f32) as usize;
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

            if rng.gen_range(0.0, 1.0) > crossover_clone_ratio {
                let parents = self
                    .individuals
                    .iter()
                    .by_ref()
                    .take(keep as usize)
                    .choose_multiple(&mut rng, 2);
                let child = Individual::breed(parents[0], parents[1]);
                self.individuals.push(child);
            } else {
                let mut child = self
                    .individuals
                    .iter()
                    .take(keep as usize)
                    .choose(&mut rng)
                    .unwrap()
                    .clone();
                child.mutate();
                self.individuals.push(child);
            }
        }

        for _ in 0..fill {
            self.individuals.push(Individual::random(3, 3 * 16)); // TODO magic
        }

        for i in 0..keep {
            if rng.gen_range(0.0, 1.0) < mutation_rate {
                self.individuals[i].mutate();
            }
        }
    }
}
