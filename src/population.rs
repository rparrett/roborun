use crate::individual::Individual;
use rand::{OsRng, Rng, sample};

pub struct Population {
    pub individuals: Vec<Individual>,
    pub num: u32
}

impl Population {
    pub fn random(num: u32) -> Population {
        let mut individuals: Vec<Individual> = Vec::new();
        for _ in 0..num {
            individuals.push(Individual::random(3, 3 * 16)); // TODO magic
        }

        Population {
            individuals: individuals,
            num: num
        }
    }

    pub fn best(&self) -> &Individual {
        self.individuals.iter().max_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap()).unwrap()
    }

    pub fn cull(&mut self) {
        let mut rng = OsRng::new().unwrap();

        // TODO magic
        let keep = (0.1 * self.num as f32) as u32;
        let randos = (self.num - keep) / 2;
        let spawn = self.num - keep - randos;

        self.individuals.sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap());
        self.individuals.truncate(keep as usize);

        for i in self.individuals.iter_mut().skip(1) {
            i.mutate();
        }
        
        for _ in 0..randos { // TODO magic
            self.individuals.push(Individual::random(3, 3 * 16)); // TODO magic
        }
        
        for _ in 0..spawn { // TODO magic
            let parents = sample(&mut rng, self.individuals.iter().by_ref().take(keep as usize), 2);
            let child = Individual::breed(parents[0], parents[1]);
            self.individuals.push(child);
        }
    }
}
