use rand::{OsRng, Rng};

#[derive(Debug, Clone)]
pub struct Individual {
    pub genes: Vec<f32>,
    pub fitness: f32,
}

impl Individual {
    pub fn random(min: i32, max: i32) -> Individual {
        let mut rng = OsRng::new().unwrap();

        let mut genes: Vec<f32> = Vec::new();

        let num: i32 = rng.gen_range(min, max + 1);

        for _ in 0..num {
            let gene: f32 = rng.gen_range(0.0, 1.0);
            genes.push(gene);
        }

        Individual { genes: genes, fitness: 0.0 }
    }

    pub fn mutate(&mut self) {
        let mut rng = OsRng::new().unwrap();

        // TODO: magic
        let mutation_magnitude = 0.2;

        let i = rng.gen_range(0, self.genes.len());

        self.genes[i] *= rng.gen_range(1.0 - mutation_magnitude, 1.0 + mutation_magnitude);

        if self.genes[i] > 1.0 {
            self.genes[i] = 1.0;
        }
        if self.genes[i] < 0.0 {
            self.genes[i] = 0.0;
        }

        // TODO: magic
        if rng.gen_range(0.0, 1.0) > 0.8 {
            self.genes.remove(rng.gen_range(0, self.genes.len()));
        }
        
        // TODO: magic
        if rng.gen_range(0.0, 1.0) > 0.8 {
            self.genes.push(rng.gen_range(0.0, 1.0));
        }

        self.fitness = 0.0;
    }

    pub fn breed(parent_a: &Individual, parent_b: &Individual) -> Individual {
        let mut rng = OsRng::new().unwrap();

        let mut child = (*parent_a).clone();
        child.fitness = 0.0;

        let take_half = parent_b.genes.len() / 2;
        let take_start = rng.gen_range(0, take_half);

        let give_half = child.genes.len() / 2;
        let give_start = rng.gen_range(0, give_half);

        child.genes.splice(
            give_start..(give_start + give_half),
            (*parent_b).genes.iter().cloned().skip(take_start).take(take_half)
        );
        child.mutate();

        child
    }
}
