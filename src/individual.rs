use rand::rngs::OsRng;
use rand::Rng;

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

        Individual {
            genes: genes,
            fitness: 0.0,
        }
    }

    pub fn mutate(&mut self) {
        let mut rng = OsRng::new().unwrap();

        // TODO: magic
        let mutation_magnitude = 0.2;
        let remove_chance = 0.2;
        let add_chance = 0.2;
        let alignment = 3;

        let i = rng.gen_range(0, self.genes.len());

        self.genes[i] *= rng.gen_range(1.0 - mutation_magnitude, 1.0 + mutation_magnitude);

        if self.genes[i] > 1.0 - std::f32::EPSILON {
            self.genes[i] = 1.0 - std::f32::EPSILON;
        }
        if self.genes[i] < 0.0 {
            self.genes[i] = 0.0;
        }

        if rng.gen_range(0.0, 1.0) < remove_chance {
            let i = rng.gen_range(0, self.genes.len() / alignment) * alignment;

            for _ in 0..alignment {
                self.genes.remove(i);
            }
        }

        if rng.gen_range(0.0, 1.0) < add_chance {
            for _ in 0..alignment {
                self.genes.push(rng.gen_range(0.0, 1.0));
            }
        }

        self.fitness = 0.0;
    }

    pub fn breed(parent_a: &Individual, parent_b: &Individual) -> Individual {
        let mut rng = OsRng::new().unwrap();

        let alignment = 3;

        let mut child = (*parent_a).clone();
        child.fitness = 0.0;

        let take_half = round_down_to_multiple(parent_b.genes.len() / 2, alignment);
        let take_start = round_down_to_multiple(rng.gen_range(0, take_half), alignment);

        let give_half = round_down_to_multiple(child.genes.len() / 2, alignment);
        let give_start = round_down_to_multiple(rng.gen_range(0, give_half), alignment);

        child.genes.splice(
            give_start..(give_start + give_half),
            (*parent_b)
                .genes
                .iter()
                .cloned()
                .skip(take_start)
                .take(take_half),
        );
        child.mutate();

        child
    }
}

fn round_down_to_multiple(num: usize, multiple: usize) -> usize {
    // we don't want zero
    if num <= multiple {
        return multiple;
    }

    let rem = num % multiple;
    if rem != 0 {
        return num - rem;
    }

    num
}
