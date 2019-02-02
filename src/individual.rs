use rand::{Rng, OsRng};

pub struct Individual {
    pub genes: Vec<f32>
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
            genes: genes
        }
    }
}
