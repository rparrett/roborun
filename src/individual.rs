use rand::rngs::OsRng;
use rand::Rng;

#[derive(Debug, Clone)]
pub struct Individual {
    pub genes: Vec<f32>,
    pub fitness: f32,
    pub alignment: usize,
}

impl Individual {
    pub fn random(min: i32, max: i32, alignment: usize) -> Individual {
        let mut rng = OsRng::new().unwrap();

        let mut genes: Vec<f32> = Vec::new();

        let num = round_down_to_multiple(rng.gen_range(min, max + 1) as usize, alignment);

        for _ in 0..num {
            let gene: f32 = rng.gen_range(0.0, 1.0);
            genes.push(gene);
        }

        Individual {
            genes,
            alignment,
            fitness: 0.0,
        }
    }

    pub fn mutate(&mut self, rate: f32) {
        let mut rng = OsRng::new().unwrap();

        // TODO: magic
        let mutation_magnitude = 0.2;

        // TODO: maybe we should do a chunked iteration here, and only
        // mutate one aspect of the gene.
        //
        // we could also randomly remove a gene by keeping a list of
        // removed indices and processing them in reverse order.

        for gene in self.genes.iter_mut() {
            if rng.gen_range(0.0, 1.0) > rate {
                continue;
            }

            *gene *= rng.gen_range(1.0 - mutation_magnitude, 1.0 + mutation_magnitude);

            if *gene > 1.0 - std::f32::EPSILON {
                *gene = 1.0 - std::f32::EPSILON;
            }
            if *gene < 0.0 {
                *gene = 0.0;
            }
        }

        self.fitness = 0.0;
    }

    pub fn one_gap_one_point(parent_a: &Individual, parent_b: &Individual) -> (Individual, Individual) {
        let mut rng = OsRng::new().unwrap();

        // we could assert(parent_a.alignment == parent_b.alignment)
        let alignment = parent_a.alignment;

        let (parent_l, parent_s) = match parent_a.genes.len() > parent_b.genes.len() {
            true => (parent_a, parent_b),
            false => (parent_b, parent_a),
        };

        let len = parent_l.genes.len();
        let diff = len - parent_s.genes.len();

        let gap_i = round_down_to_multiple(
            rng.gen_range(alignment, parent_s.genes.len() - 1),
            alignment,
        );

        let crossover_i = round_down_to_multiple(rng.gen_range(alignment, len - 1), alignment);

        let mut genes_a: Vec<f32> = Vec::new();
        let mut genes_b: Vec<f32> = Vec::new();

        for i in 0..len {
            let in_gap = i >= gap_i && i <= gap_i + diff - 1;

            let s_i = match i > gap_i {
                true => i - diff,
                false => i,
            };

            if i >= crossover_i {
                genes_a.push(*parent_l.genes.get(i).unwrap());
                if !in_gap {
                    genes_b.push(*parent_s.genes.get(s_i).unwrap());
                }
            } else {
                genes_b.push(*parent_l.genes.get(i).unwrap());
                if !in_gap {
                    genes_a.push(*parent_s.genes.get(s_i).unwrap());
                }
            }
        }

        let fitness = 0.0;
        let mut child_a = Individual {
            genes: genes_a,
            fitness,
            alignment,
        };
        let mut child_b = Individual {
            genes: genes_b,
            fitness,
            alignment,
        };

        (child_a, child_b)
    }

    pub fn messy_two_point(parent_a: &Individual, parent_b: &Individual) -> Individual {
        // this is a sort of "messy two point recombination" where the first
        // point is random-ish and the second point is always half the genome
        // length away from that, while taking care of the "alignment."
        //
        // https://journals.plos.org/plosone/article/file?id=10.1371/journal.pone.0209712&type=printable
        //
        // suggests that this is probably a terrible way to go about things.

        let mut rng = OsRng::new().unwrap();

        let mut child = (*parent_a).clone();
        child.fitness = 0.0;

        let mut take_num = round_down_to_multiple(parent_b.genes.len() / 2, child.alignment);
        if take_num < child.alignment {
            take_num = child.alignment;
        }
        let take_start = round_down_to_multiple(rng.gen_range(0, take_num), child.alignment);

        let mut give_num = round_down_to_multiple(child.genes.len() / 2, child.alignment);
        if give_num < child.alignment {
            give_num = child.alignment;
        }
        let give_start = round_down_to_multiple(rng.gen_range(0, give_num), child.alignment);

        child.genes.splice(
            give_start..(give_start + give_num),
            (*parent_b)
                .genes
                .iter()
                .cloned()
                .skip(take_start)
                .take(take_num),
        );

        child
    }
}

fn round_down_to_multiple(num: usize, multiple: usize) -> usize {
    let rem = num % multiple;
    if rem != 0 {
        return num - rem;
    }

    num
}
