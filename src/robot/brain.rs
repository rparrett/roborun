use crate::actuator::Actuator;
use nphysics3d::world::World;

#[derive(Debug)]
pub struct Actuation {
    actuator: usize,
    time: f32,
    position: f32,
}

impl Actuation {
    pub fn new(actuator: usize, time: f32, position: f32) -> Actuation {
        Actuation {
            actuator,
            time,
            position,
        }
    }
}

pub struct DumbBrain {
    time: f32,
    actuations: Vec<Actuation>,
    current_actuation: usize,
    actuators: Vec<Actuator>,
}

impl DumbBrain {
    pub fn new() -> DumbBrain {
        DumbBrain {
            time: 0.0,
            actuations: Vec::new(),
            current_actuation: 0,
            actuators: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        self.time = 0.0;
        self.current_actuation = 0;
        self.actuations.clear();
    }

    pub fn setup(&mut self) {
        self.actuations
            .sort_by(|a, b| a.time.partial_cmp(&b.time).unwrap());
    }

    pub fn len_actuators(&self) -> usize {
        self.actuators.len()
    }

    pub fn push_actuator(&mut self, actuator: Actuator) {
        self.actuators.push(actuator)
    }

    pub fn push_actuation(&mut self, actuator: usize, time: f32, position: f32) {
        self.actuations
            .push(Actuation::new(actuator, time, position))
    }

    pub fn actuator(&self, actuator: usize) -> &Actuator {
        &self.actuators[actuator]
    }

    pub fn step(&mut self, world: &mut World<f32>, elapsed: f32) {
        // Step through our list of desired actuations until our current time
        // is past the last actuation. Then reset everything.
        //
        // It's possible that an actuator could be commanded to multiple
        // positions within the same time step. In that case, all but the
        // last are effectively ignored.
        //
        // Note: the cycle resets after the last commanded actuation, even if
        // there is time remaining in the maximum cycle length. This lets bots
        // achieve a higher frequency of oscillation without explicitly encoding
        // that frequency in a gene. It would be interesting to try that approach,
        // because as genomes grow, the effective cycle length trends towards the
        // maximum.

        self.time += elapsed;

        loop {
            // This really should never happen, and we could probably just
            // unwrap.

            let actuation = self.actuations.get(self.current_actuation);
            if actuation.is_none() {
                break;
            }
            let actuation = actuation.unwrap();

            if self.time < actuation.time {
                break;
            }

            if let Some(actuator) = self.actuators.get_mut(actuation.actuator) {
                actuator.set_position(actuation.position);
            } else {
                break;
            }

            self.current_actuation += 1;

            if self.current_actuation >= self.actuations.len() {
                self.current_actuation = 0;
                self.time = 0.0;

                break;
            }
        }

        for a in self.actuators.iter_mut() {
            a.step(world);
        }
    }
}
