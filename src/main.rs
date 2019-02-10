extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate num_traits as num;
extern crate rand;
#[macro_use]
extern crate stdweb;
extern crate time;

mod actuator;
mod crucible;
#[allow(dead_code)]
mod engine;
mod individual;
#[allow(dead_code)]
mod objects;
mod population;
mod robot;
mod testbed;
mod world_owner;

use crate::testbed::Testbed;
use na::Point3;

fn main() {
    let mut testbed = Testbed::new_empty();
    testbed.look_at(Point3::new(60.0, 40.0, 30.0), Point3::new(0.0, 2.0, 0.0));
    testbed.run();
}
