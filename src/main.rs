#[macro_use]
extern crate stdweb;

extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate num_traits as num;
extern crate rand;
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

use crate::engine::GraphicsManager;
use crate::individual::Individual;
use crate::population::Population;
use crate::testbed::Testbed;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, Cylinder, ShapeHandle};
use nphysics3d::object::{BodyHandle, ColliderDesc};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use robot::Robot;
use std::cell::RefCell;

fn main() {
    let mut testbed = Testbed::new_empty();
    testbed.look_at(Point3::new(60.0, 40.0, 30.0), Point3::new(0.0, 2.0, 0.0));
    testbed.run();
}
