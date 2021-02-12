extern crate vecmath;

use vecmath::traits::Float;
use vecmath::Vector3;

use crate::geom::Poly;

pub fn add_quad<T: Float, S: Clone>(pts: [Vector3<T>; 4], surface: S, trg: &mut Vec<Poly<T, S>>) {
    trg.push(Poly::new([pts[0], pts[1], pts[2]], surface.clone()));
    trg.push(Poly::new([pts[0], pts[3], pts[2]], surface));
}
