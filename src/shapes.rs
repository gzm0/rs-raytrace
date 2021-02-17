extern crate vecmath;

use vecmath::traits::Float;
use vecmath::Vector3;

use crate::geom::Poly;

pub fn add_par<T: Float, S: Clone>(
    a: Vector3<T>,
    b_side: Vector3<T>,
    c_side: Vector3<T>,
    surface: S,
    trg: &mut Vec<Poly<T, S>>,
) {
    let b = vecmath::vec3_add(a, b_side);
    let c = vecmath::vec3_add(a, c_side);
    let d = vecmath::vec3_add(b, c_side);

    trg.push(Poly::new([a, b, d], surface.clone()));
    trg.push(Poly::new([a, c, d], surface));
}
