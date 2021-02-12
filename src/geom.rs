extern crate same;
extern crate vecmath;

use vecmath::traits::Float;
use vecmath::Vector3;

pub struct Ray<T> {
    pub orig: Vector3<T>,
    pub dir: Vector3<T>,
}

pub struct Poly<T, S> {
    points: [Vector3<T>; 3],
    plane: Plane<T>,
    pub surface: S,
}

struct Plane<T> {
    n: Vector3<T>,
    d: T,
}

struct Hit<T> {
    point: Vector3<T>,
    dist: T,
}

pub fn shoot<'a, T: Float, S, I: Iterator<Item = &'a Poly<T, S>>>(
    polys: I,
    ray: &Ray<T>,
) -> Option<(Vector3<T>, &'a Poly<T, S>)> {
    let mut closest: Option<(Hit<T>, &Poly<T, S>)> = None;

    for p in polys {
        if let Some(h) = p.hit(ray) {
            if closest.as_ref().map_or(true, |c| c.0.dist > h.dist) {
                closest = Some((h, &p));
            }
        }
    }

    return closest.map(|x| (x.0.point, x.1));
}

impl<T: Float, S> Poly<T, S> {
    pub fn new(points: [Vector3<T>; 3], surface: S) -> Poly<T, S> {
        let plane = {
            // Surface normal.
            let n = vecmath::vec3_normalized(vecmath::vec3_cross(
                vecmath::vec3_sub(points[1], points[0]),
                vecmath::vec3_sub(points[2], points[0]),
            ));

            // Surface to origin distance.
            let d = vecmath::vec3_dot(n, points[0]);

            Plane { n, d }
        };

        return Poly {
            points,
            plane,
            surface,
        };
    }

    pub fn n(&self) -> &Vector3<T> {
        return &self.plane.n;
    }

    fn hit(&self, ray: &Ray<T>) -> Option<Hit<T>> {
        let n = self.plane.n;

        let dot = vecmath::vec3_dot(n, ray.dir);

        if dot == T::zero() {
            // Ray is parallel to plane.
            return None;
        }

        // Distance of ray to hit point of the plane.
        let d = (-vecmath::vec3_dot(n, ray.orig) + self.plane.d) / dot;

        if d <= T::zero() {
            // Plane is behind the ray.
            return None;
        }

        // Hit point on the plane.
        let p = vecmath::vec3_add(ray.orig, vecmath::vec3_scale(ray.dir, d));

        for i in 0..3 {
            let edge = vecmath::vec3_sub(self.points[(i + 1) % 3], self.points[i]);
            let c = vecmath::vec3_sub(p, self.points[i]);

            if vecmath::vec3_dot(n, vecmath::vec3_cross(edge, c)) < T::zero() {
                // Point is on wrong side of edge.
                return None;
            }
        }

        return Some(Hit { point: p, dist: d });
    }
}
