extern crate image;
extern crate quaternion;
extern crate vecmath;

use image::{Rgb, RgbImage};
use vecmath::traits::Float;
use vecmath::Vector3;

use std::option::Option;

struct Ray<T> {
    orig: Vector3<T>,
    dir: Vector3<T>,
}

struct Plane<T> {
    n: Vector3<T>,
    d: T,
}

struct Poly<T, C> {
    pts: [Vector3<T>; 3],
    color: C,
}

struct Camera<T> {
    orig: Vector3<T>,
    dir: Vector3<T>,
    up: Vector3<T>,
    aperture: T, // aperture angle in radians
}

struct Scene<T, C> {
    polys: Vec<Poly<T, C>>,
    background: C,
}

const TRACE_DEPTH: u16 = 2;

fn main() {
    let mut img = RgbImage::new(500, 300);

    let cam = Camera {
        orig: [0.0, 0.0, 0.0],
        dir: [0.0, 0.0, -1.0],
        up: [0.0, 1.0, 0.0],
        aperture: 30.0 / 180.0 * std::f64::consts::PI, // deg
    };

    let scene = Scene {
        polys: vec![
            Poly {
                pts: [[1.0, 1.0, -10.0], [0.0, 0.0, -10.0], [-1.0, 1.0, -10.0]],
                color: Rgb([255, 0, 0]),
            },
            Poly {
                pts: [[1.0, 1.0, -12.0], [0.0, 3.0, -8.0], [-3.0, -3.0, -8.0]],
                color: Rgb([0, 0, 255]),
            },
            Poly {
                pts: [[2.0, 0.0, -8.0], [2.0, 0.0, -15.0], [1.5, -3.0, -15.0]],
                color: Rgb([0, 255, 0]),
            },
        ],
        background: Rgb([0, 0, 0]),
    };

    render(&scene, &cam, &mut img);

    img.save("test.png").unwrap();
}

fn render<F: Float, I: image::GenericImage>(
    scene: &Scene<F, I::Pixel>,
    camera: &Camera<F>,
    img: &mut I,
) {
    let (width, height) = img.dimensions();
    let center = vecmath::vec2_scale([F::from_u32(width), F::from_u32(height)], F::from_f64(0.5));
    let pix_ang = camera.aperture / F::from_u32(width);

    let xrot = camera.up;
    let yrot = vecmath::vec3_normalized(vecmath::vec3_cross(camera.dir, camera.up));

    for x in 0..width {
        for y in 0..height {
            let pos = [F::from_u32(x), F::from_u32(y)];
            let angles = vecmath::vec2_scale(vecmath::vec2_sub(pos, center), pix_ang);

            let q = quaternion::mul(
                quaternion::axis_angle(xrot, -angles[0]),
                quaternion::axis_angle(yrot, -angles[1]),
            );

            let r = Ray {
                orig: camera.orig,
                dir: quaternion::rotate_vector(q, camera.dir),
            };

            img.put_pixel(x, y, trace(scene, &r, TRACE_DEPTH));
        }
    }
}

fn trace<F: Float, C: Copy>(scene: &Scene<F, C>, ray: &Ray<F>, _depth: u16) -> C {
    let mut closest: Option<(F, &Poly<F, C>)> = None;

    for p in &scene.polys {
        if let Some(d) = dist(ray, &p) {
            if closest.map_or(true, |c| c.0 > d) {
                closest = Some((d, &p));
            }
        }
    }

    return closest.map_or(scene.background, |c| c.1.color);
}

fn dist<F: Float, C>(ray: &Ray<F>, poly: &Poly<F, C>) -> Option<F> {
    let pl = plane(poly);

    // Distance of origin to plane.
    let d = match intersect(ray, &pl) {
        None => return None,
        Some(d) => d,
    };

    // Hit point on the plane.
    let p = vecmath::vec3_add(ray.orig, vecmath::vec3_scale(ray.dir, d));

    for i in 0..3 {
        let opposite = vecmath::vec3_sub(poly.pts[(i + 1) % 3], poly.pts[(i + 2) % 3]);
        let ni = vecmath::vec3_cross(opposite, pl.n);

        let r = poly.pts[(i + 1) % 3];
        let pside = vecmath::vec3_sub(p, r);
        let iside = vecmath::vec3_sub(poly.pts[i], r);

        if vecmath::vec3_dot(ni, pside) / vecmath::vec3_dot(ni, iside) < F::zero() {
            return None;
        }
    }

    return Some(d);
}

fn intersect<F: Float>(ray: &Ray<F>, plane: &Plane<F>) -> Option<F> {
    let dot = vecmath::vec3_dot(plane.n, ray.dir);

    if dot == F::zero() {
        // Ray is parallel to plane.
        return None;
    }

    // Distance of ray to hit point of the plane.
    let d = (-vecmath::vec3_dot(plane.n, ray.orig) + plane.d) / dot;

    if d <= F::zero() {
        // Plane is behind the ray.
        return None;
    }

    return Some(d);
}

fn plane<T: Float, C>(poly: &Poly<T, C>) -> Plane<T> {
    // Surface normal.
    let n = vecmath::vec3_normalized(vecmath::vec3_cross(
        vecmath::vec3_sub(poly.pts[1], poly.pts[0]),
        vecmath::vec3_sub(poly.pts[0], poly.pts[2]),
    ));

    // Surface to origin distance.
    let d = vecmath::vec3_dot(n, poly.pts[0]);

    return Plane { n: n, d: d };
}
