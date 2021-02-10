extern crate image;
extern crate quaternion;
extern crate vecmath;

use image::{Rgb, RgbImage};
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

struct Poly {
    pts: [Vector3<f64>; 3],
    color: Rgb<u8>,
}

struct Camera<T> {
    orig: Vector3<T>,
    dir: Vector3<T>,
    up: Vector3<T>,
    aperture: T, // aperture angle in radians
}

struct Scene {
    polys: Vec<Poly>,
    background: Rgb<u8>,
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
        ],
        background: Rgb([0, 0, 0]),
    };

    render(&scene, &cam, &mut img);

    img.save("test.png").unwrap();
}

fn render(scene: &Scene, camera: &Camera<f64>, img: &mut RgbImage) {
    let center = vecmath::vec2_scale([img.width() as f64, img.height() as f64], 0.5);
    let pix_ang = camera.aperture / img.width() as f64;

    let xrot = camera.up;
    let yrot = vecmath::vec3_normalized(vecmath::vec3_cross(camera.dir, camera.up));

    for (x, y, c) in img.enumerate_pixels_mut() {
        let pos = [x as f64, y as f64];
        let angles = vecmath::vec2_scale(vecmath::vec2_sub(pos, center), pix_ang);

        let q = quaternion::mul(
            quaternion::axis_angle(xrot, -angles[0]),
            quaternion::axis_angle(yrot, -angles[1]),
        );

        let r = Ray {
            orig: camera.orig,
            dir: quaternion::rotate_vector(q, camera.dir),
        };

        *c = trace(scene, &r, TRACE_DEPTH);
    }
}

fn trace(scene: &Scene, ray: &Ray<f64>, _depth: u16) -> Rgb<u8> {
    let mut closest: Option<(f64, &Poly)> = None;

    for p in &scene.polys {
        if let Some(d) = dist(ray, &p) {
            if closest.map_or(true, |c| c.0 > d) {
                closest = Some((d, &p));
            }
        }
    }

    return closest.map_or(scene.background, |c| c.1.color);
}

fn dist(ray: &Ray<f64>, poly: &Poly) -> Option<f64> {
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

        if vecmath::vec3_dot(ni, pside) / vecmath::vec3_dot(ni, iside) < 0.0 {
            return None;
        }
    }

    return Some(d);
}

fn intersect(ray: &Ray<f64>, plane: &Plane<f64>) -> Option<f64> {
    let dot = vecmath::vec3_dot(plane.n, ray.dir);

    if dot == 0.0 {
        // Ray is parallel to plane.
        return None;
    }

    // Distance of ray to hit point of the plane.
    let d = (vecmath::vec3_dot(plane.n, ray.orig) + plane.d) / dot;

    if d <= 0.0 {
        // Plane is behind the ray.
        return None;
    }

    return Some(d);
}

fn plane(poly: &Poly) -> Plane<f64> {
    // Surface normal.
    let n = vecmath::vec3_normalized(vecmath::vec3_cross(
        vecmath::vec3_sub(poly.pts[1], poly.pts[0]),
        vecmath::vec3_sub(poly.pts[0], poly.pts[2]),
    ));

    // Surface to origin distance.
    let d = vecmath::vec3_dot(n, poly.pts[0]);

    return Plane { n: n, d: d };
}
