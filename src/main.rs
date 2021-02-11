extern crate image;
extern crate quaternion;
extern crate vecmath;

use image::{GenericImage, Rgb, RgbImage};
use vecmath::traits::Float;
use vecmath::Vector3;

use std::option::Option;

struct Ray<T> {
    orig: Vector3<T>,
    dir: Vector3<T>,
}

struct Poly<T, C> {
    points: [Vector3<T>; 3],
    plane: Plane<T>,
    color: C,
}

struct Plane<T> {
    n: Vector3<T>,
    d: T,
}

impl<T: Float, C> Poly<T, C> {
    fn new(points: [Vector3<T>; 3], color: C) -> Poly<T, C> {
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
            color,
        };
    }

    fn hit(&self, ray: &Ray<T>) -> Option<T> {
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

        return Some(d);
    }
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
    let mut img = RgbImage::new(1001, 601);

    let scene = Scene {
        polys: vec![
            Poly::new(
                [[2.0, 1.0, -8.0], [0.0, 0.0, -10.0], [-1.0, 1.0, -9.0]],
                Rgb([255, 0, 0]),
            ),
            Poly::new(
                [[1.0, 1.0, -12.0], [0.0, 3.0, -8.0], [-3.0, -3.0, -8.0]],
                Rgb([0, 0, 255]),
            ),
            Poly::new(
                [[2.0, 0.0, -8.0], [2.0, 0.0, -15.0], [1.5, -3.0, -15.0]],
                Rgb([0, 255, 0]),
            ),
            Poly::new(
                [[-2.0, -1.0, -2.0], [-1.0, 2.0, -12.0], [1.5, -2.0, -5.0]],
                Rgb([255, 255, 0]),
            ),
        ],
        background: Rgb([0, 0, 0]),
    };

    let front = Camera {
        orig: [0.0, 0.0, 10.0],
        dir: [0.0, 0.0, -1.0],
        up: [0.0, 1.0, 0.0],
        aperture: 30.0 / 180.0 * std::f64::consts::PI, // deg
    };

    let back = Camera {
        orig: [0.0, 0.0, -25.0],
        dir: [0.0, 0.0, 1.0],
        up: [0.0, 1.0, 0.0],
        aperture: 30.0 / 180.0 * std::f64::consts::PI, // deg
    };

    let right = Camera {
        orig: [20.0, 0.0, -10.0],
        dir: [-1.0, 0.0, 0.0],
        up: [0.0, 1.0, 0.0],
        aperture: 30.0 / 180.0 * std::f64::consts::PI, // deg
    };

    let left = Camera {
        orig: [-20.0, 0.0, -10.0],
        dir: [1.0, 0.0, 0.0],
        up: [0.0, 1.0, 0.0],
        aperture: 30.0 / 180.0 * std::f64::consts::PI, // deg
    };

    render(&scene, &front, &mut img.sub_image(0, 0, 500, 300));
    render(&scene, &back, &mut img.sub_image(0, 301, 500, 300));
    render(&scene, &right, &mut img.sub_image(501, 0, 500, 300));
    render(&scene, &left, &mut img.sub_image(501, 301, 500, 300));

    for i in 0..1001 {
        img.put_pixel(i, 300, Rgb([255, 255, 255]));
    }

    for i in 0..601 {
        img.put_pixel(500, i, Rgb([255, 255, 255]));
    }

    img.save("test.png").unwrap();
}

fn render<F: Float, I: GenericImage>(scene: &Scene<F, I::Pixel>, camera: &Camera<F>, img: &mut I) {
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
        if let Some(d) = p.hit(ray) {
            if closest.map_or(true, |c| c.0 > d) {
                closest = Some((d, &p));
            }
        }
    }

    return closest.map_or(scene.background, |c| c.1.color);
}
