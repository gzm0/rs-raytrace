extern crate image;
extern crate quaternion;
extern crate same;
extern crate vecmath;

mod geom;

use image::{GenericImage, Pixel, Rgb, RgbImage};
use vecmath::traits::Float;
use vecmath::Vector3;

use std::convert::TryInto;
use std::option::Option;

use geom::{Poly, Ray};
use same::Same;

struct Camera<T> {
    orig: Vector3<T>,
    dir: Vector3<T>,
    up: Vector3<T>,
    aperture: T, // aperture angle in radians
}

struct Scene<T, C> {
    polys: Vec<Poly<T, C>>,
    dark: C,
    bright: C,
    light: Vector3<T>,
}

struct Tracer<T> {
    all_dirs: Vec<Vector3<T>>,
    max_depth: u32,
}

impl<T: Float> Tracer<T> {
    fn new(rays: u32, max_depth: u32) -> Tracer<T> {
        let step = T::_360() / T::from_u32(rays);

        let u = [T::one(), T::zero(), T::zero()];

        let mut all_dirs = Vec::with_capacity((rays * rays).try_into().unwrap());

        for x in 0..rays {
            for y in 0..rays {
                let q = quaternion::euler_angles(
                    T::from_u32(x) * step,
                    T::from_u32(y) * step,
                    T::zero(),
                );
                all_dirs.push(quaternion::rotate_vector(q, u));
            }
        }

        return Tracer {
            all_dirs,
            max_depth,
        };
    }

    fn trace<C: Pixel<Subpixel = T>>(
        &self,
        scene: &Scene<T, C>,
        ray: &Ray<T>,
        exclude: Option<&Poly<T, C>>,
        depth: u32,
    ) -> C {
        if depth > self.max_depth {
            return scene.dark;
        }

        let polys: Box<dyn Iterator<Item = &Poly<T, C>>> = match exclude {
            Some(that_poly) => Box::new(scene.polys.iter().filter(move |x| !that_poly.same(x))),
            None => Box::new(scene.polys.iter()),
        };

        let (hit_point, poly) = match geom::shoot(polys, ray) {
            None => {
                return if vecmath::vec3_dot(ray.dir, scene.light)
                    > T::from_f64(30.0).deg_to_rad().cos()
                {
                    scene.bright
                } else {
                    scene.dark
                };
            }
            Some(hit) => hit,
        };

        let mut all_light = scene.dark;

        for dir in self.all_dirs.iter() {
            let v = vecmath::vec3_dot(*dir, *poly.n());

            if v == T::zero() {
                // Perpendicular to surface.
                continue;
            }

            if vecmath::vec3_dot(ray.dir, *poly.n()) / v > T::zero() {
                // Rays are not on the same side of the surfce.
                continue;
            }

            let r = Ray {
                orig: hit_point,
                dir: *dir,
            };

            let lambert = {
                if v < T::zero() {
                    -v
                } else {
                    v
                }
            };

            let light = self
                .trace(scene, &r, Some(poly), depth + 1)
                .map2(&poly.color, |x, y| x * y);

            all_light = all_light.map2(&light, |x, y| x + y * lambert);
        }

        return all_light;
    }
}

fn main() {
    let mut img = RgbImage::new(1001, 601);

    let scene = Scene {
        polys: vec![
            Poly::new(
                [[2.0, 1.0, -8.0], [0.0, 0.0, -10.0], [-1.0, 1.0, -9.0]],
                Rgb([0.5, 0.02, 0.02]),
            ),
            Poly::new(
                [[1.0, 1.0, -12.0], [0.0, 3.0, -8.0], [-3.0, -3.0, -8.0]],
                Rgb([0.02, 0.02, 0.5]),
            ),
            Poly::new(
                [[2.0, 0.0, -8.0], [2.0, 0.0, -15.0], [1.5, -3.0, -15.0]],
                Rgb([0.02, 0.5, 0.02]),
            ),
            Poly::new(
                [[-2.0, -1.0, -2.0], [-1.0, 2.0, -12.0], [1.5, -2.0, -5.0]],
                Rgb([0.4, 0.4, 0.02]),
            ),
            // Floor.
            Poly::new(
                [
                    [-50.0, -5.0, 50.0],
                    [-50.0, -5.0, -50.0],
                    [50.0, -5.0, -50.0],
                ],
                Rgb([0.4, 0.4, 0.4]),
            ),
            Poly::new(
                [[50.0, -5.0, -50.0], [50.0, -5.0, 50.0], [-50.0, -5.0, 50.0]],
                Rgb([0.4, 0.4, 0.4]),
            ),
        ],
        dark: Rgb([0.0, 0.0, 0.0]),
        bright: Rgb([255.0, 255.0, 255.0]),
        light: vecmath::vec3_normalized([1.0, 1.0, 1.0]),
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

    let tracer = Tracer::<f64>::new(6, 3);

    let gamma = |c: Rgb<f64>| -> Rgb<u8> {
        *Rgb::from_slice(&c.channels().iter().map(|x| (*x) as u8).collect::<Vec<u8>>())
    };

    render(
        &tracer,
        &scene,
        &front,
        gamma,
        &mut img.sub_image(0, 0, 500, 300),
    );
    render(
        &tracer,
        &scene,
        &back,
        gamma,
        &mut img.sub_image(0, 301, 500, 300),
    );
    render(
        &tracer,
        &scene,
        &right,
        gamma,
        &mut img.sub_image(501, 0, 500, 300),
    );
    render(
        &tracer,
        &scene,
        &left,
        gamma,
        &mut img.sub_image(501, 301, 500, 300),
    );

    for i in 0..1001 {
        img.put_pixel(i, 300, Rgb([255, 255, 255]));
    }

    for i in 0..601 {
        img.put_pixel(500, i, Rgb([255, 255, 255]));
    }

    img.save("test.png").unwrap();
}

fn render<F: Float, C: Pixel<Subpixel = F>, I: GenericImage, G: Fn(C) -> I::Pixel>(
    tracer: &Tracer<F>,
    scene: &Scene<F, C>,
    camera: &Camera<F>,
    gamma: G,
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

            let light = tracer.trace(scene, &r, None, 0);

            img.put_pixel(x, y, gamma(light));
        }
    }
}
