extern crate image;
extern crate quaternion;
extern crate same;
extern crate vecmath;

mod geom;
mod shapes;

use image::{GenericImage, Pixel, Rgb, RgbImage};
use vecmath::traits::Float;
use vecmath::Vector3;

use std::convert::TryInto;
use std::option::Option;
use std::sync::Arc;

use geom::{Poly, Ray};
use same::Same;

trait Black {
    fn black() -> Self;
}

impl<T: image::Primitive> Black for Rgb<T> {
    fn black() -> Rgb<T> {
        return Rgb([T::zero(), T::zero(), T::zero()]);
    }
}

struct Camera<T> {
    orig: Vector3<T>,
    dir: Vector3<T>,
    up: Vector3<T>,
    aperture: T, // aperture angle in radians
}

trait Surface<T, P> {
    fn emitted(&self) -> P;
    fn reflected(&self, n: Vector3<T>, i: Vector3<T>, o: Vector3<T>) -> P;
}

impl<T, P> Surface<T, P> for Arc<dyn Surface<T, P>> {
    fn emitted(&self) -> P {
        return (**self).emitted();
    }
    fn reflected(&self, n: Vector3<T>, i: Vector3<T>, o: Vector3<T>) -> P {
        return (**self).reflected(n, i, o);
    }
}

struct Matt<P> {
    color: P,
}

impl<'a, P: 'a + Copy + Black> Matt<P> {
    fn new<T: Float>(color: P) -> Arc<dyn 'a + Surface<T, P>> {
        Arc::new(Matt { color })
    }
}

impl<T: Float, P: Copy + Black> Surface<T, P> for Matt<P> {
    fn emitted(&self) -> P {
        return P::black();
    }
    fn reflected(&self, n: Vector3<T>, i: Vector3<T>, o: Vector3<T>) -> P {
        let v = vecmath::vec3_dot(i, n);

        if v == T::zero() {
            // Perpendicular to surface.
            return P::black();
        }

        if vecmath::vec3_dot(o, n) / v > T::zero() {
            // Rays are not on the same side of the surfce.
            return P::black();
        }

        return self.color;
    }
}

struct Light<P> {
    color: P,
}

impl<'a, P: 'a + Copy + Black> Light<P> {
    fn new<T: Float>(color: P) -> Arc<dyn 'a + Surface<T, P>> {
        Arc::new(Light { color })
    }
}

impl<T: Float, P: Copy + Black> Surface<T, P> for Light<P> {
    fn emitted(&self) -> P {
        return self.color;
    }
    fn reflected(&self, _n: Vector3<T>, _i: Vector3<T>, _o: Vector3<T>) -> P {
        return P::black();
    }
}

struct Scene<T, S> {
    polys: Vec<Poly<T, S>>,
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

    fn trace<C: Pixel<Subpixel = T> + Black + PartialEq, S: Surface<T, C>>(
        &self,
        scene: &Scene<T, S>,
        ray: &Ray<T>,
        exclude: Option<&Poly<T, S>>,
        depth: u32,
    ) -> C {
        if depth > self.max_depth {
            return C::black();
        }

        let maybe_hit = match exclude {
            Some(that_poly) => {
                let filtered = scene.polys.iter().filter(|x| !that_poly.same(x));
                geom::shoot(filtered, ray)
            }
            None => geom::shoot(scene.polys.iter(), ray),
        };

        let (hit_point, poly) = match maybe_hit {
            None => return C::black(),
            Some(hit) => hit,
        };

        let mut all_light = poly.surface.emitted();

        for dir in self.all_dirs.iter() {
            let refl = poly.surface.reflected(*poly.n(), *dir, ray.dir);

            if refl == C::black() {
                continue;
            }

            let v = vecmath::vec3_dot(*dir, *poly.n());

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
                .map2(&refl, |x, y| x * y);

            all_light = all_light.map2(&light, |x, y| x + y * lambert);
        }

        return all_light;
    }
}

fn main() {
    let mut img = RgbImage::new(1001, 601);

    let mut polys = Vec::<Poly<f64, Arc<dyn Surface<f64, Rgb<f64>>>>>::new();

    polys.push(Poly::new(
        [[2.0, 1.0, -8.0], [0.0, 0.0, -10.0], [-1.0, 1.0, -9.0]],
        Matt::new(Rgb([0.5, 0.02, 0.02])),
    ));
    polys.push(Poly::new(
        [[1.0, 1.0, -12.0], [0.0, 3.0, -8.0], [-3.0, -3.0, -8.0]],
        Matt::new(Rgb([0.02, 0.02, 0.5])),
    ));
    polys.push(Poly::new(
        [[2.0, 0.0, -8.0], [2.0, 0.0, -15.0], [1.5, -3.0, -15.0]],
        Matt::new(Rgb([0.02, 0.5, 0.02])),
    ));
    polys.push(Poly::new(
        [[-2.0, -1.0, -2.0], [-1.0, 2.0, -12.0], [1.5, -2.0, -5.0]],
        Matt::new(Rgb([0.4, 0.4, 0.02])),
    ));

    // Floor.
    shapes::add_quad(
        [
            [-50.0, -5.0, 50.0],
            [-50.0, -5.0, -50.0],
            [50.0, -5.0, -50.0],
            [50.0, -5.0, 50.0],
        ],
        Matt::new(Rgb([0.4, 0.4, 0.4])),
        &mut polys,
    );

    // Sky.
    shapes::add_quad(
        [
            [-50.0, 40.0, 50.0],
            [-50.0, 40.0, -50.0],
            [50.0, 40.0, -50.0],
            [50.0, 40.0, 50.0],
        ],
        Light::new(Rgb([255.0, 255.0, 255.0])),
        &mut polys,
    );

    let scene = Scene { polys: polys };

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

fn render<
    F: Float,
    S: Surface<F, C>,
    C: Pixel<Subpixel = F> + Black + PartialEq,
    I: GenericImage,
    G: Fn(C) -> I::Pixel,
>(
    tracer: &Tracer<F>,
    scene: &Scene<F, S>,
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
