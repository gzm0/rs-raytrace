use image::Rgb;

use vecmath::traits::Float;
use vecmath::Vector3;

use std::sync::Arc;

pub trait Black {
    fn black() -> Self;
}

impl<T: image::Primitive> Black for Rgb<T> {
    fn black() -> Rgb<T> {
        return Rgb([T::zero(), T::zero(), T::zero()]);
    }
}

pub trait Surface<T, P> {
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

pub fn matt<'a, T: Float, P: 'a + Black + Copy>(color: P) -> Arc<dyn 'a + Surface<T, P>> {
    Arc::new(Matt { color })
}

struct Matt<P> {
    color: P,
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

pub fn light<'a, T: Float, P: 'a + Copy + Black>(color: P) -> Arc<dyn 'a + Surface<T, P>> {
    Arc::new(Light { color })
}

struct Light<P> {
    color: P,
}

impl<T: Float, P: Copy + Black> Surface<T, P> for Light<P> {
    fn emitted(&self) -> P {
        return self.color;
    }
    fn reflected(&self, _n: Vector3<T>, _i: Vector3<T>, _o: Vector3<T>) -> P {
        return P::black();
    }
}
