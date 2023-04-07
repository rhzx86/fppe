use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Sub, SubAssign};

use crate::Unit;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct Vec3 {
    pub x: Unit,
    pub y: Unit,
    pub z: Unit,
}

impl Vec3 {
    pub fn new<T: Into<Unit>>(x: T, y: T, z: T) -> Self {
        Self {
            x: x.into(),
            y: y.into(),
            z: z.into(),
        }
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> Unit {
        (self.x * rhs.x) + (self.y * rhs.y) + (self.z * rhs.z)
    }

    #[inline]
    pub fn length(self) -> Unit {
        cordic::sqrt(self.dot(self))
    }

    /// Computes the Euclidean distance between two points in space.
    #[inline]
    pub fn distance(self, rhs: Self) -> Unit {
        (self - rhs).length()
    }

    /// Computes `1.0 / length()`.
    ///
    /// For valid results, `self` must _not_ be of length zero.
    #[inline]
    pub fn length_recip(self) -> Unit {
        self.length().recip()
    }

    #[must_use]
    #[inline]
    pub fn normalize(self) -> Self {
        self.mul(self.length_recip())
    }

    /// Returns the vector projection of `self` onto `rhs`.
    ///
    /// `rhs` must be normalized.
    ///
    /// # Panics
    ///
    /// Will panic if `rhs` is not normalized when `glam_assert` is enabled.
    #[must_use]
    #[inline]
    pub fn project_onto_normalized(self, rhs: Self) -> Self {
        rhs * self.dot(rhs)
    }

    /// Returns the vector projection of `self` onto `rhs`.
    ///
    /// `rhs` must be of non-zero length.
    ///
    /// # Panics
    ///
    /// Will panic if `rhs` is zero length when `glam_assert` is enabled.
    #[must_use]
    #[inline]
    pub fn project_onto(self, rhs: Self) -> Self {
        // maybe use the implementation from glam?
        self.project_onto_normalized(rhs.normalize())
    }

    /// Computes the cross product of `self` and `rhs`.
    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - rhs.y * self.z,
            y: self.z * rhs.x - rhs.z * self.x,
            z: self.x * rhs.y - rhs.x * self.y,
        }
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x.add(other.x),
            y: self.y.add(other.y),
            z: self.z.add(other.z),
        }
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, other: Self) {
        self.x.add_assign(other.x);
        self.y.add_assign(other.y);
        self.z.add_assign(other.z);
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x.sub(other.x),
            y: self.y.sub(other.y),
            z: self.z.sub(other.z),
        }
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, other: Self) {
        self.x.sub_assign(other.x);
        self.y.sub_assign(other.y);
        self.z.sub_assign(other.z);
    }
}

// Add, Sub for Vec3 and Unit

impl Add<Unit> for Vec3 {
    type Output = Self;

    fn add(self, rhs: Unit) -> Self {
        Self {
            x: self.x.add(rhs),
            y: self.y.add(rhs),
            z: self.z.add(rhs),
        }
    }
}

// impl AddAssign<Unit> for Vec3 {
//     fn add_assign(&mut self, rhs: Unit) {
//         self.x.add_assign(rhs);
//         self.y.add_assign(rhs);
//         self.z.add_assign(rhs);
//     }
// }

impl Sub<Unit> for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Unit) -> Self {
        Self {
            x: self.x.sub(rhs),
            y: self.y.sub(rhs),
            z: self.z.sub(rhs),
        }
    }
}

// impl SubAssign<Unit> for Vec3 {
//     fn sub_assign(&mut self, rhs: Unit) {
//         self.x.sub_assign(rhs);
//         self.y.sub_assign(rhs);
//         self.z.sub_assign(rhs);
//     }
// }

impl Mul<Unit> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Unit) -> Self {
        Self {
            x: self.x.mul(rhs),
            y: self.y.mul(rhs),
            z: self.z.mul(rhs),
        }
    }
}

impl MulAssign<Unit> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, rhs: Unit) {
        self.x.mul_assign(rhs);
        self.y.mul_assign(rhs);
        self.z.mul_assign(rhs);
    }
}

impl Div<Unit> for Vec3 {
    type Output = Self;

    #[inline]
    fn div(self, rhs: Unit) -> Self {
        Self {
            x: self.x.div(rhs),
            y: self.y.div(rhs),
            z: self.z.div(rhs),
        }
    }
}
