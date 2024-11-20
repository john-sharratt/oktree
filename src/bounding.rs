//! Bounding primitives.
//!
//! [`TUVec3`], [`BVec3`], [`Aabb`]

use std::{
    array::from_fn,
    fmt::{Debug, Display},
    ops::{BitAnd, Shr},
};

use num::{cast, Integer, NumCast, Unsigned as NumUnsigned};

use crate::TreeError;

pub trait Unsigned:
    Integer
    + NumUnsigned
    + NumCast
    + Shr<Self, Output = Self>
    + BitAnd<Self, Output = Self>
    + Copy
    + Display
    + Debug
    + Default
{
}
impl Unsigned for u8 {}
impl Unsigned for u16 {}
impl Unsigned for u32 {}
impl Unsigned for u64 {}
impl Unsigned for u128 {}
impl Unsigned for usize {}

/// Tree Unsigned Vec3
///
/// Inner typy shuld be any [`Unsigned`](num::Unsigned):
/// `u8`, `u16`, `u32`, `u64`, `u128`, `usize`.
#[derive(Default, Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Hash)]
pub struct TUVec3<U: Unsigned> {
    pub x: U,
    pub y: U,
    pub z: U,
}

impl<U: Unsigned> Display for TUVec3<U> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Uvec3: x: {}, y: {}, z: {}", self.x, self.y, self.z)
    }
}

impl<U: Unsigned> TUVec3<U> {
    pub fn new(x: U, y: U, z: U) -> Self {
        TUVec3 { x, y, z }
    }

    pub fn splat(size: U) -> Self {
        TUVec3 {
            x: size,
            y: size,
            z: size,
        }
    }

    pub fn zero() -> Self {
        TUVec3 {
            x: cast(0).unwrap(),
            y: cast(0).unwrap(),
            z: cast(0).unwrap(),
        }
    }

    pub fn lt(&self, other: Self) -> BVec3 {
        BVec3::new(self.x < other.y, self.y < other.y, self.z < other.z)
    }

    pub fn gt(&self, other: Self) -> BVec3 {
        BVec3::new(self.x > other.y, self.y > other.y, self.z > other.z)
    }

    pub fn le(&self, other: Self) -> BVec3 {
        BVec3::new(self.x <= other.y, self.y <= other.y, self.z <= other.z)
    }

    pub fn ge(&self, other: Self) -> BVec3 {
        BVec3::new(self.x >= other.y, self.y >= other.y, self.z >= other.z)
    }

    #[inline]
    /// Checks if [`Aabb`] creted from this [`TUVec3`] and `half_size` will have all dimensions positive.
    pub fn is_positive_aabb(&self, half_size: U) -> bool {
        self.x >= half_size && self.y >= half_size && self.z >= half_size
    }

    /// Creates [`Aabb`] with size of 1 from current [`TUVec3`].
    pub fn unit_aabb(&self) -> Aabb<U> {
        let max = TUVec3::new(
            self.x + cast(1).unwrap(),
            self.y + cast(1).unwrap(),
            self.z + cast(1).unwrap(),
        );
        Aabb { min: *self, max }
    }
}

/// Boolean Vec3 mask.
#[derive(Default, Clone, Copy, PartialEq)]
pub struct BVec3 {
    x: bool,
    y: bool,
    z: bool,
}

impl BVec3 {
    fn new(x: bool, y: bool, z: bool) -> Self {
        BVec3 { x, y, z }
    }

    pub fn all(&self) -> bool {
        self.x && self.y && self.z
    }

    pub fn any(&self) -> bool {
        self.x || self.y || self.z
    }

    pub fn none(&self) -> bool {
        !self.x && !self.y && !self.z
    }
}

/// Axis Aligned Bounding Box
///
/// Resulting Aabb should be positive and it's dimensions should be the power of 2.
/// Inner type shuld be any [`Unsigned`](num::Unsigned):
/// `u8`, `u16`, `u32`, `u64`, `u128`, `usize`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb<U: Unsigned> {
    pub min: TUVec3<U>,
    pub max: TUVec3<U>,
}

impl<U: Unsigned> Default for Aabb<U> {
    fn default() -> Self {
        Self {
            min: TUVec3::new(cast(0).unwrap(), cast(0).unwrap(), cast(0).unwrap()),
            max: TUVec3::new(cast(1).unwrap(), cast(1).unwrap(), cast(1).unwrap()),
        }
    }
}

impl<U: Unsigned> Aabb<U> {
    /// Creates a new [Aabb] object without any checks
    pub fn new_unchecked(center: TUVec3<U>, half_size: U) -> Self {
        Aabb {
            min: TUVec3::new(
                center.x - half_size,
                center.y - half_size,
                center.z - half_size,
            ),
            max: TUVec3::new(
                center.x + half_size,
                center.y + half_size,
                center.z + half_size,
            ),
        }
    }

    /// Creates a new [`Aabb`] object
    ///
    /// Checks that it's dimensions are positive
    /// and are powers of 2.
    pub fn new(center: TUVec3<U>, half_size: U) -> Result<Self, TreeError> {
        if !center.is_positive_aabb(half_size) {
            Err(TreeError::NotPositive(
                "Center: {center}, half size: {half_size}".into(),
            ))
        } else if !is_power2(half_size) {
            Err(TreeError::NotPower2("half size: {half_size}".into()))
        } else {
            Ok(Self::new_unchecked(center, half_size))
        }
    }

    pub fn center(&self) -> TUVec3<U> {
        TUVec3::new(
            (self.min.x + self.max.x) >> cast(1).unwrap(),
            (self.min.y + self.max.y) >> cast(1).unwrap(),
            (self.min.z + self.max.z) >> cast(1).unwrap(),
        )
    }

    pub fn split(&self) -> [Aabb<U>; 8] {
        let center = self.center();
        from_fn(|i| self._split(i, center))
    }

    fn _split(&self, i: usize, center: TUVec3<U>) -> Aabb<U> {
        let x_mask = (i & 0b1) != 0;
        let y_mask = (i & 0b10) != 0;
        let z_mask = (i & 0b100) != 0;

        Aabb {
            min: TUVec3::new(
                if x_mask { center.x } else { self.min.x },
                if y_mask { center.y } else { self.min.y },
                if z_mask { center.z } else { self.min.z },
            ),
            max: TUVec3::new(
                if x_mask { self.max.x } else { center.x },
                if y_mask { self.max.y } else { center.y },
                if z_mask { self.max.z } else { center.z },
            ),
        }
    }

    /// Checks if the aabb contains a [`position`](TUVec3).
    pub fn contains(&self, position: TUVec3<U>) -> bool {
        let lemin = self.min.le(position);
        let gtmax = self.max.gt(position);

        lemin.all() && gtmax.all()
    }

    pub fn unit(&self) -> bool {
        self.max.x - self.min.x == cast(1).unwrap()
    }

    pub fn size(&self) -> U {
        self.max.x - self.min.x
    }
}

/// Check if `half_size` is the power of 2.
///
/// Used in [`aabb creation`](Aabb::new) checks.
pub fn is_power2<U: Unsigned>(mut half_size: U) -> bool {
    if half_size < cast(2).unwrap() {
        return false;
    }

    while half_size > cast(1).unwrap() {
        if half_size & cast(0b1).unwrap() != cast(0).unwrap() {
            return false;
        }

        half_size = half_size >> cast(1).unwrap();
    }

    true
}

#[cfg(test)]
mod tests {
    use super::{is_power2, Aabb, TUVec3};

    #[test]
    fn test_aabb_contains() {
        let aabb = Aabb::new_unchecked(TUVec3::new(8, 8, 8), 8u16);
        assert!(aabb.contains(TUVec3::zero()));

        assert!(aabb.contains(TUVec3::new(8, 8, 8)));

        assert!(!aabb.contains(TUVec3::new(16, 16, 16)));

        assert!(!aabb.contains(TUVec3::new(0, 16, 8)));
    }

    #[test]
    fn test_ispower2() {
        assert!(!is_power2(0u32));
        assert!(!is_power2(1u32));
        assert!(is_power2(2u8));
        assert!(!is_power2(3u32));
        assert!(is_power2(4u16));
        assert!(!is_power2(5u16));
        assert!(is_power2(8u8));
        assert!(!is_power2(1023usize));
        assert!(is_power2(1024usize));
        assert!(!is_power2(1025usize));
    }

    #[test]
    fn test_aabb_constructor() {
        // Ok
        assert!(Aabb::new(TUVec3::splat(2u8), 2).is_ok());

        // Negative dimensions
        assert!(Aabb::new(TUVec3::splat(16u16), 64).is_err());

        // 7 is not the power of 2
        assert!(Aabb::new(TUVec3::splat(16u16), 7).is_err());
    }
}
