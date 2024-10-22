use std::{array::from_fn, fmt::Display, ops::Shr};

use num::{cast, Integer, NumCast, Unsigned as NumUnsigned};

pub trait Unsigned = Integer + NumUnsigned + NumCast + Shr<Self, Output = Self> + Copy + Display;

#[derive(Default, Debug, PartialEq, Clone, Copy)]
pub struct UVec3<U: Unsigned> {
    pub x: U,
    pub y: U,
    pub z: U,
}

impl<U: Unsigned> Display for UVec3<U> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Uvec3: x: {}, y: {}, z: {}", self.x, self.y, self.z)
    }
}

impl<U: Unsigned> UVec3<U> {
    pub fn new(x: U, y: U, z: U) -> Self {
        UVec3 { x, y, z }
    }

    pub fn splat(size: U) -> Self {
        UVec3 {
            x: size,
            y: size,
            z: size,
        }
    }

    pub fn zero() -> Self {
        UVec3 {
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
}

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

#[derive(Default, Clone, Copy, PartialEq)]
pub struct Aabb<U: Unsigned> {
    pub min: UVec3<U>,
    pub max: UVec3<U>,
}

impl<U: Unsigned> Aabb<U> {
    pub fn new(center: UVec3<U>, half_size: U) -> Self {
        Aabb {
            min: UVec3::new(
                center.x - half_size,
                center.y - half_size,
                center.z - half_size,
            ),
            max: UVec3::new(
                center.x + half_size,
                center.y + half_size,
                center.z + half_size,
            ),
        }
    }

    pub fn center(&self) -> UVec3<U> {
        UVec3::new(
            (self.min.x + self.max.x) >> cast(1).unwrap(),
            (self.min.y + self.max.y) >> cast(1).unwrap(),
            (self.min.z + self.max.z) >> cast(1).unwrap(),
        )
    }

    pub fn split(&self) -> [Aabb<U>; 8] {
        let center = self.center();
        from_fn(|i| self._split(i, center))
    }

    fn _split(&self, i: usize, center: UVec3<U>) -> Aabb<U> {
        let x_mask = (i & 0b1) != 0;
        let y_mask = (i & 0b10) != 0;
        let z_mask = (i & 0b100) != 0;

        Aabb {
            min: UVec3::new(
                if x_mask { center.x } else { self.min.x },
                if y_mask { center.y } else { self.min.y },
                if z_mask { center.z } else { self.min.z },
            ),
            max: UVec3::new(
                if x_mask { self.max.x } else { center.x },
                if y_mask { self.max.y } else { center.y },
                if z_mask { self.max.z } else { center.z },
            ),
        }
    }

    pub fn contains(&self, position: UVec3<U>) -> bool {
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

#[cfg(test)]
mod tests {
    use super::{Aabb, UVec3};

    #[test]
    fn test_aabb_contains() {
        let aabb = Aabb::new(UVec3::new(8, 8, 8), 8u16);
        assert!(aabb.contains(UVec3::zero()));

        assert!(aabb.contains(UVec3::new(8, 8, 8)));

        assert!(!aabb.contains(UVec3::new(16, 16, 16)));

        assert!(!aabb.contains(UVec3::new(0, 16, 8)));
    }
}
