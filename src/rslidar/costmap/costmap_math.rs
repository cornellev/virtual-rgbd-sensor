// rewriting https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/src/costmap_math.cpp in rust

#![allow(dead_code)]

use std::ops::Add;
use std::ops::Sub;

struct Point {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32
}

impl Add for Point {
    type Output = Point;
    fn add(self, rhs : Point) -> Point {
        Point {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            intensity: self.intensity + rhs.intensity
        }
    }
}

impl Sub for Point {
    type Output = Point;
    fn sub(self, rhs : Point) -> Point {
        Point {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            intensity: self.intensity - rhs.intensity
        }
    }
}


fn distance_to_line(p: Point, p_0: Point, p_1 : Point) -> f32 {
    let a = p.x - p_0.x;
    let b = p.y - p_0.y;
    let c = p_1.x - p_0.x;
    let d = p_1.y - p_0.y;
    
    let dot = a * c + b * d;
    let len_sq = c * c + d * d;
    let param = dot / len_sq;

    let mut xx = 0.0;
    let mut yy = 0.0;

    if param < 0.0 {
        xx = p_0.x;
        yy = p_0.y;
    } else if param > 1.0 {
        xx = p_1.x;
        yy = p_1.y;
    } else {
        xx = p_0.x + param * c;
        yy = p_0.y + param * d;
    }

    ((p.x - xx) * (p.x - xx) + (p.y - yy) * (p.y - yy)).sqrt()
}

trait Intersects {
    fn intersects(&self, polygon: &[Point]) -> bool;
}

// pnpoly ray casting: point (x, y) inside polygon
impl Intersects for (f32, f32) {
    fn intersects(&self, polygon: &[Point]) -> bool {
        let (testx, testy) = *self;
        let n = polygon.len();
        if n == 0 {
            return false;
        }

        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let (xi, yi) = (polygon[i].x, polygon[i].y);
            let (xj, yj) = (polygon[j].x, polygon[j].y);

            if (yi > testy) != (yj > testy)
                && testx < (xj - xi) * (testy - yi) / (yj - yi) + xi
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }
}

// polygon vs polygon: true if any vertex of either lies inside the other
impl Intersects for &[Point] {
    fn intersects(&self, polygon: &[Point]) -> bool {
        fn helper(polygon1: &[Point], polygon2: &[Point]) -> bool {
            polygon1.iter().any(|p| (p.x, p.y).intersects(polygon2))
        }
        helper(self, polygon) || helper(polygon, self)
    }
}

fn intersects<T: Intersects>(polygon: &[Point], other: T) -> bool {
    other.intersects(polygon)
}
