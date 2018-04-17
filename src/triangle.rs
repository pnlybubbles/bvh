extern crate test;
extern crate rand;

use intersection::Intersection;
use shape::*;
use constant::*;
use ray::Ray;
use math::vector::Vector3;
use math::vector::*;
use aabb::AABB;

pub struct Triangle {
  pub p0: Vector3,
  pub p1: Vector3,
  pub p2: Vector3,
  aabb: AABB,
  normal: Vector3,
}

impl Triangle {
  pub fn new(
    p0: Vector3,
    p1: Vector3,
    p2: Vector3,
  ) -> Triangle {
    Triangle {
      p0: p0,
      p1: p1,
      p2: p2,
      aabb: Self::aabb(p0, p1, p2),
      normal: Self::normal(p0, p1, p2),
    }
  }

  fn aabb(p0: Vector3, p1: Vector3, p2: Vector3) -> AABB {
    let min = Vector3::new(
      p0.x.min(p1.x).min(p2.x),
      p0.y.min(p1.y).min(p2.y),
      p0.z.min(p1.z).min(p2.z),
    );
    let max = Vector3::new(
      p0.x.max(p1.x).max(p2.x),
      p0.y.max(p1.y).max(p2.y),
      p0.z.max(p1.z).max(p2.z),
    );
    AABB {
      min: min,
      max: max,
      center: (max + min) / 2.0,
    }
  }

  fn normal(p0: Vector3, p1: Vector3, p2: Vector3) -> Vector3 {
    (p1 - p0).cross(p2 - p0).normalize()
  }
}

impl Shape for Triangle {
  fn aabb(&self) -> &AABB {
    &self.aabb
  }

  fn intersect(&self, ray: &Ray) -> Option<Intersection> {
    // Möller–Trumbore intersection algorithm
    let e1 = self.p1 - self.p0;
    let e2 = self.p2 - self.p0;
    let pv = ray.direction.cross(e2);
    let det = e1.dot(pv); // クラメルの分母
    if det.abs() < EPS {
      return None;
    }
    let invdet = 1.0 / det;
    let tv = ray.origin - self.p0;
    let u = tv.dot(pv) * invdet;
    if u < 0.0 || u > 1.0 {
      return None;
    }
    let qv = tv.cross(e1);
    let v = ray.direction.dot(qv) * invdet;
    if v < 0.0 || u + v > 1.0 {
      return None;
    }
    let t = e2.dot(qv) * invdet;
    if t < EPS {
      return None;
    }
    let p = ray.origin + ray.direction * t;
    Some(Intersection {
      distance: t,
      normal: self.normal,
      position: p,
    })
  }
}
