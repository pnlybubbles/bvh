use vector::Vector;
use ray::Ray;
use constant::*;

#[derive(Clone)]
pub struct AABB {
  pub min: Vector,
  pub max: Vector,
  pub center: Vector,
}

impl AABB {
  pub fn side(&self) -> Vector {
    Vector::from_index( |i|
      (self.max.to_array()[i] - self.min.to_array()[i]).abs()
    )
  }

  pub fn is_intersect(&self, ray: &Ray) -> bool {
    let origin = ray.origin.to_array();
    let direction = ray.direction.to_array();
    let aabb_min = self.min.to_array();
    let aabb_max = self.max.to_array();
    let mut min = -INF;
    let mut max = INF;
    for i in 0..3 {
      let t2 = (aabb_max[i] - origin[i]) / direction[i];
      if t2 < EPS { return false }
      let t1 = (aabb_min[i] - origin[i]) / direction[i];
      min = min.max(t1.min(t2));
      max = max.min(t1.max(t2));
      if min > max { return false }
    }
    true
  }
}
