extern crate ordered_float;

use math::vector::Vector3;
use ray::Ray;
use self::ordered_float::OrderedFloat;
use constant::*;

pub struct AABB {
  pub min: Vector3,
  pub max: Vector3,
  pub center: Vector3,
}

impl AABB {
  pub fn side(&self) -> Vector3 {
    Vector3::new(
      (self.max.x - self.min.x).abs(),
      (self.max.y - self.min.y).abs(),
      (self.max.z - self.min.z).abs(),
    )
  }

  pub fn merge(list: &Vec<&AABB>) -> AABB {
    let min = Vector3::new(
      *list.iter().map(|v| OrderedFloat(v.min.x)).min().unwrap(),
      *list.iter().map(|v| OrderedFloat(v.min.y)).min().unwrap(),
      *list.iter().map(|v| OrderedFloat(v.min.z)).min().unwrap(),
    );
    let max = Vector3::new(
      *list.iter().map(|v| OrderedFloat(v.max.x)).max().unwrap(),
      *list.iter().map(|v| OrderedFloat(v.max.y)).max().unwrap(),
      *list.iter().map(|v| OrderedFloat(v.max.z)).max().unwrap(),
    );
    AABB {
      min: min,
      max: max,
      center: (min + max) / 2.0,
    }
  }

  #[inline]
  pub fn is_intersect(&self, ray: &Ray) -> bool {
    let mut min = -INF;
    let mut max = INF;
    for i in 0..3 {
      let inv_d = 1.0 / ray.direction[i];
      let t1 = (self.min[i] - ray.origin[i]) * inv_d;
      let t2 = (self.max[i] - ray.origin[i]) * inv_d;
      let (t_min, t_max) = if t1 > t2 { (t2, t1) } else { (t1, t2) };
      if min < t_min {
        min = t_min
      }
      if max > t_max {
        max = t_max
      }
      if min > max { return false }
    }
    true
  }
}
