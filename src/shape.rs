use intersection::Intersection;
use ray::Ray;
use aabb::AABB;

pub trait Shape {
  fn intersect(&self, &Ray) -> Option<Intersection>;
  fn aabb(&self) -> &AABB;
}
