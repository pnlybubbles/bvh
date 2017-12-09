use intersection::Intersection;
use ray::Ray;
use aabb::AABB;

pub trait SurfaceShape: Shape {
  fn area(&self) -> f32;
}

pub trait Shape {
  fn intersect(&self, ray: &Ray) -> Option<Intersection>;
  fn aabb(&self) -> AABB;
}
