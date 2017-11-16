#![feature(box_syntax)]

mod vector;
mod shape;
mod sphere;
mod ray;
mod intersection;
mod aabb;
mod constant;
mod bvh;

use vector::*;
use sphere::Sphere;
use bvh::BVH;
use shape::Shape;
use ray::Ray;

fn main() {
  let objects: Vec<Box<Shape>> = vec![
    box Sphere { radius: 1.0, position: Vector::new(0.0, 0.0, 0.0) },
    box Sphere { radius: 1.0, position: Vector::new(2.0, 1.0, -3.0) },
    box Sphere { radius: 1.0, position: Vector::new(-1.0, 2.0, 1.0) },
    box Sphere { radius: 1.0, position: Vector::new(2.0, 3.0, -1.0) },
    box Sphere { radius: 1.0, position: Vector::new(1.0, 0.0, -1.0) },
    box Sphere { radius: 1.0, position: Vector::new(4.0, -2.0, 0.0) },
    box Sphere { radius: 1.0, position: Vector::new(-2.0, -3.0, 2.0) },
    box Sphere { radius: 1.0, position: Vector::new(1.0, -4.0, -2.0) },
    box Sphere { radius: 1.0, position: Vector::new(0.0, -2.0, -1.0) },
    box Sphere { radius: 1.0, position: Vector::new(2.0, -2.0, 1.0) },
  ];
  let ray = Ray {
    origin: Vector::new(-7.0, -5.0, -7.0),
    direction: Vector::new(1.0, 0.6, 1.0).normalize(),
  };
  {
    let maybe_intersect = objects.iter().flat_map(|v| v.intersect(&ray)).min_by(
      |a, b| {
        a.distance.partial_cmp(&b.distance).unwrap()
      },
    );
    maybe_intersect.map( |v| {
      println!("BruteForce {:?}", v.position);
    });
  }
  {
    let bvh = BVH::new(objects);
    let maybe_intersect = bvh.intersect(&ray);
    maybe_intersect.map( |v| {
      println!("BVH        {:?}", v.position)
    });
  }
}
