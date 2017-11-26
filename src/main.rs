#![feature(box_syntax)]
#![feature(test)]
#![allow(dead_code)]

extern crate tobj;
extern crate rand;
extern crate test;

mod vector;
mod shape;
mod sphere;
mod triangle;
mod ray;
mod intersection;
mod aabb;
mod constant;
mod bvh;

use vector::*;
use triangle::Triangle;
use intersection::Intersection;
use bvh::BVH;
use shape::Shape;
use ray::Ray;
use std::path::Path;
use std::rc::Rc;

fn main() {
  let objects = obj(&Path::new("models/simple/debug.obj"));
  BVH::new(objects);
}

fn brute_force(objects: &Vec<Rc<Shape>>, ray: &Ray) -> Option<Intersection> {
  objects.iter().flat_map(|v| v.intersect(&ray)).min_by(
    |a, b| {
      a.distance.partial_cmp(&b.distance).unwrap()
    },
  )
}

fn obj(path: &Path) -> Vec<Rc<Shape>> {
  let mut objects: Vec<Rc<Shape>> = Vec::new();
  let obj = tobj::load_obj(&path);
  assert!(obj.is_ok());
  let (models, _) = obj.unwrap();
  for m in models {
    let mesh = &m.mesh;
    println!("{}: {} ploygon", m.name, mesh.indices.len() / 3);
    for f in 0..mesh.indices.len() / 3 {
      let mut polygon = [Vector::zero(); 3];
      for i in 0..3 {
        let index: usize = f * 3 + i;
        polygon[i] = Vector::new(
          mesh.positions[mesh.indices[index] as usize * 3] as f64,
          mesh.positions[mesh.indices[index] as usize * 3 + 1] as f64,
          mesh.positions[mesh.indices[index] as usize * 3 + 2] as f64,
        );
      }
      objects.push(Rc::new(Triangle::new(polygon[0], polygon[1], polygon[2])));
    }
  }
  objects
}

#[cfg(test)]
mod tests {
  use super::*;
  use test::Bencher;
  use rand::Rng;
  use aabb::AABB;
  use constant::*;

  #[test]
  fn correct() {
    let objects = obj(&Path::new("models/monkey/monkey.obj"));
    let objects_ = obj(&Path::new("models/monkey/monkey.obj"));
    let bvh = BVH::new(objects);
    let mut rng = rand::XorShiftRng::new_unseeded();
    for _ in 0..10000 {
      let origin = Vector::new(
        rng.gen_range(-10.0f64, 10.0),
        rng.gen_range(-10.0f64, 10.0),
        rng.gen_range(-10.0f64, 10.0),
      );
      let direction = Vector::new(
        rng.gen_range(-1.0f64, 1.0),
        rng.gen_range(-1.0f64, 1.0),
        rng.gen_range(-1.0f64, 1.0),
      );
      let ray = Ray {
        origin: origin,
        direction: direction.normalize(),
      };
      let i1 = brute_force(&objects_, &ray);
      let i2 = bvh.intersect(&ray);
      if i1.is_some() != i2.is_some() {
        println!("origin {:?}", origin);
        println!("direction {:?}", direction);
        i1.map( |v| {
          println!("correct {:?}", v.position)
        });
        i2.map( |v| {
          println!("test {:?}", v.position)
        });
        assert!(false);
      } else {
        i1.map( |v| {
          assert!((v.position - i2.unwrap().position).norm() < EPS);
        });
      }
    }
  }

  fn random_ray_in_aabb<F, R>(aabb: &AABB, count: usize, mut rng: R, f: F)
    where
      R: Rng,
      F: Fn(&Ray),
  {
    let min = aabb.min.to_array();
    let max = aabb.max.to_array();
    let diagnal = (aabb.max - aabb.min).norm();
    for _ in 0..count {
      let from = Vector::from_index( |i|
        (max[i] - min[i]) * rng.gen_range(-1.0f64, 1.0)
      );
      let to = Vector::from_index( |i|
        (max[i] - min[i]) * rng.gen_range(-1.0f64, 1.0)
      );
      let direction = (to - from).normalize();
      let origin = from - direction * diagnal;
      let ray = Ray {
        origin: origin,
        direction: direction,
      };
      f(&ray)
    }
  }

  #[bench]
  fn bench_construct_bvh(b: &mut Bencher) {
    println!("");
    let objects = obj(&Path::new("models/bunny/bunny.obj"));
    b.iter( || {
      BVH::new(objects.iter().cloned().collect());
    })
  }

  #[bench]
  fn bench_intersection_bvh(b: &mut Bencher) {
    println!("");
    let objects = obj(&Path::new("models/monkey/monkey.obj"));
    let bvh = BVH::new(objects);
    b.iter( || {
      let mut rng = rand::XorShiftRng::new_unseeded();
      random_ray_in_aabb(&bvh.aabb(), 1000, &mut rng, |ray| {
        bvh.intersect(&ray);
      });
    })
  }

  #[bench]
  fn bench_intersection_brute_force(b: &mut Bencher) {
    println!("");
    let objects = obj(&Path::new("models/monkey/monkey.obj"));
    let aabb_list = objects.iter().map( |v| v.aabb() ).collect::<Vec<_>>();
    let aabb = AABB::merge(&aabb_list.iter().collect());
    b.iter( || {
      let mut rng = rand::XorShiftRng::new_unseeded();
      random_ray_in_aabb(&aabb, 1000, &mut rng, |ray| {
        brute_force(&objects, &ray);
      });
    })
  }
}
