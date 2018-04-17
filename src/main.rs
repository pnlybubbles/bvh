#![feature(box_syntax)]
#![feature(test)]
#![allow(dead_code)]
#![feature(sort_unstable)]

extern crate tobj;
extern crate rand;
extern crate test;
extern crate time;
extern crate bvh as svenstaro_bvh;

mod math;
mod shape;
mod triangle;
mod ray;
mod intersection;
mod aabb;
mod constant;
mod bvh;

use math::vector::*;
use triangle::Triangle;
use intersection::Intersection;
use bvh::BVH;
use shape::*;
use ray::Ray;
use std::path::Path;
use aabb::AABB;
// use constant::EPS;

fn main() {
  let split: usize = 100;
  // let mut objects = obj(&Path::new("models/happy_vrip/buddha.obj"));
  let objects = obj(&Path::new("models/happy_vrip/buddha.obj"));
  // let objects = obj(&Path::new("models/monkey/monkey.obj"));
  println!("obj loaded");
  // let objects_ = obj(&path);
  let start_time = time::now();
  let bvh = BVH::new(&objects);
  let end_time = time::now();
  let elapse_time = (end_time - start_time).num_microseconds().unwrap();
  println!("construction: {}us", elapse_time);
  println!("min {}", bvh.aabb().min);
  println!("max {}", bvh.aabb().max);
  println!("center {}", bvh.aabb().center);
  let pre = (0..split).flat_map( |i|
    (0..split).map( |j|
      get_ray_in_aabb(i, j, split - 1, bvh.aabb())
    ).collect::<Vec<_>>()
  ).collect::<Vec<_>>();
  // let intersect_count = pre.iter().flat_map( |ray| bvh.intersect(&ray) ).count();
  // println!("intersect count {}", intersect_count);
  // for ray in &pre {
  //   let bvh_i = bvh.intersect(&ray);
  //   let bf_i = brute_force(&objects_, &ray);
  //   if bvh_i.is_some() != bf_i.is_some() {
  //       panic!("Fail: no intersect")
  //   }
  //   if bvh_i.is_some() && bf_i.is_some() {
  //     if (bvh_i.unwrap().position - bf_i.unwrap().position).norm() > EPS {
  //       panic!("Fail: wrong position")
  //     }
  //   }
  // }
  // for ray in &pre {
  //   bvh.intersect(&ray);
  // }
  // panic!();
  let test_ray = get_ray_in_aabb(76, 48, split - 1, bvh.aabb());
  let test_intersect = bvh.intersect(&test_ray);
  println!("test ray origin: {} direction: {}", test_ray.origin, test_ray.direction);
  println!("test intersect coordinate {}", test_intersect.unwrap().position);
  let results = (0..10).map( |i| {
    let start_time = time::now();
    for ray in &pre {
      bvh.intersect(&ray);
    }
    let end_time = time::now();
    let elapse_time = (end_time - start_time).num_microseconds().unwrap();
    println!(
      "try {} | elapse: {}us",
      i,
      elapse_time, 
    );
    elapse_time as f64
  }).collect::<Vec<_>>();
  let n = results.len() as f64;
  let ave = results.iter().sum::<f64>() / n;
  println!("sample size: {}", n);
  println!("average: {}", ave);
  let sigma = (results.iter().map( |v| (v - ave).powi(2) ).sum::<f64>() / (n * (n - 1.0))).sqrt();
  println!("standard error: {}", sigma);
}

fn get_ray_in_aabb(i: usize, j: usize, split: usize, aabb: &AABB) -> Ray {
  let x = aabb.min.x + (aabb.max.x - aabb.min.x) * (i as f32 / split as f32);
  let y = aabb.min.y + (aabb.max.y - aabb.min.y) * (j as f32 / split as f32);
  let z = aabb.min.z;
  let origin = Vector3::new(x, y, z);
  let direction = (aabb.center - origin).normalize();
  Ray {
    origin: origin,
    direction: direction,
  }
}

fn brute_force(objects: &Vec<Box<Shape>>, ray: &Ray) -> Option<Intersection> {
  objects.iter().flat_map(|v| v.intersect(&ray)).min_by(
    |a, b| {
      a.distance.partial_cmp(&b.distance).unwrap()
    },
  )
}

fn obj(path: &Path) -> Vec<Box<Shape>> {
  let (models, _) = tobj::load_obj(&path).unwrap();
  let mut instances: Vec<Box<Shape>> = Vec::with_capacity(
    models.iter().map( |m| m.mesh.indices.len() / 3).sum()
  );
  for m in models {
    let mesh = &m.mesh;
    // println!("{}: {} ploygon", m.name, mesh.indices.len() / 3);
    for f in 0..mesh.indices.len() / 3 {
      let mut polygon = [Vector3::zero(); 3];
      for i in 0..3 {
        let index: usize = f * 3 + i;
        polygon[i] = Vector3::new(
          mesh.positions[mesh.indices[index] as usize * 3] as f32 * 100.0,
          mesh.positions[mesh.indices[index] as usize * 3 + 1] as f32 * 100.0,
          mesh.positions[mesh.indices[index] as usize * 3 + 2] as f32 * 100.0,
        );
      }
      instances.push(box Triangle::new(polygon[0], polygon[1], polygon[2]));
    }
  }
  instances
}

#[cfg(test)]
mod tests {
  use super::*;
  use test::Bencher;
  use rand::Rng;
  use aabb::AABB;
  use constant::*;
  use svenstaro_bvh::aabb::Bounded;

  #[test]
  fn correct() {
    let objects = obj(&Path::new("models/monkey/monkey.obj"));
    let objects_ = obj(&Path::new("models/monkey/monkey.obj"));
    let bvh = BVH::new(&objects);
    let mut rng = rand::XorShiftRng::new_unseeded();
    for _ in 0..10000 {
      let origin = Vector3::new(
        rng.gen_range(-10.0f32, 10.0),
        rng.gen_range(-10.0f32, 10.0),
        rng.gen_range(-10.0f32, 10.0),
      );
      let direction = Vector3::new(
        rng.gen_range(-1.0f32, 1.0),
        rng.gen_range(-1.0f32, 1.0),
        rng.gen_range(-1.0f32, 1.0),
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

  fn random_ray_in_aabb<R>(aabb: &AABB, count: usize, mut rng: R) -> Vec<Ray>
    where
      R: Rng,
  {
    let diagnal = (aabb.max - aabb.min).norm();
    (0..count).map( |_| {
      let from: Vector3 = (0..3).map( |i|
        (aabb.max[i] - aabb.min[i]) * rng.gen_range(-1.0f32, 1.0)
      ).collect::<Vec<_>>().into();
      let to: Vector3 = (0..3).map( |i|
        (aabb.max[i] - aabb.min[i]) * rng.gen_range(-1.0f32, 1.0)
      ).collect::<Vec<_>>().into();
      let direction = (to - from).normalize();
      let origin = from - direction * diagnal;
      Ray {
        origin: origin,
        direction: direction,
      }
    }).collect()
  }

  // #[bench]
  // fn bench_construct_bvh(b: &mut Bencher) {
  //   println!("");
  //   let objects = obj(&Path::new("models/bunny/bunny.obj"));
  //   b.iter( || {
  //     BVH::new(&objects);
  //   })
  // }

  #[bench]
  fn bench_intersection_bvh(b: &mut Bencher) {
    println!("");
    let objects = obj(&Path::new("models/sponza/sponza.obj"));
    let bvh = BVH::new(&objects);
    let mut rng = rand::XorShiftRng::new_unseeded();
    let random_rays = random_ray_in_aabb(&bvh.aabb(), 10000, &mut rng);
    b.iter( || {
      for ray in &random_rays {
        bvh.intersect(&ray);
      }
    });
  }

  struct SvenstaroTriangle<'a> {
    index: usize,
    triangle: &'a Box<Shape>,
  }

  impl<'a> svenstaro_bvh::aabb::Bounded for SvenstaroTriangle<'a> {
    fn aabb(&self) -> svenstaro_bvh::aabb::AABB {
      let aabb = self.triangle.aabb();
      let min = convert_nalgebra_point(aabb.min);
      let max = convert_nalgebra_point(aabb.max);
      svenstaro_bvh::aabb::AABB::with_bounds(min, max)
    }
  }

  impl<'a> svenstaro_bvh::bounding_hierarchy::BHShape for SvenstaroTriangle<'a> {
    fn set_bh_node_index(&mut self, index: usize) {
      self.index = index;
    }

    fn bh_node_index(&self) -> usize {
      self.index
    }
  }

  fn convert_nalgebra_vector(v: Vector3) -> svenstaro_bvh::nalgebra::Vector3<f32> {
    svenstaro_bvh::nalgebra::Vector3::new(v.x, v.y, v.z)
  }

  fn convert_nalgebra_point(v: Vector3) -> svenstaro_bvh::nalgebra::Point3<f32> {
    svenstaro_bvh::nalgebra::Point3::new(v.x, v.y, v.z)
  }

  fn convert_point(v: svenstaro_bvh::nalgebra::Point3<f32>) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
  }

  #[bench]
  fn bench_intersection_svenstaro_bvh(b: &mut Bencher) {
    println!("");
    let obj = obj(&Path::new("models/sponza/sponza.obj"));
    let mut objects = obj.iter().enumerate().map( |(i, v)|
      SvenstaroTriangle { index: i, triangle: v }
    ).collect::<Vec<_>>();
    let mut bounds = svenstaro_bvh::aabb::AABB::empty();
    for triangle in &objects {
      bounds.join_mut(&triangle.aabb());
    }
    let bvh = svenstaro_bvh::bvh::BVH::build(objects.as_mut_slice());
    let mut rng = rand::XorShiftRng::new_unseeded();
    let min = convert_point(bounds.min);
    let max = convert_point(bounds.max);
    let aabb = AABB {
      min: min,
      max: max,
      center: min + max / 2.0,
    };
    let rays = random_ray_in_aabb(&aabb, 10000, &mut rng);
    let random_rays = rays.iter().map( |ray| {
      let origin = convert_nalgebra_point(ray.origin);
      let direction = convert_nalgebra_vector(ray.direction);
      (svenstaro_bvh::ray::Ray::new(origin, direction), ray)
    }).collect::<Vec<_>>();
    b.iter( || {
      for &(ref ray, org_ray) in &random_rays {
        let shapes = bvh.traverse(&ray, &objects);
        shapes.iter().flat_map(|v| v.triangle.intersect(&org_ray)).min_by(
          |a, b| {
            a.distance.partial_cmp(&b.distance).unwrap()
          },
        );
      }
    });
  }

  // #[bench]
  // fn bench_intersection_brute_force(b: &mut Bencher) {
  //   println!("");
  //   let objects = obj(&Path::new("models/monkey/monkey.obj"));
  //   let aabb_list = objects.iter().map( |v| v.aabb() ).collect::<Vec<_>>();
  //   let aabb = AABB::merge(&aabb_list.iter().collect());
  //   b.iter( || {
  //     let mut rng = rand::XorShiftRng::new_unseeded();
  //     random_ray_in_aabb(&aabb, 1000, &mut rng, |ray| {
  //       brute_force(&objects, &ray);
  //     });
  //   })
  // }
}
