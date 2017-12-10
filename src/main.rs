#![feature(box_syntax)]
#![feature(test)]
#![allow(dead_code)]

extern crate tobj;
extern crate rand;
extern crate test;
extern crate time;

mod math;
mod shape;
mod sphere;
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
use std::sync::Arc;
use aabb::AABB;
// use constant::EPS;

fn main() {
  let split: usize = 100;
  // let objects = obj(&Path::new("models/happy_vrip/buddha.obj"));
  let path = Path::new("models/happy_vrip/buddha.obj");
  let objects = obj(&path);
  // let objects_ = obj(&path);
  let bvh = BVH::new(objects);
  println!("min {}", bvh.aabb().min);
  println!("max {}", bvh.aabb().max);
  println!("center {}", bvh.aabb().center);
  let pre = (0..split + 1).flat_map( |i|
    (0..split + 1).map( |j|
      get_ray_in_aabb(i, j, split, bvh.aabb())
    ).collect::<Vec<_>>()
  ).collect::<Vec<_>>();
  let intersect_count = pre.iter().flat_map( |ray| bvh.intersect(&ray) ).count();
  println!("intersect count {}", intersect_count);
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
  let test_ray = get_ray_in_aabb(76, 48, split, bvh.aabb());
  let test_intersect = bvh.intersect(&test_ray);
  println!("test ray origin: {} direction: {}", test_ray.origin, test_ray.direction);
  println!("test intersect coordinate {}", test_intersect.unwrap().position);
  let results = (0..100).map( |i| {
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

fn get_ray_in_aabb(i: usize, j: usize, split: usize, aabb: AABB) -> Ray {
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

fn brute_force(objects: &Vec<Arc<SurfaceShape + Sync + Send>>, ray: &Ray) -> Option<Intersection> {
  objects.iter().flat_map(|v| v.intersect(&ray)).min_by(
    |a, b| {
      a.distance.partial_cmp(&b.distance).unwrap()
    },
  )
}

fn obj(path: &Path) -> Vec<Arc<SurfaceShape + Sync + Send>> {
  let (models, _) = tobj::load_obj(&path).unwrap();
  let mut instances: Vec<Arc<SurfaceShape + Sync + Send>> = Vec::with_capacity(
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
      instances.push(Arc::new(Triangle::new(polygon[0], polygon[1], polygon[2])));
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

  #[test]
  fn correct() {
    let objects = obj(&Path::new("models/monkey/monkey.obj"));
    let objects_ = obj(&Path::new("models/monkey/monkey.obj"));
    let bvh = BVH::new(objects);
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
    let objects = obj(&Path::new("models/sponza/sponza.obj"));
    let bvh = BVH::new(objects);
      let mut rng = rand::XorShiftRng::new_unseeded();
    let random_rays = random_ray_in_aabb(&bvh.aabb(), 10000, &mut rng);
    b.iter( || {
      for ray in &random_rays {
        bvh.intersect(&ray);
      }
    });
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
