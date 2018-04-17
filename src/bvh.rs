extern crate ordered_float;

use aabb::AABB;
use shape::*;
use ray::Ray;
use intersection::Intersection;
use self::ordered_float::OrderedFloat;

#[derive(Clone)]
struct Leaf {
  aabb: AABB,
  index: usize,
}

trait Branch {
  fn may_intersect(&self, &Ray, &mut Vec<usize>);
  fn aabb(&self) -> &AABB;
}

impl Branch for Leaf {
  fn may_intersect(&self, ray: &Ray, candidate: &mut Vec<usize>) {
    if self.aabb.is_intersect(ray) {
      candidate.push(self.index)
    }
  }

  fn aabb(&self) -> &AABB {
    &self.aabb
  }
}

struct Node {
  aabb: AABB,
  left: Box<Branch>,
  right: Box<Branch>,
}

impl Branch for Node {
  fn may_intersect(&self, ray: &Ray, mut candidate: &mut Vec<usize>) {
    if self.aabb.is_intersect(ray) {
      self.left.may_intersect(ray, &mut candidate);
      self.right.may_intersect(ray, &mut candidate);
    }
  }

  fn aabb(&self) -> &AABB {
    &self.aabb
  }
}

pub struct BVH<'a> {
  list: &'a [Box<Shape>],
  root: Box<Branch>,
}

impl<'a> BVH<'a> {
  pub fn new(list: &'a [Box<Shape>]) -> BVH<'a> {
    // let mut indexes = (0..list.len()).enumerate().map( |(i, _)| i ).collect::<Vec<usize>>();
    let mut leaf = list.iter().enumerate().map( |(i, v)| Leaf {
      aabb: v.aabb().clone(),
      index: i,
    }).collect::<Vec<_>>();
    let root = Self::construct(&mut leaf);
    BVH {
      list: list,
      root: root, 
    }
  }

  fn construct(list: &mut [Leaf]) -> Box<Branch> {
    // セットアップ
    let len = list.len();
    // 要素が1つのときは葉
    if len == 1 {
      return box list[0].clone();
    }
    // 全体のAABBを作成
    let aabb = AABB::merge(&list.iter().map( |v| &v.aabb ).collect());
    // 最大のAABBの辺を基準にして分割する
    // 最大のAABBの辺のインデックスを求める
    let sides: [f32; 3] = aabb.side().into();
    let partition_axis = sides.iter()
      .enumerate().max_by_key( |&(_, v)| OrderedFloat(*v) )
      .map( |(i, _)| i ).unwrap_or(0);
    // 基準の軸でソート
    list.sort_unstable_by_key( |v| {
      OrderedFloat(v.aabb.center[partition_axis])
    });
    // 分割するインデックスを求める
    let partition_index = len / 2; // i-1とiの間で分割
    // 再帰的に子要素を生成
    let left = Self::construct(&mut list[0..partition_index]);
    let right = Self::construct(&mut list[partition_index..]);
    box Node {
      aabb: aabb,
      left: left,
      right: right,
    }
  }
}

impl<'a> Shape for BVH<'a> {
  fn intersect(&self, ray: &Ray) -> Option<Intersection> {
    let mut candidate = Vec::new();
    self.root.may_intersect(ray, &mut candidate);
    candidate.iter().flat_map( |&i| {
      self.list[i].intersect(&ray)
    }).min_by(
      |a, b| {
        a.distance.partial_cmp(&b.distance).unwrap()
      },
    )
  }

  fn aabb(&self) -> &AABB {
    self.root.aabb()
  }
}
