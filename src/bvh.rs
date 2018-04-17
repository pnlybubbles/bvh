extern crate ordered_float;

use aabb::AABB;
use shape::*;
use ray::Ray;
use intersection::Intersection;
use self::ordered_float::OrderedFloat;

enum Branch<'a> {
  Leaf {
    aabb: &'a AABB,
    index: usize,
  },
  Node {
    aabb: AABB,
    left: Box<Branch<'a>>,
    right: Box<Branch<'a>>,
  }
}

pub struct BVH<'a> {
  list: &'a [Box<Shape>],
  root: Box<Branch<'a>>,
}

impl<'a> BVH<'a> {
  pub fn new(list: &'a [Box<Shape>]) -> BVH<'a> {
    let mut indexes = (0..list.len()).enumerate().map( |(i, _)| i ).collect::<Vec<usize>>();
    let root = Self::construct(&list, &mut indexes);
    BVH {
      list: list,
      root: root, 
    }
  }

  fn construct(list: &'a [Box<Shape>], indexes: &mut [usize]) -> Box<Branch<'a>> {
    // セットアップ
    let len = indexes.len();
    // 要素が1つのときは葉
    if len == 1 {
      let i = indexes[0];
      return box Branch::Leaf {
        aabb: list[i].aabb(),
        index: i,
      }
    }
    // 全体のAABBを作成
    let aabb = AABB::merge(&list.iter().map( |v| v.aabb() ).collect());
    // 最大のAABBの辺を基準にして分割する
    // 最大のAABBの辺のインデックスを求める
    let sides: [f32; 3] = aabb.side().into();
    let partition_axis = sides.iter()
      .enumerate().max_by_key( |&(_, v)| OrderedFloat(*v) )
      .map( |(i, _)| i ).unwrap_or(0);
    // 基準の軸でソート
    indexes.sort_unstable_by_key( |&i| {
      OrderedFloat(list[i].aabb().center[partition_axis])
    });
    // 分割するインデックスを求める
    let partition_index = len / 2; // i-1とiの間で分割
    // 再帰的に子要素を生成
    let left = Self::construct(list, &mut indexes[0..partition_index]);
    let right = Self::construct(list, &mut indexes[partition_index..]);
    box Branch::Node {
      aabb: aabb,
      left: left,
      right: right,
    }
  }

  fn traverse(&self, branch: &Box<Branch>, ray: &Ray) -> Option<Intersection> {
    match **branch {
      Branch::Leaf { ref aabb, index } => {
        if aabb.is_intersect(ray) {
          self.list[index].intersect(ray)
        } else {
          None
        }
      },
      Branch::Node { ref aabb, ref left, ref right } => {
        if aabb.is_intersect(ray) {
          let l = self.traverse(left, ray);
          let r = self.traverse(right, ray);
          if l.is_some() && r.is_some() {
            l.and_then( |v| r.map( |w| {
              if v.distance > w.distance { w } else { v }
            }))
          } else {
            r.or(l)
          }
        } else {
          None
        }
      }
    }
  }
}

impl<'a> Shape for BVH<'a> {
  fn intersect(&self, ray: &Ray) -> Option<Intersection> {
    self.traverse(&self.root, &ray)
  }

  fn aabb(&self) -> &AABB {
    match *self.root {
      Branch::Leaf { ref aabb, .. } => {
        aabb
      },
      Branch::Node { ref aabb, .. } => {
        aabb
      }
    }
  }
}
