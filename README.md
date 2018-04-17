# BVH

Bounding volume hierarchy implementation in rust.

## Implementation

- ~~Complete binary tree~~
- Binary tree (Surface Area Heuristics)

## Benchmark

```
RUSTFLAGS='--emit asm -C target-feature=+avx' cargo bench
```

### Construction

```
test tests::bench_construct_bvh              ...
144046 triangles
bench: 697,404,302 ns/iter (+/- 130,975,216)
```

### Intersection

10000 random ray.

```
test tests::bench_intersection_bvh           ...
66450 triangles
bench:  24,281,123 ns/iter (+/- 3,389,945)
```

## See also

[LumillyRender](https://github.com/pnlybubbles/LumillyRender)
