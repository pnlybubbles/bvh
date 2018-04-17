# BVH

Bounding volume hierarchy implementation in rust.

## Implementation

- Complete binary tree

## Benchmark

```
RUSTFLAGS='--emit asm -C target-feature=+avx' cargo bench
```

### Construction

```
test tests::bench_construct_bvh
mesh: 144046 ploygon
bench: 329,071,227 ns/iter (+/- 249,062,364)
```

### Intersection

```
test tests::bench_intersection_bvh
Suzanne: 968 ploygon
bench:   1,815,536 ns/iter (+/- 619,386)
```

## See also

[LumillyRender](https://github.com/pnlybubbles/LumillyRender)
