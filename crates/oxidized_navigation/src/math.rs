use bevy::math::{IVec4, UVec4};

pub fn intersect_prop(a: IVec4, b: IVec4, c: IVec4, d: IVec4) -> bool {
    if collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b) {
        return false;
    }

    (left(a, b, c) ^ left(a, b, d)) && (left(c, d, a) ^ left(c, d, b))
}

pub fn between(a: IVec4, b: IVec4, c: IVec4) -> bool {
    if !collinear(a, b, c) {
        return false;
    }

    if a.x != b.x {
        return (a.x <= c.x && c.x <= b.x) || (a.x >= c.x && c.x >= b.x);
    }

    (a.z <= c.z && c.z <= b.z) || (a.z >= c.z && c.z >= b.z)
}

pub fn intersect(a: IVec4, b: IVec4, c: IVec4, d: IVec4) -> bool {
    intersect_prop(a, b, c, d)
        || between(a, b, c)
        || between(a, b, d)
        || between(c, d, a)
        || between(c, d, b)
}

pub fn area_sqr(a: IVec4, b: IVec4, c: IVec4) -> i32 {
    (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z)
}

pub fn collinear(a: IVec4, b: IVec4, c: IVec4) -> bool {
    area_sqr(a, b, c) == 0
}

pub fn left(a: IVec4, b: IVec4, c: IVec4) -> bool {
    area_sqr(a, b, c) < 0
}
pub fn left_on(a: IVec4, b: IVec4, c: IVec4) -> bool {
    area_sqr(a, b, c) <= 0
}

pub fn in_cone(i: usize, outline_vertices: &[UVec4], point: UVec4) -> bool {
    let point_i = outline_vertices[i];
    let point_next = outline_vertices[(i + 1) % outline_vertices.len()];
    let point_previous =
        outline_vertices[(outline_vertices.len() + i - 1) % outline_vertices.len()];

    if left_on(point_i.as_ivec4(), point.as_ivec4(), point_next.as_ivec4()) {
        return left(
            point_i.as_ivec4(),
            point.as_ivec4(),
            point_previous.as_ivec4(),
        ) && left(point.as_ivec4(), point_i.as_ivec4(), point_next.as_ivec4());
    }

    !left_on(point_i.as_ivec4(), point.as_ivec4(), point_next.as_ivec4())
        && left_on(
            point.as_ivec4(),
            point_i.as_ivec4(),
            point_previous.as_ivec4(),
        )
}
