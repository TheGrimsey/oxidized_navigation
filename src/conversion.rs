use bevy::prelude::{Transform, Vec3};
use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "xpbd")] {
        use parry3d_xpbd as parry3d;
    } else if #[cfg(feature = "rapier")] {
        use parry3d_rapier as parry3d;
    }
}

use parry3d::{
    math::Real,
    na::Point3,
    shape::{Ball, Capsule, Cone, Cuboid, Cylinder, Triangle},
};

use crate::{heightfields::TriangleCollection, Area};

pub struct GeometryCollection {
    pub transform: Transform,
    pub geometry_to_convert: GeometryToConvert,
    pub area: Option<Area>,
}

pub enum ColliderType {
    Cuboid(Cuboid),
    Ball(Ball),
    Capsule(Capsule),
    Cylinder(Cylinder),
    Cone(Cone),
    Triangle(Triangle),
}

pub enum GeometryToConvert {
    Collider(ColliderType),
    ParryTriMesh(Box<[Point3<Real>]>, Box<[[u32; 3]]>),
}

pub(super) enum Triangles {
    Triangle([Vec3; 3]),
    TriMesh(Box<[Vec3]>, Box<[[u32; 3]]>),
}

const SUBDIVISIONS: u32 = 5;

pub(super) fn convert_geometry_collections(
    geometry_collections: Vec<GeometryCollection>,
) -> Box<[TriangleCollection]> {
    geometry_collections
        .into_iter()
        .map(|geometry_collection| TriangleCollection {
            transform: geometry_collection.transform,
            triangles: convert_geometry(geometry_collection.geometry_to_convert),
            area: geometry_collection.area,
        })
        .collect()
}

pub(super) fn convert_geometry(geometry_to_convert: GeometryToConvert) -> Triangles {
    match geometry_to_convert {
        GeometryToConvert::Collider(collider) => {
            let (vertices, triangles) = match collider {
                ColliderType::Cuboid(cuboid) => cuboid.to_trimesh(),
                ColliderType::Ball(ball) => ball.to_trimesh(SUBDIVISIONS, SUBDIVISIONS),
                ColliderType::Capsule(capsule) => capsule.to_trimesh(SUBDIVISIONS, SUBDIVISIONS),
                ColliderType::Cylinder(cylinder) => cylinder.to_trimesh(SUBDIVISIONS),
                ColliderType::Cone(cone) => cone.to_trimesh(SUBDIVISIONS),
                ColliderType::Triangle(triangle) => {
                    return Triangles::Triangle(
                        triangle
                            .vertices()
                            .map(|point| Vec3::new(point.x, point.y, point.z)),
                    );
                }
            };

            let vertices = vertices
                .iter()
                .map(|point| Vec3::new(point.x, point.y, point.z))
                .collect();

            Triangles::TriMesh(vertices, triangles.into_boxed_slice())
        }
        GeometryToConvert::ParryTriMesh(vertices, triangles) => {
            let vertices = vertices
                .iter()
                .map(|point| Vec3::new(point.x, point.y, point.z))
                .collect();

            Triangles::TriMesh(vertices, triangles)
        }
    }
}
