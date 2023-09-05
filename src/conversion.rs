use bevy::prelude::{Transform, Vec3};
use parry3d::{
    math::Real,
    na::Point3,
    shape::{Ball, Capsule, Cone, Cuboid, Cylinder, Triangle},
};

use crate::{heightfields::TriangleCollection, Area};

pub(super) struct GeometryCollection {
    pub(super) transform: Transform,
    pub(super) geometry_to_convert: GeometryToConvert,
    pub(super) area: Option<Area>,
}

pub(super) enum ColliderType {
    Cuboid(Cuboid),
    Ball(Ball),
    Capsule(Capsule),
    Cylinder(Cylinder),
    Cone(Cone),
    Triangle(Triangle),
}

pub(super) enum GeometryToConvert {
    Collider(ColliderType),
    ParryTriMesh(Vec<Point3<Real>>, Vec<[u32; 3]>),
}

pub(super) enum Triangles {
    Triangle([Vec3; 3]),
    TriMesh(Vec<Vec3>, Vec<[u32; 3]>),
}

const SUBDIVISIONS: u32 = 5;

pub(super) fn convert_geometry_collections(
    mut geometry_collections: Vec<GeometryCollection>,
) -> Vec<TriangleCollection> {
    geometry_collections
        .drain(..)
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

            Triangles::TriMesh(vertices, triangles)
        }
        GeometryToConvert::ParryTriMesh(mut vertices, triangles) => {
            let vertices = vertices
                .drain(..)
                .map(|point| Vec3::new(point.x, point.y, point.z))
                .collect();

            Triangles::TriMesh(vertices, triangles)
        }
    }
}
