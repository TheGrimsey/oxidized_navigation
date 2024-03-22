//! Module for debug draws.
use bevy::{
    gizmos::{config::GizmoConfigGroup, AppGizmoBuilder},
    prelude::{
        any_with_component, App, Color, Commands, Component, Entity, Gizmos, IntoSystemConfigs,
        Plugin, Query, ReflectResource, Res, Resource, Update, Vec3,
    },
    reflect::Reflect,
    time::{Time, Timer},
};

use crate::NavMesh;

pub struct OxidizedNavigationDebugDrawPlugin;
impl Plugin for OxidizedNavigationDebugDrawPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DrawNavMesh>();
        app.register_type::<DrawNavMesh>();

        app.add_systems(
            Update,
            (
                draw_nav_mesh_system.run_if(should_draw_nav_mesh),
                draw_path_system.run_if(any_with_component::<DrawPath>),
            ),
        );

        app.init_gizmo_group::<NavigationGroup>();
    }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct NavigationGroup;

#[derive(Default, Resource, Reflect)]
#[reflect(Resource)]
/// Whether to draw the nav-mesh or not.
pub struct DrawNavMesh(pub bool);

fn should_draw_nav_mesh(draw_nav_mesh: Res<DrawNavMesh>) -> bool {
    draw_nav_mesh.0
}

fn draw_nav_mesh_system(nav_mesh: Res<NavMesh>, mut gizmos: Gizmos<NavigationGroup>) {
    if let Ok(nav_mesh) = nav_mesh.get().read() {
        for (tile_coord, tile) in nav_mesh.get_tiles().iter() {
            let tile_color = Color::Rgba {
                red: 0.0,
                green: (tile_coord.x % 10) as f32 / 10.0,
                blue: (tile_coord.y % 10) as f32 / 10.0,
                alpha: 1.0,
            };
            // Draw polygons.
            for poly in tile.polygons.iter() {
                let indices = &poly.indices;
                for i in 0..indices.len() {
                    let a = tile.vertices[indices[i] as usize];
                    let b = tile.vertices[indices[(i + 1) % indices.len()] as usize];
                    gizmos.line(a, b, tile_color);
                }
            }

            // Draw vertex points.
            for vertex in tile.vertices.iter() {
                gizmos.line(*vertex, *vertex + Vec3::Y, tile_color);
            }
        }
    }
}

#[derive(Component)]
/// Path drawing helper component. Each instance of this component will draw a path for until ``timer`` passed before being despawned.
pub struct DrawPath {
    /// Timer for how long to display path before it is despawned.
    ///
    /// If ``None`` the DrawPath entity will not be automatically despawned
    pub timer: Option<Timer>,
    /// Path to display.
    pub pulled_path: Vec<Vec3>,
    /// Color to display path as.
    pub color: Color,
}

// Helper function to draw a path for the timer's duration.
fn draw_path_system(
    mut commands: Commands,
    mut path_query: Query<(Entity, &mut DrawPath)>,
    time: Res<Time>,
    mut gizmos: Gizmos<NavigationGroup>,
) {
    path_query.iter_mut().for_each(|(entity, mut draw_path)| {
        if draw_path
            .timer
            .as_mut()
            .is_some_and(|timer| timer.tick(time.delta()).just_finished())
        {
            commands.entity(entity).despawn();
        } else {
            gizmos.linestrip(draw_path.pulled_path.clone(), draw_path.color);
        }
    });
}
