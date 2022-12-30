use bevy::prelude::{Res, ResMut};

use super::{
    get_cell_offset, DirtyTiles, NavMeshSettings, OpenSpan, OpenTile,
    TilesOpen,
};

#[derive(Default, Clone, Copy)]
struct LevelStackEntry {
    cell_index: u32,
    span_index: u32,
    index: i32,
}

pub(super) fn build_regions_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    mut open_tiles: ResMut<TilesOpen>,
    dirty_tiles: Res<DirtyTiles>,
) {
    let expand_iters = 4 + nav_mesh_settings.walkable_radius * 2;

    const LOG_NB_STACKS: i32 = 3;
    const NB_STACKS: i32 = 1 << LOG_NB_STACKS; // 8.

    for tile_coord in dirty_tiles.0.iter() {
        let Some(tile) = open_tiles.map.get_mut(tile_coord) else {
            continue;
        };

        let mut regions = vec![0; tile.span_count];
        let mut distances = vec![0; tile.span_count];

        let mut level_stacks: [Vec<LevelStackEntry>; NB_STACKS as usize] = Default::default();
        for stack in level_stacks.iter_mut() {
            stack.reserve(256);
        }
        let mut stack = Vec::with_capacity(256);

        let mut region_id = 1u16;
        let mut level = (tile.max_distance + 1) & !1u16; // Rounded.

        let mut stack_id = -1;
        while level > 0 {
            level = if level >= 2 { level - 2 } else { 0 };
            stack_id = (stack_id + 1) & (NB_STACKS - 1);

            if stack_id == 0 {
                // Sort cells by level.
                sort_cells_by_level(level, tile, &mut level_stacks, NB_STACKS, &regions);
            } else {
                // append stacks
                let prev_stack = (stack_id - 1) as usize;
                let next_stack = stack_id as usize;
                for i in 0..level_stacks[prev_stack].len() {
                    if regions[level_stacks[prev_stack][i].index as usize] != 0 {
                        continue;
                    }

                    level_stacks[next_stack].push(level_stacks[prev_stack][i]);
                }
            }

            // expand regions.
            expand_regions(
                &nav_mesh_settings,
                expand_iters,
                tile,
                &mut regions,
                &mut distances,
                &mut level_stacks[stack_id as usize],
            );

            // Mark new regions with IDs.
            for entry in level_stacks[stack_id as usize].iter() {
                if entry.index >= 0
                    && regions[entry.index as usize] == 0
                    && flood_region(
                        &nav_mesh_settings,
                        entry.cell_index,
                        entry.span_index,
                        entry.index,
                        level,
                        region_id,
                        tile,
                        &mut regions,
                        &mut distances,
                        &mut stack,
                    )
                {
                    region_id += 1;
                }
            }
        }

        // Expand regions until no empty connected cells are found.
        expand_regions_until_end(
            &nav_mesh_settings,
            tile,
            &mut regions,
            &mut distances,
            &mut stack,
        );

        // Merge regions and filter out small ones.
        merge_regions(&nav_mesh_settings, &mut regions, &mut region_id, tile);

        // Write results into spans.
        for cell in tile.cells.iter_mut() {
            for span in cell.spans.iter_mut() {
                span.region = regions[span.tile_index];
            }
        }

        tile.max_regions = region_id;
    }
}

fn sort_cells_by_level(
    start_level: u16,
    open_tile: &OpenTile,
    stacks: &mut [Vec<LevelStackEntry>],
    nb_stacks: i32, // always NB_STACKS
    regions: &[u16],
) {
    let start_level = (start_level >> 1) as i32;

    for stack in stacks.iter_mut() {
        stack.clear();
    }

    for (c_i, cell) in open_tile.cells.iter().enumerate() {
        for (s_i, span) in cell.spans.iter().enumerate() {
            if regions[span.tile_index] != 0 {
                continue;
            }

            let level = (open_tile.distances[span.tile_index] >> 1) as i32;
            let stack_id = (start_level - level).max(0);
            if stack_id >= nb_stacks {
                continue;
            }

            stacks[stack_id as usize].push(LevelStackEntry {
                cell_index: c_i as u32,
                span_index: s_i as u32,
                index: span.tile_index as i32,
            });
        }
    }
}

struct DirtyEntry {
    index: i32,
    region: u16,
    distance: u16,
}

fn expand_regions(
    nav_mesh_settings: &NavMeshSettings,
    max_iterations: u16,
    tile: &OpenTile,
    regions: &mut [u16],
    distances: &mut [u16],
    level_stack: &mut [LevelStackEntry],
) {
    for entry in level_stack
        .iter_mut()
        .filter(|entry| regions[entry.index as usize] != 0)
    {
        entry.index = -1;
    }

    let mut iter = 0;
    let mut dirty_entries = Vec::new();
    loop {
        let mut failed = 0;
        dirty_entries.clear();

        for entry in level_stack.iter_mut() {
            if entry.index < 0 {
                failed += 1;
                continue;
            }

            let mut new_region = regions[entry.index as usize];
            let mut distance = u16::MAX;
            let span = &tile.cells[entry.cell_index as usize].spans[entry.span_index as usize];

            for dir in 0..4 {
                let Some(index) = span.neighbours[dir] else {
                    continue;
                };

                let other_span = &tile.cells[(entry.cell_index as isize
                    + get_cell_offset(nav_mesh_settings, dir))
                    as usize]
                    .spans[index as usize];

                let other_region = regions[other_span.tile_index];
                let other_distance = distances[other_span.tile_index];
                if other_region > 0 && other_distance + 2 < distance {
                    new_region = other_region;
                    distance = other_distance + 2;
                }
            }

            if new_region != 0 {
                entry.index = -1;
                dirty_entries.push(DirtyEntry {
                    index: span.tile_index as i32,
                    region: new_region,
                    distance,
                });
            } else {
                failed += 1;
            }
        }

        // Copy entries that differ between src and st to keep them in sync.
        for entry in dirty_entries.iter() {
            regions[entry.index as usize] = entry.region;
            distances[entry.index as usize] = entry.distance;
        }

        if failed == level_stack.len() {
            break;
        }

        iter += 1;
        if iter >= max_iterations {
            break;
        }
    }
}

fn expand_regions_until_end(
    nav_mesh_settings: &NavMeshSettings,
    tile: &OpenTile,
    regions: &mut [u16],
    distances: &mut [u16],
    level_stack: &mut Vec<LevelStackEntry>,
) {
    level_stack.clear();

    for (c_i, cell) in tile.cells.iter().enumerate() {
        for (s_i, span) in cell.spans.iter().enumerate() {
            if regions[span.tile_index] == 0 {
                level_stack.push(LevelStackEntry {
                    cell_index: c_i as u32,
                    span_index: s_i as u32,
                    index: span.tile_index as i32,
                });
            }
        }
    }

    let mut dirty_entries = Vec::new();
    let mut failed = 0;
    while failed < level_stack.len() {
        failed = 0;
        dirty_entries.clear();

        for entry in level_stack.iter_mut() {
            if entry.index < 0 {
                failed += 1;
                continue;
            }

            let mut new_region = regions[entry.index as usize];
            let mut distance = u16::MAX;
            let span = &tile.cells[entry.cell_index as usize].spans[entry.span_index as usize];

            for dir in 0..4 {
                let Some(index) = span.neighbours[dir] else {
                    continue;
                };

                let other_span = &tile.cells[(entry.cell_index as isize
                    + get_cell_offset(nav_mesh_settings, dir))
                    as usize]
                    .spans[index as usize];

                let other_region = regions[other_span.tile_index];
                let other_distance = distances[other_span.tile_index];
                if other_region > 0 && other_distance + 2 < distance {
                    new_region = other_region;
                    distance = other_distance + 2;
                }
            }

            if new_region != 0 {
                entry.index = -1;
                dirty_entries.push(DirtyEntry {
                    index: span.tile_index as i32,
                    region: new_region,
                    distance,
                });
            } else {
                failed += 1;
            }
        }

        for entry in dirty_entries.iter() {
            regions[entry.index as usize] = entry.region;
            distances[entry.index as usize] = entry.distance;
        }

        if failed == level_stack.len() {
            break;
        }
    }
}

struct Region {
    id: u16,
    span_count: usize,
    remap: bool,
    visited: bool,
    overlap: bool,
    is_border_region: bool,
    floors: Vec<u16>,
    connections: Vec<u16>,
}

fn merge_regions(
    nav_mesh_settings: &NavMeshSettings,
    source_regions: &mut [u16],
    max_region_id: &mut u16,
    tile: &OpenTile,
) {
    let mut regions = Vec::with_capacity(*max_region_id as usize);
    for i in 0..*max_region_id {
        regions.push(Region {
            id: i,
            span_count: 0,
            remap: false,
            visited: false,
            overlap: false,
            is_border_region: false,
            floors: Vec::new(),
            connections: Vec::new(),
        });
    }

    for (c_i, cell) in tile.cells.iter().enumerate() {
        let row = c_i / nav_mesh_settings.tile_width as usize;
        let column = c_i % nav_mesh_settings.tile_width as usize;
        let is_border = row == 0
            || column == 0
            || row == (nav_mesh_settings.tile_width - 1) as usize
            || column == (nav_mesh_settings.tile_width - 1) as usize;

        for (s_i, span) in cell.spans.iter().enumerate() {
            let region_id = source_regions[span.tile_index];
            if region_id == 0 || region_id > *max_region_id {
                continue;
            }

            let region = &mut regions[region_id as usize];
            region.span_count += 1;
            region.is_border_region |= is_border;

            // Update floors
            for other_span in cell
                .spans
                .iter()
                .filter(|a| a.tile_index != span.tile_index)
            {
                let other_region_id = source_regions[other_span.tile_index];
                if other_region_id == 0 || other_region_id > *max_region_id {
                    continue;
                }
                if other_region_id == region_id {
                    region.overlap = true;
                }

                add_unique_floor_region(region, region_id);
            }

            // Contour already exists.
            if !region.connections.is_empty() {
                continue;
            }

            let dir = {
                let mut dir = None;

                for i in 0..4 {
                    if is_solid_edge(nav_mesh_settings, tile, span, c_i, i, source_regions) {
                        dir = Some(i);
                        break;
                    }
                }

                dir
            };

            if let Some(dir) = dir {
                walk_contour(
                    c_i,
                    s_i,
                    dir,
                    tile,
                    nav_mesh_settings,
                    source_regions,
                    &mut region.connections,
                )
            }
        }
    }
    // Remove too small regions
    let mut stack = Vec::with_capacity(32);
    let mut trace = Vec::with_capacity(32);
    let mut connections: Vec<u16> = Vec::with_capacity(16);

    for i in 0..*max_region_id {
        {
            let region = &mut regions[i as usize];
            if region.id == 0 || region.span_count == 0 || region.visited || region.is_border_region
            {
                continue;
            }

            region.visited = true;
        }

        let mut connects_to_border = false;

        stack.clear();
        trace.clear();

        stack.push(i);

        let mut span_count = 0;

        while let Some(r_i) = stack.pop() {
            connections.clear();
            trace.push(r_i);

            {
                let region = &regions[r_i as usize];
                connections.extend(region.connections.iter());
                span_count += region.span_count;
            }

            for connected_region in &connections {
                let connected_region = &mut regions[*connected_region as usize];

                if connected_region.visited {
                    continue;
                }

                if connected_region.is_border_region {
                    connects_to_border = true;
                    continue;
                }

                stack.push(connected_region.id);
                connected_region.visited = true;
            }
        }

        if span_count < nav_mesh_settings.min_region_area && !connects_to_border {
            for trace in &trace {
                let region = &mut regions[*trace as usize];
                region.span_count = 0;
                region.id = 0;
            }
        }
    }

    loop {
        let mut merged = false;

        for region in 0..regions.len() {
            {
                let region = &regions[region];
                if region.id == 0
                    || !region.is_border_region
                    || !region.overlap
                    || region.span_count == 0
                {
                    continue;
                }

                let connected_to_border = region.connections.contains(&0);
                if region.span_count > nav_mesh_settings.merge_region_size && connected_to_border {
                    continue;
                }
            }

            let mut smallest_region_size = usize::MAX;
            let mut merge_id = None;
            {
                let region = &regions[region];
                for connected in &region.connections {
                    let other_region = &regions[*connected as usize];
                    if other_region.id == 0 || other_region.is_border_region || other_region.overlap
                    {
                        continue;
                    }

                    if other_region.span_count < smallest_region_size
                        && can_merge_with_region(region, other_region)
                        && can_merge_with_region(other_region, region)
                    {
                        smallest_region_size = other_region.span_count;
                        merge_id = Some(other_region.id);
                    }
                }
            }

            if let Some(merge_id) = merge_id {
                let old_id = regions[region].id;

                if merge_regions_i(&mut regions, region, merge_id as usize) {
                    // Fix up regions pointing to this region.
                    for region in regions.iter_mut() {
                        if region.id == 0 || region.is_border_region {
                            continue;
                        }

                        if region.id == old_id {
                            region.id = merge_id;
                        }

                        replace_neighbour(region, old_id, merge_id);
                    }

                    merged = true;
                }
            }
        }

        if !merged {
            break;
        }
    }

    // Compress region ids.
    for region in regions.iter_mut() {
        region.remap = !(region.id == 0 || region.is_border_region);
    }

    let mut region_id_gen = 0;
    for i in 0..regions.len() {
        if !regions[i].remap {
            continue;
        }
        region_id_gen += 1;

        let old_id = regions[i].id;
        let new_id = region_id_gen;

        for region in regions.iter_mut().skip(i) {
            if region.id == old_id {
                region.id = new_id;
                region.remap = false;
            }
        }
    }
    // TODO: set max region id

    // Remap regions.
    for cell in tile.cells.iter() {
        for span in cell.spans.iter() {
            let new_region_id = regions[source_regions[span.tile_index] as usize].id;
            source_regions[span.tile_index] = new_region_id;
        }
    }
}

fn replace_neighbour(region: &mut Region, old_id: u16, new_id: u16) {
    let mut connection_changed = false;
    for connection in region.connections.iter_mut() {
        if *connection == old_id {
            *connection = new_id;
            connection_changed = true;
        }
    }
    for floor in region.floors.iter_mut() {
        if *floor == old_id {
            *floor = new_id;
        }
    }
    if connection_changed {
        remove_adjacent_connection_duplicates(region);
    }
}

fn merge_regions_i(regions: &mut [Region], a: usize, b: usize) -> bool {
    let merged_connections = {
        let a = &regions[a];
        let b = &regions[b];

        let Some(insert_point_a) = a.connections.iter().position(|i| *i == b.id) else {
            return false;
        };
        let Some(insert_point_b) = b.connections.iter().position(|i| *i == a.id) else {
            return false;
        };

        let mut merged_connections =
            Vec::with_capacity(a.connections.len() + b.connections.len() - 2);
        let size = a.connections.len() - 1;
        for i in 0..size {
            merged_connections.push(a.connections[(insert_point_a + 1 + i) % size]);
        }
        let size = b.connections.len() - 1;
        for i in 0..size {
            merged_connections.push(b.connections[(insert_point_b + 1 + i) % size]);
        }

        merged_connections
    };

    let (b_span_count, floors) = {
        let b = &mut regions[b];
        b.connections.clear();

        let count = b.span_count;
        b.span_count = 0;

        (count, b.floors.clone())
    };

    {
        let a = &mut regions[a];
        a.span_count += b_span_count;
        a.connections = merged_connections;

        remove_adjacent_connection_duplicates(a);

        // add unique floors.
        for floor in floors {
            add_unique_floor_region(a, floor);
        }
    }

    true
}

fn remove_adjacent_connection_duplicates(region: &mut Region) {
    if region.connections.len() > 1 {
        let mut i = 0;
        while i < region.connections.len() {
            let next_index = (i + 1) % region.connections.len();
            let current = region.connections[i];
            let next = region.connections[next_index];

            if current == next {
                region.connections.remove(next_index);
            } else {
                i += 1;
            }
        }
    }
}

fn can_merge_with_region(a: &Region, b: &Region) -> bool {
    !(b.connections.contains(&a.id) || a.floors.contains(&b.id))
}

fn walk_contour(
    mut cell_index: usize,
    mut span_index: usize,
    mut dir: usize,
    tile: &OpenTile,
    nav_mesh_settings: &NavMeshSettings,
    source_regions: &[u16],
    contour: &mut Vec<u16>,
) {
    let start_direction = dir;
    let start_cell = cell_index;
    let start_span = span_index;

    let span = &tile.cells[cell_index].spans[span_index];
    let mut current_region = 0;
    if let Some(index) = span.neighbours[dir] {
        let other_span = &tile.cells
            [(cell_index as isize + get_cell_offset(nav_mesh_settings, dir)) as usize]
            .spans[index as usize];

        current_region = source_regions[other_span.tile_index];
    }
    contour.push(current_region);

    loop {
        let span = &tile.cells[cell_index].spans[span_index];
        if is_solid_edge(
            nav_mesh_settings,
            tile,
            span,
            cell_index,
            dir,
            source_regions,
        ) {
            let mut r = 0;
            if let Some(index) = span.neighbours[dir] {
                let other_span = &tile.cells
                    [(cell_index as isize + get_cell_offset(nav_mesh_settings, dir)) as usize]
                    .spans[index as usize];

                r = source_regions[other_span.tile_index];
            }
            if r != current_region {
                current_region = r;
                contour.push(r);
            }

            dir = (dir + 1) & 0x3; // Rotate clock-wise.
        } else {
            // Direction is connected.
            if let Some(index) = span.neighbours[dir] {
                span_index = index.into();
            } else {
                return;
            }

            cell_index = (cell_index as isize + get_cell_offset(nav_mesh_settings, dir)) as usize;
            dir = (dir + 3) & 0x3; // Rotate COUNTER clock-wise.
        }

        if start_cell == cell_index && start_span == span_index && start_direction == dir {
            break;
        }
    }

    // Remove adjacent duplicates.
    if contour.len() > 2 {
        let mut i = 0;
        while i < contour.len() {
            let next_index = (i + 1) % contour.len();
            let current = contour[i];
            let next = contour[next_index];

            if current == next {
                contour.remove(next_index);
            } else {
                i += 1;
            }
        }
    }
}

fn is_solid_edge(
    nav_mesh_settings: &NavMeshSettings,
    tile: &OpenTile,
    span: &OpenSpan,
    c_i: usize,
    dir: usize,
    source_region: &[u16],
) -> bool {
    if let Some(index) = span.neighbours[dir] {
        let other_span = &tile.cells
            [(c_i as isize + get_cell_offset(nav_mesh_settings, dir)) as usize]
            .spans[index as usize];

        return source_region[other_span.tile_index] != source_region[span.tile_index];
    }

    true
}

fn add_unique_floor_region(region: &mut Region, region_id: u16) {
    if region.floors.contains(&region_id) {
        return;
    }

    region.floors.push(region_id);
}

fn flood_region(
    nav_mesh_settings: &NavMeshSettings,
    cell_index: u32,
    span_index: u32,
    index: i32,
    level: u16,
    region_id: u16,
    tile: &mut OpenTile,
    regions: &mut [u16],
    distances: &mut [u16],
    stack: &mut Vec<LevelStackEntry>,
) -> bool {
    stack.clear();
    stack.push(LevelStackEntry {
        cell_index,
        span_index,
        index,
    });

    regions[index as usize] = region_id;
    distances[index as usize] = 0;

    let mut count = 0;

    while let Some(entry) = stack.pop() {
        let span = &tile.cells[entry.cell_index as usize].spans[entry.span_index as usize];

        let mut adjecant_region = 0;
        for dir in 0..4 {
            let Some(index) = span.neighbours[dir] else {
                continue;
            };

            let other_cell_index =
                (entry.cell_index as isize + get_cell_offset(nav_mesh_settings, dir)) as usize;
            let other_span = &tile.cells[other_cell_index].spans[index as usize];
            let other_region = regions[other_span.tile_index];

            if other_region != 0 {
                adjecant_region = other_region;
                break;
            }

            let next_dir = (dir + 1) & 0x3;
            if let Some(index) = span.neighbours[next_dir] {
                let other_span = &tile.cells[(other_cell_index as isize
                    + get_cell_offset(nav_mesh_settings, next_dir))
                    as usize]
                    .spans[index as usize];
                let other_region = regions[other_span.tile_index];

                if other_region != 0 {
                    adjecant_region = other_region;
                    break;
                }
            }
        }

        if adjecant_region != 0 {
            regions[entry.index as usize] = 0;
            continue;
        }

        count += 1;

        for dir in 0..4 {
            let Some(index) = span.neighbours[dir] else {
                continue;
            };

            let other_cell_index =
                (entry.cell_index as isize + get_cell_offset(nav_mesh_settings, dir)) as usize;
            let other_span = &tile.cells[other_cell_index].spans[index as usize];

            if tile.distances[other_span.tile_index] >= level && regions[other_span.tile_index] == 0
            {
                regions[other_span.tile_index] = region_id;
                distances[other_span.tile_index] = 0;
                stack.push(LevelStackEntry {
                    cell_index: other_cell_index as u32,
                    span_index: index.into(),
                    index: other_span.tile_index as i32,
                })
            }
        }
    }

    count > 0
}
