use crate::{
    heightfields::{OpenSpan, OpenTile},
    Area,
};

use super::{get_neighbour_index, NavMeshSettings};

#[derive(Default, Clone, Copy)]
struct LevelStackEntry {
    cell_index: u32,
    span_index: u32,
    index: i32,
}

const EXPAND_ITERS: u16 = 8;
const LOG_NB_STACKS: i32 = 3;
const NB_STACKS: i32 = 1 << LOG_NB_STACKS; // 8.

pub fn build_regions(open_tile: &mut OpenTile, nav_mesh_settings: &NavMeshSettings) {
    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    let mut regions = vec![0; open_tile.span_count];
    let mut distances = vec![0; open_tile.span_count];
    let mut dirty_entries = Vec::with_capacity(512);

    let mut level_stacks: [Vec<LevelStackEntry>; NB_STACKS as usize] = Default::default();
    for stack in level_stacks.iter_mut() {
        stack.reserve(256);
    }
    let mut stack = Vec::with_capacity(256);

    let mut region_id = 1;
    let mut level = (open_tile.max_distance + 1) & !1u16; // Rounded.

    let mut stack_id = -1;
    while level > 0 {
        level = level.saturating_sub(2);
        stack_id = (stack_id + 1) & (NB_STACKS - 1);

        if stack_id == 0 {
            // Sort cells by level.
            sort_cells_by_level(level, open_tile, &mut level_stacks, NB_STACKS, &regions);
        } else {
            // append stacks
            let prev_stack = (stack_id - 1) as usize;
            let next_stack = stack_id as usize;
            for i in 0..level_stacks[prev_stack].len() {
                let index = level_stacks[prev_stack][i].index;
                if index < 0 || regions[index as usize] != 0 {
                    continue;
                }

                level_stacks[next_stack].push(level_stacks[prev_stack][i]);
            }
        }

        // expand regions.
        expand_regions(
            tile_side,
            EXPAND_ITERS,
            open_tile,
            &mut regions,
            &mut distances,
            &mut level_stacks[stack_id as usize],
            &mut dirty_entries,
        );

        // Mark new regions with IDs.
        for entry in level_stacks[stack_id as usize].iter() {
            if entry.index >= 0
                && regions[entry.index as usize] == 0
                && flood_region(
                    tile_side,
                    *entry,
                    level,
                    region_id,
                    open_tile,
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
        tile_side,
        open_tile,
        &mut regions,
        &mut distances,
        &mut stack,
        &mut dirty_entries,
    );

    // Merge regions and filter out small ones.
    merge_regions(
        nav_mesh_settings,
        tile_side,
        &mut regions,
        &mut region_id,
        open_tile,
    );

    // Write results into spans.
    for cell in open_tile.cells.iter_mut() {
        for span in cell.spans.iter_mut() {
            span.region = regions[span.tile_index];
        }
    }

    open_tile.max_regions = region_id;
}

fn sort_cells_by_level(
    start_level: u16,
    open_tile: &OpenTile,
    stacks: &mut [Vec<LevelStackEntry>],
    num_stacks: i32, // always NB_STACKS
    regions: &[u16],
) {
    let start_level = (start_level >> 1) as i32;

    for stack in stacks.iter_mut() {
        stack.clear();
    }

    for (c_i, cell) in open_tile.cells.iter().enumerate() {
        for (s_i, span) in cell.spans.iter().enumerate() {
            if open_tile.areas[span.tile_index].is_none() || regions[span.tile_index] != 0 {
                continue;
            }

            let level = (open_tile.distances[span.tile_index] >> 1) as i32;
            let stack_id = (start_level - level).max(0);
            if stack_id >= num_stacks {
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
    tile_side: usize,
    max_iterations: u16,
    tile: &OpenTile,
    regions: &mut [u16],
    distances: &mut [u16],
    level_stack: &mut [LevelStackEntry],
    dirty_entries: &mut Vec<DirtyEntry>,
) {
    for entry in level_stack
        .iter_mut()
        .filter(|entry| regions[entry.index as usize] != 0)
    {
        entry.index = -1;
    }

    let mut iter = 0;
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
            let area = tile.areas[span.tile_index];

            for dir in 0..4 {
                let Some(span_index) = span.neighbours[dir] else {
                    continue;
                };

                let other_span = &tile.cells
                    [get_neighbour_index(tile_side, entry.cell_index as usize, dir)]
                .spans[span_index as usize];
                let other_area = tile.areas[other_span.tile_index];
                if other_area != area {
                    continue;
                }

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
    tile_side: usize,
    tile: &OpenTile,
    regions: &mut [u16],
    distances: &mut [u16],
    level_stack: &mut Vec<LevelStackEntry>,
    dirty_entries: &mut Vec<DirtyEntry>,
) {
    level_stack.clear();

    for (c_i, cell) in tile.cells.iter().enumerate() {
        for (s_i, span) in cell.spans.iter().enumerate() {
            if regions[span.tile_index] == 0 && tile.areas[span.tile_index].is_some() {
                level_stack.push(LevelStackEntry {
                    cell_index: c_i as u32,
                    span_index: s_i as u32,
                    index: span.tile_index as i32,
                });
            }
        }
    }

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
            let area = tile.areas[span.tile_index];

            for dir in 0..4 {
                let Some(index) = span.neighbours[dir] else {
                    continue;
                };

                let other_span = &tile.cells
                    [get_neighbour_index(tile_side, entry.cell_index as usize, dir)]
                .spans[index as usize];
                let other_area = tile.areas[other_span.tile_index];
                if other_area != area {
                    continue;
                }

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
    floors: Vec<u16>,
    connections: Vec<u16>,
    area: Option<Area>,
}

fn merge_regions(
    nav_mesh_settings: &NavMeshSettings,
    tile_side: usize,
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
            floors: Vec::with_capacity(4),
            connections: Vec::with_capacity(4),
            area: None,
        });
    }

    for (c_i, cell) in tile.cells.iter().enumerate() {
        for (s_i, span) in cell.spans.iter().enumerate() {
            let region_id = source_regions[span.tile_index];
            if region_id == 0 || region_id >= *max_region_id {
                continue;
            }

            let region = &mut regions[region_id as usize];
            region.span_count += 1;

            // Update floors
            for other_span in cell
                .spans
                .iter()
                .filter(|other| other.tile_index != span.tile_index)
            {
                let other_region_id = source_regions[other_span.tile_index];
                if other_region_id == 0 || other_region_id >= *max_region_id {
                    continue;
                }
                region.overlap |= other_region_id == region_id;

                add_unique_floor_region(region, other_region_id);
            }

            // Contour already exists.
            if !region.connections.is_empty() {
                continue;
            }

            region.area = tile.areas[span.tile_index];

            let dir = {
                let mut dir = None;

                for i in 0..4 {
                    if is_solid_edge(tile_side, tile, span, c_i, i, source_regions) {
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
                    tile_side,
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
            if region.id == 0 || region.span_count == 0 || region.visited {
                continue;
            }

            region.visited = true;
        }

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

                if connected_region.visited || connected_region.id == 0 {
                    continue;
                }

                stack.push(connected_region.id);
                connected_region.visited = true;
            }
        }

        if span_count < nav_mesh_settings.min_region_area as usize {
            for trace in &trace {
                let region = &mut regions[*trace as usize];
                region.span_count = 0;
                region.id = 0;
            }
        }
    }

    // Merge regions into neighbour.
    loop {
        let mut merged = false;

        for region in 0..regions.len() {
            let merge_id = {
                let region = &regions[region];
                if region.id == 0 || region.overlap || region.span_count == 0 {
                    continue;
                }

                if region.span_count > nav_mesh_settings.max_region_area_to_merge_into as usize
                    && region.connections.contains(&0)
                {
                    continue;
                }

                let mut merge_id = None;
                let mut smallest_region_size = usize::MAX;

                for connected in &region.connections {
                    let other_region = &regions[*connected as usize];
                    if other_region.id == 0 || other_region.overlap {
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

                merge_id
            };

            if let Some(merge_id) = merge_id {
                let old_id = regions[region].id;

                if merge_regions_i(&mut regions, region, merge_id as usize) {
                    // Fix up regions pointing to this region.
                    for region in regions.iter_mut() {
                        if region.id == 0 {
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
        region.remap = region.id != 0;
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
    *max_region_id = region_id_gen;

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
        for i in 0..a.connections.len() - 1 {
            merged_connections.push(a.connections[(insert_point_a + 1 + i) % a.connections.len()]);
        }
        for i in 0..b.connections.len() - 1 {
            merged_connections.push(b.connections[(insert_point_b + 1 + i) % b.connections.len()]);
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
    if a.area != b.area {
        return false;
    }

    let mut n = 0;
    for region in a.connections.iter() {
        if *region == b.id {
            n += 1;
        }
    }
    if n > 1 {
        return false;
    }

    !a.floors.contains(&b.id)
}

fn walk_contour(
    mut cell_index: usize,
    mut span_index: usize,
    mut dir: usize,
    tile: &OpenTile,
    tile_side: usize,
    source_regions: &[u16],
    contour: &mut Vec<u16>,
) {
    let start_direction = dir;
    let start_cell = cell_index;
    let start_span = span_index;

    let span = &tile.cells[cell_index].spans[span_index];
    let mut current_region = 0;
    if let Some(span_index) = span.neighbours[dir] {
        let other_span =
            &tile.cells[get_neighbour_index(tile_side, cell_index, dir)].spans[span_index as usize];

        current_region = source_regions[other_span.tile_index];
    }
    contour.push(current_region);

    loop {
        let span = &tile.cells[cell_index].spans[span_index];
        if is_solid_edge(tile_side, tile, span, cell_index, dir, source_regions) {
            let mut r = 0;
            if let Some(span_index) = span.neighbours[dir] {
                let other_span = &tile.cells[get_neighbour_index(tile_side, cell_index, dir)].spans
                    [span_index as usize];

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

            cell_index = get_neighbour_index(tile_side, cell_index, dir);
            dir = (dir + 3) & 0x3; // Rotate COUNTER clock-wise.
        }

        if start_cell == cell_index && start_span == span_index && start_direction == dir {
            break;
        }
    }

    // Remove adjacent duplicates.
    if contour.len() > 1 {
        let mut i = 0;
        while i < contour.len() {
            let next_index = (i + 1) % contour.len();

            if contour[i] == contour[next_index] {
                contour.remove(next_index);
            } else {
                i += 1;
            }
        }
    }
}

fn is_solid_edge(
    tile_side: usize,
    tile: &OpenTile,
    span: &OpenSpan,
    c_i: usize,
    dir: usize,
    source_region: &[u16],
) -> bool {
    let mut region = 0;
    if let Some(span_index) = span.neighbours[dir] {
        let other_span =
            &tile.cells[get_neighbour_index(tile_side, c_i, dir)].spans[span_index as usize];

        region = source_region[other_span.tile_index];
    }

    region != source_region[span.tile_index]
}

fn add_unique_floor_region(region: &mut Region, region_id: u16) {
    if region.floors.contains(&region_id) {
        return;
    }

    region.floors.push(region_id);
}

#[allow(clippy::too_many_arguments)]
fn flood_region(
    tile_side: usize,
    entry: LevelStackEntry,
    level: u16,
    region_id: u16,
    tile: &OpenTile,
    regions: &mut [u16],
    distances: &mut [u16],
    stack: &mut Vec<LevelStackEntry>,
) -> bool {
    stack.clear();
    stack.push(entry);

    let span = &tile.cells[entry.cell_index as usize].spans[entry.span_index as usize];
    let area = tile.areas[span.tile_index];

    regions[entry.index as usize] = region_id;
    distances[entry.index as usize] = 0;

    let lev = level.saturating_sub(2);
    let mut expanded_any = false;

    while let Some(entry) = stack.pop() {
        let span = &tile.cells[entry.cell_index as usize].spans[entry.span_index as usize];

        let mut has_adjecant_region = false;
        for dir in 0..4 {
            let Some(span_index) = span.neighbours[dir] else {
                continue;
            };

            let other_cell_index = get_neighbour_index(tile_side, entry.cell_index as usize, dir);
            let other_span = &tile.cells[other_cell_index].spans[span_index as usize];
            let other_region = regions[other_span.tile_index];
            let other_area = tile.areas[other_span.tile_index];

            if other_area != area {
                continue;
            }

            if other_region != 0 && other_region != region_id {
                has_adjecant_region = true;
                break;
            }

            let next_dir = (dir + 1) & 0x3;
            if let Some(span_index) = other_span.neighbours[next_dir] {
                let other_span = &tile.cells
                    [get_neighbour_index(tile_side, other_cell_index, next_dir)]
                .spans[span_index as usize];
                let other_region = regions[other_span.tile_index];
                let other_area = tile.areas[other_span.tile_index];

                if other_area != area {
                    continue;
                }

                if other_region != 0 && other_region != region_id {
                    has_adjecant_region = true;
                    break;
                }
            }
        }

        if has_adjecant_region {
            regions[entry.index as usize] = 0;
            continue;
        }

        expanded_any = true;

        // Expand neighbours.
        for dir in 0..4 {
            let Some(span_index) = span.neighbours[dir] else {
                continue;
            };

            let other_cell_index = get_neighbour_index(tile_side, entry.cell_index as usize, dir);
            let other_span = &tile.cells[other_cell_index].spans[span_index as usize];
            let other_area = tile.areas[other_span.tile_index];
            if other_area != area {
                continue;
            }

            if tile.distances[other_span.tile_index] >= lev && regions[other_span.tile_index] == 0 {
                regions[other_span.tile_index] = region_id;
                distances[other_span.tile_index] = 0;
                stack.push(LevelStackEntry {
                    cell_index: other_cell_index as u32,
                    span_index: span_index.into(),
                    index: other_span.tile_index as i32,
                })
            }
        }
    }

    expanded_any
}
