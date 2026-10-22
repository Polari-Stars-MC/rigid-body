//! Minecraft 地形快照解包 → 逐材质贪心合并 → 碰撞体构建器。
//!
//! 布局与 Java 侧 `TerrainChunkSnapshot.capture` 对应（小端，整包一块内存）：
//! ```text
//! i32 magic "TSNP", i32 sectionCount, i32 minSectionY,
//! i32 materialCount, [f64; 3] * materialCount,            密度 / 摩擦 / 弹性
//! i32 paletteCount, (i32 stateId, i32 materialIndex) * paletteCount,
//! i32 nonFullCount, [i32 stateId, i32 boxCount, [f64; 6] * boxCount] * nonFullCount,
//! sectionCount 段 MC 原样字节
//! ```
//!
//! 段字节就是 MC `PalettedContainer.write` 的输出：u8 bits + 调色板 + varint longCount + long[]。
//! bits==0 是单值段（只写一个 varint），1..8 是显式调色板，>=9 走全局调色板（流里没有调色板段）。
//!
//! 解压在主线程一次完成，逐格分流到"每材质一张整区块布尔网格"；多线程只做贪心合并，
//! 每条线程负责一种材质。材质键只看摩擦/弹性，密度不参与。

use std::collections::{HashMap, HashSet};

use rayon::prelude::*;

use rapier3d::math::{Pose, Vector};
use rapier3d::prelude::{ColliderBuilder, SharedShape};

use crate::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, ffi_guard, set_error};
use crate::rapier::ffi::ColliderBuilderHandle;

pub const MAGIC: i32 = 0x5453_4E50; // "TSNP"

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlockMaterial {
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Debug, Default)]
pub struct ChunkHeader {
    pub section_count: u32,
    pub min_section_y: i32,
    pub non_full_boxes: HashMap<i32, Vec<Aabb>>,
    pub sections_offset: usize,
}

struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn i32(&mut self) -> Result<i32, &'static str> {
        let end = self.pos.checked_add(4).ok_or("offset overflow")?;
        let bytes = self
            .data
            .get(self.pos..end)
            .ok_or("unexpected end of chunk snapshot")?;
        self.pos = end;
        Ok(i32::from_le_bytes(bytes.try_into().unwrap()))
    }

    fn f64(&mut self) -> Result<f64, &'static str> {
        let end = self.pos.checked_add(8).ok_or("offset overflow")?;
        let bytes = self
            .data
            .get(self.pos..end)
            .ok_or("unexpected end of chunk snapshot")?;
        self.pos = end;
        Ok(f64::from_le_bytes(bytes.try_into().unwrap()))
    }

    /// MC 的 varint（低位在前，7 位一组）
    fn varint(&mut self) -> Result<i32, &'static str> {
        let mut value = 0u32;
        for shift in (0..35).step_by(7) {
            let byte = *self.data.get(self.pos).ok_or("unexpected end of varint")?;
            self.pos += 1;
            value |= ((byte & 0x7F) as u32) << shift;
            if byte & 0x80 == 0 {
                return Ok(value as i32);
            }
        }
        Err("varint too long")
    }
}

/// 解 Java 侧快照头部：magic + 子区块信息 + 不完整方块表。
/// 材质表不再跟着区块走，改成 Java 在数据包加载后一次性塞进来的全局表（见 [`set_global_tables`]）。
pub fn read_header(data: &[u8]) -> Result<(ChunkHeader, MaterialTable), &'static str> {
    let mut cursor = Cursor::new(data);
    if cursor.i32()? != MAGIC {
        return Err("bad magic: not a Minecraft terrain snapshot");
    }

    let mut header = ChunkHeader {
        section_count: cursor.i32()? as u32,
        min_section_y: cursor.i32()?,
        ..Default::default()
    };

    // 材质表在全局共享内存里；没设过表就没法建碰撞体
    let materials = global_tables().ok_or("global material tables are not set")?;

    let non_full_count = cursor.i32()?;
    header
        .non_full_boxes
        .reserve(non_full_count.max(0) as usize);
    for _ in 0..non_full_count {
        let state_id = cursor.i32()?;
        let box_count = cursor.i32()?.max(0) as usize;
        let mut boxes = Vec::with_capacity(box_count);
        for _ in 0..box_count {
            boxes.push(Aabb {
                min: [cursor.f64()?, cursor.f64()?, cursor.f64()?],
                max: [cursor.f64()?, cursor.f64()?, cursor.f64()?],
            });
        }
        header.non_full_boxes.insert(state_id, boxes);
    }

    header.sections_offset = cursor.pos;
    Ok((header, materials))
}

/// 包内材质表：每 3 个 f64 一种材质（密度/摩擦/弹性）。
#[derive(Debug, Default, Clone)]
pub struct MaterialTable {
    entries: Vec<BlockMaterial>,
    /// 方块状态 id -> 材质下标（全局调色盘，Java 侧建好后整块推过来）
    palette: Vec<i32>,
}

impl MaterialTable {
    /// entries: 每 3 个 f64 一种材质；palette: 方块状态 id -> 材质下标。
    pub fn from_parts(entries: Vec<BlockMaterial>, palette: Vec<i32>) -> Self {
        Self { entries, palette }
    }

    pub fn get(&self, index: u32) -> Option<BlockMaterial> {
        self.entries.get(index as usize).copied()
    }

    /// 状态 id 直接查全局调色盘（区块里不再带配对表）
    pub fn material_of(&self, _header: &ChunkHeader, state_id: i32) -> Option<BlockMaterial> {
        if state_id < 0 {
            return None;
        }
        let index = *self.palette.get(state_id as usize)?;
        if index < 0 {
            return None; // 负数 = 空气/没有碰撞形状的方块，这一格不参与
        }
        self.get(index as u32)
    }
}

/// Java 传过来的全局表（数据包加载 / 重载时会被重新推送）。
static GLOBAL_TABLES: std::sync::Mutex<Option<MaterialTable>> = std::sync::Mutex::new(None);

fn global_tables() -> Option<MaterialTable> {
    GLOBAL_TABLES.lock().ok().and_then(|guard| guard.clone())
}

/// 指针布局（Java 侧 `TerrainMaterialTable.publishSharedTable` 写的）：
/// `+0 i32 材质条数`、`+4 i32 调色盘长度`、`+8 起 24×条数 字节（每 3 个 f64 一种材质）`、
/// 其后 `4×长度 字节（方块状态 id -> 材质下标）`。
///
/// Java 侧换表时会 free 掉旧的那块，所以这里必须**拷一份**回去，不能存指针。
///
/// # Safety
///
/// `pointer` 必须指向上面这个布局、且长度正确的一块可读内存。
pub unsafe fn set_global_tables(pointer: *const u8) -> bool {
    if pointer.is_null() {
        return false;
    }
    unsafe {
        let material_count =
            i32::from_le_bytes(std::slice::from_raw_parts(pointer, 4).try_into().unwrap()).max(0)
                as usize;
        let palette_len = i32::from_le_bytes(
            std::slice::from_raw_parts(pointer.add(4), 4)
                .try_into()
                .unwrap(),
        )
        .max(0) as usize;

        let mut entries = Vec::with_capacity(material_count);
        for i in 0..material_count {
            let base = pointer.add(8 + i * 24);
            let read = |offset: usize| {
                f64::from_le_bytes(
                    std::slice::from_raw_parts(base.add(offset), 8)
                        .try_into()
                        .unwrap(),
                )
            };
            entries.push(BlockMaterial {
                density: read(0),
                friction: read(8),
                restitution: read(16),
            });
        }

        let palette_base = pointer.add(8 + material_count * 24);
        let mut palette = Vec::with_capacity(palette_len);
        for i in 0..palette_len {
            let value = i32::from_le_bytes(
                std::slice::from_raw_parts(palette_base.add(i * 4), 4)
                    .try_into()
                    .unwrap(),
            );
            palette.push(value);
        }

        if let Ok(mut guard) = GLOBAL_TABLES.lock() {
            *guard = Some(MaterialTable::from_parts(entries, palette));
        }
    }
    true
}

/// 材质键（只看摩擦/弹性）
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MaterialKey {
    pub friction: u64,
    pub restitution: u64,
}

impl MaterialKey {
    pub fn of(material: BlockMaterial) -> Self {
        Self {
            friction: material.friction.to_bits(),
            restitution: material.restitution.to_bits(),
        }
    }

    pub fn to_pair(self) -> (f64, f64) {
        (
            f64::from_bits(self.friction),
            f64::from_bits(self.restitution),
        )
    }
}

#[derive(Debug, Clone)]
pub struct MaterialBoxes {
    pub friction: f64,
    pub restitution: f64,
    pub boxes: Vec<Aabb>,
}

/// 一段（16 高）的解包元数据。
struct SectionMeta {
    bits: usize,
    palette: Vec<i32>,
    longs_offset: usize,
    long_count: usize,
    /// 全局调色板：值是方块状态 id，流里没有调色板段
    global: bool,
}

impl SectionMeta {
    fn per_long(&self) -> usize {
        (64 / self.bits.max(1)).max(1)
    }

    fn mask(&self) -> u64 {
        if self.bits >= 64 {
            u64::MAX
        } else {
            (1u64 << self.bits) - 1
        }
    }

    /// 取这一格里调色板/全局的下标（解码越界返回 -1）。
    #[inline]
    fn value(&self, data: &[u8], cell: usize) -> i32 {
        if self.long_count == 0 {
            // 单值段：存储恒为 0，所有格子都指向调色板第 0 项（那一项存的是方块状态 id）
            return if self.palette.is_empty() { -1 } else { 0 };
        }
        let per_long = self.per_long();
        let long_index = cell / per_long;
        if long_index >= self.long_count {
            return -1;
        }
        let offset = (cell % per_long) * self.bits;
        let base = self.longs_offset + long_index * 8;
        let Some(bytes) = data.get(base..base + 8) else {
            return -1;
        };
        // MC 用 FriendlyByteBuf 写 long，netty 默认大端
        let word = u64::from_be_bytes(bytes.try_into().unwrap());
        ((word >> offset) & self.mask()) as i32
    }
}

/// 整区块的一张布尔网格：stride = x + z*16 + y*256，y 从 min_y 起
pub struct Grid {
    min_y: i32,
    height: usize,
    cells: Vec<bool>,
}

impl Grid {
    fn new(min_y: i32, height: usize) -> Self {
        Self {
            min_y,
            height,
            cells: vec![false; 16 * 16 * height],
        }
    }

    #[inline]
    fn set(&mut self, x: usize, y: i32, z: usize) {
        let row = (y - self.min_y) as usize;
        if row >= self.height {
            return;
        }
        self.cells[x + z * 16 + row * 256] = true;
    }
}

/// 逐段读头部：bits / 调色板 / long 段位置，并从调色板收集材质键。
fn read_sections(
    data: &[u8],
    header: &ChunkHeader,
    table: &MaterialTable,
) -> Result<(Vec<SectionMeta>, Vec<MaterialKey>), &'static str> {
    let mut cursor = Cursor::new(data);
    cursor.pos = header.sections_offset;
    let mut metas = Vec::with_capacity(header.section_count as usize);
    let mut keys: Vec<MaterialKey> = Vec::new();

    for _ in 0..header.section_count {
        let bits = *cursor.data.get(cursor.pos).ok_or("unexpected end (bits)")? as usize;
        cursor.pos += 1;

        let mut palette: Vec<i32> = Vec::new();
        if bits == 0 {
            // 单值段：只写一个 varint（就是那个值本身），没有 size 前缀
            palette.push(cursor.varint()?);
        } else if bits <= 8 {
            let palette_len = cursor.varint()?.max(0) as usize;
            palette.reserve(palette_len);
            for _ in 0..palette_len {
                palette.push(cursor.varint()?);
            }
        }

        // MC 的 Data::write = writeByte(bits) + palette.write + writeLongArray，
        // writeLongArray 自带 varint 长度（长度 0 也写），所以这里必须无条件读一个
        let long_count = cursor.varint()?.max(0) as usize;
        let longs_offset = cursor.pos;
        cursor.pos = cursor
            .pos
            .checked_add(long_count * 8)
            .ok_or("offset overflow")?;
        if cursor.pos > cursor.data.len() {
            return Err("unexpected end (longs)");
        }

        for state_id in &palette {
            if let Some(material) = table.material_of(header, *state_id) {
                let key = MaterialKey::of(material);
                if !keys.contains(&key) {
                    keys.push(key);
                }
            }
        }

        metas.push(SectionMeta {
            bits,
            palette,
            longs_offset,
            long_count,
            global: bits >= 9,
        });
    }

    Ok((metas, keys))
}

/// 主线程：解压整区块，逐格分流到各材质网格（每种材质一张整区块布尔数组）。
pub fn extract_grids(
    data: &[u8],
) -> Result<(Vec<MaterialKey>, Vec<Grid>, Vec<Vec<Aabb>>, i32, usize), &'static str> {
    let (header, table) = read_header(data)?;
    let min_y = header.min_section_y * 16;
    let height = header.section_count as usize * 16;
    let (metas, mut keys) = read_sections(data, &header, &table)?;

    // 全局调色板段没法从调色板里看出材质，只能先解出用到的状态 id（很少见）
    for meta in &metas {
        if !meta.global || meta.long_count == 0 {
            continue;
        }
        let mut seen: HashSet<i32> = HashSet::new();
        for cell in 0..4096usize {
            let value = meta.value(data, cell);
            if value >= 0 {
                seen.insert(value);
            }
        }
        for state_id in seen {
            if let Some(material) = table.material_of(&header, state_id) {
                let key = MaterialKey::of(material);
                if !keys.contains(&key) {
                    keys.push(key);
                }
            }
        }
    }

    let mut grids: Vec<Grid> = keys.iter().map(|_| Grid::new(min_y, height)).collect();
    // 不完整方块不进布尔网格：平移到格子位置后直接进同一个碰撞体
    let mut extra: Vec<Vec<Aabb>> = keys.iter().map(|_| Vec::new()).collect();

    // 唯一一次解压，逐格分流
    for (section, meta) in metas.iter().enumerate() {
        if meta.long_count == 0 && meta.palette.is_empty() {
            continue;
        }
        let section_y = min_y + (section as i32) * 16;

        let mut targets: Vec<usize> = vec![usize::MAX; meta.palette.len()];
        let mut shape_boxes: Vec<Option<&Vec<Aabb>>> = vec![None; meta.palette.len()];
        for (i, state_id) in meta.palette.iter().enumerate() {
            if let Some(material) = table.material_of(&header, *state_id) {
                let key = MaterialKey::of(material);
                if let Some(slot) = keys.iter().position(|k| *k == key) {
                    targets[i] = slot;
                }
            }
            shape_boxes[i] = header.non_full_boxes.get(state_id);
        }

        for cell in 0..4096usize {
            let raw = meta.value(data, cell);
            if raw < 0 {
                continue;
            }
            let (slot, local_boxes) = if meta.global {
                let Some(material) = table.material_of(&header, raw) else {
                    continue;
                };
                let key = MaterialKey::of(material);
                let Some(slot) = keys.iter().position(|k| *k == key) else {
                    continue;
                };
                (slot, header.non_full_boxes.get(&raw))
            } else {
                let index = raw as usize;
                if index >= targets.len() {
                    continue;
                }
                (targets[index], shape_boxes[index])
            };
            if slot == usize::MAX {
                continue;
            }

            let x = cell & 15;
            let z = (cell >> 4) & 15;
            let y = (cell >> 8) & 15;
            match local_boxes {
                Some(list) => {
                    let base_y = (section_y + y as i32) as f64;
                    for local in list {
                        extra[slot].push(Aabb {
                            min: [
                                x as f64 + local.min[0],
                                base_y + local.min[1],
                                z as f64 + local.min[2],
                            ],
                            max: [
                                x as f64 + local.max[0],
                                base_y + local.max[1],
                                z as f64 + local.max[2],
                            ],
                        });
                    }
                }
                None => grids[slot].set(x, section_y + y as i32, z),
            }
        }
    }

    Ok((keys, grids, extra, min_y, height))
}

/// 多线程：每种材质各自在整区块范围贪心合并成盒。
pub fn greedy_merge(
    keys: &[MaterialKey],
    grids: Vec<Grid>,
    extra: Vec<Vec<Aabb>>,
    min_y: i32,
    _height: usize,
) -> Vec<MaterialBoxes> {
    keys.par_iter()
        .zip(grids.into_par_iter())
        .zip(extra.into_par_iter())
        .map(|((key, grid), extra)| {
            let (friction, restitution) = key.to_pair();
            let mut boxes = merge_grid(&grid, min_y);
            boxes.extend(extra);
            MaterialBoxes {
                friction,
                restitution,
                boxes,
            }
        })
        .collect()
}

fn merge_grid(grid: &Grid, min_y: i32) -> Vec<Aabb> {
    let h = grid.height;
    let mut used = vec![false; 16 * 16 * h];
    let mut boxes = Vec::new();
    for y in 0..h {
        for z in 0..16 {
            for x in 0..16 {
                let start = x + z * 16 + y * 256;
                if !grid.cells[start] || used[start] {
                    continue;
                }
                let mut ex = x;
                while ex + 1 < 16
                    && grid.cells[(ex + 1) + z * 16 + y * 256]
                    && !used[(ex + 1) + z * 16 + y * 256]
                {
                    ex += 1;
                }
                let mut ez = z;
                'z: while ez + 1 < 16 {
                    for xx in x..=ex {
                        let i = xx + (ez + 1) * 16 + y * 256;
                        if !grid.cells[i] || used[i] {
                            break 'z;
                        }
                    }
                    ez += 1;
                }
                let mut ey = y;
                'y: while ey + 1 < h {
                    for zz in z..=ez {
                        for xx in x..=ex {
                            let i = xx + zz * 16 + (ey + 1) * 256;
                            if !grid.cells[i] || used[i] {
                                break 'y;
                            }
                        }
                    }
                    ey += 1;
                }
                for yy in y..=ey {
                    for zz in z..=ez {
                        for xx in x..=ex {
                            used[xx + zz * 16 + yy * 256] = true;
                        }
                    }
                }
                boxes.push(Aabb {
                    min: [x as f64, (min_y + y as i32) as f64, z as f64],
                    max: [
                        (ex + 1) as f64,
                        (min_y + ey as i32 + 1) as f64,
                        (ez + 1) as f64,
                    ],
                });
            }
        }
    }
    boxes
}

/// 由区块快照直接产出碰撞体构建器（不碰刚体），交给 Java 决定挂谁身上。
///
/// 满方块是贪心合并出的大盒，不完整方块是它自己的形状盒，两者进同一个 compound。
///
/// # 与上游 rapier 0.35 的语义差异（合并自 DeepSpace-Studio/mps）
///
/// 原实现给**每个子形状**挂自己的 `ShapeMaterial`（per-shape friction /
/// restitution）。rapier 0.36 移除了 `ShapeMaterial`：材质现在只存在于
/// collider 上、且按 manifold 生效（见 `ColliderMaterial`），compound 的子形状
/// 无法再各自带材质。
///
/// 这里保留原语义的方式是：**每个材质组返回一个独立的 ColliderBuilder**，
/// 由调用方把它们挂成同一个刚体的多个 collider。如果一个 compound 里混了多种
/// 材质，调用 `build_terrain_collider` 的旧路径只能拿到第一组的系数——所以要
/// 逐材质正确，必须改用 [`build_terrain_colliders`]。
///
/// 单材质区块（绝大多数情况）行为与原实现完全一致。
pub fn build_terrain_collider(data: &[u8]) -> Result<ColliderBuilder, &'static str> {
    let mut builders = build_terrain_colliders(data)?;
    // 第一组是最大的一组（`greedy_merge` 按材质分组，顺序稳定）。
    Ok(builders.remove(0))
}

/// 按材质分组产出碰撞体：每个材质一个 `ColliderBuilder`。
///
/// 这是 rapier 0.36 下保留「不同方块不同摩擦」的唯一正确方式——材质挂在
/// collider 上，所以不同材质必须是不同的 collider。返回的顺序与
/// `greedy_merge` 一致（先出现的材质组在前），空输入报错。
pub fn build_terrain_colliders(data: &[u8]) -> Result<Vec<ColliderBuilder>, &'static str> {
    let (keys, grids, extra, min_y, height) = extract_grids(data)?;
    let materials = greedy_merge(&keys, grids, extra, min_y, height);

    let mut out: Vec<ColliderBuilder> = Vec::new();
    for material in materials {
        let mut shapes: Vec<(Pose, SharedShape)> = Vec::new();
        for b in material.boxes {
            let half = Vector::new(
                (b.max[0] - b.min[0]) * 0.5,
                (b.max[1] - b.min[1]) * 0.5,
                (b.max[2] - b.min[2]) * 0.5,
            );
            if half.x <= 0.0 || half.y <= 0.0 || half.z <= 0.0 {
                continue;
            }
            let center = Vector::new(
                (b.min[0] + b.max[0]) * 0.5,
                (b.min[1] + b.max[1]) * 0.5,
                (b.min[2] + b.max[2]) * 0.5,
            );
            shapes.push((Pose::from_translation(center), SharedShape::cuboid(half.x, half.y, half.z)));
        }
        if shapes.is_empty() {
            continue;
        }
        // 材质挂在 collider 上（0.36 的模型），而不是每个子形状上。
        out.push(
            ColliderBuilder::new(SharedShape::compound(shapes))
                .friction(material.friction)
                .restitution(material.restitution),
        );
    }

    if out.is_empty() {
        return Err("chunk snapshot has no collision shapes");
    }
    Ok(out)
}

/// 由 Java 侧保留内存里的快照直接建碰撞体构建器（不挂刚体）。
///
/// # Safety
///
/// `data` 必须指向至少 `len` 个可读字节，且在一次调用期间保持有效。
#[unsafe(no_mangle)]
pub extern "C" fn terrain_collider_builder_create(
    data: *const u8,
    len: usize,
) -> *mut ColliderBuilderHandle {
    ffi_guard(std::ptr::null_mut(), || {
        if data.is_null() || len == 0 {
            set_error(ERR_NULL_POINTER, "terrain snapshot is null");
            return std::ptr::null_mut();
        }
        let bytes = unsafe { std::slice::from_raw_parts(data, len) };
        let Ok(builder) = build_terrain_collider(bytes) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid terrain snapshot");
            return std::ptr::null_mut();
        };
        Box::into_raw(Box::new(ColliderBuilderHandle {
            inner: builder,
            voxel_source: None,
        }))
    })
}
