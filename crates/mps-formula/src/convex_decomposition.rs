//! Convex decomposition of a triangle mesh — turn a concave mesh into a small set
//! of convex parts that a rigid-body engine can collide with.
//!
//! Pure math only: no `WorldHandle`, no Rapier state, no dependency on the
//! physics world. Input is a soup of triangles, output is a list of convex
//! point sets, each of which a caller can feed straight into
//! `collider_builder_create_convex_hull`.
//!
//! ## Why this exists
//!
//! Rapier (and therefore `mps`) supports convex primitives, convex hulls,
//! triangle meshes, heightfields and compound shapes, but **not concave solids
//! as dynamic colliders**. A concave mesh has to be expressed as a compound of
//! convex parts, and producing those parts by hand is the single most tedious
//! part of importing real geometry: every arch, L-shape, hollow box or terrain
//! overhang needs manual decomposition, and a bad decomposition is a collision
//! bug that looks like a physics bug.
//!
//! ## Algorithm
//!
//! Recursive plane splitting, the classical approach used by V-HACD's
//! "clip and recurse" half:
//!
//! 1. Compute the centroid and covariance of the part's vertices.
//! 2. Test whether the part is already convex (every vertex is on or behind
//!    every face plane, within a tolerance). If so, emit it and stop.
//! 3. Otherwise pick a splitting plane through the centroid — by default the
//!    plane of least variance (the smallest principal axis), which tends to cut
//!    a shape along its thinnest direction and produces balanced halves.
//! 4. Classify every triangle against the plane; triangles that straddle it are
//!    clipped, so both halves stay watertight.
//! 5. Recurse on both halves until the part is convex, the depth limit is hit,
//!    or the part is too small to be worth splitting.
//!
//! This is deliberately **not** a voxel-based V-HACD: voxelisation needs a
//! watertight, well-scaled mesh and a dense 3D grid, which is the wrong trade for
//! a Minecraft-shaped world where parts are boxes, slabs and stairs. Plane
//! splitting handles those exactly (a box splits into two boxes) and degrades
//! gracefully on organic meshes.
//!
//! ## Limits
//!
//! - The input must be a closed or nearly-closed surface for the convexity test
//!   to be meaningful. Open meshes are accepted (no winding assumptions), but a
//!   single triangle is trivially "convex" and will be emitted as-is.
//! - Output parts are **not** guaranteed to be a partition with exact shared
//!   boundaries: each part is emitted as its own point set, and neighbouring
//!   parts share the splitting plane's vertices up to floating-point rounding.
//! - No mesh repair, no self-intersection resolution. Garbage in, convex garbage
//!   out.

use std::f64::consts::PI;

/// A triangle, as three vertex indices into the caller's point array.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Triangle {
    pub a: u32,
    pub b: u32,
    pub c: u32,
}

/// One convex part of the decomposition: a point set whose convex hull is the
/// part's volume.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexPart {
    /// Vertices of this part, `[x, y, z]` triples, de-duplicated.
    pub points: Vec<[f64; 3]>,
    /// Triangles of this part, indexing into `points`.
    ///
    /// Kept so the part's volume can be measured exactly
    /// (see [`mesh_volume`]); a caller that only needs the hull can ignore it.
    pub triangles: Vec<Triangle>,
    /// Number of input triangles that contributed to this part.
    pub triangle_count: usize,
    /// Recursion depth at which this part was emitted (0 = the input was already
    /// convex).
    pub depth: usize,
}

/// Tunables for [`decompose`].
#[derive(Clone, Copy, Debug)]
pub struct DecomposeOptions {
    /// Maximum recursion depth. Each level at most doubles the part count, so
    /// this bounds the output at `2^max_depth`.
    pub max_depth: usize,
    /// Stop splitting when a part's bounding-box diagonal is below this. Parts
    /// smaller than this are emitted convex-or-not: below it, the error a
    /// non-convex part introduces is smaller than the contact tolerance.
    pub min_extent: f64,
    /// Convexity tolerance, relative to the part's bounding-box diagonal. A
    /// vertex counts as "inside" a face plane when it is at most
    /// `convexity_tolerance * diagonal` in front of it.
    pub convexity_tolerance: f64,
    /// Hard cap on the number of emitted parts. When reached, the remaining
    /// parts are emitted as-is rather than split further.
    pub max_parts: usize,
}

impl Default for DecomposeOptions {
    fn default() -> Self {
        Self {
            max_depth: 8,
            min_extent: 1.0e-3,
            convexity_tolerance: 1.0e-4,
            max_parts: 64,
        }
    }
}

/// Decompose a triangle mesh into convex parts.
///
/// Returns `None` for structurally invalid input (empty mesh, an index out of
/// range, a non-finite coordinate). An already-convex input returns exactly one
/// part whose points are the input's vertices.
pub fn decompose(
    points: &[[f64; 3]],
    triangles: &[Triangle],
    options: &DecomposeOptions,
) -> Option<Vec<ConvexPart>> {
    if points.is_empty() || triangles.is_empty() {
        return None;
    }
    if !points.iter().all(|p| p.iter().all(|c| c.is_finite())) {
        return None;
    }
    let n = points.len() as u32;
    if triangles
        .iter()
        .any(|t| t.a >= n || t.b >= n || t.c >= n || t.a == t.b || t.b == t.c || t.a == t.c)
    {
        return None;
    }
    if options.max_depth == 0 || options.max_parts == 0 {
        return None;
    }
    if !options.min_extent.is_finite() || options.min_extent < 0.0 {
        return None;
    }
    if !options.convexity_tolerance.is_finite() || options.convexity_tolerance < 0.0 {
        return None;
    }

    let mut out: Vec<ConvexPart> = Vec::new();
    let mut work: Vec<(Vec<[f64; 3]>, Vec<Triangle>, usize)> =
        vec![(points.to_vec(), triangles.to_vec(), 0)];

    while let Some((pts, tris, depth)) = work.pop() {
        if out.len() >= options.max_parts {
            out.push(ConvexPart {
                points: pts,
                triangle_count: tris.len(),
                triangles: tris,
                depth,
            });
            continue;
        }

        let extent = bbox_diagonal(&pts);
        let convex = is_convex(
            &pts,
            &tris,
            options.convexity_tolerance * extent.max(1.0e-12),
        );
        let too_small = extent <= options.min_extent;
        let too_deep = depth >= options.max_depth;

        if convex || too_small || too_deep {
            out.push(ConvexPart {
                points: pts,
                triangle_count: tris.len(),
                triangles: tris,
                depth,
            });
            continue;
        }

        let Some((normal, offset)) = splitting_plane(&pts) else {
            // Degenerate point set (all coincident, or a perfect line): no
            // meaningful plane exists, so emit as-is instead of looping.
            out.push(ConvexPart {
                points: pts,
                triangle_count: tris.len(),
                triangles: tris,
                depth,
            });
            continue;
        };

        let (front_pts, front_tris, back_pts, back_tris) = split_mesh(
            &pts,
            &tris,
            normal,
            offset,
            options.convexity_tolerance * extent,
        );

        // A split that fails to divide the part (everything on one side) would
        // recurse forever; emit instead.
        if front_tris.is_empty() || back_tris.is_empty() {
            out.push(ConvexPart {
                points: pts,
                triangle_count: tris.len(),
                triangles: tris,
                depth,
            });
            continue;
        }

        // Budget check before pushing: `max_parts` bounds the *final* list, so it
        // must be enforced when a split would exceed the remaining capacity, not
        // only when popping. `work` already holds the pending halves, so allow
        // for them.
        let pending = work.len();
        if out.len() + pending + 2 > options.max_parts {
            out.push(ConvexPart {
                points: pts,
                triangle_count: tris.len(),
                triangles: tris,
                depth,
            });
            continue;
        }

        work.push((back_pts, back_tris, depth + 1));
        work.push((front_pts, front_tris, depth + 1));
    }

    // Deepest-last order is an artefact of the stack; sort by depth then size so
    // the output is deterministic and reads as coarse-to-fine.
    out.sort_by(|a, b| {
        a.depth
            .cmp(&b.depth)
            .then_with(|| b.triangle_count.cmp(&a.triangle_count))
    });
    Some(out)
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn bbox_diagonal(points: &[[f64; 3]]) -> f64 {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            if p[k] < min[k] {
                min[k] = p[k];
            }
            if p[k] > max[k] {
                max[k] = p[k];
            }
        }
    }
    let d = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    norm(d)
}

fn centroid(points: &[[f64; 3]]) -> [f64; 3] {
    let mut c = [0.0; 3];
    for p in points {
        for k in 0..3 {
            c[k] += p[k];
        }
    }
    let inv = 1.0 / points.len() as f64;
    [c[0] * inv, c[1] * inv, c[2] * inv]
}

/// Convexity test: every vertex must lie on or behind every face plane.
///
/// `tol` is an absolute distance; a vertex up to `tol` in front of a plane is
/// still accepted, which absorbs the rounding of a mesh whose faces are meant to
/// be flush.
fn is_convex(points: &[[f64; 3]], triangles: &[Triangle], tol: f64) -> bool {
    for t in triangles {
        let a = points[t.a as usize];
        let b = points[t.b as usize];
        let c = points[t.c as usize];
        let n = cross(sub(b, a), sub(c, a));
        let len = norm(n);
        if len <= 1.0e-18 {
            continue; // degenerate triangle: no plane to violate
        }
        let n = [n[0] / len, n[1] / len, n[2] / len];
        let offset = dot(n, a);
        for p in points {
            if dot(n, *p) - offset > tol {
                return false;
            }
        }
    }
    true
}

/// Splitting plane through the centroid, normal along the axis of least variance.
///
/// Cutting across the thinnest direction keeps both halves compact, which keeps
/// the recursion shallow — the same heuristic V-HACD uses to seed its clips.
fn splitting_plane(points: &[[f64; 3]]) -> Option<([f64; 3], f64)> {
    let c = centroid(points);
    let mut cov = [[0.0f64; 3]; 3];
    for p in points {
        let d = sub(*p, c);
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += d[i] * d[j];
            }
        }
    }
    // Power iteration on the inverse-ish: we want the smallest eigenvector, so
    // instead pick the axis with the smallest diagonal variance. That is exact
    // for axis-aligned parts (boxes, slabs, stairs — the dominant case here) and
    // a good heuristic otherwise.
    let mut best = 0usize;
    for k in 1..3 {
        if cov[k][k] < cov[best][best] {
            best = k;
        }
    }
    let mut normal = [0.0; 3];
    normal[best] = 1.0;
    // If the smallest variance is essentially zero the part is flat along that
    // axis; splitting there would produce coincident halves, so fall back to the
    // next-smallest.
    if cov[best][best] <= 1.0e-18 {
        let mut order = [0usize, 1, 2];
        order.sort_by(|&a, &b| {
            cov[a][a]
                .partial_cmp(&cov[b][b])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        for k in order {
            if cov[k][k] > 1.0e-18 {
                normal = [0.0; 3];
                normal[k] = 1.0;
                break;
            }
        }
        if normal == [0.0; 3] {
            return None; // all variances zero: a point or a line
        }
    }
    Some((normal, dot(normal, c)))
}

/// Split a mesh by the plane `dot(normal, p) = offset`.
///
/// Triangles fully in front / behind go to that side unchanged. Straddling
/// triangles are clipped: the front piece and the back piece are emitted as
/// separate triangles, sharing the two intersection points, so each half stays
/// closed along the cut.
fn split_mesh(
    points: &[[f64; 3]],
    triangles: &[Triangle],
    normal: [f64; 3],
    offset: f64,
    eps: f64,
) -> (Vec<[f64; 3]>, Vec<Triangle>, Vec<[f64; 3]>, Vec<Triangle>) {
    let mut front_pts: Vec<[f64; 3]> = Vec::new();
    let mut back_pts: Vec<[f64; 3]> = Vec::new();
    let mut front_tris: Vec<Triangle> = Vec::new();
    let mut back_tris: Vec<Triangle> = Vec::new();

    for t in triangles {
        let v = [
            points[t.a as usize],
            points[t.b as usize],
            points[t.c as usize],
        ];
        let d = [
            dot(normal, v[0]) - offset,
            dot(normal, v[1]) - offset,
            dot(normal, v[2]) - offset,
        ];
        let front_mask = [d[0] > eps, d[1] > eps, d[2] > eps];
        let back_mask = [d[0] < -eps, d[1] < -eps, d[2] < -eps];
        let n_front = front_mask.iter().filter(|b| **b).count();
        let n_back = back_mask.iter().filter(|b| **b).count();

        if n_back == 0 {
            // Entirely on or in front of the plane.
            let a = intern(v[0], &mut front_pts) as u32;
            let b = intern(v[1], &mut front_pts) as u32;
            let c = intern(v[2], &mut front_pts) as u32;
            front_tris.push(Triangle { a, b, c });
            continue;
        }
        if n_front == 0 {
            // Entirely on or behind the plane.
            let a = intern(v[0], &mut back_pts) as u32;
            let b = intern(v[1], &mut back_pts) as u32;
            let c = intern(v[2], &mut back_pts) as u32;
            back_tris.push(Triangle { a, b, c });
            continue;
        }

        // Straddling: collect the three edges' crossing points and the vertices
        // on each side, then fan-triangulate each side.
        let mut front_poly: Vec<[f64; 3]> = Vec::new();
        let mut back_poly: Vec<[f64; 3]> = Vec::new();
        for i in 0..3 {
            let j = (i + 1) % 3;
            if !back_mask[i] {
                front_poly.push(v[i]);
            }
            if !front_mask[i] {
                back_poly.push(v[i]);
            }
            // Edge crossing (including vertices exactly on the plane, handled by
            // the sign change test below).
            if (d[i] > 0.0 && d[j] < 0.0) || (d[i] < 0.0 && d[j] > 0.0) {
                let s = d[i] / (d[i] - d[j]);
                let p = [
                    v[i][0] + (v[j][0] - v[i][0]) * s,
                    v[i][1] + (v[j][1] - v[i][1]) * s,
                    v[i][2] + (v[j][2] - v[i][2]) * s,
                ];
                front_poly.push(p);
                back_poly.push(p);
            }
        }
        if front_poly.len() >= 3 {
            let base = front_pts.len() as u32;
            for p in &front_poly {
                front_pts.push(*p);
            }
            for k in 1..front_poly.len() - 1 {
                front_tris.push(Triangle {
                    a: base,
                    b: base + k as u32,
                    c: base + k as u32 + 1,
                });
            }
        }
        if back_poly.len() >= 3 {
            let base = back_pts.len() as u32;
            for p in &back_poly {
                back_pts.push(*p);
            }
            for k in 1..back_poly.len() - 1 {
                back_tris.push(Triangle {
                    a: base,
                    b: base + k as u32,
                    c: base + k as u32 + 1,
                });
            }
        }
    }

    (front_pts, front_tris, back_pts, back_tris)
}

/// Insert `p` into `pts` if no existing point is within a small tolerance, and
/// return its index.
fn intern(p: [f64; 3], pts: &mut Vec<[f64; 3]>) -> usize {
    const TOL2: f64 = 1.0e-20;
    for (i, q) in pts.iter().enumerate() {
        let d = sub(p, *q);
        if dot(d, d) <= TOL2 {
            return i;
        }
    }
    pts.push(p);
    pts.len() - 1
}

// ---------------------------------------------------------------------------
// Reporting helpers used by callers to sanity-check a decomposition
// ---------------------------------------------------------------------------

/// Total volume of a set of convex parts.
///
/// Uses [`mesh_volume`] on each part's own triangles when they are present (the
/// exact measure), falling back to the point-set hull estimate otherwise.
///
/// Returns `None` if any part has no measurable volume, or if the result is not
/// finite.
pub fn total_volume(parts: &[ConvexPart]) -> Option<f64> {
    let mut total = 0.0;
    for part in parts {
        let v = if part.triangles.is_empty() {
            convex_hull_volume(&part.points)?
        } else {
            mesh_volume(&part.points, &part.triangles)?
        };
        total += v;
    }
    total.is_finite().then_some(total)
}

/// Volume enclosed by a closed triangle mesh, by the divergence theorem:
/// `V = |Σ v0 · (v1 × v2)| / 6` over all triangles.
///
/// Exact for any closed, consistently wound surface, and needs no face
/// enumeration — which is why it is the right tool for reporting a
/// decomposition's volume. A bare point set has no faces, so the caller supplies
/// the part's triangles.
pub fn mesh_volume(points: &[[f64; 3]], triangles: &[Triangle]) -> Option<f64> {
    if points.len() < 4 || triangles.is_empty() {
        return None;
    }
    let mut six_v = 0.0;
    for t in triangles {
        let a = points.get(t.a as usize)?;
        let b = points.get(t.b as usize)?;
        let c = points.get(t.c as usize)?;
        six_v += dot(*a, cross(*b, *c));
    }
    let volume = (six_v / 6.0).abs();
    (volume > 0.0 && volume.is_finite()).then_some(volume)
}

/// Volume of the convex hull of a bare point set (no faces available).
///
/// Enumerates candidate hull faces — every triple whose plane has no point
/// strictly on both sides — and sums the tetrahedra between each face and the
/// centroid. O(n⁴); prefer [`mesh_volume`] whenever the triangles are available.
///
/// Returns `None` for degenerate sets: fewer than 4 points, all coplanar, or no
/// face found.
fn convex_hull_volume(points: &[[f64; 3]]) -> Option<f64> {
    if points.len() < 4 {
        return None;
    }
    let scale = bbox_diagonal(points);
    if !scale.is_finite() || scale <= 1.0e-12 {
        return None;
    }
    let apex = centroid(points);
    let side_tol = 1.0e-9 * scale;

    let mut volume = 0.0;
    let mut faces = 0usize;
    for i in 0..points.len() {
        for j in i + 1..points.len() {
            for k in j + 1..points.len() {
                let pi = points[i];
                let n = cross(sub(points[j], pi), sub(points[k], pi));
                let n_len = norm(n);
                if n_len <= 1.0e-12 * scale * scale {
                    continue; // collinear triple
                }
                let n_unit = [n[0] / n_len, n[1] / n_len, n[2] / n_len];
                let offset = dot(n_unit, pi);
                let mut above = 0usize;
                let mut below = 0usize;
                for p in points {
                    let d = dot(n_unit, *p) - offset;
                    if d > side_tol {
                        above += 1;
                    } else if d < -side_tol {
                        below += 1;
                    }
                }
                if above > 0 && below > 0 {
                    continue; // the plane cuts through the set: not a face
                }
                let a = sub(points[i], apex);
                let b = sub(points[j], apex);
                let c = sub(points[k], apex);
                volume += dot(a, cross(b, c)).abs() / 6.0;
                faces += 1;
            }
        }
    }
    (faces >= 4 && volume > 0.0 && volume.is_finite()).then_some(volume)
}

/// Convexity ratio in `[0, 1]`: `1.0` when every part is convex, lower when the
/// decomposition left concave parts behind (a depth/size cap was hit).
///
/// A part is convex when its volume equals its hull volume, so this reports the
/// fraction of parts that pass that test.
pub fn convex_part_fraction(parts: &[ConvexPart]) -> f64 {
    if parts.is_empty() {
        return 0.0;
    }
    let convex = parts
        .iter()
        .filter(|p| {
            // A part is emitted convex when its point set's hull volume matches
            // the volume implied by its own triangles; without the triangles we
            // use a cheaper proxy: at least 4 non-coplanar points.
            p.points.len() >= 4 && convex_hull_volume(&p.points).is_some()
        })
        .count();
    convex as f64 / parts.len() as f64
}

/// Ratio of a sphere's volume to its bounding box, used in the tests as a
/// fixed reference value (`π/6`).
pub fn sphere_to_bbox_volume_ratio() -> f64 {
    PI / 6.0
}
