use std::slice;

use rapier3d::math::{Pose, Rotation, Vector};
use rapier3d::prelude::{ColliderBuilder, SharedShape};
use smallvec::SmallVec;

use crate::rapier::ffi::{
    ColliderBuilderHandle, ColliderHandleRaw, MAX_OUTPUT_CAPACITY, NeuralActivation,
    NeuralBoundsDesc, QueryFilterDesc, WorldHandle, neural_activation_from_raw,
    pack_collider_handle, quat_finite, quat_to_rapier, query_filter_from_desc, vec3_finite,
};

const EPSILON: f64 = 1.0e-9;
const MAX_SAMPLE_RESOLUTION: u32 = 64;
const MAX_HIDDEN_WIDTH: u32 = 256;
const MAX_HIDDEN_LAYERS: u32 = 8;

struct NeuralWeights<'a> {
    values: &'a [f64],
    offset: usize,
}

impl<'a> NeuralWeights<'a> {
    fn new(values: &'a [f64]) -> Self {
        Self { values, offset: 0 }
    }

    fn take(&mut self) -> Option<f64> {
        let value = *self.values.get(self.offset)?;
        self.offset += 1;
        value.is_finite().then_some(value)
    }

    fn is_done(&self) -> bool {
        self.offset == self.values.len()
    }
}

fn activate(value: f64, activation: NeuralActivation) -> f64 {
    match activation {
        NeuralActivation::Relu => value.max(0.0),
        NeuralActivation::Tanh => value.tanh(),
        NeuralActivation::Sin => value.sin(),
        NeuralActivation::Linear => value,
    }
}

fn required_weight_count(hidden_width: usize, hidden_layers: usize) -> Option<usize> {
    if hidden_layers == 0 {
        return Some(4);
    }

    let input = hidden_width.checked_mul(3)?.checked_add(hidden_width)?;
    let hidden = hidden_layers.checked_sub(1)?.checked_mul(
        hidden_width
            .checked_mul(hidden_width)?
            .checked_add(hidden_width)?,
    )?;
    input
        .checked_add(hidden)?
        .checked_add(hidden_width)?
        .checked_add(1)
}

fn eval_layer(
    weights: &mut NeuralWeights<'_>,
    input: &[f64],
    output_width: usize,
    activation: NeuralActivation,
) -> Option<SmallVec<[f64; 32]>> {
    let mut output = SmallVec::<[f64; 32]>::with_capacity(output_width);
    for _ in 0..output_width {
        let mut value = 0.0;
        for input_value in input {
            value += weights.take()? * input_value;
        }
        value += weights.take()?;
        output.push(activate(value, activation));
    }
    Some(output)
}

fn eval_network(direction: Vector, desc: NeuralBoundsDesc, weights: &[f64]) -> Option<f64> {
    let hidden_width = desc.hidden_width as usize;
    let hidden_layers = desc.hidden_layers as usize;
    if hidden_layers == 0 {
        let mut reader = NeuralWeights::new(weights);
        let raw = reader.take()? * direction.x
            + reader.take()? * direction.y
            + reader.take()? * direction.z
            + reader.take()?;
        return reader.is_done().then_some(raw.max(0.0));
    }

    let mut reader = NeuralWeights::new(weights);
    let mut layer = eval_layer(
        &mut reader,
        &[direction.x, direction.y, direction.z],
        hidden_width,
        neural_activation_from_raw(desc.activation),
    )?;
    for _ in 1..hidden_layers {
        layer = eval_layer(
            &mut reader,
            &layer,
            hidden_width,
            neural_activation_from_raw(desc.activation),
        )?;
    }

    let mut raw = 0.0;
    for value in &layer {
        raw += reader.take()? * value;
    }
    raw += reader.take()?;
    reader.is_done().then_some(raw.max(0.0))
}

fn normalized(value: Vector) -> Option<Vector> {
    let len = value.length();
    (len > EPSILON).then_some(value / len)
}

fn push_obb_corners(
    points: &mut SmallVec<[Vector; 128]>,
    desc: NeuralBoundsDesc,
    rotation: Rotation,
) {
    for x in [-desc.half_extents.x, desc.half_extents.x] {
        for y in [-desc.half_extents.y, desc.half_extents.y] {
            for z in [-desc.half_extents.z, desc.half_extents.z] {
                points.push(rotation * Vector::new(x, y, z));
            }
        }
    }
}

fn sample_directions(resolution: u32) -> SmallVec<[Vector; 128]> {
    let rings = resolution.clamp(2, MAX_SAMPLE_RESOLUTION) as usize;
    let segments = rings * 2;
    let mut directions = SmallVec::<[Vector; 128]>::with_capacity((rings - 1) * segments + 6);

    directions.extend([
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(-1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.0, -1.0, 0.0),
        Vector::new(0.0, 0.0, 1.0),
        Vector::new(0.0, 0.0, -1.0),
    ]);

    for ring in 1..rings {
        let phi = std::f64::consts::PI * ring as f64 / rings as f64;
        let y = phi.cos();
        let ring_scale = phi.sin();
        for segment in 0..segments {
            let theta = std::f64::consts::TAU * segment as f64 / segments as f64;
            directions.push(Vector::new(
                ring_scale * theta.cos(),
                y,
                ring_scale * theta.sin(),
            ));
        }
    }

    directions
}

fn validate_desc(desc: NeuralBoundsDesc) -> Option<()> {
    if !vec3_finite(desc.center)
        || !vec3_finite(desc.half_extents)
        || !quat_finite(desc.rotation)
        || desc.half_extents.x <= 0.0
        || desc.half_extents.y <= 0.0
        || desc.half_extents.z <= 0.0
    {
        return None;
    }
    if desc.hidden_layers > MAX_HIDDEN_LAYERS || desc.hidden_width > MAX_HIDDEN_WIDTH {
        return None;
    }
    if desc.hidden_layers > 0 && desc.hidden_width == 0 {
        return None;
    }
    if !desc.output_scale.is_finite()
        || !desc.padding.is_finite()
        || desc.output_scale < 0.0
        || desc.padding < 0.0
    {
        return None;
    }
    Some(())
}

fn neural_points(desc: NeuralBoundsDesc, weights: &[f64]) -> Option<SmallVec<[Vector; 128]>> {
    validate_desc(desc)?;
    let required = required_weight_count(desc.hidden_width as usize, desc.hidden_layers as usize)?;
    if weights.len() != required || !weights.iter().all(|value| value.is_finite()) {
        return None;
    }

    let rotation = quat_to_rapier(desc.rotation);
    let mut points = SmallVec::<[Vector; 128]>::new();
    push_obb_corners(&mut points, desc, rotation);

    for direction in sample_directions(desc.sample_resolution) {
        let direction = normalized(direction)?;
        let base_radius = direction.x.abs() * desc.half_extents.x
            + direction.y.abs() * desc.half_extents.y
            + direction.z.abs() * desc.half_extents.z;
        let expansion = eval_network(direction, desc, weights)? * desc.output_scale + desc.padding;
        points.push(rotation * (direction * (base_radius + expansion)));
    }

    Some(points)
}

fn neural_shape(desc: NeuralBoundsDesc, weights: &[f64]) -> Option<(Pose, SharedShape)> {
    let points = neural_points(desc, weights)?;
    let shape = SharedShape::convex_hull(points.as_slice())?;
    Some((
        Pose::from_parts(
            Vector::new(desc.center.x, desc.center.y, desc.center.z),
            Rotation::IDENTITY,
        ),
        shape,
    ))
}

fn weights_from_raw(weights: *const f64, weight_count: u32) -> Option<&'static [f64]> {
    if weights.is_null() {
        return None;
    }
    if weight_count == 0 || weight_count > 1_048_576 {
        return None;
    }
    Some(unsafe { slice::from_raw_parts(weights, weight_count as usize) })
}

fn builder_from_neural(desc: NeuralBoundsDesc, weights: &[f64]) -> *mut ColliderBuilderHandle {
    let Some((pose, shape)) = neural_shape(desc, weights) else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(ColliderBuilderHandle {
        inner: ColliderBuilder::new(shape).position(pose),
    }))
}

fn intersect_neural_count(
    world: *const WorldHandle,
    desc: NeuralBoundsDesc,
    weights: &[f64],
    filter: QueryFilterDesc,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    let Some((pose, shape)) = neural_shape(desc, weights) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    query.intersect_shape(pose, shape.as_ref()).count() as u32
}

fn intersect_neural(
    world: *const WorldHandle,
    desc: NeuralBoundsDesc,
    weights: &[f64],
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    if out_handles.is_null() || capacity == 0 || capacity > MAX_OUTPUT_CAPACITY {
        return 0;
    }
    let Some((pose, shape)) = neural_shape(desc, weights) else {
        return 0;
    };

    let query = world.inner.broad_phase.as_query_pipeline(
        world.inner.narrow_phase.query_dispatcher(),
        &world.inner.bodies,
        &world.inner.colliders,
        query_filter_from_desc(filter),
    );

    let out = unsafe { slice::from_raw_parts_mut(out_handles, capacity as usize) };
    let mut written = 0usize;
    for (handle, _) in query.intersect_shape(pose, shape.as_ref()) {
        if written >= out.len() {
            break;
        }
        out[written] = pack_collider_handle(handle);
        written += 1;
    }

    written as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn neural_bounds_required_weight_count(
    hidden_width: u32,
    hidden_layers: u32,
) -> u32 {
    if hidden_layers > MAX_HIDDEN_LAYERS || hidden_width > MAX_HIDDEN_WIDTH {
        return 0;
    }
    if hidden_layers > 0 && hidden_width == 0 {
        return 0;
    }
    required_weight_count(hidden_width as usize, hidden_layers as usize)
        .and_then(|value| u32::try_from(value).ok())
        .unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn collider_builder_create_neural_bounds(
    desc: NeuralBoundsDesc,
    weights: *const f64,
    weight_count: u32,
) -> *mut ColliderBuilderHandle {
    let Some(weights) = weights_from_raw(weights, weight_count) else {
        return std::ptr::null_mut();
    };
    builder_from_neural(desc, weights)
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_neural_bounds_count(
    world: *const WorldHandle,
    desc: NeuralBoundsDesc,
    weights: *const f64,
    weight_count: u32,
    filter: QueryFilterDesc,
) -> u32 {
    let Some(weights) = weights_from_raw(weights, weight_count) else {
        return 0;
    };
    intersect_neural_count(world, desc, weights, filter)
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_neural_bounds_count_all(
    world: *const WorldHandle,
    desc: NeuralBoundsDesc,
    weights: *const f64,
    weight_count: u32,
) -> u32 {
    query_intersect_neural_bounds_count(
        world,
        desc,
        weights,
        weight_count,
        QueryFilterDesc::default(),
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_neural_bounds(
    world: *const WorldHandle,
    desc: NeuralBoundsDesc,
    weights: *const f64,
    weight_count: u32,
    filter: QueryFilterDesc,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    let Some(weights) = weights_from_raw(weights, weight_count) else {
        return 0;
    };
    intersect_neural(world, desc, weights, filter, out_handles, capacity)
}

#[unsafe(no_mangle)]
pub extern "C" fn query_intersect_neural_bounds_all(
    world: *const WorldHandle,
    desc: NeuralBoundsDesc,
    weights: *const f64,
    weight_count: u32,
    out_handles: *mut ColliderHandleRaw,
    capacity: u32,
) -> u32 {
    query_intersect_neural_bounds(
        world,
        desc,
        weights,
        weight_count,
        QueryFilterDesc::default(),
        out_handles,
        capacity,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::collider::{collider_builder_destroy, world_insert_collider};
    use crate::rapier::ffi::{Bool, Quat, Vec3};

    fn identity_rotation() -> Quat {
        Quat {
            i: 0.0,
            j: 0.0,
            k: 0.0,
            w: 1.0,
        }
    }

    fn desc() -> NeuralBoundsDesc {
        NeuralBoundsDesc {
            center: Vec3::default(),
            half_extents: Vec3 {
                x: 1.0,
                y: 0.5,
                z: 1.5,
            },
            rotation: identity_rotation(),
            sample_resolution: 8,
            hidden_width: 4,
            hidden_layers: 1,
            activation: NeuralActivation::Relu as u32,
            output_scale: 0.1,
            padding: 0.02,
        }
    }

    #[test]
    fn required_weight_count_matches_layout() {
        assert_eq!(neural_bounds_required_weight_count(0, 0), 4);
        assert_eq!(neural_bounds_required_weight_count(4, 1), 21);
        assert_eq!(neural_bounds_required_weight_count(4, 2), 41);
    }

    #[test]
    fn neural_bounds_builds_collider() {
        let desc = desc();
        let weights = vec![0.0; neural_bounds_required_weight_count(4, 1) as usize];
        let builder =
            collider_builder_create_neural_bounds(desc, weights.as_ptr(), weights.len() as u32);
        assert!(!builder.is_null());
        collider_builder_destroy(builder);
    }

    #[test]
    fn neural_bounds_query_intersects_inserted_collider() {
        let desc = desc();
        let weights = vec![0.0; neural_bounds_required_weight_count(4, 1) as usize];
        let builder = crate::rapier::collider::collider_builder_build(
            collider_builder_create_neural_bounds(desc, weights.as_ptr(), weights.len() as u32),
        );
        assert!(!builder.is_null());

        let world = crate::rapier::world::world_create(Vec3::default());
        let collider = world_insert_collider(world, builder);
        assert_ne!(collider, 0);
        crate::rapier::world::world_step(world, 1.0 / 60.0);

        let filter = QueryFilterDesc {
            use_groups: Bool::FALSE,
            ..QueryFilterDesc::default()
        };
        assert_eq!(
            query_intersect_neural_bounds_count(
                world,
                desc,
                weights.as_ptr(),
                weights.len() as u32,
                filter
            ),
            1
        );

        crate::rapier::world::world_destroy(world);
    }
}
