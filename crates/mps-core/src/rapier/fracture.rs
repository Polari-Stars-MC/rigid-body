use std::slice;

use rapier3d::prelude::{ColliderBuilder, FixedJointBuilder, RigidBodyBuilder, Vector};

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, FractureEnergyReport, FractureFragmentDesc, FractureMaterial, FractureModeReport,
    FractureReplaceReport, GriffithReport, ImpulseJointHandleRaw, MinerDamageReport,
    RigidBodyHandleRaw, SnCurveReport, StressIntensityReport, WorldHandle,
    pack_impulse_joint_handle, pack_rigid_body_handle, unpack_rigid_body_handle, vec3_finite,
    vec3_to_rapier,
};

use crate::rapier::math::{finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;
const MAX_FRAGMENTS: u32 = 4096;

fn material_valid(material: FractureMaterial) -> bool {
    finite_positive(material.youngs_modulus)
        && material.poisson_ratio.is_finite()
        && material.poisson_ratio > -1.0
        && material.poisson_ratio < 0.5
        && finite_non_negative(material.fracture_toughness)
        && finite_non_negative(material.surface_energy)
        && finite_non_negative(material.density)
}

fn fragment_valid(fragment: FractureFragmentDesc) -> bool {
    vec3_finite(fragment.local_center)
        && vec3_finite(fragment.half_extents)
        && vec3_finite(fragment.initial_velocity)
        && fragment.half_extents.x > 0.0
        && fragment.half_extents.y > 0.0
        && fragment.half_extents.z > 0.0
        && finite_non_negative(fragment.density)
}

#[unsafe(no_mangle)]
pub extern "C" fn fracture_stress_intensity_factor(
    stress: f64,
    crack_length: f64,
    geometry_factor: f64,
    fracture_toughness: f64,
    out_report: *mut StressIntensityReport,
) -> Bool {
    if !stress.is_finite()
        || !finite_positive(crack_length)
        || !finite_positive(geometry_factor)
        || !finite_non_negative(fracture_toughness)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid stress intensity parameters");
        return Bool::FALSE;
    }
    let stress_intensity = geometry_factor * stress * (std::f64::consts::PI * crack_length).sqrt();
    let safety_factor = if stress_intensity.abs() > EPSILON {
        fracture_toughness / stress_intensity.abs()
    } else {
        f64::INFINITY
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "stress intensity output is null");
        return Bool::FALSE;
    };
    *out_report = StressIntensityReport {
        stress_intensity,
        critical: Bool::from(stress_intensity.abs() >= fracture_toughness),
        safety_factor,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fracture_griffith_criterion(
    stress: f64,
    crack_length: f64,
    material: FractureMaterial,
    out_report: *mut GriffithReport,
) -> Bool {
    if !stress.is_finite() || !finite_positive(crack_length) || !material_valid(material) {
        set_error(ERR_INVALID_ARGUMENT, "invalid Griffith fracture parameters");
        return Bool::FALSE;
    }
    let critical_stress = (2.0 * material.youngs_modulus * material.surface_energy
        / (std::f64::consts::PI * crack_length))
        .sqrt();
    let energy_release_rate =
        std::f64::consts::PI * crack_length * stress * stress / material.youngs_modulus;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Griffith output is null");
        return Bool::FALSE;
    };
    *out_report = GriffithReport {
        critical_stress,
        energy_release_rate,
        critical_energy_release_rate: 2.0 * material.surface_energy,
        will_fracture: Bool::from(stress.abs() >= critical_stress),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fracture_miner_damage(
    cycle_counts: *const f64,
    cycles_to_failure: *const f64,
    count: u32,
    out_report: *mut MinerDamageReport,
) -> Bool {
    if count == 0 || count > 1_000_000 {
        set_error(ERR_CAPACITY, "invalid fatigue bin count");
        return Bool::FALSE;
    }
    if cycle_counts.is_null() || cycles_to_failure.is_null() {
        set_error(ERR_NULL_POINTER, "fatigue arrays are null");
        return Bool::FALSE;
    }
    let cycle_counts = unsafe { slice::from_raw_parts(cycle_counts, count as usize) };
    let cycles_to_failure = unsafe { slice::from_raw_parts(cycles_to_failure, count as usize) };
    let mut damage = 0.0;
    for (&n, &nf) in cycle_counts.iter().zip(cycles_to_failure) {
        if !finite_non_negative(n) || !finite_positive(nf) {
            set_error(ERR_INVALID_ARGUMENT, "invalid fatigue cycle data");
            return Bool::FALSE;
        }
        damage += n / nf;
    }
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Miner damage output is null");
        return Bool::FALSE;
    };
    *out_report = MinerDamageReport {
        damage,
        remaining_life_fraction: (1.0 - damage).max(0.0),
        failed: Bool::from(damage >= 1.0),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fracture_sn_curve_life(
    stress_amplitude: f64,
    coefficient: f64,
    exponent: f64,
    endurance_limit: f64,
    out_report: *mut SnCurveReport,
) -> Bool {
    if !finite_positive(stress_amplitude)
        || !finite_positive(coefficient)
        || !finite_positive(exponent)
        || !finite_non_negative(endurance_limit)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid S-N curve parameters");
        return Bool::FALSE;
    }
    let cycles_to_failure = if endurance_limit > 0.0 && stress_amplitude <= endurance_limit {
        f64::INFINITY
    } else {
        coefficient / stress_amplitude.powf(exponent)
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "S-N curve output is null");
        return Bool::FALSE;
    };
    *out_report = SnCurveReport {
        cycles_to_failure,
        infinite_life: Bool::from(cycles_to_failure.is_infinite()),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fracture_energy_release(
    strain_energy: f64,
    new_surface_area: f64,
    surface_energy: f64,
    kinetic_energy: f64,
    out_report: *mut FractureEnergyReport,
) -> Bool {
    if !finite_non_negative(strain_energy)
        || !finite_positive(new_surface_area)
        || !finite_non_negative(surface_energy)
        || !finite_non_negative(kinetic_energy)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid fracture energy parameters");
        return Bool::FALSE;
    }
    let surface_energy_required = new_surface_area * surface_energy;
    let available_energy = strain_energy + kinetic_energy;
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "fracture energy output is null");
        return Bool::FALSE;
    };
    *out_report = FractureEnergyReport {
        available_energy,
        surface_energy_required,
        fragment_kinetic_energy: (available_energy - surface_energy_required).max(0.0),
        will_fracture: Bool::from(available_energy >= surface_energy_required),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn fracture_mode_from_stress(
    tensile_stress: f64,
    shear_stress: f64,
    compressive_stress: f64,
    out_report: *mut FractureModeReport,
) -> Bool {
    if !finite_non_negative(tensile_stress)
        || !finite_non_negative(shear_stress)
        || !finite_non_negative(compressive_stress)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid fracture mode stress values");
        return Bool::FALSE;
    }
    let (mode, driving_stress) =
        if tensile_stress >= shear_stress && tensile_stress >= compressive_stress {
            (1, tensile_stress)
        } else if shear_stress >= compressive_stress {
            (2, shear_stress)
        } else {
            (3, compressive_stress)
        };
    let total = tensile_stress + shear_stress + compressive_stress;
    let mixed_mode_ratio = if total > EPSILON {
        driving_stress / total
    } else {
        0.0
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "fracture mode output is null");
        return Bool::FALSE;
    };
    *out_report = FractureModeReport {
        mode,
        driving_stress,
        mixed_mode_ratio,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn world_replace_body_with_fracture_fragments(
    world: *mut WorldHandle,
    source_body: RigidBodyHandleRaw,
    fragments: *const FractureFragmentDesc,
    fragment_count: u32,
    connect_fragments: Bool,
    remove_source: Bool,
    out_body_handles: *mut RigidBodyHandleRaw,
    out_joint_handles: *mut ImpulseJointHandleRaw,
    capacity: u32,
    out_report: *mut FractureReplaceReport,
) -> Bool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "world is null");
        return Bool::FALSE;
    };
    if fragment_count == 0 || fragment_count > MAX_FRAGMENTS || capacity < fragment_count {
        set_error(ERR_CAPACITY, "invalid fracture fragment capacity");
        return Bool::FALSE;
    }
    if fragments.is_null() || out_body_handles.is_null() {
        set_error(ERR_NULL_POINTER, "fracture fragment pointers are null");
        return Bool::FALSE;
    }
    if connect_fragments.0 != 0 && out_joint_handles.is_null() {
        set_error(ERR_NULL_POINTER, "fracture joint output is null");
        return Bool::FALSE;
    }
    let Some(source) = world
        .inner
        .bodies
        .get(unpack_rigid_body_handle(source_body))
    else {
        set_error(ERR_NOT_FOUND, "source body was not found");
        return Bool::FALSE;
    };
    let source_pos = *source.position();
    let source_linvel = source.linvel();
    let source_angvel = source.angvel();
    let fragments = unsafe { slice::from_raw_parts(fragments, fragment_count as usize) };
    let out_bodies = unsafe { slice::from_raw_parts_mut(out_body_handles, capacity as usize) };
    let out_joints = if connect_fragments.0 != 0 {
        Some(unsafe { slice::from_raw_parts_mut(out_joint_handles, capacity as usize) })
    } else {
        None
    };

    let mut created_bodies = Vec::with_capacity(fragment_count as usize);
    for (index, fragment) in fragments.iter().copied().enumerate() {
        if !fragment_valid(fragment) {
            set_error(ERR_INVALID_ARGUMENT, "invalid fracture fragment");
            return Bool::FALSE;
        }
        let local = vec3_to_rapier(fragment.local_center);
        let world_center = source_pos * local;
        let body = RigidBodyBuilder::dynamic()
            .translation(world_center)
            .rotation(source_pos.rotation.to_scaled_axis())
            .linvel(source_linvel + vec3_to_rapier(fragment.initial_velocity))
            .angvel(source_angvel)
            .build();
        let body_handle = world.inner.bodies.insert(body);
        let collider = ColliderBuilder::cuboid(
            fragment.half_extents.x,
            fragment.half_extents.y,
            fragment.half_extents.z,
        )
        .density(fragment.density)
        .friction(fragment.friction.max(0.0))
        .restitution(fragment.restitution.max(0.0))
        .build();
        world
            .inner
            .colliders
            .insert_with_parent(collider, body_handle, &mut world.inner.bodies);
        let packed = pack_rigid_body_handle(body_handle);
        out_bodies[index] = packed;
        created_bodies.push((body_handle, local));
    }

    let mut joint_count = 0u32;
    if let Some(out_joints) = out_joints {
        for i in 1..created_bodies.len() {
            let (body1, local1) = created_bodies[i - 1];
            let (body2, local2) = created_bodies[i];
            let builder = FixedJointBuilder::new()
                .local_anchor1(Vector::ZERO)
                .local_anchor2(local1 - local2);
            let joint = world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), true);
            out_joints[i - 1] = pack_impulse_joint_handle(joint);
            joint_count += 1;
        }
    }

    if remove_source.0 != 0 {
        world.inner.bodies.remove(
            unpack_rigid_body_handle(source_body),
            &mut world.inner.islands,
            &mut world.inner.colliders,
            &mut world.inner.impulse_joints,
            &mut world.inner.multibody_joints,
            true,
        );
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = FractureReplaceReport {
            fragment_count,
            joint_count,
            removed_source: remove_source,
        };
    }
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::Vec3;
    use crate::rapier::world::world_create;

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[test]
    fn fracture_formulas_work() {
        let mut intensity = StressIntensityReport::default();
        assert_eq!(
            fracture_stress_intensity_factor(100.0, 0.01, 1.0, 10.0, &mut intensity),
            Bool::TRUE
        );
        assert!(intensity.stress_intensity > 0.0);
        assert_eq!(intensity.critical, Bool::TRUE);

        let material = FractureMaterial {
            youngs_modulus: 200.0e9,
            poisson_ratio: 0.3,
            fracture_toughness: 50.0e6,
            surface_energy: 10.0,
            density: 7850.0,
        };
        let mut griffith = GriffithReport::default();
        assert_eq!(
            fracture_griffith_criterion(1.0e6, 0.01, material, &mut griffith),
            Bool::TRUE
        );
        assert!(griffith.critical_stress > 0.0);
        assert_eq!(griffith.critical_energy_release_rate, 20.0);

        let cycles = [100.0, 50.0];
        let lives = [1000.0, 500.0];
        let mut damage = MinerDamageReport::default();
        assert_eq!(
            fracture_miner_damage(
                cycles.as_ptr(),
                lives.as_ptr(),
                cycles.len() as u32,
                &mut damage
            ),
            Bool::TRUE
        );
        assert!((damage.damage - 0.2).abs() < 1.0e-12);
        assert_eq!(damage.failed, Bool::FALSE);

        let mut sn = SnCurveReport::default();
        assert_eq!(
            fracture_sn_curve_life(50.0, 1.0e12, 3.0, 100.0, &mut sn),
            Bool::TRUE
        );
        assert_eq!(sn.infinite_life, Bool::TRUE);

        let mut energy = FractureEnergyReport::default();
        assert_eq!(
            fracture_energy_release(120.0, 10.0, 8.0, 0.0, &mut energy),
            Bool::TRUE
        );
        assert_eq!(energy.will_fracture, Bool::TRUE);
        assert_eq!(energy.fragment_kinetic_energy, 40.0);

        let mut mode = FractureModeReport::default();
        assert_eq!(
            fracture_mode_from_stress(1.0, 3.0, 2.0, &mut mode),
            Bool::TRUE
        );
        assert_eq!(mode.mode, 2);
    }

    #[test]
    fn fracture_replaces_body_with_connected_fragments() {
        let world = world_create(v3(0.0, -9.81, 0.0));
        assert!(!world.is_null());
        let world = unsafe { &mut *world };

        let source = world
            .inner
            .bodies
            .insert(RigidBodyBuilder::dynamic().build());
        world.inner.colliders.insert_with_parent(
            ColliderBuilder::cuboid(1.0, 1.0, 1.0).density(1.0).build(),
            source,
            &mut world.inner.bodies,
        );

        let fragments = [
            FractureFragmentDesc {
                local_center: v3(-0.5, 0.0, 0.0),
                half_extents: v3(0.25, 0.5, 0.5),
                initial_velocity: v3(-1.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
            FractureFragmentDesc {
                local_center: v3(0.5, 0.0, 0.0),
                half_extents: v3(0.25, 0.5, 0.5),
                initial_velocity: v3(1.0, 0.0, 0.0),
                density: 1.0,
                friction: 0.5,
                restitution: 0.1,
            },
        ];
        let mut bodies = [0; 2];
        let mut joints = [0; 2];
        let mut report = FractureReplaceReport::default();
        assert_eq!(
            world_replace_body_with_fracture_fragments(
                world,
                pack_rigid_body_handle(source),
                fragments.as_ptr(),
                fragments.len() as u32,
                Bool::TRUE,
                Bool::TRUE,
                bodies.as_mut_ptr(),
                joints.as_mut_ptr(),
                bodies.len() as u32,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.fragment_count, 2);
        assert_eq!(report.joint_count, 1);
        assert_eq!(report.removed_source, Bool::TRUE);
        assert!(bodies.iter().all(|handle| *handle != 0));
        assert_ne!(joints[0], 0);
        assert_eq!(world.inner.bodies.len(), 2);
    }
}
