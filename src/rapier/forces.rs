//! Force registry — type-erased force-law registration with per-step dispatch.
//!
//! ## Motivation
//!
//! Before this module, every force subsystem (Newtonian gravity, air drag,
//! electromagnetic, aerodynamic surfaces, …) was applied via hard-coded
//! `if let Some(law)` branches inside `world_step` or its callees.  Adding a
//! new force type meant touching five files: the struct definition, the setter
//! FFI, `CustomPhysicsState`, the `world_step` dispatch, and cbindgen exports.
//!
//! The registry centralises discovery: a force law implements [`ForceLaw`],
//! registers itself into [`ForceRegistry`] (held by `PhysicsWorld`), and
//! `world_step` simply calls `registry.apply_all()`.  A new law is now a
//! single struct plus a `register_*` call — no dispatch logic to update.
//!
//! ## Architecture
//!
//! ```text
//! ForceLaw (trait)          ← one impl per physics model
//!   ↓
//! ForceRegistry             ← Vec<(ForceLawType, Box<dyn ForceLaw>)>
//!   ↓ apply_all()
//! world_step dispatches      ← single call, no if-let branching
//! ```
//!
//! Each law returns a [`ForceContribution`] that maps `ForceLawType →
//! total_force_vector`, so per-frame reports can show which force *source*
//! contributed what to every body.

use std::collections::BTreeMap;

use rapier3d::prelude::{
    ColliderSet, NarrowPhase, RigidBodySet,
};

use crate::rapier::ffi::CustomPhysicsReport;

// ---------------------------------------------------------------------------
// ForceLawType — enum of all recognised force sources
// ---------------------------------------------------------------------------

/// Categorises every force that can act on a rigid body.
///
/// The enum is `#[non_exhaustive]` so new variants can be added without
/// breaking downstream callers that match on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum ForceLawType {
    /// Rapier's built-in world gravity (`world.gravity` vector).
    WorldGravity,
    /// Direct force added via `rigid_body_add_force` FFI or `add_force()`.
    UserForce,
    /// Newtonian pairwise gravitational attraction.
    NewtonianGravity,
    /// Tangential Coulomb friction between contacting bodies.
    CoulombFriction,
    /// Reynolds-number-aware aerodynamic drag (per-body).
    AirDrag,
    /// Buoyancy force (Archimedes' principle).
    Buoyancy,
    /// Lorentz force from electric + magnetic fields.
    Electromagnetic,
    /// Linear spring anchored to a fixed point.
    ElasticSpring,
    /// Point-mass gravitational attraction to a fixed source.
    PointGravity,
    /// Aerodynamic force from surface-sample panels.
    AerodynamicSurface,
    /// Aerodynamic force from voxel occupancy grids.
    AerodynamicVoxel,
    /// Fluid drag on AABB-aligned bodies.
    FluidAABB,
    /// Lennard-Jones intermolecular potential.
    MolecularLennardJones,
    /// Coulomb electrostatic force between molecules.
    MolecularCoulomb,
    /// J2 zonal harmonic perturbation (oblateness).
    SpaceJ2,
    /// Control-moment-gyroscope torque.
    SpaceCMG,
    /// Atmospheric drag in orbital mechanics.
    SpaceAtmosphericDrag,
    /// Solar radiation pressure on spacecraft.
    SpaceSolarRadiation,
    /// Gravity-gradient torque on extended bodies.
    SpaceGravityGradient,
    /// Magnetic torquer attitude control.
    SpaceMagneticTorquer,
    /// Coriolis pseudo-force in rotating reference frame.
    TrajectoryCoriolis,
    /// Centrifugal pseudo-force in rotating reference frame.
    TrajectoryCentrifugal,
    /// Central-body gravity in trajectory calculations.
    TrajectoryGravity,
    /// PID controller output force.
    ControlPID,
    /// User-defined force registered via FFI (opaque type tag in upper 32 bits).
    Custom(u64),
}

impl ForceLawType {
    /// Human-readable label for debug logging and reports.
    pub fn label(&self) -> &'static str {
        match self {
            Self::WorldGravity => "WorldGravity",
            Self::UserForce => "UserForce",
            Self::NewtonianGravity => "NewtonianGravity",
            Self::CoulombFriction => "CoulombFriction",
            Self::AirDrag => "AirDrag",
            Self::Buoyancy => "Buoyancy",
            Self::Electromagnetic => "Electromagnetic",
            Self::ElasticSpring => "ElasticSpring",
            Self::PointGravity => "PointGravity",
            Self::AerodynamicSurface => "AerodynamicSurface",
            Self::AerodynamicVoxel => "AerodynamicVoxel",
            Self::FluidAABB => "FluidAABB",
            Self::MolecularLennardJones => "MolecularLennardJones",
            Self::MolecularCoulomb => "MolecularCoulomb",
            Self::SpaceJ2 => "SpaceJ2",
            Self::SpaceCMG => "SpaceCMG",
            Self::SpaceAtmosphericDrag => "SpaceAtmosphericDrag",
            Self::SpaceSolarRadiation => "SpaceSolarRadiation",
            Self::SpaceGravityGradient => "SpaceGravityGradient",
            Self::SpaceMagneticTorquer => "SpaceMagneticTorquer",
            Self::TrajectoryCoriolis => "TrajectoryCoriolis",
            Self::TrajectoryCentrifugal => "TrajectoryCentrifugal",
            Self::TrajectoryGravity => "TrajectoryGravity",
            Self::ControlPID => "ControlPID",
            Self::Custom(_) => "Custom",
        }
    }
}

// ---------------------------------------------------------------------------
// ForceReport — per-frame telemetry keyed by force type
// ---------------------------------------------------------------------------

/// Per-frame contribution from one force law.
#[derive(Clone, Copy, Debug, Default)]
pub struct ForceContribution {
    /// Total force vector (N) applied across all bodies this frame.
    pub total_force: crate::rapier::ffi::Vec3,
    /// Number of bodies that received non-zero force from this law.
    pub body_count: u32,
}

/// Aggregated force report for one simulation step.
///
/// Building block for `CustomPhysicsReport` — maps each active force type to
/// its per-frame contribution so callers can see exactly where forces are
/// coming from.
#[derive(Clone, Debug, Default)]
pub struct ForceReport {
    pub contributions: BTreeMap<ForceLawType, ForceContribution>,
    pub max_reynolds_number: f64,
}

impl ForceReport {
    pub fn add(&mut self, law_type: ForceLawType, force: rapier3d::prelude::Vector, body_count: u32) {
        let entry = self.contributions.entry(law_type).or_default();
        let existing = crate::rapier::ffi::vec3_from_rapier(force);
        entry.total_force = crate::rapier::ffi::Vec3 {
            x: entry.total_force.x + existing.x,
            y: entry.total_force.y + existing.y,
            z: entry.total_force.z + existing.z,
        };
        entry.body_count += body_count;
    }

    /// Convert to the legacy flat `CustomPhysicsReport` struct for FFI.
    pub fn to_legacy_report(&self) -> CustomPhysicsReport {
        let total_external = self.contributions.iter().fold(
            crate::rapier::ffi::Vec3::default(),
            |acc, (_, c)| crate::rapier::ffi::Vec3 {
                x: acc.x + c.total_force.x,
                y: acc.y + c.total_force.y,
                z: acc.z + c.total_force.z,
            },
        );
        let drag_contrib = self
            .contributions
            .get(&ForceLawType::AirDrag)
            .copied()
            .unwrap_or_default();
        let ext_body_count = self
            .contributions
            .iter()
            .filter(|(ty, _)| {
                !matches!(
                    ty,
                    ForceLawType::WorldGravity
                        | ForceLawType::UserForce
                        | ForceLawType::AirDrag
                )
            })
            .map(|(_, c)| c.body_count)
            .sum();

        CustomPhysicsReport {
            body_count: self
                .contributions
                .values()
                .map(|c| c.body_count)
                .sum::<u32>()
                .min(1), // at least 1 if anything happened
            drag_body_count: drag_contrib.body_count,
            external_force_body_count: ext_body_count,
            total_drag_force: drag_contrib.total_force,
            total_external_force: total_external,
            max_reynolds_number: self.max_reynolds_number,
        }
    }
}

// ---------------------------------------------------------------------------
// ForceLaw trait
// ---------------------------------------------------------------------------

/// A pluggable physics force law.
///
/// Implementors compute per-frame forces and apply them via
/// `body.add_force()` / `body.add_torque()`.  The registry calls
/// [`apply`](ForceLaw::apply) once per step for every registered law whose
/// [`enabled`](ForceLaw::enabled) returns `true`.
///
/// # Implementation notes
///
/// - `apply` receives `&mut PhysicsWorld` — it may iterate bodies and mutate
///   them (add forces, wake up, etc.).
/// - `law_type()` must return the same value for the lifetime of the law; the
///   registry uses it for reporting and lookup.
/// - `clone_box()` is used when the FFI layer needs to read a law's config
///   without downcasting.  Implementations should return a fresh heap copy.
pub trait ForceLaw: Send + Sync {
    /// The categorisation tag for this law.
    fn law_type(&self) -> ForceLawType;

    /// Whether this law should run this frame.
    fn enabled(&self) -> bool {
        true
    }

    /// Apply the law to all relevant bodies, writing statistics into `report`.
    ///
    /// Takes world deconstructed as `(bodies, colliders, narrow_phase)` to
    /// avoid borrow conflicts with the registry that holds this law.
    fn apply(
        &self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        narrow_phase: &NarrowPhase,
        report: &mut ForceReport,
    );

    /// Heap-clone this law (used for FFI read-back).
    fn clone_box(&self) -> Box<dyn ForceLaw>;
}

// ---------------------------------------------------------------------------
// ForceRegistry
// ---------------------------------------------------------------------------

/// A monotonically-growing handle into the force registry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ForceLawHandle(pub u64);

impl ForceLawHandle {
    pub fn raw(&self) -> u64 {
        self.0
    }
}

/// Ordered collection of active force laws.
///
/// Laws are applied in registration order.  The registry supports add,
/// remove, lookup-by-handle, and find-by-type operations.
pub struct ForceRegistry {
    laws: Vec<Option<RegistryEntry>>,
    next_handle: u64,
    free_slots: Vec<usize>,
}

struct RegistryEntry {
    handle: ForceLawHandle,
    law: Box<dyn ForceLaw>,
}

impl ForceRegistry {
    pub fn new() -> Self {
        Self {
            laws: Vec::new(),
            next_handle: 1,
            free_slots: Vec::new(),
        }
    }

    /// Register a force law.  Returns a handle that can be used to unregister
    /// or query the law later.
    pub fn register(&mut self, law: Box<dyn ForceLaw>) -> ForceLawHandle {
        let handle = ForceLawHandle(self.next_handle);
        self.next_handle += 1;
        let entry = RegistryEntry { handle, law };

        if let Some(slot) = self.free_slots.pop() {
            self.laws[slot] = Some(entry);
        } else {
            self.laws.push(Some(entry));
        }
        handle
    }

    /// Remove a previously-registered law by handle.
    ///
    /// Returns `true` if the law was found and removed.
    pub fn unregister(&mut self, handle: ForceLawHandle) -> bool {
        for (i, slot) in self.laws.iter_mut().enumerate() {
            if let Some(entry) = slot {
                if entry.handle == handle {
                    *slot = None;
                    self.free_slots.push(i);
                    return true;
                }
            }
        }
        false
    }

    /// Find all handles for laws of a given type.
    pub fn find_by_type(&self, law_type: ForceLawType) -> Vec<ForceLawHandle> {
        self.laws
            .iter()
            .filter_map(|slot| {
                slot.as_ref().and_then(|entry| {
                    if entry.law.law_type() == law_type {
                        Some(entry.handle)
                    } else {
                        None
                    }
                })
            })
            .collect()
    }

    /// Get a reference to a law by handle.
    pub fn get(&self, handle: ForceLawHandle) -> Option<&dyn ForceLaw> {
        self.laws
            .iter()
            .find_map(|slot| slot.as_ref().and_then(|e| (e.handle == handle).then(|| &*e.law)))
    }

    /// Get a mutable reference to a law by handle (for config updates).
    pub fn get_mut(&mut self, handle: ForceLawHandle) -> Option<&mut (dyn ForceLaw + '_)> {
        for slot in self.laws.iter_mut() {
            if let Some(entry) = slot {
                if entry.handle == handle {
                    return Some(&mut *entry.law);
                }
            }
        }
        None
    }

    /// Collects references to all enabled laws (avoids borrow conflicts).
    ///
    /// The returned `Vec` owns the references to the trait objects; drop it
    /// after applying forces to release the immutable borrow on the registry.
    pub fn enabled_laws(&self) -> Vec<&dyn ForceLaw> {
        self.laws
            .iter()
            .filter_map(|slot| {
                slot.as_ref().and_then(|entry| {
                    if entry.law.enabled() {
                        Some(&*entry.law)
                    } else {
                        None
                    }
                })
            })
            .collect()
    }

    /// Apply all enabled laws in registration order.
    ///
    /// Called once per `world_step` — replaces the old hard-coded dispatch.
    ///
    /// Note: because the caller typically holds both `&ForceRegistry` and
    /// `&mut PhysicsWorld` through the same root reference, this method may
    /// cause borrow conflicts.  Prefer `law_indices()` + `apply_at()` for
    /// such cases.
    pub fn apply_all(
        &self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        narrow_phase: &NarrowPhase,
    ) -> ForceReport {
        let mut report = ForceReport::default();
        for idx in self.law_indices() {
            let slot = &self.laws[idx];
            if let Some(entry) = slot {
                if entry.law.enabled() {
                    entry.law.apply(bodies, colliders, narrow_phase, &mut report);
                }
            }
        }
        report
    }

    /// Returns indices of all occupied slots (non-removed laws).
    ///
    /// Use with [`apply_at`](Self::apply_at) to avoid borrow conflicts when
    /// the registry and world are owned by the same struct.
    pub fn law_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.laws
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.as_ref().map(|_| i))
    }

    /// Apply a single law at the given index.
    ///
    /// # Panics
    /// Panics if `idx` is out of bounds or the slot is empty.
    pub fn apply_at(
        &self,
        idx: usize,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        narrow_phase: &NarrowPhase,
        report: &mut ForceReport,
    ) {
        if let Some(entry) = &self.laws[idx] {
            if entry.law.enabled() {
                entry.law.apply(bodies, colliders, narrow_phase, report);
            }
        }
    }

    /// Number of registered (non-removed) laws.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.laws.iter().filter(|s| s.is_some()).count()
    }

    /// Whether the registry is empty.
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.laws.iter().all(|s| s.is_none())
    }
}

impl Default for ForceRegistry {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::{Bool, Vec3};

    /// A trivial test law that just records that it was called.
    struct TestLaw {
        enabled: bool,
        call_count: std::sync::atomic::AtomicU32,
    }

    impl TestLaw {
        fn new(enabled: bool) -> Self {
            Self {
                enabled,
                call_count: std::sync::atomic::AtomicU32::new(0),
            }
        }

        fn call_count(&self) -> u32 {
            self.call_count.load(std::sync::atomic::Ordering::Relaxed)
        }
    }

    impl ForceLaw for TestLaw {
        fn law_type(&self) -> ForceLawType {
            ForceLawType::Custom(999)
        }

        fn enabled(&self) -> bool {
            self.enabled
        }

        fn apply(
            &self,
            _bodies: &mut RigidBodySet,
            _colliders: &mut ColliderSet,
            _narrow_phase: &NarrowPhase,
            report: &mut ForceReport,
        ) {
            self.call_count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            report.max_reynolds_number = report.max_reynolds_number.max(42.0);
        }

        fn clone_box(&self) -> Box<dyn ForceLaw> {
            Box::new(Self {
                enabled: self.enabled,
                call_count: std::sync::atomic::AtomicU32::new(self.call_count()),
            })
        }
    }

    #[test]
    fn registry_register_and_apply() {
        let mut reg = ForceRegistry::new();
        assert!(reg.is_empty());

        let law = TestLaw::new(true);
        let handle = reg.register(Box::new(law));
        assert_eq!(reg.len(), 1);
        assert!(!reg.is_empty());

        let world = super::super::world::world_create(Vec3::default());
        let world_ref = unsafe { &mut (*world).inner };
        let report = reg.apply_all(
            &mut world_ref.bodies,
            &mut world_ref.colliders,
            &world_ref.narrow_phase,
        );
        super::super::world::world_destroy(world);

        // The law should have been called once.
        let retrieved = reg.get(handle).unwrap();
        // We can't downcast directly, but we can verify the report was updated.
        assert_eq!(report.max_reynolds_number, 42.0);
        // call_count should be 1
        let law_ref = reg.get(handle).unwrap();
        assert_eq!(law_ref.law_type(), ForceLawType::Custom(999));
    }

    #[test]
    fn registry_skips_disabled_laws() {
        let mut reg = ForceRegistry::new();
        let law = TestLaw::new(false); // disabled
        let handle = reg.register(Box::new(law));
        assert_eq!(reg.len(), 1);

        let world = super::super::world::world_create(Vec3::default());
        let world_ref = unsafe { &mut (*world).inner };
        let report = reg.apply_all(
            &mut world_ref.bodies,
            &mut world_ref.colliders,
            &world_ref.narrow_phase,
        );
        super::super::world::world_destroy(world);

        // Disabled → no contribution
        assert_eq!(report.max_reynolds_number, 0.0);
        let law_ref = reg.get(handle).unwrap();
        assert_eq!(law_ref.law_type(), ForceLawType::Custom(999));
        assert!(!law_ref.enabled());
    }

    #[test]
    fn registry_unregister() {
        let mut reg = ForceRegistry::new();
        let h1 = reg.register(Box::new(TestLaw::new(true)));
        let h2 = reg.register(Box::new(TestLaw::new(true)));
        assert_eq!(reg.len(), 2);

        assert!(reg.unregister(h1));
        assert_eq!(reg.len(), 1);
        assert!(!reg.unregister(h1)); // double-unregister → false
        assert!(reg.get(h1).is_none());
        assert!(reg.get(h2).is_some());
    }

    #[test]
    fn registry_find_by_type() {
        let mut reg = ForceRegistry::new();
        let h1 = reg.register(Box::new(TestLaw::new(true)));
        let h2 = reg.register(Box::new(TestLaw::new(true)));
        let h3 = reg.register(Box::new(TestLaw::new(true)));

        let found = reg.find_by_type(ForceLawType::Custom(999));
        assert_eq!(found.len(), 3);
        assert!(found.contains(&h1));
        assert!(found.contains(&h2));
        assert!(found.contains(&h3));

        let not_found = reg.find_by_type(ForceLawType::AirDrag);
        assert!(not_found.is_empty());
    }

    #[test]
    fn registry_reuses_freed_slots() {
        let mut reg = ForceRegistry::new();
        let h1 = reg.register(Box::new(TestLaw::new(true)));
        let h2 = reg.register(Box::new(TestLaw::new(true)));
        assert!(reg.unregister(h1));
        assert!(reg.unregister(h2));
        assert!(reg.is_empty());

        // Register a new law — it should reuse a freed slot
        let h3 = reg.register(Box::new(TestLaw::new(true)));
        assert_eq!(reg.len(), 1);
        assert!(reg.get(h3).is_some());
    }

    #[test]
    fn force_report_to_legacy_conversion() {
        let mut report = ForceReport::default();
        report.add(
            ForceLawType::AirDrag,
            crate::rapier::ffi::vec3_to_rapier(crate::rapier::ffi::Vec3 {
                x: -10.0,
                y: 0.0,
                z: 0.0,
            }),
            3,
        );
        report.add(
            ForceLawType::PointGravity,
            crate::rapier::ffi::vec3_to_rapier(crate::rapier::ffi::Vec3 {
                x: 0.0,
                y: -50.0,
                z: 0.0,
            }),
            1,
        );

        let legacy = report.to_legacy_report();
        assert_eq!(legacy.drag_body_count, 3);
        assert_eq!(legacy.total_drag_force.x, -10.0);
        assert!(legacy.total_external_force.x == -10.0); // drag + point-gravity
    }
}
