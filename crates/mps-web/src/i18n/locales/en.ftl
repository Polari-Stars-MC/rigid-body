# MPS Motion Physics System — English translations
# Fluent .ftl format (https://projectfluent.org/)

# ---- Navigation ----
nav-home = Home
nav-quickstart = Quickstart
nav-architecture = Architecture
nav-gravity = Gravity
nav-integrators = Integrators
nav-formula = Formula
nav-voxel = Voxel
nav-soft-body = Soft Body
nav-events = Events
nav-arena = Arena
nav-batch = Batch Colliders
nav-cosmos = Cosmos
nav-cosmos-class = Functionality
nav-jni = JNI
nav-ffm = FFM
nav-api = API
nav-more = More ▾

# ── Short labels for galaxy planet buttons (≈60px diameter, 2-4 chars ideal) ──
nav-planet-home = Home
nav-planet-quickstart = Start
nav-planet-architecture = Arch
nav-planet-gravity = Gravity
nav-planet-integrators = Integr
nav-planet-formula = Formula
nav-planet-api = API
nav-planet-voxel = Voxel
nav-planet-events = Events
nav-planet-arena = Arena
nav-planet-batch = Batch
nav-planet-cosmos = Cosmos
nav-planet-jni = JNI
nav-planet-ffm = FFM

# ---- Language switcher ----
lang-zh = 中文
lang-en = English

# ---- Home page ----
home-hero-tag = / MPS PHYSICS OBSERVATORY
home-hero-title = Motion Physics System (Meters Per Second)
home-hero-desc = High-precision Rust physics engine based on { $rapier }. Full API exposed via C FFI ({ $ffi } functions) and Java JNI ({ $jni } methods). { $tests } tests, { $gravity } gravity models, { $integrators } symplectic integrators, zero-copy shared-memory Arena, { $modules } formula modules and { $bodies } celestial bodies.
home-cta-quickstart = Quickstart
home-cta-api = API Reference

home-stat-tests = Tests
home-stat-formula-fns = Formula Fns
home-stat-formula-modules = Formula Modules
home-stat-celestial = Celestial Bodies

home-section-directory = Module Directory
home-section-formula-modules = Formula Modules ({ $count })
home-section-key-features = Key Features
home-section-architecture = Architecture

home-mod-core-title = Core Engine
home-mod-core-desc = World, rigid bodies, colliders, joints, queries, controllers
home-mod-cosmos-title = Cosmos Rigid Body
home-mod-cosmos-desc = CosmosWorld, Verlet orbit integration, n-body gravity, perturbations
home-mod-physics-title = Physics Systems
home-mod-physics-desc = Gravity, terrain, force registry, events, aerodynamics, fluid
home-mod-formula-title = Domain Formulas
home-mod-formula-desc = 107 pure formula modules (557 functions) — spaceflight, astrophysics, nuclear, relativity, quantum, and more.
home-mod-integration-title = Integration
home-mod-integration-desc = Arena shared memory, JNI/FFM bindings, Java ecosystem
home-mod-reference-title = Reference
home-mod-reference-desc = Full API tables, precision & performance, optimization guide

home-feat-gravity-title = High-Precision Gravity
home-feat-gravity-desc = Spherical harmonics (EGM2008 8×8), ellipsoidal gravity, J2-J6 zonal harmonics, quadrupole tensor. Auto-selects optimal model by orbital altitude.
home-feat-integrators-title = Symplectic Integrators
home-feat-integrators-desc = Leapfrog, Yoshida 4th order, Forest-Ruth 8th order. Kahan compensation: 15→30 significant digits. Post-Newtonian 1PN+2PN corrections.
home-feat-celestial-title = Built-in Celestials
home-feat-celestial-desc = 10 solar system bodies with precision data (JPL DE441). Earth EGM2008, Moon LP165 + 12 Mascons (GRAIL), Mars Mars50c.
home-feat-terrain-title = Terrain Gravity
home-feat-terrain-desc = Polyhedral gravity (Werner-Scheeres), DEM terrain mass distribution, FFT acceleration. Lunar Mascon model prevents low-orbit decay.
home-feat-registry-title = ForceRegistry
home-feat-registry-desc = Typed force registry. Any force implementing ForceLaw trait auto-dispatches; world step auto-aggregates reports, no manual dispatch needed.
home-feat-jni-title = JNI + Shared Memory
home-feat-jni-desc = Java 21 JNI full binding ({ $count } methods). Shared-memory Arena (DirectByteBuffer) for zero-JNI read/write, only 1 world_step call per frame.

home-callout = All formulas live in a standalone crate { $crate } — pure Rust, no Rapier or WorldHandle dependency.

# ---- Formula mini-stat labels (home module grid) ----
formula-cat-spaceflight = Spaceflight
formula-cat-nuclear = Nuclear
formula-cat-mechanics = Mechanics
formula-cat-astrophysics = Astrophysics
formula-cat-relativity = Relativity
formula-cat-quantum = Quantum
formula-cat-electromagnetism = Electromagnetism
formula-cat-fluid = Fluid Dynamics

# ---- Quickstart page ----
quickstart-tag = / QUICKSTART
quickstart-title = Quickstart
quickstart-desc = Set up the MPS physics engine development environment from scratch.
quickstart-step1-title = Install Rust Toolchain
quickstart-step1-desc = Install Rust 1.75+ and cargo. Recommend using rustup.
quickstart-step2-title = Clone the Repository
quickstart-step2-desc = git clone and enter the rigid-body directory.
quickstart-step3-title = Build the Core Library
quickstart-step3-desc = cargo build --workspace compiles all crates.
quickstart-step4-title = Run Tests
quickstart-step4-desc = cargo test --workspace executes { $tests } integration tests.
quickstart-step5-title = Generate C Headers
quickstart-step5-desc = cargo build -p mps-core triggers cbindgen to generate rigid_body.h.

# ---- Architecture page ----
arch-tag = // MPS
arch-title = Architecture Overview
arch-desc = Crate layering and per-frame data flow from Java top to Rapier bottom.
arch-stack-title = Crate stack
arch-stack-lead = Top-to-bottom: Java bindings → C ABI → mps-core → Rapier3D-f64.
arch-stack-diagram = Java 21 JNI / Java 25 FFM
  └─ Rust C ABI (720 functions)
       ├─ mps-formula  — { $modules } pure formula modules (557 functions)
       ├─ mps-core     — physics engine + Rapier wrapper (World, bodies, colliders, queries, events)
       ├─ mps-cosmos   — cosmos rigid body (separate world, Verlet orbit integration)
       ├─ mps-jni      — JNI bindings ({ $methods } methods, incl. cosmos batch)
       ├─ mps-ffm      — FFM metadata
       └─ mps-test     — integration tests (incl. cosmos 19)
arch-layers-title = Layer responsibilities
arch-layer-formula-title = mps-formula
arch-layer-formula-desc = { $modules } pure formula modules (spaceflight / astrophysics / nuclear etc.). No WorldHandle dependency; callable from anywhere.
arch-layer-core-title = mps-core
arch-layer-core-desc = Rapier3D-f64 wrapper + shared Arena + C-ABI. World, bodies, colliders, joints, queries, force registry, event system.
arch-layer-cosmos-title = mps-cosmos
arch-layer-cosmos-desc = CosmosWorld: separate physics domain. Verlet orbit integration + n-body mutual gravity. Does not share World with mps-core.
arch-layer-jni-title = mps-jni
arch-layer-jni-desc = Java 21 JNI full bindings, { $methods } methods (including cosmos batch API).
arch-layer-test-title = mps-test
arch-layer-test-desc = { $tests } integration tests, also serves as the module-mirror crate verifying source structure.
arch-layer-ffm-title = mps-ffm
arch-layer-ffm-desc = Java 25 Foreign Function & Memory API metadata; gradually replacing JNI.
arch-flow-title = Per-frame data flow
arch-flow-lead = Java frame-renderer side, 5-step MPS loop:
arch-flow-step-1 = Java reads Arena data in the DirectByteBuffer, updates body target positions / forces.
arch-flow-step-2 = Exactly one world_step(world, dt) FFI call — the only sync point across the boundary.
arch-flow-step-3 = mps-core dispatches internally: ForceRegistry aggregates forces → Rapier collision solver → joint solver → event dispatch.
arch-flow-step-4 = Arena layout: position / velocity / force-accumulator SoA writes go directly back into the DirectByteBuffer.
arch-flow-step-5 = Java side reads Arena zero-copy, renders / extracts physics. Events drained from ring buffer.
arch-tenets-title = Design tenets
arch-tenet-zero-copy = Zero-copy: Java and Rust swap body state via shared memory Arena; no per-object JNI read/write.
arch-tenet-formula-pure = Pure formula layer: mps-formula doesn't depend on WorldHandle or Rapier; pure-function callable in any context.
arch-tenet-ffi-stable = Stable C ABI: cbindgen generates rigid_body.h; every breaking change must bump the banner version.
arch-build-title = Build pipeline
arch-build-cbindgen = mps-core's build.rs triggers cbindgen at build time, emitting pub C-ABI types/functions into rigid_body.h.
arch-build-xtask = xtask auto-counts TEST_COUNT / JNI_METHOD_COUNT / CORE_FFI_COUNT and refreshes mps-web/src/metrics.rs.

# ---- Gravity page ----
grav-tag = // mps-core
grav-title = Gravity Models
grav-desc = The gravity stack in mps-core is a pluggable ForceLaw: every model is registered through world_set_*_gravity and seeds the same Newtonian baseline, so you can swap fidelity without touching the solver.
grav-models-title = Model catalogue
grav-models-lead = Seven field models span the full fidelity range — from a two-body point mass to GRAIL-grade lunar Mascons. Pick by orbital regime; the engine can also auto-select.
grav-col-name = Model
grav-col-use = Best for
grav-col-cost = Cost
grav-row-newton = Deep-space cruise and two-body propagation where analytical gravity is enough.
grav-row-sh = Low-Earth-orbit precision; EGM2008 spherical-harmonic expansion (8×8 by default).
grav-row-ellipsoid = Fast first-order Earth oblateness for region-scale sims that don't need full SH.
grav-row-zonal = Sectoral / zonal terms from J2 up to J6; mid-altitude satellite dynamics.
grav-row-quad = Arbitrary quadrupole tensor for elongated or irregular primaries.
grav-row-poly = Asteroid and irregular-body surfaces via the Werner–Scheeres polyhedron method.
grav-row-mascon = Low lunar orbit using 12 GRAIL mass-anomaly tiles; prevents low-pass orbit decay.
grav-bodies-title = Built-in bodies
grav-body-earth-title = Earth (EGM2008)
grav-body-earth-desc = 8×8 spherical-harmonic truncation at Jason / CHAMP / GRACE orbit-simulation grade.
grav-body-moon-title = Moon (LP165 + Mascon)
grav-body-moon-desc = LP165 spherical harmonics plus 12 GRAIL Mascon tiles; keeps low lunar orbit from crashing into terrain.
grav-body-mars-title = Mars (Mars50c)
grav-body-mars-desc = 50th-order harmonic truncation, with polar oblateness and a seasonal CO₂-cap approximation.
grav-body-sun-title = Sun (point source)
grav-body-sun-desc = JPL DE441 solar GM = 1.32712440018e20 m³/s² — the origin of every planetary perturbation.
grav-auto-title = Auto model selection
grav-auto-lead = When enabled, the engine switches the active field model from the body's current altitude and reference radius.
grav-auto-note = Below ~200 km it uses spherical harmonics + Mascon; mid orbit falls back to J2–J6; high / deep-space uses the point mass. The cross-validation path (world_set_cross_validate_gravity) diffs two models per step for regression checks.
grav-api-title = C ABI
grav-api-desc = All gravity models register through the mps-core C-ABI and share one ForceLawType::NewtonianGravity slot, so only one law is active at a time:
grav-bodies-grid-title = Body catalogue

# ---- Integrators page ----
int-tag = // mps-core
int-title = Symplectic Integrators
int-desc = Leapfrog, Yoshida-4 and Forest–Ruth 8th-order symplectic steppers, plus Kahan-compensated variants and post-Newtonian corrections — all in mps-formula::integrators.
int-why-title = Why symplectic
int-why-lead = Classical RK4 bleeds energy on long-duration orbital arcs and the orbit slowly spirals.
int-why-body = Symplectic steppers preserve the Hamiltonian structure: the energy error stays bounded and oscillates instead of growing, so 10⁴-year propagations stay closed. Every stepper ships a _kahan variant that lifts f64 precision from ~15 to ~30 significant digits.
int-catalog-title = Integrator catalogue
int-col-name = Stepper
int-col-order = Order
int-col-notes = Notes
int-row-leapfrog = Kick-Drift-Kick; 2nd-order, time-reversible; the everyday orbit workhorse.
int-row-yoshida4 = 4th-order symmetric split (3 substeps); energy error ~O(dt⁴).
int-row-forest-ruth = 8th-order Forest–Ruth class; deep-space high-precision planet approximation.
int-kahan-title = Kahan error compensation
int-kahan-lead = Every symplectic stepper has a _kahan variant; the Kahan summation compensates f64 truncation.
int-kahan-li-1 = Standard form stays stable to ~15 significant digits.
int-kahan-li-2 = The Kahan variant raises that to ~30 significant digits.
int-kahan-li-3 = Use only when long-double-equivalent precision is required — about 2× slower.
int-kahan-note = Kahan keeps a running compensation term c = (sum - t) - y for every addition.
int-pn-title = Post-Newtonian corrections
int-pn-lead = Near a large-GM body the relativistic term is no longer negligible.
int-pn-li-1pn = post_newtonian_1pn — first-order post-Newtonian (Mercury perihelion advance).
int-pn-li-2pn = post_newtonian_2pn — second-order post-Newtonian (Lense–Thirring on high-eccentricity orbits).
int-pn-li-full = post_newtonian_full — all PN terms combined.
int-adaptive-title = Adaptive step control
int-adaptive-desc = adaptive_step_size + step_accepted let the stepper grow or shrink dt from a local error estimate:
int-diag-title = Numerical diagnostics
int-diag-energy = Specific energy ε = v²/2 − GM/r — must stay conserved.
int-diag-am = Specific angular momentum h = r × v — strictly preserved by symplectic methods.
int-diag-kepler = keplerian_elements() converts to six elements to monitor semi-major-axis drift.

# ---- Formula page ----
form-tag = // mps-formula
form-title = Formula Modules
form-desc = A standalone pure-Rust crate: 107 formula modules and 557 public functions spanning spaceflight to quantum physics, with zero Rapier or WorldHandle dependency.
form-intro-pure = All formulas live in the standalone crate mps-formula — pure Rust, no Rapier or WorldHandle dependency.
form-mod-kepler = kepler.rs — Kepler equation iterative solver / element conversion
form-mod-dynamics = dynamics.rs — orbit dynamics / two-centre approximation
form-mod-perturbation = perturbation.rs — third-body perturbation / drag / solar pressure
form-mod-propulsion = propulsion.rs — Tsiolkovsky / propellant budget
form-mod-rotation = rotation.rs — rigid-body attitude dynamics / quaternions
form-mod-thermal = thermal.rs — thermal balance / solar irradiance
form-mod-debris = debris.rs — debris cloud evolution / collision probability
form-mod-gnss = gnss.rs — GNSS pseudorange / multi-frequency
form-mod-trajectory = trajectory.rs — Lambert problem / transfer orbit
form-mod-astrophysics = astrophysics.rs — stellar structure / luminosity function
form-mod-stellar = stellar.rs — main sequence / H-R diagram / evolution
form-mod-galactic = galactic_dynamics.rs — rotation curves / density-wave theory
form-mod-cosmology = cosmology.rs — FLRW metric / Hubble flow
form-mod-helio = heliophysics.rs — solar wind / corona parameters
form-mod-high-energy = high_energy_astro.rs — blackbody / synchrotron / inverse Compton
form-mod-celestial = celestial_data.rs — JPL DE441 10-body precision parameters
form-mod-planetary = planetary_science.rs — planetary interiors / tides
form-mod-mechanics = material_mechanics.rs — stress / strain / yield
form-mod-material = material_mechanics (subset) — elasticity tensor / anisotropy
form-mod-biomech = biomechanics.rs — joint torque / muscle models
form-mod-control = control_theory.rs — PID / LQR / state-space
form-mod-chaos = chaos.rs — Lorenz / Rössler attractors
form-mod-topology = topology.rs — homotopy / topological invariants
form-mod-softbody = softbody.rs — soft body / spring-mass
form-mod-relativity = relativity.rs — Lorentz transform / time dilation
form-mod-transmission = transmission.rs — signal transmission / link budget
form-mod-quantum = quantum.rs — Schrödinger equation / Hamiltonian / spin
form-mod-em = electromagnetism.rs — Maxwell equations / Poynting vector
form-mod-nuclear = nuclear.rs — decay / half-life / cross section
form-mod-fluid = fluid.rs — Navier–Stokes / Euler / Bernoulli
form-mod-plasma = plasma.rs — plasma frequency / MHD
form-mod-superfluidity = superfluidity.rs — superfluid helium / two-fluid model
form-mod-continuum = continuum.rs — continuum mechanics
form-mod-physchem-title = Physics + Chemistry
form-mod-physchem = physchem.rs — reaction kinetics / equilibrium constant
form-mod-thermo = thermodynamics.rs — entropy / enthalpy / engine efficiency
form-mod-molecular = molecular.rs — ideal gas / molecular dynamics
form-mod-wave-optics = wave_optics.rs — interference / diffraction / polarization
form-mod-acoustics = acoustics.rs — acoustics / Doppler
form-mod-aero = aerodynamics.rs — lift / drag / airfoils
form-support-title = Supporting modules
form-support-intro = A few shared primitives inside mps-formula that all 107 modules reuse:
form-call-title = Calling from Java
form-call-desc = All formula functions are exposed via C ABI with no WorldHandle dependency:

# ---- Voxel page ----
vox-tag = // mps-core
vox-title = Voxel System
vox-desc = Dense voxel grid → collider build → polyhedron-gravity bridge, all inside mps-core.
vox-overview-title = Overview
vox-overview-lead = VoxelGrid + build_voxel_collider is the end-to-end pipeline from voxel map to simulation.
vox-overview-body = Good for lunar-lander surface representation, irregular-terrain elevation maps, and proximity fly-by sims that need fast near-field collision.
vox-grid-title = VoxelGrid data model
vox-grid-desc = rapier::voxel::VoxelGrid<'a> borrows cells / dims / origin / scale:
vox-grid-li-1 = cells: 8- or 16-bit occupancy (0 = empty, non-0 = solid)
vox-grid-li-2 = dims + origin + scale: world-frame dimensions / origin / metres-per-voxel
vox-grid-li-3 = Borrow semantics: grid doesn't own cells; can come directly from mmap or Java ByteBuffer
vox-build-title = build_voxel_collider
vox-build-lead = The entry function turns the grid into a Rapier collider and inserts it into the World.
vox-build-note = Internally MarchingCubes + convex decomposition: roughly 1 m³ → 1 convex hull; complexity O(N log N).
vox-terrain-title = Terrain-gravity bridge
vox-terrain-desc = Voxel grids can be fed directly into rapier::terrain_gravity polyhedron gravity:
vox-terrain-li-direct = terrain_gravity_direct — brute O(N²) vertex sum; baseline / validation.
vox-terrain-li-fft = terrain_gravity_fft — frequency-domain convolution O(N log N); preferred for large grids.
vox-terrain-li-poly = polyhedron_gravity — Werner–Scheeres raw API.
vox-cases-title = Typical use cases
vox-case-lunar-title = Lunar surface landing
vox-case-lunar-desc = 100 m lunar grid + 12 Mascon blocks; simulate lander gravity-gradient disturbance on descent.
vox-case-terrain-title = Terrain region mapping
vox-case-terrain-desc = DEM elevation → VoxelGrid → simulation; pairs well with land-flight trajectory.
vox-case-proximity-title = Near-field collision performance
vox-case-proximity-desc = Because convex decomposition yields a bounded collider count, n-body mutual collision is O(N·log M).

# ---- Events page ----
evt-tag = // mps-core
evt-title = Event System
evt-desc = Collision and contact-force events with three dispatch modes and a C callback ABI — built on a lockless SPSC ring buffer.
evt-types-title = Event types
evt-types-lead = Two physics event records are emitted by world_step and drained through the event rings.
evt-col-type = Type
evt-col-fields = Fields
evt-row-collision = started, collider1, collider2, sensor, removed
evt-row-contact = collider1, collider2, total_force, max_force_direction, max_force_magnitude
evt-modes-title = Dispatch modes
evt-mode-poll-title = Poll
evt-mode-poll-desc = Events are pushed to a background Vec; the application drains them on demand between frames.
evt-mode-callback-title = Callback
evt-mode-callback-desc = A direct unsafe extern "C" callback fires in real time; keep the callback free of heavy Rust work.
evt-mode-both-title = Both
evt-mode-both-desc = Events are both queued and dispatched to the callback — used for tests and hot replay.
evt-ring-title = Ring buffer
evt-ring-desc = The high-frequency path uses an SPSC EventRing<T>, lockless single-producer / single-consumer.
evt-ring-li-1 = MAX_EVENT_RECORDS = 16 384 — capacity of one ring.
evt-ring-li-2 = Producer: the Rapier callback thread inside world_step.
evt-ring-li-3 = Consumer: the Java render-frame thread; drain() returns the latest N.
evt-forces-title = ForceLaw dispatch
evt-forces-lead = rapier::forces::ForceRegistry takes typed registrations; every step aggregates a ForceReport.
evt-force-coulomb = CoulombFrictionLaw — static / dynamic friction with a velocity threshold.
evt-force-airdrag = AirDragLaw — Reynolds-adaptive Stokes / Newton regime switching.
evt-force-external = ExternalForceLaw — buoyancy + electromagnetic + spring + gravity composite.
evt-force-newton = NewtonGravityLaw — body-to-body N-body pairwise attraction with scalable G.
evt-force-custom = Custom ForceLaw trait — any force struct is scheduled automatically after register().
evt-forces-note = ForceReport carries per-body total force / torque / type labels; handy for a debug overlay.
evt-abi-title = C callback ABI
evt-abi-desc = Collision / contact-force callback signatures are exposed as unsafe extern "C":

# ---- Arena page ----
arena-tag = // mps-core
arena-title = Shared Memory Arena
arena-desc = A DirectByteBuffer zero-copy bridge between Java and the Rust rigid-body state — one world_step call per frame, no per-object JNI.
arena-why-title = Why an Arena
arena-why-lead = Classic JNI per-object get/setField costs 10 ms+ per frame at 1000 bodies.
arena-why-body = The shared Arena lays out rigid-body SoA data in one direct byte buffer; Java reads and writes without crossing the Rust boundary. Only one world_step FFI call happens per frame.
arena-layout-title = Memory layout
arena-layout-desc = rapier::shared_arena header + SoA slot array + recycle ring.
arena-mods-title = Sub-modules
arena-mod-header = header.rs — magic / version / capacity validation.
arena-mod-layout = layout.rs — BodySlot field offset constants.
arena-mod-ring = ring.rs — SPSC event ring reuse.
arena-mod-holes = holes.rs — O(1) recycling of freed slots.
arena-flow-title = Per-frame protocol
arena-flow-step-1 = Java side: arena_write_* sets this frame's target positions / forces / torques.
arena-flow-step-2 = A single world_step(world, dt) FFI call triggers the simulation.
arena-flow-step-3 = arena_read_* reads updated positions / velocities straight from the DirectByteBuffer.
arena-flow-note = No GetFieldID / CallObjectMethod anywhere; the JNI reference table stays empty, so there is no GC risk.
arena-java-title = Java side example
arena-java-desc = DirectByteBuffer + JNI is the standard Java 21 idiom:

# ---- Cosmos page ----
cosmos-tag = // mps-cosmos
cosmos-title = Cosmos Rigid Body
cosmos-desc = CosmosWorld — a separate orbital-scale physics domain, independent of mps-core's Rapier World.
cosmos-what-title = What Cosmos is
cosmos-what-lead = mps-cosmos provides an orbital physics domain that does not share a World with mps-core.
cosmos-what-body = Unlike mps-core's Rapier stepping, CosmosWorld uses Verlet integration (verlet_step) and pairwise n-body mutual gravity (n_body_acceleration_reduce), with an SIMD far-field monopole path (far_field_monopole_simd) for the high-count case. It is built for long-duration orbital evolution without rigid-body contact solving.
cosmos-mods-title = Sub-modules
cosmos-mod-kepler-title = kepler.rs
cosmos-mod-kepler-desc = Kepler equation iteration / six-element ↔ Cartesian conversion.
cosmos-mod-dynamics-title = dynamics.rs
cosmos-mod-dynamics-desc = Two-centre approximation / third-body perturbation / drag.
cosmos-mod-perturbation-title = perturbation.rs
cosmos-mod-perturbation-desc = Tidal / Yarkovsky / Poynting–Robertson.
cosmos-mod-propulsion-title = propulsion.rs
cosmos-mod-propulsion-desc = Tsiolkovsky equation / specific impulse / propellant budget.
cosmos-mod-rotation-title = rotation.rs
cosmos-mod-rotation-desc = Quaternion attitude dynamics / spin stabilization.
cosmos-mod-thermal-title = thermal.rs
cosmos-mod-thermal-desc = Thermal balance / solar irradiance / eclipse cycle.
cosmos-mod-debris-title = debris.rs
cosmos-mod-debris-desc = Debris cloud evolution / Kessler syndrome model.
cosmos-mod-gnss-title = gnss.rs
cosmos-mod-gnss-desc = L1 / L5 pseudorange / multi-frequency ionosphere-free solution.
cosmos-nbody-title = n-body mutual gravity
cosmos-nbody-lead = Every CosmosWorld frame computes GM · m / r² pairwise across all bodies, O(N²).
cosmos-nbody-note = For N ≤ 20 the direct pairwise is faster than a BH tree; this is exactly the scale of a solar-system-level simulation.
cosmos-bodies-title = Body catalogue ({ $count })
cosmos-bodies-desc = JPL DE441 provides the GM, orbital elements, and phase of 10 primary bodies; injected into CosmosWorld at init.
cosmos-jni-title = JNI integration
cosmos-jni-desc = mps-jni exposes cosmosWorld* methods; bulk state read-back goes through cosmosWorldDynamicBodySnapshot / cosmosWorldDynamicBodySnapshotCount over the Arena (no per-body JNI).
cosmos-arena-title = Shared-memory Arena (same as core)
cosmos-arena-desc = Cosmos ships its own SharedArena (magic COSMAREN) — the orbital-scale twin of mps-core's shared_arena. Java reads/writes body state through a DirectByteBuffer with no per-body JNI, and cosmosWorldGetArenaDirectByteBuffer parallels worldGetArenaDirectByteBuffer.
cosmos-class-title = Functionality by category
cosmos-class-lead = Cosmos reuses several mps-core capabilities at orbital scale. Each group below maps a capability to its source module.
cosmos-class-world-title = World & Bodies
cosmos-class-world-desc = world.rs / bodies.rs — CosmosWorld, central body + sun, the 10-body DE441 catalogue, and batch insertion (cosmos_world_add_n_body).
cosmos-class-gravity-title = Gravity
cosmos-class-gravity-desc = gravity.rs — Newtonian point-mass and pairwise n-body mutual gravity (n_body_acceleration_reduce) with a SIMD far-field monopole path (far_field_monopole_simd).
cosmos-class-integrator-title = Integrators
cosmos-class-integrator-desc = integrator.rs — verlet_step, n-body acceleration reduce, and high-order + Kahan-compensated steppers (advance_highorder_kahan) for long arcs.
cosmos-class-orbit-title = Orbit & Diagnostics
cosmos-class-orbit-desc = orbit.rs / orbit_diagnostics.rs — six-element conversion, Hill radius (cosmos_hill_radius_for), and state snapshots for monitoring drift.
cosmos-class-flight-title = Flight & Perturbation
cosmos-class-flight-desc = flight/* (dynamics, trim, stability) and perturbation/* — two-centre dynamics, third-body / solar-pressure / drag perturbations, and attitude trim.
cosmos-class-arena-title = Shared-memory Arena
cosmos-class-arena-desc = arena.rs + ffi.rs — SharedArena (COSMAREN) with a seqlock-guarded body slot; exposed to Java as a DirectByteBuffer via cosmos_world_get_shared_arena_address / _size, mirroring core's arena bridge.

# ---- Batch collider page ----
batch-tag = // Box3D
batch-title = Batch Collider Pipeline
batch-desc = Box3D-style batch insertion + same-material merge + physics-feel presets; one ColliderSet::insert amortises N shapes.
batch-pipeline-title = Pipeline
batch-pipeline-lead = The upper layer pushes ColliderRequest records into a ColliderBatch manager, which merges compatible static shapes into a compound and inserts in one shot.
batch-step-1-title = Build requests
batch-step-1-desc = Populate a ColliderRequest array — shape, pose, material, collision groups, parent body.
batch-step-2-title = Choose preset
batch-step-2-desc = Pass a Box3DPreset to set default friction/restitution/density/erosion/damping/CCD substeps/solver iterations.
batch-step-3-title = Merge and insert
batch-step-3-desc = Same-material static shapes merge into a single compound collider; different materials or dynamic shapes are grouped and inserted separately.
batch-step-4-title = Return handles
batch-step-4-desc = Returns ColliderHandleRaw for each generated collider; the caller can use them for queries and further operations.
batch-request-title = ColliderRequest fields
batch-request-lead = Each request is a #[repr(C)] flat struct; build a contiguous array and pass (ptr, count) to the FFI.
batch-col-field = Field
batch-col-type = Type
batch-col-desc = Description
batch-col-scenario = Scenario
batch-col-result = Result
batch-field-shape = Shape descriptor (shape_type + a/b/c/d floats)
batch-field-translation = Local translation relative to the merged collider origin
batch-field-rotation = Unit quaternion local rotation
batch-field-friction = Coulomb friction coefficient (>= 0)
batch-field-restitution = Coefficient of restitution (>= 0, typically < 1)
batch-field-density = Mass density (>= 0; ignored for static shapes)
batch-field-collision-groups = Collision group memberships bitmask
batch-field-solver-groups = Solver group memberships bitmask
batch-field-body-parent = When non-zero, attaches collider to the given rigid body
batch-field-is-sensor = When non-zero, the collider is a sensor (no collision response)
batch-field-erosion-margin = Erosion margin; only meaningful for round shapes; 0 = no erosion
batch-preset-title = Box3D physics-feel presets
batch-preset-lead = Three built-in presets cover common sandbox physics scenarios; also available via FFI constructors.
batch-preset-default-title = Default
batch-preset-default-desc = Balanced — moderate friction, slight bounce, gentle damping. Good for general sandbox use.
batch-preset-sticky-title = Sticky
batch-preset-sticky-desc = No bounce, high friction. Good for ground/walls and static geometry.
batch-preset-bouncy-title = Bouncy
batch-preset-bouncy-desc = Low friction, high restitution, more CCD substeps. Good for bouncing/stacking demos.
batch-merge-title = Merge strategy
batch-merge-lead = The manager groups by material, collision groups, sensor flag, and parent body; same-group static shapes merge into a compound.
batch-merge-same-material = Same material + same collision groups + static
batch-merge-compound = Merged into a single compound collider (one insert)
batch-merge-diff-material = Different material or different collision groups
batch-merge-separate = Each gets its own collider (multiple inserts)
batch-merge-dynamic-parent = Attached to a dynamic rigid body
batch-merge-attach = Attached via insert_with_parent to the parent body
batch-merge-sensor = Sensor flag is true
batch-merge-sensor-result = Sensor collider does not participate in collision response, only triggers events
batch-erosion-title = Erosion
batch-erosion-lead = Rapier/parry has no built-in clone_eroded API; we rebuild the shape as its round variant with border_radius = erosion_margin.
batch-erosion-cuboid = Converts a hard-edge cuboid to a round cuboid, reducing jitter when stacked.
batch-erosion-cylinder = Cylinder to round cylinder; edge contact is smoother.
batch-erosion-cone = Cone to round cone; the tip is blunted to prevent penetration.
batch-erosion-note = Ball / Capsule shapes are already round; erosion does not change their geometry. Ball and unsupported shapes fall back to shape_from_desc.
batch-ffi-title = FFI entry points
batch-ffi-lead = All pub extern "C" fn; payloads are #[repr(C)] flat structs; cbindgen generates the rigid_body.h header.
batch-limits-title = Capacity limits
batch-limit-max-requests = MAX_BATCH_REQUESTS = 100 000 — maximum requests per batch.
batch-limit-max-compound = MAX_COMPOUND_PARTS = 50 000 — maximum parts in a single compound.
batch-limit-erosion-zero = When erosion_margin = 0, round-variant rebuild is skipped and the original shape is used directly.
batch-example-title = Rust usage example
batch-example-lead = Build a ColliderRequest array, pass to batch_add_colliders; same-material shapes auto-merge into a compound.

# ---- JNI page ----
jni-tag = // mps-jni
jni-title = Java JNI Bindings
jni-desc = mps-jni exports { $methods } methods to org.polaris2023.mps.rapier.RapierNative via the jni! / jni_e_c! macros.
jni-codegen-title = Macro code generation
jni-codegen-lead = Two declarative macros wrap a Rust closure into an export_named JNI symbol; type tables live in @ty / @default arms.
jni-codegen-body = jni! handles plain methods that don't need JNIEnv; jni_e_c! adds the env / class types for callback-install methods and reuses jni!'s type table (see OPTIMIZATION.md §5.A).
jni-codegen-note = The macro body wraps the closure in catch_unwind(AssertUnwindSafe); on failure it falls back to @default instead of aborting the JVM process.
jni-panic-title = Panic isolation
jni-panic-lead = Any Rust panic is mapped by catch_unwind to ERR_INTERNAL and a zero value of the declared return type.
jni-panic-body = A JVM abort is almost unrecoverable, so each export fn has a panic net; the side effect is dirty state — callers should bracket suspect calls between world_step and check last_error_code().
jni-mangle-title = Symbol mangling
jni-mangle-desc = The export_name must match the Java class FQN exactly: '.' becomes underscore, and '_' inside an identifier becomes '_1'.
jni-col-class = Java class
jni-col-symbol = Exported symbol prefix
jni-mangle-note = Forgetting _1 makes System.load succeed but dlsym fail → UnsatisfiedLinkError; the library name is mps_rigid_body (mps-jni), so no _1 is needed anywhere — underscore mangling only applies to class-FQN packages like cn_1grass_1block.
jni-groups-title = API surface groups ({ $ffi } extern C entries)
jni-group-abi-title = ABI / version
jni-group-abi-desc = abiVersion / abiSupportsFfm / abiSupportsJni / last_error_code / last_error_message / clear.
jni-group-world-title = world_*
jni-group-world-desc = create / step / destroy / gravity / force-law install / event registration. 117 FFI entries.
jni-group-rb-title = rigid_body_*
jni-group-rb-desc = builder chain / state read-write / axis locks / mass_properties. 62 FFI entries.
jni-group-collider-title = collider_*
jni-group-collider-desc = shape build / friction restitution / collision groups / sensors. 75 FFI entries.
jni-group-query-title = query_*
jni-group-query-desc = ray_cast / shape_cast / point / intersection / R-tree culling. 58 FFI entries.
jni-group-events-title = events / ForceLaw
jni-group-events-desc = collision + contact-force ring buffer; Coulomb / AirDrag / Newton / custom force-law install.
jni-group-forces-title = Physics force laws (C1–C4 expansion)
jni-group-forces-desc = solar-wind dynamic pressure / Eddington radiation pressure / X-ray irradiation / pulsar magnetic-dipole torque / Jeans escape / MOND gravity.
jni-group-aero-title = Aerodynamics / fluid
jni-group-aero-desc = aero_apply_surfaces / aero_apply_voxel_grid / fluid AABB drag and buoyancy.
jni-group-arena-title = Arena zero-copy bridge
jni-group-arena-desc = arenaAsDirectByteBuffer / arena_read_double / arena_write_double — read/write physics state without JNI round-trips.
jni-group-cosmos-title = cosmos_*
jni-group-cosmos-desc = CosmosWorld create / celestial registration / n-body mutual gravity / Verlet advance / step_n batch.
jni-group-spaceflight-title = spaceflight_*
jni-group-spaceflight-desc = orbital perturbation / specific impulse / propellant budget / write acceleration output to a native buffer (out_accel).
jni-group-disaster-title = disaster_*
jni-group-disaster-desc = disaster_add_typhoon / disaster_add_tornado / disaster_add_hail / disaster_apply_forces: typhoon, tornado and hailstorm fields driving rigid bodies.
jni-group-volcano-title = volcano_*
jni-group-volcano-desc = volcano_add / volcano_apply: eruptive plume flow, lava-bomb ejecta and progressive rigid-body melting.
jni-handle-title = Handle packing
jni-handle-lead = RigidBodyHandle folds into a single jlong: high 32 bits store the index, low 32 bits the generation, matching Rapier's into_raw_parts() order.
jni-handle-note = Not splitting into two jints keeps ABI alignment with RigidBodyHandleRaw (a single u64) and avoids a generation race between two JNI reads.
jni-arena-title = Zero-copy Arena bridge
jni-arena-lead = arenaAsDirectByteBuffer uses NewDirectByteBuffer (a standard JNI API since Java 1.4) to expose the native Arena memory directly as a java.nio.ByteBuffer.
jni-arena-body = The Java side reads/writes purely through DoubleBuffer with no JNI upcalls; the per-frame native→jdoubleArray copy disappears, and the hot path barely sees JNI scheduling.
jni-deploy-title = Deployment
jni-deploy-lib = The cargo build --release -p mps-jni product mps_rigid_body.dll goes into src/main/resources/natives/.
jni-deploy-load = On the Java side, System.load("mps_rigid_body") per architecture; on failure UnsatisfiedLinkError will carry the offending symbol name.
jni-deploy-version = ABI is negotiated via abiVersion(); mps-web self-reports v{ $version }, any runtime mismatch should cause the caller to abort.
# ---- Debug replay page ----
nav-debug-replay = Debug Replay
replay-tag = // MPS
replay-title = Debug Recording & Replay
replay-desc = Jolt-Viewer-style capture/replay: the debugRecording* JNI surface records a ring buffer of 13-lane body snapshots and exports a replay blob for a canvas renderer.
replay-lifecycle-title = Recording lifecycle
replay-lifecycle-lead = A ring buffer with Jolt-Viewer overflow semantics: once full, the oldest frame is evicted, so a recording never grows unbounded.
replay-lifecycle-note = capture is a silent no-op while not recording, so the hot path can call it unconditionally after every step. max_frames is capped at MAX_OUTPUT_CAPACITY; start() returns 0 and sets ERR_INVALID_ARGUMENT on a bad capacity.
replay-blob-title = Export blob layout
replay-blob-desc = debugRecordingExport is two-phase: call with out = 0 to query the total f64 slot count, allocate, then call again. All values are f64 (handles and the magic are carried as raw bits).
replay-col-slot = Slot
replay-col-value = Value
replay-col-meaning = Meaning
replay-row-frame-count = Number of retained frames; [1] is the count of oldest frames evicted by the ring (0 unless the ring overflowed).
replay-row-dropped = Frame header. The magic slot carries 0x4d50534452303131 ("MPSDR01") as f64 bits — verify before decoding.
replay-row-magic = Number of bodies in this frame (fixed + dynamic, the full snapshot).
replay-row-body-count = Frame delta time in seconds — the step dt at capture time.
replay-row-dt = Packed RigidBodyHandleRaw u64 (same packing as worldBodySnapshot: high 32 = index, low 32 = generation).
replay-row-handle = pos3 + quat4 (i,j,k,w) + linvel3 + angvel3 — identical 13-lane layout to worldBodySnapshot.
replay-row-lanes = 13 f64 per body, offset 0 = x, 1 = y, 2 = z, 3..7 = quaternion, 8..11 = linear velocity, 11..14... — see rigid_body.h `BodySnapshot` doc for the authoritative lane table.
replay-blob-note = The blob is little-endian native order; read it through a DoubleBuffer with ByteOrder.nativeOrder(). Frame reads during recording are also available via debugRecordingFrame (layout without the leading frame_count/dropped slots).
replay-java-title = Minimal Java replay loop
replay-java-lead = No new dependency: Unsafe.allocateMemory for the blob, DoubleBuffer for decoding, and any Canvas/BufferedImage renderer doing a fixed-axis orthographic projection.
replay-java-note = The example keeps only poses for rendering; keep the velocity lanes when you need motion vectors or trail rendering. Handles are stable across frames, so entity colors/labels survive the whole recording.
replay-recenter-title = Floating origin interaction
replay-recenter-lead = A long-haul flight recentering shifts every body by the same offset; recorded frame coordinates jump by that offset exactly once.
replay-recenter-note = Render relative-to-pivot instead of absolute coordinates and the jump disappears from the picture. cosmosWorldRecenter must be called between steps, never mid-step.
# ---- FFM page ----
ffm-tag = // MPS
ffm-title = Java FFM Bindings
ffm-desc = ABI probing and version negotiation entrypoint for the Foreign Function & Memory API (JEP 454), targeting Java 25+ callers.
ffm-what-title = FFM's role here
ffm-what-lead = mps-ffm is a three-function lightweight crate: it only exposes the ABI version and per-end capability bits for runtime negotiation.
ffm-what-body = The actual rigid_body.h entries are still down-called by the Java side directly through Linker.downcallHandle; crates/mps-ffm does NOT replicate any method signature. Its job is to let Java get abi_version() the moment the .dll/.so is loaded, and decide whether to take the FFM or JNI path.
ffm-surface-title = ABI surface
ffm-surface-lead = Three #[unsafe(no_mangle)] extern C functions with #[repr(C)] Bool return values.
ffm-surface-note = Pure ABI probing with no world access; Java Linker down-calls these first, then switches into world_create / world_step etc.
ffm-vs-title = JNI vs FFM comparison
ffm-col-feature = Feature
ffm-row-min-java = Minimum Java
ffm-row-binding = Binding style
ffm-row-jni-bind = Java native methods + javah header generation
ffm-row-ffm-bind = Linker.downcallHandle + FunctionDescriptor
ffm-row-overhead = Call overhead
ffm-row-jni-over = High (Env per call)
ffm-row-ffm-over = Near-native (compile-time ABI direct link)
ffm-row-memory = Memory management
ffm-row-jni-mem = GetXxxArrayElements + release
ffm-row-ffm-mem = MemorySegment slicing the Arena directly
ffm-row-panic = Panic handling
ffm-row-jni-panic = catch_unwind maps to ERR_INTERNAL
ffm-row-ffm-panic = Caller must respect Rust conventions on its own (UB otherwise)
ffm-layout-title = Linker downcall layout
ffm-layout-desc = The Java side writes a FunctionDescriptor from the C ABI describing args and return types; the Linker produces a method handle invoked via invokeExact.
ffm-header-title = C ABI input
ffm-header-lead = Java treats rigid_body.h as the contract and mirrors #[repr(C)] structs via memory layouts.
ffm-header-cbindgen = The cbindgen-generated rigid_body.h (~5400 lines) is the single source of truth.
ffm-header-structs = Vec3 / Quat / ShapeDesc / event records are all #[repr(C)] flat; Java MemoryLayout computes offsets.
ffm-header-load = Linker + SymbolLookup.loaderLookup() loads mps_rigid_body with no JNIEnv dependency.
ffm-header-note = Any cbindgen field change forces a Java layout update; abi_version() is the version gatekeeper.
ffm-alloc-title = Allocation strategy
ffm-alloc-segment-title = MemorySegment
ffm-alloc-segment-desc = Each call allocates a temporary segment; the arena allocator closes them together with a clear lifetime.
ffm-alloc-arena-title = Shared Arena
ffm-alloc-arena-desc = Bulk state read/write goes through the mps-core Arena; Java gets a DirectByteBuffer wrapper, zero-copy.
ffm-alloc-shared-title = Foundation
ffm-alloc-shared-desc = JNI and FFM both share the mps-core Arena state — different routes, same destination.
ffm-status-title = Current status
ffm-status-body = mps-ffm remains a capability-probing crate; the full JEP 454 downcall bindings are constructed at runtime by the Java side — no generator path yet.
# ---- API Reference page ----
api-tag = // MPS
api-title = API Reference
api-desc = crates/mps-core/include/rigid_body.h exposes { $total } pub extern C entries.
api-header-title = Header surface
api-header-lead = cbindgen 0.29.4 generates it from mps-core's build.rs; { $total } FFI functions plus all #[repr(C)] types are listed.
api-header-body = Do NOT hand-edit — any change comes from a pub extern C in mps-core's rapier module; build regenerates it.
api-prefix-title = Function prefix groups
api-prefix-lead = Each export function name has a physics-subsystem prefix; grepping the prefix lists all entries of that module.
api-col-prefix = Prefix
api-col-count = Count
api-col-domain = Responsibility
api-row-world = World / gravity / force laws / events / step
api-row-rigid = RigidBody state / mass / axis locks
api-row-collider = shapes / friction / collision groups / sensors
api-row-query = ray cast / shape cast / point projection / sweeps
api-row-disaster = Typhoon / tornado / hailstorm field sources (world side).
api-row-volcano = Volcanic eruption: plume / lava bombs / melting (world side).
api-prefix-note = The sum matches CORE_FFI_COUNT; unlisted prefixes (aero_ / fluid_ / trajectory_ / anvilkit_ / cosmos_ / molecular_ / events / force law) live in their domain sections.
api-handles-title = Common handle types
api-col-type = Type
api-col-scope = Scope
api-handle-world = Physics world; lifecycle from world_create to world_destroy.
api-handle-rigid = Rigid body index+generation, packed as u64, reused across calls.
api-handle-collider = Collider index+generation; may be detached from its parent rigid body.
api-handle-rb-build = Builder chain; ownership transfers on insert_with_parent.
api-handle-col-build = Collider builder; ownership transfers to world on build_insert.
api-handle-joint = Joint builder; voided after world_add_impulse_joint succeeds.
api-handle-rtree = Broad-phase R-tree for collider/query culling.
api-handle-crbtree = C-side red-black tree for joints / collision pairs, O(log n) queries.
api-handle-cc = Character controller handle wrapping capsule scan + contact solve.
api-records-title = Flat record types
api-records-lead = All #[repr(C)] flat, written cleanly by Java / JNI / FFM ends alike.
api-record-vec3 = Vec3 — three f64 axes in x/y/z order.
api-record-quat = Quat — (i, j, k, w) quaternion; builders convert to axis-angle.
api-record-aabb = AabbDesc — min/max + user tag for query ranges.
api-record-shape = ShapeDesc — shape_type + a/b/c/d params, covering sphere / cuboid / capsule / cone etc.
api-record-event = CollisionEventRecord / ContactForceEventRecord — event ring buffer payloads.
api-record-filter = QueryFilterDesc / InteractionGroupsDesc — query filter bitmasks.
api-error-title = Error reporting
api-error-lead = A thread-local error code + message; always check the function return value first.
api-error-note = Subsequent calls in the same frame overwrite the previous error; you must last_error_clear() immediately after a read to judge the next failure cleanly.
api-lifecycle-title = World lifecycle call sample
api-stability-title = ABI stability contract
api-stability-cbindgen = rigid_body.h is cbindgen-generated; skipping hand-edits is a hard error.
api-stability-repr = All public structs are #[repr(C)] with fixed field order, alignment, and padding.
api-stability-version = abi_version() is the mandatory negotiation entry; callers must abort on runtime mismatch.
api-stability-redline = Formula modules may only expose crate-internal pub fn; only pub extern C fn enter rigid_body.h. The C ABI red line is guarded by cargo build -p mps-core + a header git diff.
# ---- 404 page ----
not-found-title = Page Not Found
not-found-desc = The page you requested does not exist. Please return to the home page.
not-found-back = Back to Home

# ---- Footer ----
footer-text = MPS Motion Physics System v{ $version } — GitHub
nav-group-overview = Overview
nav-group-core = mps-core
nav-group-cosmos = mps-cosmos
nav-group-formula = mps-formula
nav-group-jni = mps-jni
nav-group-ffm = mps-ffm
# ---- Soft Body (Phases 0–21) ----
soft-tag = Soft Body
soft-title = Soft Body Physics
soft-desc = XPBD / MassSpring deformable bodies — cloth, tetrahedral volume meshes, voxel terrain, 22 capability upgrades (Phases 0–21), plus a zero-fork FFI safety line (Phases 22–25): contact-force readback, per-particle impulse, AABB readback, deep clone, binary state save/restore, and per-particle velocity write.

soft-overview-title = Overview
soft-overview-lead = A soft body is a collection of particles connected by distance constraints (XPBD) and/or springs (MassSpring), optionally wrapped in a triangle shell or a tetrahedral volume mesh.
soft-overview-body = Every soft body owns an independent gravity field, a sleep/wake state, and a set of XPBD distance constraints plus MassSpring springs. The solver is selected per body via soft_body_configure_solver — XPBD for stiff structural cloth/flesh, MassSpring for bouncy rope/jelly. All state is exposed read-write through the Arena, so Java reads particles/tetrahedra/triangles/edges with zero per-object JNI.

soft-solver-title = Solver
soft-solver-desc = Two solvers share the same particle buffer; switch with soft_body_configure_solver(world, id, solver, iterations, dt).
soft-solver-li-1 = XPBD — rigid-compliance distance constraints with per-constraint compliance and compression; pairs with tetrahedral volume conservation for incompressible flesh.
soft-solver-li-2 = MassSpring — Hookean springs (soft_body_add_spring) with per-spring stiffness; cheap and stable for ropes, cloth, and gelatin.
soft-solver-li-3 = Per-constraint anisotropic compliance (soft_body_set_distance_constraint_compliance) lets an edge resist stretch differently along its axis for directional stiffness.

soft-data-title = Data Model
soft-data-desc = A soft body is four parallel arrays plus two constraint sets; all are Arena-readable.
soft-data-li-1 = Particles — soft_body_add_particle(pos, inv_mass, pinned); read via soft_body_read_particles.
soft-data-li-2 = Tetrahedra — soft_body_add_tetrahedron(a,b,c,d) for volume meshes; rest volume cached for conservation; read via soft_body_read_tetrahedra.
soft-data-li-3 = Triangles — soft_body_add_triangle(a,b,c) build the shell; read via soft_body_read_triangles.
soft-data-li-4 = Edges — springs and distance constraints; read via soft_body_read_edges.

soft-cap-title = Capability Matrix (Phases 0–21)
soft-cap-lead = Each card maps to a real soft_body_* FFI delivered in the workspace. Phases 22–25 add a zero-fork FFI safety line (see below).

soft-cap-01-title = Base Body & Particles
soft-cap-01-desc = soft_body_create + soft_body_add_particle; free or pinned particles with independent inv_mass. The foundation every later feature builds on.
soft-cap-02-title = Triangle Shell
soft-cap-02-desc = soft_body_add_triangle registers the 3 structural edges as distance constraints; the shell drives cloth and surface contact.
soft-cap-03-title = Tetrahedral Volume Mesh
soft-cap-03-desc = soft_body_add_tetrahedron + soft_body_build_tetra_mesh build an incompressible volumetric body; rest volumes are cached for volume conservation.
soft-cap-04-title = Springs (MassSpring)
soft-cap-04-desc = soft_body_add_spring + soft_body_set_spring_stiffness — Hookean links for ropes, cloth, and jelly with tunable stiffness.
soft-cap-05-title = Distance Constraints
soft-cap-05-desc = soft_body_add_distance_constraint with per-constraint compliance and compression; the XPBD structural backbone.
soft-cap-06-title = Cloth & Bending
soft-cap-06-desc = soft_body_add_bending adds angular bending resistance on top of the triangle shell for stiff fabric and fol-do-not-collapse surfaces.
soft-cap-07-title = Wind Field
soft-cap-07-desc = soft_body_apply_wind + soft_body_clear_wind — aerodynamic drag on the shell using triangle normals; flags via soft_body_apply_wind_flag.
soft-cap-08-title = Sleep Diagnostics
soft-cap-08-desc = soft_body_is_sleeping / soft_body_sleep / soft_body_wake — persistent-island sleep states keep a settled body off the solver.
soft-cap-09-title = Rigid Anchoring
soft-cap-09-desc = soft_body_attach_particle / soft_body_detach_particle bind a particle to a rigid body (fixed point, rope end, pinned cloth corner).
soft-cap-10-title = Tearing
soft-cap-10-desc = soft_body_set_tear_strain — a distance constraint breaks when its strain exceeds the threshold, so cloth rips under load.
soft-cap-11-title = Plasticity
soft-cap-11-desc = soft_body_set_plasticity — constraints retain a fraction of deformation as permanent offset, so soft bodies dent and stay dented.
soft-cap-12-title = Pressurization
soft-cap-12-desc = soft_body_set_pressure — an internal pressure force inflates the tetrahedral mesh (balloons, airbags, bladders).
soft-cap-13-title = Self-Collision
soft-cap-13-desc = soft_body_set_self_collision — particles repel each other within a radius; keeps a folded body from passing through itself.
soft-cap-14-title = Soft–Soft Collision
soft-cap-14-desc = soft_body_set_cross_collision — two soft bodies resolve contact against each other (piling, stacking, squishing).
soft-cap-15-title = Independent Gravity
soft-cap-15-desc = soft_body_set_gravity — each body carries its own gravity vector, decoupled from the world gravity used by rigid bodies.
soft-cap-16-title = Volume Conservation
soft-cap-16-desc = soft_body_set_volume_conservation + soft_body_total_volume — an XPBD constraint holds total tetrahedral volume (incompressible flesh, water balloons).
soft-cap-17-title = Cohesion
soft-cap-17-desc = soft_body_set_cohesion — nearby particles inside a capture radius are pulled together (surface tension, sticky droplets, wet sand).
soft-cap-18-title = Structural Damping
soft-cap-18-desc = soft_body_set_damping — velocity-proportional damping suppresses jitter and rings the body to rest.
soft-cap-19-title = Anisotropic Compliance
soft-cap-19-desc = soft_body_set_distance_constraint_compliance — per-edge directional compliance for stiff-warp / soft-weft cloth and directional flesh.
soft-cap-20-title = Soft–Soft Friction
soft-cap-20-desc = soft_body_set_self_collision_friction + soft_body_set_cross_collision_friction — Coulomb tangential damping on self and cross contacts (μ ∈ [0,1]).
soft-cap-21-title = Adaptive Tetrahedral Subdivision
soft-cap-21-desc = soft_body_subdivide_tetrahedra — barycentric 1→4 split of tets whose longest edge exceeds a threshold; sub-volumes sum to the parent so volume stays conserved.
soft-cap-22-title = Read/Write API
soft-cap-22-desc = soft_body_read_particles / _read_tetrahedra / _read_triangles / _read_edges + soft_body_get_particle — the full body state flows through the zero-copy Arena.

soft-p25-title = FFI Safety Line (Phases 22–25)
soft-p25-lead = Six pure mps-core additions — each walks the SoftBody public fields directly, none touches the rapier3d fork. They expose state read-back, clone, binary (de)serialization, and direct velocity write for save/restore, replay, and networked soft-body snapshots.
soft-p25-1-title = Contact-Force Readback
soft-p25-1-desc = soft_body_read_contact_force — per-particle total contact impulse from the collision proxy, split by which collider it hit. Read-only diagnostics for grasp/squish force.
soft-p25-2-title = Per-Particle Impulse
soft-p25-2-desc = soft_body_apply_particle_impulse — p.vel += J·inv_mass; pinned (inv_mass==0) is a no-op. Kick a single node without rebuilding the body.
soft-p25-3-title = AABB / Centroid Readback
soft-p25-3-desc = soft_body_read_aabb — min/max corner and centroid from particle positions; any out pointer may be null to skip it.
soft-p25-4-title = Deep Clone
soft-p25-4-desc = soft_body_clone — SoftBody::clone into a fresh id with collide=false, so the copy integrates independently and never shares the source proxy.
soft-p25-5-title = Binary State Save / Restore
soft-p25-5-desc = soft_body_state_size + soft_body_save_state + soft_body_restore_state — hand-rolled little-endian blob over every public field (Option/enum/RigidBodyHandle packed via into_raw_parts). Corrupt magic/version/truncation returns FALSE with no half-built body left behind.
soft-p25-6-title = Per-Particle Velocity Write
soft-p25-6-desc = soft_body_set_particle_velocity — overwrite particle.vel; pinned / out-of-range / unknown id return FALSE. The write counterpart to soft_body_get_particle.

soft-p25-map-title = Phase 25 FFI <-> JNI  (zero-fork, mps-core only)
soft-p25-map-note = Each C FFI maps 1:1 to a Java JNI method. All eight walk the SoftBody public fields directly and never touch the rapier3d fork. Return + guard column shows the success type and the failure path.
soft-p25-map-body = FFI                                      JNI                               ret / guard
  soft_body_read_contact_force        softBodyReadContactForce        u32 count / bad id -> 0
  soft_body_apply_particle_impulse     softBodyApplyParticleImpulse     bool / pinned skip, bad id -> false
  soft_body_read_aabb                  softBodyReadAabb                 bool / null out ptr ok
  soft_body_clone                      softBodyClone                    u32 new id / fail -> u32::MAX
  soft_body_state_size                 softBodyStateSize                u32 bytes / fail -> u32::MAX
  soft_body_save_state                 softBodySaveState                u32 written / small buf -> u32::MAX
  soft_body_restore_state              softBodyRestoreState             u32 new id / bad magic -> u32::MAX
  soft_body_set_particle_velocity      softBodySetParticleVelocity      bool / pinned|oob|bad id -> false

soft-api-title = FFI Surface
soft-api-desc = The soft-body subsystem is exposed symmetrically across C FFI, Java JNI, and the integration tests.
soft-api-stat-ffi = C FFI functions
soft-api-stat-jni = JNI methods
soft-api-stat-tests = integration tests

# ---- Cosmos sub-pages (Plan D split) ----
cosmos-land-title = Feature pages
cosmos-land-lead = Cosmos is split into six capability pages — click through for the real functions and FFI behind each.
nav-cosmos-world = World & Bodies
nav-cosmos-gravity = Gravity & n-body
nav-cosmos-integrator = Integrators
nav-cosmos-orbit = Orbit & Diagnostics
nav-cosmos-flight = Flight & Perturbation
nav-cosmos-arena = Arena & JNI
cw-tag = // mps-cosmos
cw-title = World & Bodies
cw-desc = CosmosWorld — an independent orbital-scale world, separate from mps-core's Rapier World.
cw-overview-title = Overview
cw-overview-lead = A CosmosWorld owns its own integration domain: a central body, the Sun, and a catalogue of celestial bodies.
cw-overview-body = Unlike mps-core, there is no rigid-body contact solver — CosmosWorld advances bodies purely through gravity + integrator. Bodies are inserted individually (cosmos_world_insert_body) or as gravity sources (cosmos_world_insert_body_as_gravity_source), and the Sun / central body are configured via cosmos_world_set_sun_position / cosmos_world_set_central_body.
cw-bodies-title = Celestial catalogue
cw-bodies-desc = cosmos_world_add_celestial injects a body from the JPL DE441 dataset (GM, elements, phase). The 10 primary bodies are registered at init; user bodies are added on top.
cw-batch-title = Batch insertion
cw-batch-desc = cosmos_world_add_n_body submits many spacecraft states in one call rather than per-body, matching the cosmosWorldDynamicBodySnapshot bulk read-back pattern for tracking-queue workloads.
cw-ffi-title = C FFI surface
cw-ffi-desc = The world layer exposes the create / build / insert / step lifecycle.
cw-ffi-1 = cosmos_world_create / cosmos_world_destroy — own the orbital world.
cw-ffi-2 = cosmos_satellite_builder / cosmos_fixed_body_builder — satellite or fixed (celestial) body.
cw-ffi-3 = cosmos_builder_set_gravity_scale / _set_linear_damping / _set_angular_damping / _lock_translations — body parameters.
cw-ffi-4 = cosmos_world_insert_body / cosmos_world_insert_body_as_gravity_source — add to the world.
cw-ffi-5 = cosmos_world_set_central_body / cosmos_world_set_sun_position / cosmos_world_set_perturbation — domain config.
cw-ffi-6 = cosmos_world_step / cosmos_world_step_n — advance one or N frames.
cg-tag = // mps-cosmos
cg-title = Gravity & n-body
cg-desc = Newtonian point-mass gravity, pairwise n-body mutual gravity, and an SIMD far-field monopole path.
cg-overview-title = Overview
cg-overview-lead = Every frame computes GM·m/r² across all bodies — the signature trait of CosmosWorld.
cg-overview-body = gravity.rs models three regimes: a single point-mass acceleration (point_mass_acceleration), the full pairwise mutual sum (n_body_acceleration / n_body_acceleration_reduce, O(N²)), and a SIMD far-field monopole approximation (far_field_monopole_simd) for the high-count case. The near/far split is governed by near_field_threshold, monopole, and irregular.
cg-fn-title = Functions
cg-fn-desc = Pure helpers in gravity.rs; bodies call them during step.
cg-fn-1 = point_mass_acceleration / celestial_acceleration — acceleration from one source.
cg-fn-2 = n_body_acceleration / n_body_acceleration_reduce — pairwise mutual gravity over all bodies.
cg-fn-3 = far_field_monopole_simd — SIMD monopole sum for the far field.
cg-fn-4 = gm_from_mass — GM from mass via the gravitational constant.
cg-fn-5 = monopole / irregular / near_field_threshold — toggle near-field vs far-field treatment.
cg-ffi-title = Gravity source FFI
cg-ffi-desc = cosmos_world_insert_body_as_gravity_source lets a body attract others without being integrated; cosmos_hill_radius_for (orbit page) sizes the sphere of influence.
ci-tag = // mps-cosmos
ci-title = Integrators
ci-desc = Verlet stepping plus high-order and Kahan-compensated steppers for long arcs.
ci-overview-title = Overview
ci-overview-lead = CosmosWorld integrates with velocity-Verlet by default, with higher-order options for precision.
ci-overview-body = integrator.rs provides verlet_step as the baseline, and explicit_highorder_step / advance_highorder for higher-order accuracy. advance_highorder_kahan / explicit_highorder_kahan_step add Kahan summation compensation so long-duration arcs (years of simulation) do not drift from floating-point round-off. total_acceleration folds gravity + perturbation into one vector; snapshot_source_positions caches source positions for the far field.
ci-fn-title = Functions
ci-fn-desc = The stepper family in integrator.rs.
ci-fn-1 = verlet_step — baseline velocity-Verlet advance.
ci-fn-2 = explicit_highorder_step / advance_highorder — higher-order integration.
ci-fn-3 = explicit_highorder_kahan_step / advance_highorder_kahan — Kahan-compensated, low drift over long arcs.
ci-fn-4 = total_acceleration — gravity + perturbation summed.
ci-fn-5 = snapshot_source_positions — cache source positions for the far field.
ci-toggle-title = Parallel & SIMD toggles
ci-toggle-desc = nb_parallel_enabled switches the pairwise sum to a rayon parallel path; ff_simd_enabled turns on the SIMD far-field monopole. Both default on for the solar-system scale.
co-tag = // mps-cosmos
co-title = Orbit & Diagnostics
co-desc = Six-element conversion, Hill radius, mean motion, eccentricity vector, Kozai period, and state snapshots.
co-overview-title = Overview
co-overview-lead = orbit.rs / orbit_diagnostics.rs turn state vectors into classical elements and back, and expose diagnostics for monitoring drift.
co-overview-body = The six Keplerian elements convert to/from Cartesian via orbit.rs. orbit_diagnostics.rs adds mean_motion, mean_motion_ratio, eccentricity_vector, and kozai_period (the Kozai–Lidov cycle period). cosmos_hill_radius_for sizes the Hill sphere of influence for a body. State snapshots (cosmos_world_dynamic_body_snapshot / _count) let Java read every body's position/velocity without per-body JNI.
co-fn-title = Functions
co-fn-desc = Element and diagnostic helpers.
co-fn-1 = Six-element ↔ Cartesian conversion (orbit.rs).
co-fn-2 = cosmos_hill_radius_for — Hill sphere radius for a body.
co-fn-3 = mean_motion / mean_motion_ratio — orbital rate and resonance ratio.
co-fn-4 = eccentricity_vector — orbit shape / orientation.
co-fn-5 = kozai_period — Kozai–Lidov oscillation period.
co-snap-title = State snapshots
co-snap-desc = cosmos_world_dynamic_body_snapshot_count + cosmos_world_dynamic_body_snapshot bulk-export every dynamic body's state through the Arena for drift monitoring and plotting.
cf-tag = // mps-cosmos
cf-title = Flight & Perturbation
cf-desc = Two-centre dynamics, trim, longitudinal stability, and environmental perturbations.
cf-dyn-title = Flight dynamics
cf-dyn-lead = flight/dynamics.rs — two-centre approximation, third-body and drag forces.
cf-dyn-desc = total_forces_and_moments and simulate_one_step integrate an aircraft/scraft state; from_body / linvel_body convert frames; flat_plate_area / default_airfoil size the lifting surface. Validates input via valid.
cf-trim-title = Trim
cf-trim-desc = flight/trim.rs — hover_target / level_flight_target define the desired equilibrium; trim solves control surfaces for it.
cf-stab-title = Stability
cf-stab-desc = flight/stability.rs — linearize builds the state matrix; longitudinal_modes / longitudinal_submatrix expose the short/phugoid modes; power_iteration finds the dominant eigenvalue.
cf-pert-title = Perturbation
cf-pert-desc = perturbation/* — atmospheric drag and solar radiation pressure as forces (reuses mps_formula::spaceflight), injected before each step.
ca-tag = // mps-cosmos
ca-title = Arena & JNI
ca-desc = Cosmos ships its own SharedArena (COSMAREN) — the orbital-scale twin of mps-core's shared_arena.
ca-overview-title = Overview
ca-overview-lead = Java reads/writes body state through a DirectByteBuffer with no per-body JNI call.
ca-overview-body = arena.rs holds a seqlock-guarded body slot; ffi.rs exposes the address and size so Java maps it as a DirectByteBuffer. This mirrors mps-core's arena bridge but is sized for orbital bodies. cosmos_world_get_shared_arena_address / _size parallel worldGetArenaDirectByteBuffer.
ca-ffi-title = Arena FFI
ca-ffi-desc = Create / destroy / query the shared arena.
ca-ffi-1 = cosmos_world_create_shared_arena / cosmos_world_destroy_shared_arena — own the Arena.
ca-ffi-2 = cosmos_world_get_shared_arena_address / cosmos_world_get_shared_arena_size — map into Java.
ca-ffi-3 = cosmos_world_dynamic_body_snapshot(_count) — bulk body state through the Arena.
ca-jni-title = JNI batch API
ca-jni-desc = mps-jni exposes cosmosWorld* methods; bulk state read-back goes through cosmosWorldDynamicBodySnapshot / cosmosWorldDynamicBodySnapshotCount over the Arena (no per-body JNI).

# ---- Cosmos sub-page FFI<->JNI maps ----
cw-map-title = FFI <-> JNI  (src: implementation module)
cw-map-note = C FFI (snake_case) maps 1:1 to Java JNI (camelCase). After a builder is inserted, ownership transfers to the world. The src column names the real implementation module in mps-cosmos.
cw-map-body = FFI                                   JNI                          src
  cosmos_world_create                  cosmosWorldCreate            world.rs
  cosmos_world_destroy                 cosmosWorldDestroy           world.rs
  cosmos_satellite_builder            cosmosSatelliteBuilder       bodies.rs
  cosmos_fixed_body_builder           cosmosFixedBodyBuilder       bodies.rs
  cosmos_builder_set_gravity_scale     cosmosBuilderSetGravityScale bodies.rs
  cosmos_builder_set_linear_damping    cosmosBuilderSetLinearDamping bodies.rs
  cosmos_builder_set_angular_damping   cosmosBuilderSetAngularDamping bodies.rs
  cosmos_builder_lock_translations     cosmosBuilderLockTranslations bodies.rs
  cosmos_world_insert_body             cosmosWorldInsertBody        world.rs
  cosmos_world_insert_body_as_gravity_source cosmosWorldInsertBodyAsGravitySource world.rs + gravity.rs
  cosmos_world_set_central_body        cosmosWorldSetCentralBody    world.rs
  cosmos_world_set_sun_position        cosmosWorldSetSunPosition    world.rs
  cosmos_world_set_perturbation        cosmosWorldSetPerturbation   world.rs + perturbation/
  cosmos_world_step                    cosmosWorldStep              world.rs -> integrator.rs
  cosmos_world_step_n                  cosmosWorldStepN             world.rs -> integrator.rs
  cosmos_world_add_celestial          cosmosWorldAddCelestial       world.rs + gravity.rs
  cosmos_world_add_n_body              cosmosWorldAddNBody          world.rs + gravity.rs
cg-map-title = FFI <-> JNI  (src: implementation module)
cg-map-note = Gravity source registration is the only FFI in this layer; the acceleration functions run inside cosmosWorldStep. The src column names where they live.
cg-map-body = FFI                                                       JNI  src
  cosmos_world_insert_body_as_gravity_source  cosmosWorldInsertBodyAsGravitySource  world.rs -> gravity.rs
  # point_mass_acceleration / n_body_acceleration_reduce / far_field_monopole_simd
  #   are called inside cosmosWorldStep - implemented in gravity.rs + integrator.rs
  # no standalone FFI/JNI.
ci-map-title = FFI <-> JNI  (src: implementation module)
ci-map-note = Stepping is the only FFI in this layer; the stepper family is selected internally by orbit_integration / verlet_substeps. The src column names where they live.
ci-map-body = FFI                          JNI                 src
  cosmos_world_step    cosmosWorldStep    world.rs -> integrator.rs
  cosmos_world_step_n  cosmosWorldStepN  world.rs -> integrator.rs
  # verlet_step / explicit_highorder_step / advance_highorder_kahan
  #   live in integrator.rs, chosen by orbit_integration + verlet_substeps
  # no standalone JNI.
co-map-title = FFI <-> JNI  (src: implementation module)
co-map-note = Hill radius is FFI-only (internal diagnostics, in world.rs -> orbit_diagnostics.rs); snapshots are the JNI bulk path (ffi.rs + arena.rs layout).
co-map-body = FFI                                                   JNI  src
  cosmos_hill_radius_for  (FFI only)  world.rs -> orbit_diagnostics.rs
  cosmos_world_dynamic_body_snapshot        cosmosWorldDynamicBodySnapshot       ffi.rs + arena.rs
  cosmos_world_dynamic_body_snapshot_count  cosmosWorldDynamicBodySnapshotCount  ffi.rs + arena.rs
  # mean_motion / eccentricity_vector / kozai_period are pure fns in
  # orbit.rs / orbit_diagnostics.rs, read back via the snapshot / Arena.
cf-map-title = FFI <-> JNI  (src: implementation module)
cf-map-note = flight/* and perturbation/* are compute-only; they run before cosmosWorldStep and have no standalone FFI/JNI. The src column names where they live.
cf-map-body = module                         functions                              src
  flight/dynamics   total_forces_and_moments / simulate_one_step  flight/dynamics.rs
  flight/trim      trim (hover_target / level_flight_target)      flight/trim.rs
  flight/stability linearize / longitudinal_modes / power_iteration flight/stability.rs
  perturbation     atmospheric_drag_force / solar pressure        perturbation/
  # all injected before cosmosWorldStep; results read back via
  # cosmosWorldDynamicBodySnapshot through the Arena (arena.rs).
ca-map-title = FFI <-> JNI  (src: implementation module)
ca-map-note = The arena exposes its address/size to Java as a DirectByteBuffer; snapshots are the bulk read-back path (arena.rs).
ca-map-body = FFI                                              JNI                              src
  cosmos_world_create_shared_arena     cosmosWorldCreateSharedArena     world.rs -> arena.rs
  cosmos_world_destroy_shared_arena    cosmosWorldDestroySharedArena    world.rs -> arena.rs
  cosmos_world_get_shared_arena_address cosmosWorldGetSharedArenaAddress arena.rs
  cosmos_world_get_shared_arena_size     cosmosWorldGetSharedArenaSize     arena.rs
  cosmos_world_dynamic_body_snapshot        cosmosWorldDynamicBodySnapshot       ffi.rs + arena.rs
  cosmos_world_dynamic_body_snapshot_count  cosmosWorldDynamicBodySnapshotCount  ffi.rs + arena.rs

moons-title = Natural Satellites (Moons)
moons-desc = Precision data for { $count } regular moons of the major planets, preloaded in `mps_formula::celestial_data::MOONS` (NASA Planetary Fact Sheet / JPL).
moons-catalog-title = Satellite catalogue
moons-catalog-lead = Each moon is a point-mass gravity source. GM is in 10^9 m3/s2, radius and semi-major axis in km, orbital period in days (retrograde orbits use the absolute magnitude).
moons-source-note = Sources: NASA Planetary Fact Sheet, JPL. Irregular inner satellites carry small eccentricities; values are mean elements.
moons-col-planet = Parent planet
moons-col-name = Moon
moons-col-gm = GM (10^9 m3/s2)
moons-col-radius = Radius (km)
moons-col-sma = Semi-major axis (km)
moons-col-period = Period (days)
moons-ffi-title = Inject a moon into a CosmosWorld
moons-ffi-lead = Moons reuse the celestial gravity infrastructure via `add_moon`, which converts a `Moon` to a `CelestialBody` (max_degree=0, no spherical harmonics) and registers it as a gravity source.
moons-ffi-body = cosmos_world_add_moon(world: *mut CosmosWorld, moon_index: i32) -> i32
    // moon_index = index into MOONS; returns source index or -1 (out of range / null world)
    let idx = cosmos_world_add_moon(world, 0); // Earth's Moon
# character_body / sensor_zone / vehicle_controller (Phase 3c/3d/3e)
nav-character-body = Character Body
char-tag = New body type
char-title = Character Body
char-desc = A kinematic character controller that drives a kinematic-position-based rigid body - walk, slide, and step over terrain without tunnelling.
char-overview-title = Overview
char-overview-lead = The character body wraps rapier's KinematicCharacterController and a kinematic rigid body.
char-overview-body = You create a character with a shape plus translation, then each step call character_body_move with the desired translation. The controller shape-casts against the world to resolve collisions, slopes and steps, and the result is written back to the kinematic body.
char-api-title = C ABI
char-api-desc = Four entry points cover creation, stepping, position read-back and teardown.
char-cap-title = Capabilities
char-cap-lead = What the character body gives you out of the box.
char-cap-01-title = Collision-safe movement
char-cap-01-desc = shape_cast based resolve means the character never tunnels through walls or floors.
char-cap-02-title = Grounded detection
char-cap-02-desc = character_body_move returns an EffectiveCharacterMovement with the resolved translation and a grounded flag.
char-cap-03-title = No self-collision
char-cap-03-desc = the character's own collider is intentionally omitted so its shape-cast does not catch itself.
char-cap-04-title = Toggleable push
char-cap-04-desc = set_apply_impulses_to_dynamic_bodies lets you disable the momentum transfer, so the character ghosts through dynamic bodies without shoving them.
char-col-title = Collision readback & terrain gravity
char-col-lead = Inspect what the character hit and add slope/terrain-aware free-fall.
char-col-body = character_body_move captures every contact into a ring buffer. Read it back with collision_count + get_collision (each entry names the collider handle, the hit normal and the remaining translation). move_with_terrain adds the world's registered terrain-gravity acceleration to the desired motion, so avatars slide down hills the way they do on a Minecraft terrain collider.
char-col-01-title = Collision enumeration
char-col-01-desc = collision_count returns the number of contacts captured by the last move; get_collision(index) reads one without copying the whole buffer.
char-col-02-title = Terrain-aware move
char-col-02-desc = move_with_terrain folds the terrain-gravity source into the desired displacement; with no source registered it is bit-identical to move.
char-col-03-title = Push dynamic bodies
char-col-03-desc = solve_impulses drives each touched dynamic body toward the character.s intended velocity (from the blocked translation), so the character physically shoves crates and other rigid bodies.
nav-sensor-zone = Sensor Zone
sensor-tag = New body type
sensor-title = Sensor Trigger Zone
sensor-desc = A sensor collider polled for overlaps - no physical response, just trigger events. The fourth body type.
sensor-overview-title = Overview
sensor-overview-lead = The sensor zone is a sensor collider plus an overlap cache.
sensor-overview-body = After each step, call sensor_zone_poll to intersect the zone shape against all world colliders via the broad-phase query pipeline. Overlapping colliders are written into a sticky current set and the zone is marked triggered.
sensor-api-title = C ABI
sensor-api-desc = Nine entry points cover creation, polling, contact enumeration, transform control and teardown.
sensor-cap-title = Capabilities
sensor-cap-lead = What the sensor zone gives you.
sensor-cap-01-title = Overlap detection
sensor-cap-01-desc = poll computes the live set of overlapping colliders via intersect_shape on the broad-phase query pipeline.
sensor-cap-02-title = Sticky trigger flag
sensor-cap-02-desc = is_triggered stays true once any overlap has ever occurred, so event edges are easy to detect.
sensor-cap-03-title = Movable and toggleable
sensor-cap-03-desc = set_translation moves the zone, set_enabled toggles it without recreation.
sensor-cap-04-title = Rising-edge trigger
sensor-cap-04-desc = set_edge switches is_triggered to edge mode: it fires only on the poll where an overlap first appears, then stays false until the zone is emptied and re-entered.
sensor-cap-05-title = Consume and clear
sensor-cap-04-desc = set_edge switches is_triggered to edge mode: it fires only on the poll where an overlap first appears, then stays false until the zone is emptied and re-entered.
sensor-cap-05-desc = sensor_zone_consume read-and-clears the one-shot edge latch (TRUE exactly once per entry); sensor_zone_clear re-arms the zone. No fork changes.
nav-vehicle-controller = Vehicle Controller
nav-force-queue = Force Queue
veh-tag = New body type
veh-title = Ray-Cast Vehicle Controller
veh-desc = A dynamic chassis body driven by rapier's ray-cast vehicle controller - suspension, wheels, engine force and steering. The fifth body type.
veh-overview-title = Overview
veh-overview-lead = The vehicle controller owns a dynamic chassis rigid body plus rapier's DynamicRayCastVehicleController.
veh-overview-body = You create a chassis with a shape plus translation, add wheels, then each step call vehicle_controller_update with dt. The controller ray-casts the wheels against the world and applies suspension and traction forces to the chassis.
veh-api-title = C ABI
veh-api-desc = Eleven entry points cover creation, wheel setup, force or steer or brake, stepping, read-back and teardown.
veh-cap-title = Capabilities
veh-cap-lead = What the vehicle controller gives you.
veh-cap-01-title = Ray-cast suspension
veh-cap-01-desc = each wheel is a ray against the ground; suspension stiffness, compression, damping and travel are tunable per wheel.
veh-cap-02-title = Drive and steer
veh-cap-02-desc = set_engine_force drives the wheel along the forward axis; set_steering and set_brake control direction and grip.
veh-cap-03-title = Ground telemetry
veh-cap-03-desc = wheel_on_ground and wheel_contact_normal report per-wheel contact so you can build traction logic.

# ── Character Body: Minecraft-style tuning (added in Phase 3c batch)
char-mc-title = Minecraft-style tuning
char-mc-lead = Every knob a Minecraft-style mod packs into its character controller is exposed as a one-line setter, mirror-compatible with Mojang's KinematicCharacterController.
char-mc-01-title = Up axis and offset
char-mc-01-desc = set_up rotates the controller for non-Y-up worlds; set_offset_absolute / set_offset_relative shift the capsule against its rigid-body frame.
char-mc-02-title = Auto-step
char-mc-02-desc = set_autostep toggles stair/step climbing with max height, min width and an include-dynamic flag that lets the character mount moving platforms.
char-mc-03-title = Snap to ground
char-mc-03-desc = set_snap_to_ground keeps the controller glued to slopes and stairs within a configurable distance, so it never floats on vertical seams.
char-mc-04-title = Slope limits and slide
char-mc-04-desc = set_slope_angles clamps the max climb and min slide angles; set_slide lets the controller slide down otherwise-unclimbable slopes. is_grounded / is_on_ground / is_sliding_down_slope read the last move.
char-mc-note = is_on_ground is the hybrid Minecraft jump-gate: grounded OR resting on a slope (translation.y >= 0). is_grounded tracks rapier's raw grounded flag; read both when you build a jump system.

nav-changelog = Changelog
changelog-tag = ROADMAP
changelog-title = Changelog
changelog-desc = Every capability below is backed by committed code: { $jni } JNI bindings over { $ffi } C-ABI functions, exercised by { $tests } integration tests — none of it is aspirational.
changelog-grid-title = Capability milestones
changelog-grid-lead = The workspace has grown from a thin Rapier wrapper into a full body-type + controller toolkit. Each card is a real, shipped phase.
changelog-c1-title = Soft bodies (Phase 3)
changelog-c1-desc = rope / grid / SPH-fluid FFI, skinning + skeleton binding, soft-soft collision via proxy collider isolation, and fluid-rigid coupling so SPH particles stack behind terrain.
changelog-c2-title = Body types (Phase 3c-e)
changelog-c2-desc = character_body, sensor_zone, and a raycast vehicle controller — each with a dedicated C ABI surface and a matching JNI binding.
changelog-c3-title = Character feel (Minecraft)
changelog-c3-desc = autostep / snap-to-ground / slope clamps / slide, plus the hybrid is_on_ground jump-gate, tuned for blocky-world movement.
changelog-c4-title = Sensor edge-trigger
changelog-c4-desc = rising-edge latch with consume / clear so a trigger fires once and re-arms cleanly — no more double events.
changelog-c5-title = World-handle JNI parity
changelog-c5-desc = a full FFI<->JNI audit closed the gap: 149 world-handle FFI now have matching JNI, including the geometric-intersection family (capsule / ssv / ellipsoid / prism / cylinder / spherical-shell / neural-bounds).
changelog-c6-title = Query surface
changelog-c6-desc = sphere / obb / aabb / voxel / ray / point casts plus the new geometric shape sweeps, all filterable and array-friendly.
changelog-c7-title = Natural disasters & volcano
changelog-c7-desc = Typhoon / tornado / hailstorm disaster_* FFI and volcanic-eruption volcano_* FFI: plume flow, lava-bomb ejecta, rigid-body heating and progressive melting, with 18 JNI methods and doc pages.
changelog-total-title = Current scale
changelog-total-lead = Live numbers from the workspace: { $jni } JNI methods, { $ffi } C-ABI functions, { $tests } integration tests.
changelog-stat-jni = JNI methods
changelog-stat-ffi = C-ABI functions
changelog-stat-tests = integration tests

# ---- Force Queue page ----
force-queue-tag = // MPS
force-queue-title = Zero-Copy Force Queue
force-queue-desc = Shared-memory lock-free queue for Java-to-Rust force application — single producer (Java), single consumer (Rust in world_step). No JNI overhead on the hot path; forces flow through a DirectByteBuffer-backed ring buffer.

force-queue-overview-title = Overview
force-queue-overview-lead = The force queue eliminates per-force JNI calls by batching all forces into a single shared-memory ring buffer that Java writes and Rust consumes once per frame.
force-queue-overview-body = Java allocates the queue via ForceQueue (JNI) or ForceQueueFFM (FFM), writes body_id + force[3] (+ torque[3] optional) into slots, sets the bitmap bit, and advances head. The native world_step calls rigid_body_consume_force_queue once, which drains [tail, head), applies forces to Rapier bodies, clears bits, and advances tail. Zero JNI on the hot path.

force-queue-layout-title = Memory Layout
force-queue-layout-desc = The queue is a single contiguous allocation with three regions:
force-queue-layout-diagram = ForceQueueHeader (64 bytes, #[repr(C, align(64))])\n+----------+----------+----------+----------+--------+-----------+\n| capacity | head     | tail     |generation| stride |  flags    |\n|  u64     |  u64     |  u64     |   u64    |  u32   |   u32     |\n+----------+----------+----------+----------+--------+-----------+\nBitmap: (capacity + 63) / 64  x  u64  (1 bit per slot)\nPayload: capacity x stride x 8 bytes (f64 per component)\nstride = 6 (body_id + force[3])  or  7 (body_id + force[3] + torque[3])
force-queue-layout-note = Header is cache-line aligned (64 bytes). Bitmap follows immediately. Payload follows bitmap. All offsets computable from header fields — no separate pointers needed.

force-queue-sync-title = Synchronization Model
force-queue-sync-lead = Single-producer / single-consumer lock-free protocol — no CAS loops, no mutexes:
force-queue-sync-li-1 = Java writes payload + sets bitmap bit with release ordering
force-queue-sync-li-2 = Java advances head with release store
force-queue-sync-li-3 = Rust loads head with acquire load (pairs with Java release)
force-queue-sync-li-4 = Rust processes [tail, head), clears bitmap bits with release
force-queue-sync-li-5 = Rust advances tail with release store
force-queue-sync-li-6 = Generation counter on head wrap resolves ABA without heavyweight primitives

force-queue-stride-title = Stride Modes
force-queue-stride-desc = Two payload formats controlled by the stride field:
force-queue-stride-col-mode = Stride
force-queue-stride-col-desc = Layout
force-queue-stride-col-use = Use case
force-queue-stride-6-desc = body_id (f64) + force.x + force.y + force.z (6 f64 = 48 bytes)
force-queue-stride-6-use = Pure force application (most common)
force-queue-stride-7-desc = body_id + force.x + force.y + force.z + torque.x + torque.y + torque.z (7 f64 = 56 bytes)
force-queue-stride-7-use = Force + torque in one slot (e.g., thruster with moment arm)

force-queue-ffi-title = C FFI Surface
force-queue-ffi-desc = One entry point exposed in rigid_body.h:

force-queue-jni-title = Java Producer (JNI)
force-queue-jni-desc = ForceQueue.java wraps a DirectByteBuffer and uses VarHandle for atomic head/bitmap access:

force-queue-ffm-title = Java Producer (FFM / Java 25+)
force-queue-ffm-desc = ForceQueueFFM.java uses MemorySegment + VarHandle for the same protocol without JNIEnv:

force-queue-test-title = Integration Tests
force-queue-test-desc = mps-test/src/rapier/force_queue_integration.rs covers the full cycle:

force-queue-perf-title = Performance Notes
force-queue-perf-li-1 = Single world_step call per frame — zero JNI on force hot path
force-queue-perf-li-2 = Batch N forces → 1 FFI call; amortizes FFI overhead over thousands of bodies
force-queue-perf-li-3 = Cache-line aligned header + bitmap; false-sharing minimized
force-queue-perf-li-4 = Capacity power-of-2 → fast modulo via bitmask (no division)

# ---- Natural Disasters ----
nav-disasters = Disasters
dis-tag = Physics extension
dis-title = Natural Disaster Simulation
dis-desc = Typhoon / tornado / hailstorm sources attached to a WorldHandle: Rankine-vortex wind fields, hail impacts and cycrostrophic pressure drops, driving every dynamic rigid body per frame.
dis-model-title = Three disaster models
dis-model-lead = Pure field math lives in mps_formula::disasters (no Rapier state); the world-side disaster_* C ABI turns fields into forces.
dis-model-typhoon-title = Typhoon / hurricane (tropical cyclone)
dis-model-typhoon-desc = Rankine-like vortex: linear growth inside the radius of maximum wind, (Rm/r)^0.6 decay outside; radial inflow + storm translation + power-law boundary-layer height profile. spin = ±1 selects the hemisphere.
dis-model-tornado-title = Tornado
dis-model-tornado-desc = Rankine combined vortex on a vertical axis with a sinusoidal core-concentrated updraft, linear fade above the funnel top; cycrostrophic pressure drop Δp = ½ρv² is sampleable.
dis-model-hail-title = Hailstorm
dis-model-hail-desc = A horizontal disc fallout zone: hailstones falling at quadratic-drag terminal velocity apply deterministic capture impulses to dynamic bodies inside; intensity controls expected impacts per m² per second.
dis-model-note = Typhoon and hurricane are the same model under regional names — the NW Pacific calls it a typhoon, the Atlantic a hurricane; the spin sign selects the hemisphere.
dis-frame-title = Per-frame call order
dis-frame-lead = Disasters are a "field → force" pipeline: advance the storms, apply the field as forces, then run the physics step.
dis-frame-note = Pass airDensity = 0 to disasterApplyForces for the ISA sea-level default 1.225 kg/m³; both counters are per-call.
dis-api-title = C ABI overview
dis-api-note = Every entry point returns a u8 status code (ERR_OK = 0), wrapped in ffi_guard so panics never cross the FFI boundary. Vec3 splits into three doubles at the JNI layer.
dis-java-title = Java example
dis-java-lead = mps-jni exports disasterAddTyphoon / disasterApplyForces and friends on RapierNative; out parameters are native buffer pointers allocated by Java.
dis-java-note = Allocate out buffers with ByteBuffer.allocateDirect or Unsafe.allocateMemory, written in native f64 byte order.
dis-det-title = Deterministic replay
dis-det-lead = Hail impact counts and lateral scatter are generated deterministically by splitmix64 seeded from (source id + body handle).
dis-det-body = Same world state + same call sequence = bit-identical results — ideal for replay and testing. The wind field itself is a pure-function sample, deterministic by construction.

# ---- Volcano ----
nav-volcano = Volcano
vol-tag = Physics extension
vol-title = Volcanic Eruption Simulation
vol-desc = Eruptive plume flow + lava-bomb ejecta + thermal exposure: updraft drag and impacts on bodies inside the plume or near the vent, plus progressive melting by temperature (mass loss until the body is disabled in place).
vol-model-title = Three sub-models
vol-model-lead = Pure field math lives in mps_formula::volcano; the world-side volcano_* C ABI holds the volcano sources and per-body thermal / melt state.
vol-model-plume-title = Eruptive plume
vol-model-plume-desc = A linearly widening Gaussian column: core updraft decays to zero at 1.2 column heights, mushroom-cap radial outflow at the top; temperature falls from lava temperature to ambient along the same profile.
vol-model-bomb-title = Lava bombs
vol-model-bomb-desc = Deterministic near-vent ejecta (splitmix64): up-dominant capture impulses tilted radially outward with jitter, delivered as an equivalent force through the add_force pipeline.
vol-model-melt-title = Progressive melting
vol-model-melt-desc = Newton exchange towards the exposure temperature; above the melt point mass is lost linearly in superheat (set_additional_mass shrinks step by step); at 5% remaining the body is disabled in place and reported.
vol-melt-title = Melt pipeline
vol-melt-lead = Temperature and mass scale integrate explicitly per frame; all parameters (melt point / melt rate / exchange rate) are passed per apply call — give different materials different melt points under the same volcano.
vol-melt-note = massScale ≤ 5% counts as fully melted; the final frame can overshoot slightly below the threshold. Melted bodies stay disabled; volcanoClear resets thermal state but does not revive them.
vol-api-title = C ABI overview
vol-api-note = Pass lava_temperature = 0 for the basaltic default 1473.15 K; max_updraft may be 0 (lava lake: no wind, full thermal exposure); ejecta_rate = 0 disables lava bombs.
vol-java-title = Java example
vol-java-lead = mps-jni exports volcanoAdd / volcanoApply and friends on RapierNative; per-body thermal state is keyed by the packed body handle inside the world.
vol-java-note = volcanoBodyTemperature suits UI health bars / smoke effects; volcanoSetBodyTemperature enables scripted pre-heating (or igniting flammables).
vol-det-title = Lazy-refresh pitfalls
vol-det-lead = Same implementation notes as the disaster module: rigid-body mass properties are lazily computed until the first step.
vol-det-body = body.mass() reports 0 on the first frame → the mass reference is captured lazily; world_com is a stale origin on the first frame → sampling uses translation(); apply_impulse would lose the first frame → impulses flow through the add_force pipeline. volcano_apply handles all of this internally — Java code needs to know nothing about it.
