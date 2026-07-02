//! Plasma physics:
//! - Debye shielding & plasma parameters (Debye length, plasma frequency, thermal velocity)
//! - Particle-in-cell (PIC) building blocks: Boris pusher, charge deposition, field interpolation
//! - Vlasov equation moment computation (density, bulk velocity, temperature, heat flux)
//! - Self-consistent field solve (Poisson solver on grid)
//! - Magnetic reconnection: X-point detection, reconnection rate estimation
//!
//! All functions are FFI-exported with C-compatible types.

use crate::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, BorisPusherParams, ChargeDensityCell, GridField, MagneticXPoint, PicParticle,
    PicStepReport, PlasmaParamsReport, VlasovMomentReport,
};

use crate::rapier::math::{KahanSum, finite, finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-14;
const MASS_EPSILON: f64 = 1.0e-30;
const PI: f64 = std::f64::consts::PI;
const VACUUM_PERMITTIVITY: f64 = 8.854_187_812_8e-12;
const VACUUM_PERMEABILITY: f64 = 1.256_637_062_12e-6;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;
const ELECTRON_MASS: f64 = 9.109_383_56e-31;
const ELECTRON_CHARGE: f64 = 1.602_176_634e-19;
const BOLTZMANN: f64 = 1.380_649e-23;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn write_out<T: Copy>(out: *mut T, value: T) -> Bool {
    let Some(out) = (unsafe { out.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "output pointer is null");
        return Bool::FALSE;
    };
    *out = value;
    clear_error();
    Bool::TRUE
}

fn pic_particle_valid(p: &PicParticle) -> bool {
    p.x.is_finite()
        && p.y.is_finite()
        && p.z.is_finite()
        && p.vx.is_finite()
        && p.vy.is_finite()
        && p.vz.is_finite()
        && p.charge.is_finite()
        && p.mass.is_finite()
        && p.mass > 0.0
        && p.weight.is_finite()
        && p.weight > 0.0
}

fn vec3_length_sq(x: f64, y: f64, z: f64) -> f64 {
    x * x + y * y + z * z
}

fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

// ===========================================================================
// A. Debye shielding & plasma parameters
// ===========================================================================

/// Compute Debye length, plasma frequency, and related plasma parameters.
///
///   λ_D = sqrt(ε₀ k_B T_e / (n_e e²))
///   ω_pe = sqrt(n_e e² / (ε₀ m_e))
///   ω_pi = sqrt(n_i Z² e² / (ε₀ m_i))
///   N_D = (4π/3) n_e λ_D³
///   v_th = sqrt(k_B T_e / m_e)
///
/// `electron_density` — n_e (m⁻³)
/// `electron_temperature` — T_e (K)
/// `ion_density` — n_i (m⁻³, typically ≈ n_e)
/// `ion_mass` — m_i (kg, e.g. 1.672e-27 for protons)
/// `ion_charge_state` — Z (e.g. 1 for singly ionised)
#[unsafe(no_mangle)]
pub extern "C" fn pl_plasma_params(
    electron_density: f64,
    electron_temperature: f64,
    ion_density: f64,
    ion_mass: f64,
    ion_charge_state: f64,
    out_params: *mut PlasmaParamsReport,
) -> Bool {
    if !finite_positive(electron_density) || !finite_positive(electron_temperature) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "electron density and temperature must be positive",
        );
        return Bool::FALSE;
    }
    if !finite_non_negative(ion_density) {
        set_error(ERR_INVALID_ARGUMENT, "ion_density must be non-negative and finite");
        return Bool::FALSE;
    }
    if ion_density > 0.0 && (!finite_positive(ion_mass) || !finite_non_negative(ion_charge_state))
    {
        set_error(ERR_INVALID_ARGUMENT, "ion mass must be positive when ion density > 0");
        return Bool::FALSE;
    }
    if !finite_non_negative(ion_charge_state) {
        set_error(ERR_INVALID_ARGUMENT, "ion_charge_state must be non-negative and finite");
        return Bool::FALSE;
    }

    let eps0 = VACUUM_PERMITTIVITY;
    let e = ELECTRON_CHARGE;
    let me = ELECTRON_MASS;
    let kb = BOLTZMANN;

    // Debye length
    let debye_len = if electron_density > EPSILON {
        (eps0 * kb * electron_temperature / (electron_density * e * e)).sqrt()
    } else {
        f64::INFINITY
    };

    // Electron plasma frequency
    let omega_pe = if electron_density > EPSILON {
        (electron_density * e * e / (eps0 * me)).sqrt()
    } else {
        0.0
    };

    // Ion plasma frequency
    let omega_pi = if ion_density > EPSILON && ion_mass > MASS_EPSILON && ion_charge_state > EPSILON {
        let z = ion_charge_state;
        (ion_density * z * z * e * e / (eps0 * ion_mass)).sqrt()
    } else {
        0.0
    };

    // Debye sphere count
    let nd = if debye_len.is_finite() && electron_density > EPSILON {
        (4.0 * PI / 3.0) * electron_density * debye_len.powi(3)
    } else {
        0.0
    };

    // Thermal velocity
    let v_th = if electron_temperature > EPSILON {
        (kb * electron_temperature / me).sqrt()
    } else {
        0.0
    };

    write_out(
        out_params,
        PlasmaParamsReport {
            debye_length: debye_len,
            plasma_frequency: omega_pe,
            ion_plasma_frequency: omega_pi,
            debye_sphere_count: nd,
            thermal_velocity: v_th,
        },
    )
}

/// Compute the Debye length directly from density and temperature.
#[unsafe(no_mangle)]
pub extern "C" fn pl_debye_length(density: f64, temperature: f64) -> f64 {
    if !finite_positive(density) || !finite_positive(temperature) {
        set_error(ERR_INVALID_ARGUMENT, "density and temperature must be positive");
        return f64::NAN;
    }
    clear_error();
    (VACUUM_PERMITTIVITY * BOLTZMANN * temperature / (density * ELECTRON_CHARGE * ELECTRON_CHARGE))
        .sqrt()
}

/// Compute the plasma frequency from density.
#[unsafe(no_mangle)]
pub extern "C" fn pl_plasma_frequency(density: f64) -> f64 {
    if !finite_positive(density) {
        set_error(ERR_INVALID_ARGUMENT, "density must be positive");
        return f64::NAN;
    }
    clear_error();
    (density * ELECTRON_CHARGE * ELECTRON_CHARGE / (VACUUM_PERMITTIVITY * ELECTRON_MASS)).sqrt()
}

// ===========================================================================
// B. Boris particle pusher (PIC particle advance)
// ===========================================================================

/// Advance a single particle by one time step using the Boris algorithm.
///
/// The Boris pusher is a second-order accurate, symplectic integrator for
/// charged particle motion in electromagnetic fields:
///
///   1. Half-step acceleration from E-field
///   2. Rotation by B-field (gyration)
///   3. Half-step acceleration from E-field (completing the step)
///
/// References:
///   Birdsall & Langdon, "Plasma Physics via Computer Simulation"
#[unsafe(no_mangle)]
pub extern "C" fn pl_boris_push(
    particle: PicParticle,
    field: GridField,
    params: BorisPusherParams,
    out_particle: *mut PicParticle,
) -> Bool {
    if !pic_particle_valid(&particle) {
        set_error(ERR_INVALID_ARGUMENT, "invalid particle");
        return Bool::FALSE;
    }
    if !finite_positive(params.dt) {
        set_error(ERR_INVALID_ARGUMENT, "dt must be positive");
        return Bool::FALSE;
    }
    if !params.charge_to_mass_ratio.is_finite() {
        set_error(ERR_INVALID_ARGUMENT, "charge_to_mass_ratio must be finite");
        return Bool::FALSE;
    }

    let dt = params.dt;
    let qm = params.charge_to_mass_ratio; // q/m

    // Normalise: v → v⁻ (half-step back)
    let v_minus_x = particle.vx + 0.5 * dt * qm * field.ex;
    let v_minus_y = particle.vy + 0.5 * dt * qm * field.ey;
    let v_minus_z = particle.vz + 0.5 * dt * qm * field.ez;

    // Rotation by B-field
    // t = (q/m) * B * dt/2
    let tx = 0.5 * dt * qm * field.bx;
    let ty = 0.5 * dt * qm * field.by;
    let tz = 0.5 * dt * qm * field.bz;
    let t_sq = vec3_length_sq(tx, ty, tz);

    // v' = v⁻ + v⁻ × t
    let v_prime_x = v_minus_x + (v_minus_y * tz - v_minus_z * ty);
    let v_prime_y = v_minus_y + (v_minus_z * tx - v_minus_x * tz);
    let v_prime_z = v_minus_z + (v_minus_x * ty - v_minus_y * tx);

    // s = 2t / (1 + t²)
    let s_scale = 2.0 / (1.0 + t_sq);
    let sx = tx * s_scale;
    let sy = ty * s_scale;
    let sz = tz * s_scale;

    // v⁺ = v⁻ + v' × s
    let v_plus_x = v_minus_x + (v_prime_y * sz - v_prime_z * sy);
    let v_plus_y = v_minus_y + (v_prime_z * sx - v_prime_x * sz);
    let v_plus_z = v_minus_z + (v_prime_x * sy - v_prime_y * sx);

    // v^{n+1} = v⁺ + half-step E push
    let v_next_x = v_plus_x + 0.5 * dt * qm * field.ex;
    let v_next_y = v_plus_y + 0.5 * dt * qm * field.ey;
    let v_next_z = v_plus_z + 0.5 * dt * qm * field.ez;

    // Position update: x^{n+1} = x^n + dt * v^{n+1}
    let x_next = particle.x + dt * v_next_x;
    let y_next = particle.y + dt * v_next_y;
    let z_next = particle.z + dt * v_next_z;

    write_out(
        out_particle,
        PicParticle {
            x: x_next,
            y: y_next,
            z: z_next,
            vx: v_next_x,
            vy: v_next_y,
            vz: v_next_z,
            charge: particle.charge,
            mass: particle.mass,
            weight: particle.weight,
        },
    )
}

/// Interpolate the electromagnetic field from a grid cell to the particle
/// position using first-order (linear / area-weighted) interpolation.
///
/// `grid` — pointer to a 3D array of `GridField` of size nx × ny × nz,
/// stored in row-major order (x-fastest, then y, then z).
/// `cell_size` — grid cell size (uniform in all directions).
/// `origin_x/y/z` — position of grid cell centre (0,0,0).
///
/// Returns the interpolated field at the particle position.
#[unsafe(no_mangle)]
pub extern "C" fn pl_interpolate_field(
    grid: *const GridField,
    nx: u32,
    ny: u32,
    nz: u32,
    cell_size: f64,
    origin_x: f64,
    origin_y: f64,
    origin_z: f64,
    particle_x: f64,
    particle_y: f64,
    particle_z: f64,
    out_field: *mut GridField,
) -> Bool {
    if grid.is_null() {
        set_error(ERR_NULL_POINTER, "grid pointer is null");
        return Bool::FALSE;
    }
    if nx < 2 || ny < 2 || nz < 2 {
        set_error(ERR_INVALID_ARGUMENT, "grid must have at least 2 cells in each dimension");
        return Bool::FALSE;
    }
    if !finite_positive(cell_size) {
        set_error(ERR_INVALID_ARGUMENT, "cell_size must be positive and finite");
        return Bool::FALSE;
    }
    if !finite(origin_x) || !finite(origin_y) || !finite(origin_z)
        || !finite(particle_x) || !finite(particle_y) || !finite(particle_z)
    {
        set_error(ERR_INVALID_ARGUMENT, "all coordinates must be finite");
        return Bool::FALSE;
    }

    let cells = unsafe { std::slice::from_raw_parts(grid, (nx as usize) * (ny as usize) * (nz as usize)) };

    // Compute grid indices (cell-centre coordinates)
    let ix_f = (particle_x - origin_x) / cell_size;
    let iy_f = (particle_y - origin_y) / cell_size;
    let iz_f = (particle_z - origin_z) / cell_size;

    let ix = clamp(ix_f.floor(), 0.0, (nx as f64) - 2.0) as usize;
    let iy = clamp(iy_f.floor(), 0.0, (ny as f64) - 2.0) as usize;
    let iz = clamp(iz_f.floor(), 0.0, (nz as f64) - 2.0) as usize;

    // Local coordinates within the cell [0, 1)
    let wx = ix_f - ix as f64;
    let wy = iy_f - iy as f64;
    let wz = iz_f - iz as f64;

    let nx_u = nx as usize;
    let ny_u = ny as usize;

    // Trilinear interpolation: E(P) = Σ Σ Σ w_i·w_j·w_k · E(i,j,k)
    let idx000 = iz * nx_u * ny_u + iy * nx_u + ix;
    let idx001 = iz * nx_u * ny_u + iy * nx_u + ix + 1;
    let idx010 = iz * nx_u * ny_u + (iy + 1) * nx_u + ix;
    let idx011 = iz * nx_u * ny_u + (iy + 1) * nx_u + ix + 1;
    let idx100 = (iz + 1) * nx_u * ny_u + iy * nx_u + ix;
    let idx101 = (iz + 1) * nx_u * ny_u + iy * nx_u + ix + 1;
    let idx110 = (iz + 1) * nx_u * ny_u + (iy + 1) * nx_u + ix;
    let idx111 = (iz + 1) * nx_u * ny_u + (iy + 1) * nx_u + ix + 1;

    let w000 = (1.0 - wx) * (1.0 - wy) * (1.0 - wz);
    let w001 = wx * (1.0 - wy) * (1.0 - wz);
    let w010 = (1.0 - wx) * wy * (1.0 - wz);
    let w011 = wx * wy * (1.0 - wz);
    let w100 = (1.0 - wx) * (1.0 - wy) * wz;
    let w101 = wx * (1.0 - wy) * wz;
    let w110 = (1.0 - wx) * wy * wz;
    let w111 = wx * wy * wz;

    let fx = |c: &GridField| c.ex;
    let fy = |c: &GridField| c.ey;
    let fz = |c: &GridField| c.ez;
    let bx = |c: &GridField| c.bx;
    let by = |c: &GridField| c.by;
    let bz = |c: &GridField| c.bz;

    let interp = |f: fn(&GridField) -> f64| -> f64 {
        w000 * f(&cells[idx000])
            + w001 * f(&cells[idx001])
            + w010 * f(&cells[idx010])
            + w011 * f(&cells[idx011])
            + w100 * f(&cells[idx100])
            + w101 * f(&cells[idx101])
            + w110 * f(&cells[idx110])
            + w111 * f(&cells[idx111])
    };

    write_out(
        out_field,
        GridField {
            ex: interp(fx),
            ey: interp(fy),
            ez: interp(fz),
            bx: interp(bx),
            by: interp(by),
            bz: interp(bz),
        },
    )
}

// ===========================================================================
// C. Charge deposition (particle → grid)
// ===========================================================================

/// Deposit a single particle's charge and current onto a grid cell using
/// first-order (Cloud-in-Cell) weighting.
///
/// The charge density contribution is: ρ = q · w / V_cell
/// The current density contribution is: j = ρ · v
///
/// `cell_volume` — volume of a single grid cell (m³).
#[unsafe(no_mangle)]
pub extern "C" fn pl_deposit_particle(
    particle: PicParticle,
    cell_size: f64,
    cell_volume: f64,
    out_density: *mut ChargeDensityCell,
) -> Bool {
    if !pic_particle_valid(&particle) {
        set_error(ERR_INVALID_ARGUMENT, "invalid particle");
        return Bool::FALSE;
    }
    if !finite_positive(cell_size) || !finite_positive(cell_volume) {
        set_error(ERR_INVALID_ARGUMENT, "cell_size and cell_volume must be positive");
        return Bool::FALSE;
    }

    let rho = particle.charge * particle.weight / cell_volume;
    let jx = rho * particle.vx;
    let jy = rho * particle.vy;
    let jz = rho * particle.vz;

    write_out(
        out_density,
        ChargeDensityCell {
            rho,
            jx,
            jy,
            jz,
        },
    )
}

// ===========================================================================
// D. Vlasov equation → moment computation
// ===========================================================================

/// Compute velocity moments of a distribution function from a set of
/// macroparticles. This is a reduced representation of the Vlasov equation:
///
///   n = Σ wⱼ
///   u = (1/n) Σ wⱼ vⱼ
///   T = (m/3n) Σ wⱼ |vⱼ − u|²    (isotropic temperature)
///   q = (m/2) Σ wⱼ (vⱼ − u)² (vⱼ − u)   (heat flux)
///
/// `particles` — pointer to array of `PicParticle`.
/// `count` — number of particles.
/// `out_moments` — computed moments.
#[unsafe(no_mangle)]
pub extern "C" fn pl_vlasov_moments(
    particles: *const PicParticle,
    count: u32,
    out_moments: *mut VlasovMomentReport,
) -> Bool {
    if particles.is_null() {
        set_error(ERR_NULL_POINTER, "particles pointer is null");
        return Bool::FALSE;
    }
    if count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "count must be > 0");
        return Bool::FALSE;
    }

    let parts = unsafe { std::slice::from_raw_parts(particles, count as usize) };

    // First pass: compute density and bulk velocity
    let mut total_weight = 0.0_f64;
    let mut weighted_vx = 0.0_f64;
    let mut weighted_vy = 0.0_f64;
    let mut weighted_vz = 0.0_f64;
    let mut _total_mass_weight = 0.0_f64;

    for p in parts {
        if !pic_particle_valid(p) {
            continue;
        }
        total_weight += p.weight;
        weighted_vx += p.weight * p.vx;
        weighted_vy += p.weight * p.vy;
        weighted_vz += p.weight * p.vz;
        _total_mass_weight += p.weight * p.mass;
    }

    if total_weight < EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "total particle weight is zero");
        return Bool::FALSE;
    }

    let density = total_weight;
    let ux = weighted_vx / total_weight;
    let uy = weighted_vy / total_weight;
    let uz = weighted_vz / total_weight;

    // Second pass: temperature (average kinetic energy in the bulk frame)
    let mut thermal_energy = 0.0_f64;
    let mut hfx = 0.0_f64;
    let mut hfy = 0.0_f64;
    let mut hfz = 0.0_f64;

    for p in parts {
        if !pic_particle_valid(p) {
            continue;
        }
        let dvx = p.vx - ux;
        let dvy = p.vy - uy;
        let dvz = p.vz - uz;
        let ke = 0.5 * p.mass * (dvx * dvx + dvy * dvy + dvz * dvz);
        thermal_energy += p.weight * ke;

        // Heat flux: (m/2) * w * |dv|² * dv
        let hf_mag = 0.5 * p.mass * p.weight * (dvx * dvx + dvy * dvy + dvz * dvz);
        hfx += hf_mag * dvx;
        hfy += hf_mag * dvy;
        hfz += hf_mag * dvz;
    }

    // Temperature: T = (2/3) * (average kinetic energy in bulk frame) / k_B
    // Here we report temperature in energy units (J) as the isotropic pressure / density
    let temperature = if total_weight > EPSILON {
        (2.0 / 3.0) * thermal_energy / total_weight
    } else {
        0.0
    };

    // Normalise heat flux by density
    let qnorm = if total_weight > EPSILON {
        1.0 / total_weight
    } else {
        1.0
    };

    write_out(
        out_moments,
        VlasovMomentReport {
            density,
            ux,
            uy,
            uz,
            temperature,
            qx: hfx * qnorm,
            qy: hfy * qnorm,
            qz: hfz * qnorm,
        },
    )
}

// ===========================================================================
// E. Self-consistent Poisson solve (simplified)
// ===========================================================================

/// Solve Poisson's equation ∇²φ = −ρ/ε₀ on a 1D grid using a simple
/// tridiagonal (finite-difference) solver with Dirichlet boundary conditions
/// (φ = 0 at both ends).
///
/// `rho` — charge density array (C/m³), length `n`.
/// `dx` — grid spacing (m).
/// `phi_out` — pre-allocated output array for potential φ (V).
/// `e_out` — pre-allocated output array for electric field E = −dφ/dx (V/m).
///
/// Returns Bool::TRUE on success.
#[unsafe(no_mangle)]
pub extern "C" fn pl_poisson_solve_1d(
    rho: *const f64,
    n: u32,
    dx: f64,
    phi_out: *mut f64,
    e_out: *mut f64,
) -> Bool {
    if rho.is_null() || phi_out.is_null() || e_out.is_null() {
        set_error(ERR_NULL_POINTER, "pointer is null");
        return Bool::FALSE;
    }
    if n < 3 {
        set_error(ERR_INVALID_ARGUMENT, "n must be >= 3");
        return Bool::FALSE;
    }
    if !finite_positive(dx) {
        set_error(ERR_INVALID_ARGUMENT, "dx must be positive and finite");
        return Bool::FALSE;
    }

    let n_u = n as usize;
    let rho_arr = unsafe { std::slice::from_raw_parts(rho, n_u) };
    let phi = unsafe { std::slice::from_raw_parts_mut(phi_out, n_u) };
    let e_arr = unsafe { std::slice::from_raw_parts_mut(e_out, n_u) };

    let eps0 = VACUUM_PERMITTIVITY;
    let dx2 = dx * dx;

    // Thomas algorithm for tridiagonal system:
    // φ_{i-1} - 2φ_i + φ_{i+1} = −ρ_i dx² / ε₀
    // Boundary: φ_0 = φ_{n-1} = 0

    phi[0] = 0.0;
    phi[n_u - 1] = 0.0;

    // Forward sweep
    let mut c_prime = vec![0.0_f64; n_u];
    let mut d_prime = vec![0.0_f64; n_u];

    // For i = 1 (first interior point)
    // a_i = 1, b_i = −2, c_i = 1
    // c'_1 = c_1 / b_1
    // d'_1 = d_1 / b_1
    if n_u > 2 {
        let rhs = -rho_arr[1] * dx2 / eps0;
        c_prime[1] = 1.0 / (-2.0);
        d_prime[1] = rhs / (-2.0);

        for i in 2..n_u - 1 {
            let rhs = -rho_arr[i] * dx2 / eps0;
            let denom = -2.0 - c_prime[i - 1];
            if denom.abs() < EPSILON {
                set_error(ERR_INVALID_ARGUMENT, "singular tridiagonal matrix");
                return Bool::FALSE;
            }
            c_prime[i] = 1.0 / denom;
            d_prime[i] = (rhs - d_prime[i - 1]) / denom;
        }

        // Back substitution
        for i in (1..n_u - 1).rev() {
            phi[i] = d_prime[i] - c_prime[i] * phi[i + 1];
        }
    }

    // Electric field: E_i = −(φ_{i+1} − φ_{i-1}) / (2dx)
    e_arr[0] = -(phi[1] - phi[0]) / dx;
    for i in 1..n_u - 1 {
        e_arr[i] = -(phi[i + 1] - phi[i - 1]) / (2.0 * dx);
    }
    e_arr[n_u - 1] = -(phi[n_u - 1] - phi[n_u - 2]) / dx;

    clear_error();
    Bool::TRUE
}

// ===========================================================================
// F. Magnetic reconnection: X-point detection
// ===========================================================================

/// Detect a magnetic X-point (reconnection site) in a 2D plane from the
/// magnetic field components Bx, By on a regular grid.
///
/// An X-point is characterised by B = 0 (or very small) with a hyperbolic
/// null topology: Bx ∝ (x − x₀), By ∝ −(y − y₀) (or rotated).
///
/// `bx_grid` — pointer to Bx array (T), size nx × ny, row-major.
/// `by_grid` — pointer to By array (T).
/// `nx`, `ny` — grid dimensions.
/// `cell_size` — uniform grid cell size (m).
/// `origin_x`, `origin_y` — position of grid cell (0,0) centre.
/// `threshold` — maximum |B| at a null point (T).
///
/// Returns the first X-point found (if any).
#[unsafe(no_mangle)]
pub extern "C" fn pl_find_xpoint(
    bx_grid: *const f64,
    by_grid: *const f64,
    nx: u32,
    ny: u32,
    cell_size: f64,
    origin_x: f64,
    origin_y: f64,
    threshold: f64,
    out_xpoint: *mut MagneticXPoint,
) -> Bool {
    if bx_grid.is_null() || by_grid.is_null() {
        set_error(ERR_NULL_POINTER, "grid pointer is null");
        return Bool::FALSE;
    }
    if nx < 3 || ny < 3 {
        set_error(ERR_INVALID_ARGUMENT, "grid must be at least 3×3");
        return Bool::FALSE;
    }
    if !finite_positive(cell_size) || !finite_non_negative(threshold) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "cell_size must be positive, threshold non-negative",
        );
        return Bool::FALSE;
    }
    if !finite(origin_x) || !finite(origin_y) {
        set_error(ERR_INVALID_ARGUMENT, "origin coordinates must be finite");
        return Bool::FALSE;
    }

    let nu = (nx as usize) * (ny as usize);
    let bx = unsafe { std::slice::from_raw_parts(bx_grid, nu) };
    let by = unsafe { std::slice::from_raw_parts(by_grid, nu) };

    let idx = |ix: usize, iy: usize| -> usize { iy * (nx as usize) + ix };

    // Scan interior cells for B ≈ 0
    for iy in 1..(ny as usize - 1) {
        for ix in 1..(nx as usize - 1) {
            let b_mag_sq = bx[idx(ix, iy)] * bx[idx(ix, iy)]
                + by[idx(ix, iy)] * by[idx(ix, iy)];

            if b_mag_sq > threshold * threshold {
                continue;
            }

            // Compute Jacobian (gradient of B) using centred differences
            let dbx_dx = (bx[idx(ix + 1, iy)] - bx[idx(ix - 1, iy)]) / (2.0 * cell_size);
            let dbx_dy = (bx[idx(ix, iy + 1)] - bx[idx(ix, iy - 1)]) / (2.0 * cell_size);
            let dby_dx = (by[idx(ix + 1, iy)] - by[idx(ix - 1, iy)]) / (2.0 * cell_size);
            let dby_dy = (by[idx(ix, iy + 1)] - by[idx(ix, iy - 1)]) / (2.0 * cell_size);

            // For an X-point, the determinant of ∇B should be negative
            // (one eigenvalue positive, one negative — hyperbolic null)
            let det = dbx_dx * dby_dy - dbx_dy * dby_dx;

            if det < 0.0 {
                // Compute position of the null
                let x_pos = origin_x + (ix as f64) * cell_size;
                let y_pos = origin_y + (iy as f64) * cell_size;

                // Shear angle: angle between the separatrices
                let trace = dbx_dx + dby_dy;
                let discriminant = (trace * trace - 4.0 * det).max(0.0);
                let shear_angle = discriminant.sqrt().atan(); // hyperbolic angle

                // Reconnection rate: E = ηJ at the null
                // Simplified estimate using the in-plane current at the null
                let jz = (dby_dx - dbx_dy) / VACUUM_PERMEABILITY;
                let resistivity = 1.0e-4; // typical anomalous resistivity (Ω·m)
                let reconnection_rate = (resistivity * jz / SPEED_OF_LIGHT).abs();

                return write_out(
                    out_xpoint,
                    MagneticXPoint {
                        x: x_pos,
                        y: y_pos,
                        z: 0.0,
                        shear_angle,
                        reconnection_rate,
                        valid: Bool::TRUE,
                    },
                );
            }
        }
    }

    // No X-point found
    write_out(
        out_xpoint,
        MagneticXPoint {
            valid: Bool::FALSE,
            ..MagneticXPoint::default()
        },
    )
}

/// Compute the Sweet–Parker reconnection rate estimate.
///
///   R = v_in / v_A = 1 / √S
///
/// where S = μ₀ L v_A / η is the Lundquist number.
///
/// `lundquist_number` — S = μ₀ L_A v_A / η (dimensionless).
#[unsafe(no_mangle)]
pub extern "C" fn pl_sweet_parker_rate(lundquist_number: f64) -> f64 {
    if !finite_positive(lundquist_number) {
        set_error(ERR_INVALID_ARGUMENT, "Lundquist number must be positive and finite");
        return f64::NAN;
    }
    clear_error();
    1.0 / lundquist_number.sqrt()
}

/// Compute the Petschek fast reconnection rate estimate.
///
///   R ≈ π / (4 ln S)
///
/// `lundquist_number` — S (dimensionless).
#[unsafe(no_mangle)]
pub extern "C" fn pl_petschek_rate(lundquist_number: f64) -> f64 {
    if !finite_positive(lundquist_number) || lundquist_number <= 1.0 {
        set_error(
            ERR_INVALID_ARGUMENT,
            "Lundquist number must be > 1",
        );
        return f64::NAN;
    }
    clear_error();
    PI / (4.0 * lundquist_number.ln())
}

/// Compute the Alfvén speed v_A = B / √(μ₀ n m_i).
#[unsafe(no_mangle)]
pub extern "C" fn pl_alfven_speed(
    magnetic_field: f64,
    density: f64,
    ion_mass: f64,
) -> f64 {
    if !finite_non_negative(magnetic_field) {
        set_error(ERR_INVALID_ARGUMENT, "magnetic_field must be non-negative");
        return f64::NAN;
    }
    if !finite_positive(density) || !finite_positive(ion_mass) {
        set_error(ERR_INVALID_ARGUMENT, "density and ion_mass must be positive");
        return f64::NAN;
    }
    clear_error();
    magnetic_field / (VACUUM_PERMEABILITY * density * ion_mass).sqrt()
}

/// Compute the Lundquist number S = μ₀ L v_A / η.
#[unsafe(no_mangle)]
pub extern "C" fn pl_lundquist_number(
    length_scale: f64,
    alfven_speed: f64,
    resistivity: f64,
) -> f64 {
    if !finite_positive(length_scale) || !finite_positive(alfven_speed)
        || !finite_positive(resistivity)
    {
        set_error(ERR_INVALID_ARGUMENT, "all parameters must be positive and finite");
        return f64::NAN;
    }
    clear_error();
    VACUUM_PERMEABILITY * length_scale * alfven_speed / resistivity
}

// ===========================================================================
// G. PIC simulation step summary
// ===========================================================================

/// Compute a summary report for a PIC simulation step from an array of
/// particles and a field grid.
///
/// `particles` — pointer to array of `PicParticle`.
/// `particle_count` — number of particles.
/// `grid` — pointer to array of `GridField`.
/// `grid_cells` — total number of grid cells.
#[unsafe(no_mangle)]
pub extern "C" fn pl_pic_step_report(
    particles: *const PicParticle,
    particle_count: u32,
    grid: *const GridField,
    grid_cells: u32,
    out_report: *mut PicStepReport,
) -> Bool {
    if particles.is_null() || grid.is_null() {
        set_error(ERR_NULL_POINTER, "input pointer is null");
        return Bool::FALSE;
    }
    if particle_count == 0 {
        set_error(ERR_INVALID_ARGUMENT, "particle_count must be > 0");
        return Bool::FALSE;
    }
    if grid_cells == 0 {
        set_error(ERR_INVALID_ARGUMENT, "grid_cells must be > 0");
        return Bool::FALSE;
    }

    let parts = unsafe { std::slice::from_raw_parts(particles, particle_count as usize) };
    let cells = unsafe { std::slice::from_raw_parts(grid, grid_cells as usize) };

    let mut total_ke_acc = KahanSum::default();
    let mut max_e = 0.0_f64;
    let mut max_b = 0.0_f64;

    for p in parts {
        if pic_particle_valid(p) {
            let v2 = p.vx * p.vx + p.vy * p.vy + p.vz * p.vz;
            total_ke_acc.add(0.5 * p.mass * p.weight * v2);
        }
    }

    for c in cells {
        let e_mag = (c.ex * c.ex + c.ey * c.ey + c.ez * c.ez).sqrt();
        let b_mag = (c.bx * c.bx + c.by * c.by + c.bz * c.bz).sqrt();
        if e_mag > max_e {
            max_e = e_mag;
        }
        if b_mag > max_b {
            max_b = b_mag;
        }
    }

    // Field energy density: (ε₀ E² + B²/μ₀) / 2 per cell
    // Report the total as approximate
    let mut field_energy_acc = KahanSum::default();
    for c in cells {
        let e2 = c.ex * c.ex + c.ey * c.ey + c.ez * c.ez;
        let b2 = c.bx * c.bx + c.by * c.by + c.bz * c.bz;
        field_energy_acc.add(0.5 * VACUUM_PERMITTIVITY * e2 + 0.5 * b2 / VACUUM_PERMEABILITY);
    }

    write_out(
        out_report,
        PicStepReport {
            particle_count,
            max_density: 0.0,
            max_electric_field: max_e,
            max_magnetic_field: max_b,
            total_kinetic_energy: total_ke_acc.value(),
            total_field_energy: field_energy_acc.value(),
        },
    )
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rapier::ffi::GridField;

    #[test]
    fn debye_length_positive() {
        let d = pl_debye_length(1e20, 1e4);
        assert!(d.is_finite() && d > 0.0);
        // For a typical fusion plasma: n_e = 10²⁰, T_e = 10⁴ K
        // λ_D ≈ √(ε₀ kT / n e²) ≈ 7e-6 m
        assert!(d > 1e-7 && d < 1e-4);
    }

    #[test]
    fn plasma_frequency_positive() {
        let f = pl_plasma_frequency(1e20);
        assert!(f.is_finite() && f > 0.0);
        // ω_pe ≈ √(10²⁰ * e² / ε₀ m_e) ≈ 5.6e11 rad/s
        assert!(f > 1e10 && f < 1e13);
    }

    #[test]
    fn plasma_params_self_consistent() {
        let mut params = PlasmaParamsReport::default();
        assert_eq!(
            pl_plasma_params(1e20, 1e4, 1e20, 1.672e-27, 1.0, &mut params),
            Bool::TRUE
        );
        assert!(params.debye_length > 0.0);
        assert!(params.plasma_frequency > 0.0);
        assert!(params.ion_plasma_frequency > 0.0);
        // ω_pi should be much smaller than ω_pe
        assert!(params.ion_plasma_frequency < params.plasma_frequency);
        assert!(params.debye_sphere_count > 0.0);
        assert!(params.thermal_velocity > 0.0);
    }

    #[test]
    fn boris_push_conserves_energy_in_b_field() {
        // Pure B-field: kinetic energy should be conserved
        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 1e6, vy: 0.0, vz: 0.0,
            charge: -ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1.0,
        };
        let field = GridField {
            ex: 0.0, ey: 0.0, ez: 0.0,
            bx: 0.0, by: 0.0, bz: 1.0, // 1 T along z
        };
        let params = BorisPusherParams {
            dt: 1e-11,
            charge_to_mass_ratio: -ELECTRON_CHARGE / ELECTRON_MASS,
        };

        let mut next = PicParticle::default();
        let ke_initial = 0.5 * ELECTRON_MASS * 1e12;

        let mut ke_min = f64::MAX;
        let mut ke_max = 0.0_f64;

        let mut p = particle;
        for _ in 0..100 {
            assert_eq!(pl_boris_push(p, field, params, &mut next), Bool::TRUE);
            let ke = 0.5 * ELECTRON_MASS * (next.vx * next.vx + next.vy * next.vy + next.vz * next.vz);
            if ke < ke_min {
                ke_min = ke;
            }
            if ke > ke_max {
                ke_max = ke;
            }
            p = next;
        }
        // Energy should be conserved to within 1% over 100 gyro-steps
        let drift = (ke_max - ke_min) / ke_initial;
        assert!(drift < 0.01, "energy drift {drift} > 1%");
    }

    #[test]
    fn boris_push_accelerates_in_e_field() {
        const E_FIELD: f64 = 1e5; // 100 kV/m
        const DT: f64 = 1e-9;
        const STEPS: u32 = 10;

        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 0.0, vy: 0.0, vz: 0.0,
            charge: -ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1.0,
        };
        let field = GridField {
            ex: E_FIELD, ey: 0.0, ez: 0.0,
            bx: 0.0, by: 0.0, bz: 0.0,
        };
        let params = BorisPusherParams {
            dt: DT,
            charge_to_mass_ratio: -ELECTRON_CHARGE / ELECTRON_MASS,
        };

        let mut p = particle;
        for _ in 0..STEPS {
            let mut next = PicParticle::default();
            assert_eq!(pl_boris_push(p, field, params, &mut next), Bool::TRUE);
            p = next;
        }
        // After N steps in constant E-field: v = qE/m * N*dt
        let expected_v = -ELECTRON_CHARGE / ELECTRON_MASS * E_FIELD * DT * (STEPS as f64);
        assert!(
            (p.vx - expected_v).abs() / expected_v.abs() < 0.01,
            "velocity should be ≈{expected_v}, got {}",
            p.vx
        );
    }

    #[test]
    fn interpolate_field_trilinear() {
        let mut grid = [GridField::default(); 27]; // 3×3×3
        // Set a uniform E-field in the x-direction
        for cell in grid.iter_mut() {
            cell.ex = 1.0;
        }
        let mut out = GridField::default();
        assert_eq!(
            pl_interpolate_field(
                grid.as_ptr(), 3, 3, 3, 1.0,
                0.0, 0.0, 0.0,
                0.5, 0.5, 0.5,
                &mut out,
            ),
            Bool::TRUE
        );
        assert!((out.ex - 1.0).abs() < 1e-12);
    }

    #[test]
    fn deposit_particle_charge_conserved() {
        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 1e5, vy: 0.0, vz: 0.0,
            charge: ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1e10,
        };
        let mut density = ChargeDensityCell::default();
        assert_eq!(
            pl_deposit_particle(particle, 1e-3, 1e-9, &mut density),
            Bool::TRUE
        );
        // Total charge in cell: q*w = e * 1e10
        let total_charge = density.rho * 1e-9; // rho * V_cell
        assert!((total_charge - ELECTRON_CHARGE * 1e10).abs() / (ELECTRON_CHARGE * 1e10) < 1e-10);
    }

    #[test]
    fn vlasov_moments_single_particle() {
        // Single particle has zero temperature in its own bulk frame
        let particle = PicParticle {
            x: 0.0, y: 0.0, z: 0.0,
            vx: 1e5, vy: 2e5, vz: -1e5,
            charge: ELECTRON_CHARGE,
            mass: ELECTRON_MASS,
            weight: 1.0,
        };
        let mut moments = VlasovMomentReport::default();
        assert_eq!(
            pl_vlasov_moments(&particle, 1, &mut moments),
            Bool::TRUE
        );
        assert!((moments.density - 1.0).abs() < 1e-12);
        assert!((moments.ux - 1e5).abs() < 1.0);
        assert!((moments.uy - 2e5).abs() < 1.0);
        assert!((moments.uz + 1e5).abs() < 1.0);
        // For a single particle, T = 0 (no thermal spread around bulk)
        assert!(moments.temperature == 0.0 || moments.temperature.abs() < 1e-20);

        // With multiple particles having different velocities, T > 0
        let particles = [
            PicParticle {
                vx: 1e5, vy: 0.0, vz: 0.0,
                ..particle
            },
            PicParticle {
                vx: -1e5, vy: 0.0, vz: 0.0,
                ..particle
            },
        ];
        let mut moments2 = VlasovMomentReport::default();
        assert_eq!(
            pl_vlasov_moments(particles.as_ptr(), 2, &mut moments2),
            Bool::TRUE
        );
        assert!(
            moments2.temperature > 0.0,
            "temperature should be positive for thermal spread"
        );
        // Bulk velocity should be zero (symmetric)
        assert!(moments2.ux.abs() < 1.0);
    }

    #[test]
    fn poisson_solve_1d_linear_potential() {
        // Uniform charge density → parabolic potential
        let n = 10u32;
        let rho = vec![1.0; n as usize];
        let mut phi = vec![0.0_f64; n as usize];
        let mut e = vec![0.0_f64; n as usize];

        assert_eq!(
            pl_poisson_solve_1d(rho.as_ptr(), n, 0.1, phi.as_mut_ptr(), e.as_mut_ptr()),
            Bool::TRUE
        );
        // φ should be symmetric (parabolic) with maximum at the centre
        assert!(phi[0] == 0.0);
        assert!(phi[n as usize - 1] == 0.0);
        for val in phi.iter().take(n as usize - 1).skip(1) {
            assert!(*val > 0.0, "potential should be positive inside");
        }
        // E-field should be anti-symmetric
        assert!((e[0] + e[n as usize - 1]).abs() < 1e-10 || e[0] * e[n as usize - 1] < 0.0);
    }

    #[test]
    fn find_xpoint_detects_null() {
        // Create a 2D hyperbolic null: Bx = y, By = x
        let nx = 10u32;
        let ny = 10u32;
        let cell = 1.0;
        let mut bx = vec![0.0_f64; (nx * ny) as usize];
        let mut by = vec![0.0_f64; (nx * ny) as usize];

        for iy in 0..ny {
            for ix in 0..nx {
                let idx = (iy * nx + ix) as usize;
                let x = (ix as f64) - (nx as f64 / 2.0);
                let y = (iy as f64) - (ny as f64 / 2.0);
                bx[idx] = y;
                by[idx] = x;
            }
        }

        let mut xpoint = MagneticXPoint::default();
        assert_eq!(
            pl_find_xpoint(
                bx.as_ptr(), by.as_ptr(),
                nx, ny, cell,
                -(nx as f64 / 2.0), -(ny as f64 / 2.0),
                1.0,
                &mut xpoint,
            ),
            Bool::TRUE
        );
        assert_eq!(xpoint.valid, Bool::TRUE);
    }

    #[test]
    fn sweet_parker_and_petschek_rates() {
        let s = 1e8;
        let r_sp = pl_sweet_parker_rate(s);
        assert!(r_sp.is_finite() && r_sp > 0.0);
        // Sweet–Parker: R = 1/√S = 1e-4
        assert!((r_sp - 1e-4).abs() < 1e-6);

        let r_pet = pl_petschek_rate(s);
        assert!(r_pet.is_finite() && r_pet > 0.0);
        // Petschek: R ≈ π / (4 ln S) ≈ 0.043
        assert!(r_pet > r_sp);
    }

    #[test]
    fn alfven_speed_finite() {
        let v = pl_alfven_speed(1.0, 1e20, 1.672e-27);
        assert!(v.is_finite() && v > 0.0);
        // v_A ≈ 69 km/s for B=1T, n=10²⁰, m_i=proton
        assert!(v > 1e3 && v < 1e7, "v_A = {v} should be ~2.2e6");
    }

    #[test]
    fn pic_step_report_generates_stats() {
        let particles = [
            PicParticle {
                x: 0.0, y: 0.0, z: 0.0,
                vx: 1e5, vy: 0.0, vz: 0.0,
                charge: ELECTRON_CHARGE, mass: ELECTRON_MASS, weight: 1.0,
            },
        ];
        let cells = [GridField {
            ex: 1e4, ey: 0.0, ez: 0.0,
            bx: 1.0, by: 0.0, bz: 0.0,
        }];
        let mut report = PicStepReport::default();
        assert_eq!(
            pl_pic_step_report(
                particles.as_ptr(), 1,
                cells.as_ptr(), 1,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(report.particle_count, 1);
        assert!(report.total_kinetic_energy > 0.0);
        assert!(report.max_electric_field > 0.0);
        assert!(report.total_field_energy > 0.0);
    }

    #[test]
    fn null_pointer_rejected() {
        let p = PicParticle::default();
        let f = GridField::default();
        let bp = BorisPusherParams::default();
        assert_eq!(
            pl_boris_push(p, f, bp, std::ptr::null_mut()),
            Bool::FALSE
        );
    }
}