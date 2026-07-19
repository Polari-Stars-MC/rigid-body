/* tslint:disable */
/* eslint-disable */

/**
 * Snapshot entry for one body — used for batch sync with Three.js.
 * Layout per body: 13 f64 values.
 */
export class BodySnapshotEntry {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Angular velocity (x, y, z) at offset 10
     */
    angvel_x: number;
    angvel_y: number;
    angvel_z: number;
    /**
     * Position (x, y, z) at offset 0
     */
    pos_x: number;
    pos_y: number;
    pos_z: number;
    /**
     * Rotation quaternion (i, j, k, w) at offset 3
     */
    rot_i: number;
    rot_j: number;
    rot_k: number;
    rot_w: number;
    /**
     * Linear velocity (x, y, z) at offset 7
     */
    vel_x: number;
    vel_y: number;
    vel_z: number;
}

/**
 * Body status (matches rapier3d BodyStatus).
 */
export enum BodyStatus {
    Dynamic = 0,
    Fixed = 1,
    KinematicPositionBased = 2,
    KinematicVelocityBased = 3,
}

export enum CelestialBody {
    Sun = 0,
    Mercury = 1,
    Venus = 2,
    Earth = 3,
    Moon = 4,
    Mars = 5,
    Jupiter = 6,
    Saturn = 7,
    Uranus = 8,
    Neptune = 9,
}

export class CelestialParams {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    equatorial_radius: number;
    flattening: number;
    gm: number;
    j2: number;
    j3: number;
    j4: number;
    j5: number;
    j6: number;
    max_degree: number;
    ref_radius: number;
    rotation_rate: number;
}

/**
 * Descriptor for creating a collider.
 */
export class ColliderDescriptor {
    free(): void;
    [Symbol.dispose](): void;
    constructor();
    /**
     * Shape parameters: (radius, half_x, half_y, half_z) depending on type.
     */
    a: number;
    b: number;
    c: number;
    collision_group: number;
    collision_mask: number;
    d: number;
    density: number;
    friction: number;
    is_sensor: boolean;
    restitution: number;
    rotation: Quat;
    shape_type: ShapeType;
    translation: Vec3;
}

/**
 * Collision event record.
 */
export class CollisionEvent {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    collider1: bigint;
    collider2: bigint;
    sensor: boolean;
    started: boolean;
}

/**
 * Contact force event record.
 */
export class ContactForceEvent {
    private constructor();
    free(): void;
    [Symbol.dispose](): void;
    collider1: bigint;
    collider2: bigint;
    force_x: number;
    force_y: number;
    force_z: number;
    max_force_dir_x: number;
    max_force_dir_y: number;
    max_force_dir_z: number;
    max_force_magnitude: number;
    total_force_magnitude: number;
}

/**
 * A physics world with configurable gravity and body management.
 */
export class PhysicsWorld {
    free(): void;
    [Symbol.dispose](): void;
    add_force(handle: bigint, fx: number, fy: number, fz: number, wake_up: boolean): void;
    add_torque(handle: bigint, tx: number, ty: number, tz: number, wake_up: boolean): void;
    apply_impulse(handle: bigint, ix: number, iy: number, iz: number, wake_up: boolean): void;
    /**
     * Cast a ray into the world. Returns collider handle or 0.
     */
    cast_ray(ox: number, oy: number, oz: number, dx: number, dy: number, dz: number, max_toi: number): bigint;
    clear_events(): void;
    /**
     * Create a dynamic box at position with given half-extents and mass.
     */
    create_dynamic_box(px: number, py: number, pz: number, hx: number, hy: number, hz: number, mass: number): bigint;
    /**
     * Create a dynamic sphere at position with given radius and mass.
     */
    create_dynamic_sphere(px: number, py: number, pz: number, radius: number, mass: number): bigint;
    /**
     * Create a ground plane (halfspace) collider attached to a fixed body.
     */
    create_ground_plane(nx: number, ny: number, nz: number, dist: number): bigint;
    /**
     * Destroy the world.
     */
    destroy(): void;
    /**
     * Enable/disable CCD. FFI: rigid_body_enable_ccd(world, handle, Bool)
     */
    enable_ccd(handle: bigint, enabled: boolean): void;
    get_body_angular_velocity(handle: bigint): Vec3;
    /**
     * Returns body count as i32 (matches FFI).
     */
    get_body_count(): number;
    get_body_linear_velocity(handle: bigint): Vec3;
    get_body_rotation(handle: bigint): Quat;
    /**
     * Batch snapshot for Three.js sync.
     * Returns Float64Array: [pos(3), quat(4), vel(3), angvel(3)] per body = 13 f64 each.
     */
    get_body_snapshot(): Float64Array;
    get_body_translation(handle: bigint): Vec3;
    /**
     * Get parameters of a built-in celestial body.
     */
    get_celestial_params(body_id: number): CelestialParams | undefined;
    /**
     * Returns collider count as i32 (matches FFI).
     */
    get_collider_count(): number;
    get_collision_event(index: number): CollisionEvent;
    get_collision_event_count(): number;
    get_collision_events(): Array<any>;
    /**
     * Insert a collider from a descriptor. Optionally attach to a parent body.
     */
    insert_collider(desc: ColliderDescriptor, parent_body: bigint): bigint;
    insert_rigid_body(desc: RigidBodyDescriptor): bigint;
    /**
     * Count bodies intersecting an AABB.
     */
    intersect_aabb_count(min_x: number, min_y: number, min_z: number, max_x: number, max_y: number, max_z: number): number;
    constructor(gx: number, gy: number, gz: number);
    /**
     * Register a celestial body's gravity. Returns the force law handle (>0) or 0 on error.
     */
    register_celestial_gravity(body_id: number, degree: number): bigint;
    /**
     * Remove a collider by handle.
     */
    remove_collider(handle: bigint): void;
    remove_rigid_body(handle: bigint, remove_colliders: boolean): void;
    set_body_translation(handle: bigint, x: number, y: number, z: number): void;
    set_gravity(x: number, y: number, z: number): void;
    /**
     * Put a body to sleep. FFI: rigid_body_sleep(world, handle) -> Bool
     */
    sleep(handle: bigint): void;
    step(dt: number): void;
    /**
     * Wake up a body. FFI: rigid_body_wake_up(world, handle, strong: Bool)
     */
    wake_up(handle: bigint): void;
}

/**
 * Quaternion (i, j, k, w) — compatible with Three.js ordering.
 */
export class Quat {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Identity quaternion.
     */
    static identity(): Quat;
    constructor(i: number, j: number, k: number, w: number);
    i: number;
    j: number;
    k: number;
    w: number;
}

/**
 * Descriptor for creating a rigid body.
 */
export class RigidBodyDescriptor {
    free(): void;
    [Symbol.dispose](): void;
    constructor();
    additional_mass: number;
    angular_damping: number;
    angular_velocity: Vec3;
    can_sleep: boolean;
    ccd_enabled: boolean;
    gravity_scale: number;
    linear_damping: number;
    linear_velocity: Vec3;
    rotation: Quat;
    status: BodyStatus;
    translation: Vec3;
}

/**
 * Shape types for colliders.
 */
export enum ShapeType {
    Ball = 0,
    Cuboid = 1,
    Capsule = 2,
    Cylinder = 3,
    Cone = 4,
    Halfspace = 5,
    Heightfield = 6,
    ConvexHull = 7,
    TriangleMesh = 8,
}

/**
 * 3D vector (matches rapier3d/Three.js convention: right-handed Y-up).
 */
export class Vec3 {
    free(): void;
    [Symbol.dispose](): void;
    constructor(x: number, y: number, z: number);
    x: number;
    y: number;
    z: number;
}

/**
 * Initialize the WASM module (sets panic hook).
 */
export function init(): void;

/**
 * Returns the version string.
 */
export function version(): string;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_bodysnapshotentry_free: (a: number, b: number) => void;
    readonly __wbg_colliderdescriptor_free: (a: number, b: number) => void;
    readonly __wbg_collisionevent_free: (a: number, b: number) => void;
    readonly __wbg_contactforceevent_free: (a: number, b: number) => void;
    readonly __wbg_get_bodysnapshotentry_angvel_x: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_angvel_y: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_angvel_z: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_pos_x: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_pos_y: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_pos_z: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_rot_i: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_rot_j: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_rot_k: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_rot_w: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_vel_x: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_vel_y: (a: number) => number;
    readonly __wbg_get_bodysnapshotentry_vel_z: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_collision_group: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_collision_mask: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_density: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_is_sensor: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_rotation: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_shape_type: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_translation: (a: number) => number;
    readonly __wbg_get_collisionevent_collider1: (a: number) => bigint;
    readonly __wbg_get_collisionevent_collider2: (a: number) => bigint;
    readonly __wbg_get_collisionevent_sensor: (a: number) => number;
    readonly __wbg_get_collisionevent_started: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_angular_damping: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_angular_velocity: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_can_sleep: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_ccd_enabled: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_gravity_scale: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_linear_damping: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_linear_velocity: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_status: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_translation: (a: number) => number;
    readonly __wbg_quat_free: (a: number, b: number) => void;
    readonly __wbg_rigidbodydescriptor_free: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_angvel_x: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_angvel_y: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_angvel_z: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_pos_x: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_pos_y: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_pos_z: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_rot_i: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_rot_j: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_rot_k: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_rot_w: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_vel_x: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_vel_y: (a: number, b: number) => void;
    readonly __wbg_set_bodysnapshotentry_vel_z: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_collision_group: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_collision_mask: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_density: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_is_sensor: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_rotation: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_shape_type: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_translation: (a: number, b: number) => void;
    readonly __wbg_set_collisionevent_collider1: (a: number, b: bigint) => void;
    readonly __wbg_set_collisionevent_collider2: (a: number, b: bigint) => void;
    readonly __wbg_set_collisionevent_sensor: (a: number, b: number) => void;
    readonly __wbg_set_collisionevent_started: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_angular_damping: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_angular_velocity: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_can_sleep: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_ccd_enabled: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_gravity_scale: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_linear_damping: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_linear_velocity: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_status: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_translation: (a: number, b: number) => void;
    readonly __wbg_vec3_free: (a: number, b: number) => void;
    readonly colliderdescriptor_new: () => number;
    readonly quat_identity: () => number;
    readonly quat_new: (a: number, b: number, c: number, d: number) => number;
    readonly rigidbodydescriptor_new: () => number;
    readonly vec3_new: (a: number, b: number, c: number) => number;
    readonly __wbg_get_colliderdescriptor_a: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_b: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_c: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_d: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_friction: (a: number) => number;
    readonly __wbg_get_colliderdescriptor_restitution: (a: number) => number;
    readonly __wbg_get_contactforceevent_collider1: (a: number) => bigint;
    readonly __wbg_get_contactforceevent_collider2: (a: number) => bigint;
    readonly __wbg_get_contactforceevent_force_x: (a: number) => number;
    readonly __wbg_get_contactforceevent_force_y: (a: number) => number;
    readonly __wbg_get_contactforceevent_force_z: (a: number) => number;
    readonly __wbg_get_contactforceevent_max_force_dir_x: (a: number) => number;
    readonly __wbg_get_contactforceevent_max_force_dir_y: (a: number) => number;
    readonly __wbg_get_contactforceevent_max_force_dir_z: (a: number) => number;
    readonly __wbg_get_contactforceevent_max_force_magnitude: (a: number) => number;
    readonly __wbg_get_contactforceevent_total_force_magnitude: (a: number) => number;
    readonly __wbg_get_quat_i: (a: number) => number;
    readonly __wbg_get_quat_j: (a: number) => number;
    readonly __wbg_get_quat_k: (a: number) => number;
    readonly __wbg_get_quat_w: (a: number) => number;
    readonly __wbg_get_rigidbodydescriptor_additional_mass: (a: number) => number;
    readonly __wbg_get_vec3_x: (a: number) => number;
    readonly __wbg_get_vec3_y: (a: number) => number;
    readonly __wbg_get_vec3_z: (a: number) => number;
    readonly __wbg_set_colliderdescriptor_a: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_b: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_c: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_d: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_friction: (a: number, b: number) => void;
    readonly __wbg_set_colliderdescriptor_restitution: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_collider1: (a: number, b: bigint) => void;
    readonly __wbg_set_contactforceevent_collider2: (a: number, b: bigint) => void;
    readonly __wbg_set_contactforceevent_force_x: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_force_y: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_force_z: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_max_force_dir_x: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_max_force_dir_y: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_max_force_dir_z: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_max_force_magnitude: (a: number, b: number) => void;
    readonly __wbg_set_contactforceevent_total_force_magnitude: (a: number, b: number) => void;
    readonly __wbg_set_quat_i: (a: number, b: number) => void;
    readonly __wbg_set_quat_j: (a: number, b: number) => void;
    readonly __wbg_set_quat_k: (a: number, b: number) => void;
    readonly __wbg_set_quat_w: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_additional_mass: (a: number, b: number) => void;
    readonly __wbg_set_vec3_x: (a: number, b: number) => void;
    readonly __wbg_set_vec3_y: (a: number, b: number) => void;
    readonly __wbg_set_vec3_z: (a: number, b: number) => void;
    readonly __wbg_set_rigidbodydescriptor_rotation: (a: number, b: number) => void;
    readonly __wbg_get_rigidbodydescriptor_rotation: (a: number) => number;
    readonly init: () => void;
    readonly physicsworld_cast_ray: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => bigint;
    readonly query_cast_ray: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly physicsworld_clear_events: (a: number) => void;
    readonly world_clear_events: (a: number) => void;
    readonly physicsworld_create_dynamic_box: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => bigint;
    readonly physicsworld_create_dynamic_sphere: (a: number, b: number, c: number, d: number, e: number, f: number) => bigint;
    readonly physicsworld_create_ground_plane: (a: number, b: number, c: number, d: number, e: number) => bigint;
    readonly physicsworld_get_collision_event: (a: number, b: number) => number;
    readonly world_get_collision_event: (a: number, b: number, c: number) => void;
    readonly physicsworld_get_collision_event_count: (a: number) => number;
    readonly world_collision_event_count: (a: number) => number;
    readonly physicsworld_get_collision_events: (a: number) => any;
    readonly physicsworld_insert_collider: (a: number, b: number, c: bigint) => bigint;
    readonly physicsworld_intersect_aabb_count: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly query_intersect_aabb_count: (a: number, b: number, c: number) => number;
    readonly physicsworld_remove_collider: (a: number, b: bigint) => void;
    readonly world_remove_collider: (a: number, b: bigint, c: number) => number;
    readonly version: () => [number, number];
    readonly world_destroy: (a: number) => void;
    readonly rigid_body_builder_create: (a: number) => number;
    readonly rigid_body_builder_set_pose: (a: number, b: number, c: number) => void;
    readonly rigid_body_builder_set_linvel: (a: number, b: number) => void;
    readonly rigid_body_builder_set_angvel: (a: number, b: number) => void;
    readonly rigid_body_builder_set_additional_mass: (a: number, b: number) => void;
    readonly rigid_body_builder_set_linear_damping: (a: number, b: number) => void;
    readonly rigid_body_builder_set_angular_damping: (a: number, b: number) => void;
    readonly rigid_body_builder_build: (a: number) => number;
    readonly world_insert_rigid_body: (a: number, b: number) => bigint;
    readonly collider_builder_create_ex: (a: number) => number;
    readonly collider_builder_set_translation: (a: number, b: number) => void;
    readonly collider_builder_set_pose: (a: number, b: number, c: number) => void;
    readonly collider_builder_set_friction: (a: number, b: number) => void;
    readonly collider_builder_set_restitution: (a: number, b: number) => void;
    readonly collider_builder_set_density: (a: number, b: number) => void;
    readonly collider_builder_set_sensor: (a: number, b: number) => void;
    readonly collider_builder_set_collision_groups: (a: number, b: number) => void;
    readonly collider_builder_build: (a: number) => number;
    readonly world_insert_collider_with_parent: (a: number, b: number, c: bigint) => bigint;
    readonly celestial_get_body: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly world_register_celestial_gravity: (a: number, b: number, c: number) => bigint;
    readonly __wbg_physicsworld_free: (a: number, b: number) => void;
    readonly physicsworld_add_force: (a: number, b: bigint, c: number, d: number, e: number, f: number) => void;
    readonly rigid_body_add_force: (a: number, b: bigint, c: number, d: number) => number;
    readonly physicsworld_add_torque: (a: number, b: bigint, c: number, d: number, e: number, f: number) => void;
    readonly rigid_body_add_torque: (a: number, b: bigint, c: number, d: number) => number;
    readonly physicsworld_apply_impulse: (a: number, b: bigint, c: number, d: number, e: number, f: number) => void;
    readonly rigid_body_apply_impulse: (a: number, b: bigint, c: number, d: number) => number;
    readonly physicsworld_destroy: (a: number) => void;
    readonly physicsworld_enable_ccd: (a: number, b: bigint, c: number) => void;
    readonly rigid_body_enable_ccd: (a: number, b: bigint, c: number) => number;
    readonly physicsworld_get_body_angular_velocity: (a: number, b: bigint) => number;
    readonly rigid_body_get_angvel: (a: number, b: number, c: bigint) => void;
    readonly physicsworld_get_body_count: (a: number) => number;
    readonly world_get_rigid_body_set_size: (a: number) => number;
    readonly physicsworld_get_body_linear_velocity: (a: number, b: bigint) => number;
    readonly rigid_body_get_linvel: (a: number, b: number, c: bigint) => void;
    readonly physicsworld_get_body_rotation: (a: number, b: bigint) => number;
    readonly rigid_body_get_rotation: (a: number, b: number, c: bigint) => void;
    readonly physicsworld_get_body_snapshot: (a: number) => any;
    readonly world_body_snapshot: (a: number, b: number, c: number, d: number) => number;
    readonly physicsworld_get_body_translation: (a: number, b: bigint) => number;
    readonly rigid_body_get_translation: (a: number, b: number, c: bigint) => void;
    readonly physicsworld_get_collider_count: (a: number) => number;
    readonly world_get_collider_set_size: (a: number) => number;
    readonly physicsworld_insert_rigid_body: (a: number, b: number) => bigint;
    readonly physicsworld_new: (a: number, b: number, c: number) => number;
    readonly world_create: (a: number) => number;
    readonly physicsworld_remove_rigid_body: (a: number, b: bigint, c: number) => void;
    readonly world_remove_rigid_body: (a: number, b: bigint, c: number) => number;
    readonly physicsworld_set_body_translation: (a: number, b: bigint, c: number, d: number, e: number) => void;
    readonly rigid_body_set_translation: (a: number, b: bigint, c: number, d: number) => number;
    readonly physicsworld_set_gravity: (a: number, b: number, c: number, d: number) => void;
    readonly world_set_gravity: (a: number, b: number) => void;
    readonly physicsworld_sleep: (a: number, b: bigint) => void;
    readonly rigid_body_sleep: (a: number, b: bigint) => number;
    readonly physicsworld_step: (a: number, b: number) => void;
    readonly world_step: (a: number, b: number) => void;
    readonly physicsworld_wake_up: (a: number, b: bigint) => void;
    readonly rigid_body_wake_up: (a: number, b: bigint, c: number) => number;
    readonly __wbg_celestialparams_free: (a: number, b: number) => void;
    readonly __wbg_get_celestialparams_equatorial_radius: (a: number) => number;
    readonly __wbg_get_celestialparams_flattening: (a: number) => number;
    readonly __wbg_get_celestialparams_gm: (a: number) => number;
    readonly __wbg_get_celestialparams_j2: (a: number) => number;
    readonly __wbg_get_celestialparams_j3: (a: number) => number;
    readonly __wbg_get_celestialparams_j4: (a: number) => number;
    readonly __wbg_get_celestialparams_j5: (a: number) => number;
    readonly __wbg_get_celestialparams_j6: (a: number) => number;
    readonly __wbg_get_celestialparams_max_degree: (a: number) => number;
    readonly __wbg_get_celestialparams_ref_radius: (a: number) => number;
    readonly __wbg_get_celestialparams_rotation_rate: (a: number) => number;
    readonly __wbg_set_celestialparams_equatorial_radius: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_flattening: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_gm: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_j2: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_j3: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_j4: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_j5: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_j6: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_max_degree: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_ref_radius: (a: number, b: number) => void;
    readonly __wbg_set_celestialparams_rotation_rate: (a: number, b: number) => void;
    readonly physicsworld_get_celestial_params: (a: number, b: number) => number;
    readonly physicsworld_register_celestial_gravity: (a: number, b: number, c: number) => bigint;
    readonly sf_biot_savart_velocity: (a: number, b: number, c: number) => number;
    readonly sf_circulation_around_loop: (a: number, b: number, c: number, d: number) => number;
    readonly sf_circulation_quantum: () => number;
    readonly sf_gp_amplitude_evolution: (a: number, b: number, c: number, d: number) => number;
    readonly sf_gp_energy_density: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly sf_gp_grid_sample: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number) => number;
    readonly sf_gp_order_parameter: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly sf_healing_length: (a: number, b: number, c: number) => number;
    readonly sf_helium_mass: () => number;
    readonly sf_helium_scattering_length: () => number;
    readonly sf_quantum_number_estimate: (a: number, b: number, c: number, d: number) => number;
    readonly sf_sound_speed: (a: number, b: number, c: number) => number;
    readonly sf_vortex_reconnection: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly sf_vortex_ring_velocity: (a: number, b: number) => number;
    readonly sf_vortex_tangle_stats: (a: number, b: number, c: number, d: number) => number;
    readonly topology_compliance_sensitivity: (a: number, b: number, c: number) => number;
    readonly topology_density_filter_2d: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly topology_density_to_voxels: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly topology_oc_update: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly topology_runtime_shape_density_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly topology_simp_material: (a: number, b: number, c: number) => number;
    readonly topology_simp_stiffness: (a: number, b: number, c: number, d: number) => number;
    readonly transmission_archimedean_spiral_evaluate: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly transmission_archimedean_spiral_radius: (a: number, b: number, c: number, d: number) => number;
    readonly transmission_cycloidal_cam_evaluate: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly transmission_gear_evaluate: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly transmission_gear_target_angle: (a: number, b: number, c: number, d: number) => number;
    readonly transmission_screw_evaluate: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly transmission_screw_target_translation: (a: number, b: number, c: number, d: number) => number;
    readonly aero_apply_surfaces: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly aero_apply_surfaces_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly aero_apply_voxel_grid: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number) => number;
    readonly aero_apply_voxel_grid_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number) => number;
    readonly aero_estimate_surface_force: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly rigid_body_add_force_at_point: (a: number, b: bigint, c: number, d: number, e: number) => number;
    readonly rigid_body_add_force_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_add_torque_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_apply_impulse_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_apply_torque_impulse: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_apply_torque_impulse_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_builder_destroy: (a: number) => void;
    readonly rigid_body_builder_set_additional_mass_properties: (a: number, b: number, c: number, d: number) => void;
    readonly rigid_body_builder_set_can_sleep: (a: number, b: number) => void;
    readonly rigid_body_builder_set_enabled_rotations: (a: number, b: number, c: number, d: number) => void;
    readonly rigid_body_builder_set_gravity_scale: (a: number, b: number) => void;
    readonly rigid_body_builder_set_rotation: (a: number, b: number) => void;
    readonly rigid_body_builder_set_translation: (a: number, b: number) => void;
    readonly rigid_body_builder_set_user_data: (a: number, b: bigint, c: bigint) => void;
    readonly rigid_body_destroy_raw: (a: number) => void;
    readonly rigid_body_enable_ccd_flag: (a: number, b: bigint, c: number) => number;
    readonly rigid_body_get_angvel_out: (a: number, b: bigint, c: number) => void;
    readonly rigid_body_get_force: (a: number, b: number, c: bigint) => void;
    readonly rigid_body_get_linvel_out: (a: number, b: bigint, c: number) => void;
    readonly rigid_body_get_rotation_out: (a: number, b: bigint, c: number) => void;
    readonly rigid_body_get_status: (a: number, b: bigint) => number;
    readonly rigid_body_get_translation_out: (a: number, b: bigint, c: number) => void;
    readonly rigid_body_is_sleeping: (a: number, b: bigint) => number;
    readonly rigid_body_is_sleeping_flag: (a: number, b: bigint) => number;
    readonly rigid_body_reset_force: (a: number, b: bigint, c: number) => number;
    readonly rigid_body_reset_torque: (a: number, b: bigint, c: number) => number;
    readonly rigid_body_set_angvel: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_angvel_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_linvel: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_linvel_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_pose: (a: number, b: bigint, c: number, d: number, e: number) => number;
    readonly rigid_body_set_pose_flag: (a: number, b: bigint, c: number, d: number, e: number) => number;
    readonly rigid_body_set_rotation: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_rotation_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_status: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_set_translation_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly rigid_body_sleep_flag: (a: number, b: bigint) => number;
    readonly rigid_body_wake_up_flag: (a: number, b: bigint, c: number) => number;
    readonly rtree_clear: (a: number) => void;
    readonly rtree_create: () => number;
    readonly rtree_destroy: (a: number) => void;
    readonly rtree_insert: (a: number, b: bigint, c: number) => number;
    readonly rtree_len: (a: number) => number;
    readonly rtree_query_aabb: (a: number, b: number, c: number, d: number) => number;
    readonly rtree_query_aabb_count: (a: number, b: number) => number;
    readonly rtree_rebuild: (a: number) => void;
    readonly rtree_remove: (a: number, b: bigint) => number;
    readonly rtree_update: (a: number, b: bigint, c: number) => number;
    readonly world_copy_rigid_body: (a: number, b: bigint) => number;
    readonly world_remove_rigid_body_flag: (a: number, b: bigint, c: number) => number;
    readonly fluid_apply_aabb_forces: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number) => number;
    readonly fluid_apply_aabb_forces_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number) => number;
    readonly fluid_bernoulli_pressure: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly fluid_bernoulli_report: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly fluid_estimate_aabb_forces: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly fluid_navier_stokes_simplified_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly fluid_sph_estimate_density: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly fluid_sph_estimate_forces: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly fluid_sph_poly6_kernel: (a: number, b: number) => number;
    readonly fluid_sph_spiky_gradient: (a: number, b: number, c: number) => number;
    readonly fluid_sph_viscosity_laplacian: (a: number, b: number) => number;
    readonly molecular_apply_pair_forces: (a: number, b: bigint, c: bigint, d: number, e: number, f: number, g: number, h: number) => number;
    readonly molecular_apply_pair_forces_flag: (a: number, b: bigint, c: bigint, d: number, e: number, f: number, g: number, h: number) => number;
    readonly molecular_coulomb_force: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly molecular_coulomb_potential: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly molecular_lennard_jones_force: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly molecular_lennard_jones_potential: (a: number, b: number, c: number) => number;
    readonly molecular_pair_interaction: (a: number, b: number, c: number, d: number) => number;
    readonly molecular_vacuum_coulomb_constant: () => number;
    readonly softbody_mass_spring_forces: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly softbody_predict_positions: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly softbody_solve_sphere_collision_constraints: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly softbody_solve_xpbd_bending_constraints: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly softbody_solve_xpbd_distance_constraints: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly softbody_solve_xpbd_volume_constraints: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly softbody_update_velocities: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly collider_builder_create_capsule: (a: number) => number;
    readonly collider_builder_create_cylinder: (a: number) => number;
    readonly collider_builder_create_ellipsoid: (a: number) => number;
    readonly collider_builder_create_fdh: (a: number, b: number, c: number, d: number) => number;
    readonly collider_builder_create_kdop: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_prism: (a: number) => number;
    readonly collider_builder_create_spherical_shell: (a: number) => number;
    readonly collider_builder_create_ssv: (a: number) => number;
    readonly query_cast_ray_out: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => bigint;
    readonly query_cast_rays: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly query_cast_shape: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => void;
    readonly query_cast_shape_out: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => bigint;
    readonly query_intersect_aabb: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_aabb_count_all: (a: number, b: number) => number;
    readonly query_intersect_aabb_counts: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly query_intersect_aabb_rigid_bodies_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_aabb_rigid_bodies: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_aabb_rigid_body_count_all: (a: number, b: number) => number;
    readonly query_intersect_aabb_rigid_body_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_capsule: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_capsule_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_capsule_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_capsule_count_all: (a: number, b: number) => number;
    readonly query_intersect_cylinder: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_cylinder_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_cylinder_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_cylinder_count_all: (a: number, b: number) => number;
    readonly query_intersect_ellipsoid: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_ellipsoid_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_ellipsoid_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_ellipsoid_count_all: (a: number, b: number) => number;
    readonly query_intersect_obb: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_obb_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_obb_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_obb_count_all: (a: number, b: number) => number;
    readonly query_intersect_obb_counts: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly query_intersect_point_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_prism: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_prism_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_prism_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_prism_count_all: (a: number, b: number) => number;
    readonly query_intersect_sphere: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_sphere_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_sphere_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_sphere_count_all: (a: number, b: number) => number;
    readonly query_intersect_sphere_counts: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly query_intersect_spherical_shell: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_spherical_shell_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_spherical_shell_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_spherical_shell_count_all: (a: number, b: number) => number;
    readonly query_intersect_ssv: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_ssv_all: (a: number, b: number, c: number, d: number) => number;
    readonly query_intersect_ssv_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_ssv_count_all: (a: number, b: number) => number;
    readonly query_project_point: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => void;
    readonly query_project_point_out: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => bigint;
    readonly rel_beta_from_gamma: (a: number) => number;
    readonly rel_effective_potential: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly rel_gravitational_time_dilation: (a: number, b: number, c: number, d: number) => number;
    readonly rel_gravitational_time_dilation_simple: (a: number, b: number) => number;
    readonly rel_invariant_mass: (a: number, b: number, c: number, d: number) => number;
    readonly rel_length_contraction: (a: number, b: number, c: number) => number;
    readonly rel_light_deflection_angle: (a: number, b: number, c: number) => number;
    readonly rel_lorentz_boost: (a: number, b: number) => number;
    readonly rel_lorentz_factor: (a: number, b: number) => number;
    readonly rel_particle_properties: (a: number, b: number, c: number) => number;
    readonly rel_rapidity: (a: number) => number;
    readonly rel_schwarzschild_metric: (a: number, b: number, c: number, d: number) => number;
    readonly rel_schwarzschild_radius: (a: number, b: number) => number;
    readonly rel_speed_of_light: () => number;
    readonly rel_transform_four_vector: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly rel_velocity_addition: (a: number, b: number, c: number) => number;
    readonly space_airlock_depressurization: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_apply_atmospheric_drag_to_body: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly space_atmospheric_drag_acceleration: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_apply_atmospheric_drag_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly space_apply_cmg_torque_to_body: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_cmg_exchange: (a: number, b: number, c: number, d: number) => number;
    readonly space_apply_cmg_torque_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_apply_gravity_gradient_torque_to_body: (a: number, b: bigint, c: number, d: number, e: number, f: number) => number;
    readonly space_gravity_gradient_torque: (a: number, b: number, c: number, d: number) => number;
    readonly space_apply_gravity_gradient_torque_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number) => number;
    readonly space_apply_j2_force_to_body: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly space_j2_acceleration: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_apply_j2_force_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly space_apply_magnetic_torquer_to_body: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_magnetic_torquer_dipole: (a: number, b: number, c: number, d: number) => number;
    readonly space_apply_magnetic_torquer_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_apply_solar_radiation_pressure_to_body: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly space_solar_radiation_pressure_acceleration: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_apply_solar_radiation_pressure_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly space_arm_first_joint_inverse: (a: number, b: number) => number;
    readonly space_arm_third_joint_angle: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_artificial_potential_guidance: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_atmospheric_density_scale_height: (a: number, b: number, c: number, d: number) => number;
    readonly space_atomic_oxygen_erosion: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_bang_off_bang_profile: (a: number, b: number, c: number, d: number) => number;
    readonly space_battery_equivalent_circuit: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly space_cmg_robust_pseudoinverse_diag: (a: number, b: number, c: number, d: number) => number;
    readonly space_co2_mass_balance: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_contact_force_hunt_crossley: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_cw_derivative: (a: number, b: number, c: number) => number;
    readonly space_debris_collision_probability: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_dh_transform: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_docking_buffer_energy: (a: number, b: number, c: number, d: number) => number;
    readonly space_docking_glideslope_command: (a: number, b: number, c: number) => number;
    readonly space_ekf_gain_scalar: (a: number, b: number, c: number) => number;
    readonly space_ekf_predict_scalar: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_ekf_update_scalar: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_elements_to_state: (a: number, b: number, c: number) => number;
    readonly space_flexible_mode_derivative: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_friis_link: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_friis_wavelength_from_frequency: (a: number) => number;
    readonly space_gnss_double_difference_carrier_phase: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_gnss_pseudorange: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_hall_thruster_performance: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_heat_pipe_thermal_resistance: (a: number, b: number, c: number, d: number) => number;
    readonly space_hohmann_transfer: (a: number, b: number, c: number, d: number) => number;
    readonly space_kepler_period: (a: number, b: number) => number;
    readonly space_kepler_semi_major_axis: (a: number, b: number) => number;
    readonly space_lambert_time_elliptic: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_least_squares_attitude_two_vector: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_triad_attitude: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_manipulator_dynamics_diag: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_mass_properties_two_body: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_quaternion_derivative: (a: number, b: number, c: number) => number;
    readonly space_radar_range_rate: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_radiation_absorbed_dose: (a: number, b: number, c: number) => number;
    readonly space_radiator_power: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_rigid_body_euler_derivative: (a: number, b: number, c: number, d: number) => number;
    readonly space_sabatier_methane_rate: (a: number, b: number, c: number, d: number) => number;
    readonly space_sagnac_phase_rate: (a: number, b: number, c: number) => number;
    readonly space_semi_major_axis_decay_rate: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_sgp4_j2_secular_rates: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_single_phase_loop_heat_transfer: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_slosh_pendulum_derivative: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly space_solar_array_pd_torque: (a: number, b: number, c: number, d: number) => number;
    readonly space_solar_panel_power: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly space_spe_oxygen_rate: (a: number, b: number, c: number, d: number) => number;
    readonly space_state_to_elements: (a: number, b: number, c: number) => number;
    readonly space_structural_natural_frequency: (a: number, b: number, c: number) => number;
    readonly space_surface_charging_current_balance: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_thermal_balance: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly space_tsiolkovsky_delta_v: (a: number, b: number, c: number, d: number) => number;
    readonly space_variational_two_body: (a: number, b: number, c: number, d: number) => number;
    readonly space_whipple_critical_projectile_diameter: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly trajectory_apply_forces_to_body: (a: number, b: bigint, c: number, d: number, e: number) => number;
    readonly trajectory_apply_forces_to_body_flag: (a: number, b: bigint, c: number, d: number, e: number) => number;
    readonly trajectory_estimate_forces: (a: number, b: number, c: number) => number;
    readonly trajectory_glide_estimate: (a: number, b: number, c: number) => number;
    readonly trajectory_glide_integrate_step: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly trajectory_integrate_step: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly acoustic_contact_material_excitation: (a: number, b: number, c: number, d: number) => number;
    readonly acoustic_detect_resonance: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly acoustic_generalized_modal_analysis: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly acoustic_modal_synthesis_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly acoustic_spatialize_mono_sample: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly acoustic_structural_mode_sdof: (a: number, b: number, c: number, d: number) => number;
    readonly acoustic_wave_equation_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly gravity_ellipsoid: (a: number, b: number, c: number) => number;
    readonly gravity_quadrupole_tensor: (a: number, b: number, c: number, d: number) => number;
    readonly gravity_spherical_harmonics: (a: number, b: number, c: number, d: number) => number;
    readonly gravity_zonal_harmonics: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly physchem_catalyst_rate_multiplier: (a: number, b: number, c: number) => number;
    readonly physchem_concentration_buoyancy: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly physchem_gray_scott_reaction_terms: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly physchem_gray_scott_step_2d: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number) => number;
    readonly physchem_reaction_diffusion_explicit: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly pl_alfven_speed: (a: number, b: number, c: number) => number;
    readonly pl_boris_push: (a: number, b: number, c: number, d: number) => number;
    readonly pl_debye_length: (a: number, b: number) => number;
    readonly pl_deposit_particle: (a: number, b: number, c: number, d: number) => number;
    readonly pl_find_xpoint: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly pl_interpolate_field: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number) => number;
    readonly pl_lundquist_number: (a: number, b: number, c: number) => number;
    readonly pl_petschek_rate: (a: number) => number;
    readonly pl_pic_step_report: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly pl_plasma_frequency: (a: number) => number;
    readonly pl_plasma_params: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly pl_poisson_solve_1d: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly pl_sweet_parker_rate: (a: number) => number;
    readonly pl_vlasov_moments: (a: number, b: number, c: number) => number;
    readonly terrain_gravity_dem: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly terrain_gravity_dem_fft: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly terrain_lunar_mascon_count: () => number;
    readonly terrain_lunar_mascon_get: (a: number, b: number) => number;
    readonly terrain_lunar_mascon_gravity: (a: number, b: number) => number;
    readonly terrain_polyhedron_gravity: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly character_controller_collision_count: (a: number) => number;
    readonly character_controller_create: () => number;
    readonly character_controller_destroy: (a: number) => void;
    readonly character_controller_get_collision: (a: number, b: number, c: number) => void;
    readonly character_controller_move_shape: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => void;
    readonly character_controller_set_autostep: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly character_controller_set_offset_absolute: (a: number, b: number) => void;
    readonly character_controller_set_offset_relative: (a: number, b: number) => void;
    readonly character_controller_set_slide: (a: number, b: number) => void;
    readonly character_controller_set_slope_angles: (a: number, b: number, c: number) => void;
    readonly character_controller_set_snap_to_ground: (a: number, b: number, c: number) => void;
    readonly character_controller_set_up: (a: number, b: number) => void;
    readonly character_controller_solve_impulses: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly collider_builder_create_neural_bounds: (a: number, b: number, c: number) => number;
    readonly fracture_energy_release: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly fracture_griffith_criterion: (a: number, b: number, c: number, d: number) => number;
    readonly fracture_miner_damage: (a: number, b: number, c: number, d: number) => number;
    readonly fracture_mode_from_stress: (a: number, b: number, c: number, d: number) => number;
    readonly fracture_sn_curve_life: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly fracture_stress_intensity_factor: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly neural_bounds_required_weight_count: (a: number, b: number) => number;
    readonly query_intersect_neural_bounds: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly query_intersect_neural_bounds_all: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly query_intersect_neural_bounds_count: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_neural_bounds_count_all: (a: number, b: number, c: number, d: number) => number;
    readonly world_replace_body_with_fracture_fragments: (a: number, b: bigint, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly biomechanics_hill_force_length_factor: (a: number, b: number, c: number) => number;
    readonly biomechanics_hill_force_velocity_factor: (a: number, b: number) => number;
    readonly biomechanics_hill_muscle_evaluate: (a: number, b: number, c: number) => number;
    readonly biomechanics_hill_three_element_force: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly biomechanics_muscle_joint_torque: (a: number, b: number) => number;
    readonly biomechanics_skeletal_joint_limit: (a: number, b: number, c: number, d: number) => number;
    readonly chaos_bifurcation_lorenz: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly chaos_detect: (a: number, b: number, c: number, d: number) => number;
    readonly chaos_lyapunov_rosenstein: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly chaos_double_pendulum_accel: (a: number, b: number, c: number) => number;
    readonly chaos_double_pendulum_integrate: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly chaos_double_pendulum_step: (a: number, b: number, c: number) => number;
    readonly chaos_logistic_bifurcation: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly chaos_logistic_iterate: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly chaos_logistic_step: (a: number, b: number, c: number) => number;
    readonly chaos_lorenz_integrate: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly chaos_lorenz_integrate_count: (a: number, b: number) => number;
    readonly chaos_lorenz_step: (a: number, b: number, c: number) => number;
    readonly chaos_lyapunov_lorenz: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly last_error_code: () => number;
    readonly last_error_message: () => number;
    readonly thermal_fem_diffusion_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly thermal_fourier_conduction: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly thermal_phase_change: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly thermal_phase_condition: (a: number, b: number, c: number, d: number) => number;
    readonly thermal_stefan_boltzmann_radiation: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly thermal_stress_from_expansion: (a: number, b: number, c: number, d: number) => number;
    readonly thermal_thermoelastic_stress_strain: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly wo_fresnel_diffraction_point: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly wo_fresnel_grid: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number) => number;
    readonly wo_fresnel_zone: (a: number, b: number, c: number, d: number) => number;
    readonly wo_fresnel_zone_sum: (a: number, b: number, c: number, d: number) => number;
    readonly wo_huygens_fresnel: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly wo_kirchhoff_diffraction_point: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly wo_plane_wave: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly wo_spherical_wave: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly wo_thin_film_interference: (a: number, b: number, c: number) => number;
    readonly wo_thin_film_spectrum: (a: number, b: number, c: number, d: number) => number;
    readonly wo_wavelength: (a: number) => number;
    readonly wo_wavenumber: (a: number) => number;
    readonly wo_young_slit_pattern: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number) => number;
    readonly wo_young_slit_point: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly last_error_clear: () => void;
    readonly collider_builder_create_voxel_aabb: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_voxel_aabb_auto: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_voxel_obb: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_voxel_obb_auto: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_voxels: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly collider_builder_create_voxels_auto: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
    readonly joint_builder_create: (a: number, b: number, c: number, d: number) => number;
    readonly joint_builder_destroy: (a: number) => void;
    readonly joint_builder_set_contacts_enabled: (a: number, b: number) => void;
    readonly joint_builder_set_limits: (a: number, b: number, c: number, d: number) => void;
    readonly joint_builder_set_local_anchor1: (a: number, b: number) => void;
    readonly joint_builder_set_local_anchor2: (a: number, b: number) => void;
    readonly joint_builder_set_motor_position: (a: number, b: number, c: number, d: number, e: number) => void;
    readonly joint_builder_set_motor_velocity: (a: number, b: number, c: number, d: number) => void;
    readonly query_intersect_voxel_aabb: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_voxel_aabb_count: (a: number, b: number, c: number) => number;
    readonly query_intersect_voxel_obb: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly query_intersect_voxel_obb_count: (a: number, b: number, c: number) => number;
    readonly voxel_aabb_build_stats: (a: number, b: number, c: number, d: number) => void;
    readonly voxel_aabb_build_stats_out: (a: number, b: number, c: number, d: number) => void;
    readonly voxel_build_stats: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => void;
    readonly voxel_obb_build_stats: (a: number, b: number, c: number, d: number) => void;
    readonly voxel_obb_build_stats_out: (a: number, b: number, c: number, d: number) => void;
    readonly world_insert_dynamic_voxel_obb: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => bigint;
    readonly world_insert_impulse_joint: (a: number, b: bigint, c: bigint, d: number, e: number) => bigint;
    readonly world_insert_static_voxel_aabb: (a: number, b: number, c: number, d: number, e: number, f: number) => bigint;
    readonly world_remove_impulse_joint: (a: number, b: bigint, c: number) => number;
    readonly astro_barnes_hut_should_open: (a: number, b: number, c: number) => number;
    readonly astro_fmm_monopole_acceleration: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly astro_nbody_barnes_hut_accelerations: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly astro_nbody_direct_accelerations: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly astro_orbital_resonance_detect: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly astro_relativistic_orbit_correction: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly astro_roche_limit: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly em_faraday_induction: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly em_fdtd_yee_update: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number) => number;
    readonly em_lorentz_force: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly em_magnetic_flux: (a: number, b: number, c: number, d: number) => number;
    readonly em_maxwell_point_update: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number) => number;
    readonly em_vacuum_permeability: () => number;
    readonly em_vacuum_permittivity: () => number;
    readonly world_clear_air_drag_law: (a: number) => void;
    readonly world_clear_contact_pair_filter_callback: (a: number) => void;
    readonly world_clear_coulomb_friction_law: (a: number) => void;
    readonly world_clear_event_rings: (a: number) => void;
    readonly world_clear_external_force_law: (a: number) => void;
    readonly world_clear_newton_gravity_law: (a: number) => void;
    readonly world_collision_event_ring_len: (a: number) => number;
    readonly world_collision_event_ring_stats: (a: number, b: number) => number;
    readonly world_contact_force_event_count: (a: number) => number;
    readonly world_contact_force_event_ring_len: (a: number) => number;
    readonly world_contact_force_event_ring_stats: (a: number, b: number) => number;
    readonly world_drain_collision_event_ring: (a: number, b: number, c: number) => number;
    readonly world_drain_contact_force_event_ring: (a: number, b: number, c: number) => number;
    readonly world_get_air_drag_law: (a: number, b: number) => number;
    readonly world_get_collision_events: (a: number, b: number, c: number) => number;
    readonly world_get_contact_force_event: (a: number, b: number, c: number) => void;
    readonly world_get_contact_force_events: (a: number, b: number, c: number) => number;
    readonly world_get_coulomb_friction_law: (a: number, b: number) => number;
    readonly world_get_custom_physics_report: (a: number, b: number) => number;
    readonly world_get_external_force_law: (a: number, b: number) => number;
    readonly world_get_newton_gravity_law: (a: number, b: number) => number;
    readonly world_init_collision_event_ring: (a: number, b: number) => number;
    readonly world_init_contact_force_event_ring: (a: number, b: number) => number;
    readonly world_register_collision_callback: (a: number, b: number, c: number) => bigint;
    readonly world_register_contact_force_callback: (a: number, b: number, c: number) => bigint;
    readonly world_set_air_drag_law: (a: number, b: number) => number;
    readonly world_set_air_drag_law_flag: (a: number, b: number) => number;
    readonly world_set_contact_pair_filter_callback: (a: number, b: number, c: number) => void;
    readonly world_set_coulomb_friction_law: (a: number, b: number) => number;
    readonly world_set_coulomb_friction_law_flag: (a: number, b: number) => number;
    readonly world_set_event_dispatch_mode: (a: number, b: number) => number;
    readonly world_set_external_force_law: (a: number, b: number) => number;
    readonly world_set_external_force_law_flag: (a: number, b: number) => number;
    readonly world_set_intersection_pair_filter_callback: (a: number, b: number, c: number) => void;
    readonly world_set_newton_gravity_law: (a: number, b: number) => number;
    readonly world_set_newton_gravity_law_flag: (a: number, b: number) => number;
    readonly world_unregister_callback: (a: number, b: bigint) => void;
    readonly world_clear_intersection_pair_filter_callback: (a: number) => void;
    readonly integrator_keplerian_elements: (a: number, b: number, c: number, d: number) => number;
    readonly integrator_leapfrog_step: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly integrator_post_newtonian: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly integrator_specific_energy: (a: number, b: number, c: number, d: number) => number;
    readonly collider_builder_create: (a: number, b: number) => number;
    readonly collider_builder_create_convex_hull: (a: number, b: number) => number;
    readonly collider_builder_create_discrete_obb: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_double_bv: (a: number, b: number) => number;
    readonly collider_builder_create_edge_bvh: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly collider_builder_create_fused_collapsing_bounds: (a: number, b: number, c: number) => number;
    readonly collider_builder_create_halfspace: (a: number) => number;
    readonly collider_builder_create_heightmap: (a: number, b: number, c: number, d: number) => number;
    readonly collider_builder_create_medial_spheres: (a: number, b: number) => number;
    readonly collider_builder_create_obb: (a: number) => number;
    readonly collider_builder_create_point_cloud_bounds: (a: number, b: number) => number;
    readonly collider_builder_create_skewed_obb: (a: number, b: number, c: number, d: number) => number;
    readonly collider_builder_create_sphere: (a: number) => number;
    readonly collider_builder_destroy: (a: number) => void;
    readonly collider_builder_set_active_events: (a: number, b: number) => void;
    readonly collider_builder_set_active_hooks: (a: number, b: number) => void;
    readonly collider_builder_set_contact_force_event_threshold: (a: number, b: number) => void;
    readonly collider_builder_set_rotation: (a: number, b: number) => void;
    readonly collider_builder_set_solver_groups: (a: number, b: number) => void;
    readonly collider_destroy_raw: (a: number) => void;
    readonly collider_get_density: (a: number, b: bigint) => number;
    readonly collider_get_rotation: (a: number, b: number, c: bigint) => void;
    readonly collider_get_rotation_out: (a: number, b: bigint, c: number) => void;
    readonly collider_get_shape_count: (a: number, b: bigint) => number;
    readonly collider_get_translation: (a: number, b: number, c: bigint) => void;
    readonly collider_get_translation_out: (a: number, b: bigint, c: number) => void;
    readonly collider_set_active_events: (a: number, b: bigint, c: number) => number;
    readonly collider_set_active_events_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_active_hooks: (a: number, b: bigint, c: number) => number;
    readonly collider_set_active_hooks_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_collision_groups: (a: number, b: bigint, c: number) => number;
    readonly collider_set_collision_groups_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_contact_force_event_threshold: (a: number, b: bigint, c: number) => number;
    readonly collider_set_contact_force_event_threshold_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_friction: (a: number, b: bigint, c: number) => number;
    readonly collider_set_friction_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_pose: (a: number, b: bigint, c: number, d: number) => number;
    readonly collider_set_pose_flag: (a: number, b: bigint, c: number, d: number) => number;
    readonly collider_set_restitution: (a: number, b: bigint, c: number) => number;
    readonly collider_set_restitution_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_rotation: (a: number, b: bigint, c: number) => number;
    readonly collider_set_sensor: (a: number, b: bigint, c: number) => number;
    readonly collider_set_sensor_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_solver_groups: (a: number, b: bigint, c: number) => number;
    readonly collider_set_solver_groups_flag: (a: number, b: bigint, c: number) => number;
    readonly collider_set_translation: (a: number, b: bigint, c: number) => number;
    readonly world_copy_collider: (a: number, b: bigint) => number;
    readonly world_insert_collider: (a: number, b: number) => bigint;
    readonly world_insert_dynamic_cuboids: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number) => bigint;
    readonly world_insert_static_trimesh: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => bigint;
    readonly world_remove_collider_flag: (a: number, b: bigint, c: number) => number;
    readonly celestial_get_sh_coeff_count: (a: number) => number;
    readonly celestial_get_sh_coeffs: (a: number, b: number, c: number, d: number) => number;
    readonly crb_tree_clear: (a: number) => void;
    readonly crb_tree_create: () => number;
    readonly crb_tree_destroy: (a: number) => void;
    readonly crb_tree_insert: (a: number, b: bigint, c: number) => number;
    readonly crb_tree_insert_flag: (a: number, b: bigint, c: number) => number;
    readonly crb_tree_len: (a: number) => number;
    readonly crb_tree_query_aabb: (a: number, b: number, c: number, d: number) => number;
    readonly crb_tree_query_aabb_count: (a: number, b: number) => number;
    readonly crb_tree_remove: (a: number, b: bigint) => number;
    readonly crb_tree_update: (a: number, b: bigint, c: number) => number;
    readonly world_body_snapshot_count: (a: number) => number;
    readonly world_dynamic_body_snapshot: (a: number, b: number, c: number, d: number) => number;
    readonly world_dynamic_body_snapshot_count: (a: number) => number;
    readonly world_get_force_registry_count: (a: number) => number;
    readonly world_get_force_registry_typed_count: (a: number, b: number) => number;
    readonly world_get_gravity: (a: number, b: number) => void;
    readonly world_get_gravity_out: (a: number, b: number) => void;
    readonly world_get_integration_parameters: (a: number, b: number, c: number) => number;
    readonly world_set_integration_parameters: (a: number, b: number, c: number, d: number) => number;
    readonly world_update_body_poses: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly world_update_body_velocities: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly continuum_deformation_gradient: (a: number, b: number, c: number, d: number) => number;
    readonly continuum_linear_elastic_constitutive_matrix: (a: number, b: number, c: number, d: number) => number;
    readonly continuum_linear_tetra_element_stiffness: (a: number, b: number, c: number, d: number, e: number) => number;
    readonly continuum_tetra_strain_displacement_matrix: (a: number, b: number, c: number, d: number) => number;
    readonly continuum_newmark_beta_solve: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number, o: number) => number;
    readonly continuum_tetra_shape_functions: (a: number, b: number, c: number) => number;
    readonly continuum_tetra_volume: (a: number) => number;
    readonly control_lqr_like_stabilizing_input: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number) => number;
    readonly control_mpc_solve_box_qp: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number) => number;
    readonly control_pid_step: (a: number, b: number, c: number, d: number, e: number, f: number) => number;
    readonly control_state_space_step: (a: number, b: number, c: number, d: number, e: number, f: number, g: number, h: number, i: number, j: number, k: number, l: number, m: number, n: number) => number;
    readonly quantum_harmonic_oscillator_report: (a: number, b: number, c: number) => number;
    readonly quantum_rectangular_barrier_probability: (a: number) => number;
    readonly quantum_rectangular_barrier_tunneling: (a: number, b: number) => number;
    readonly quantum_reduced_planck_constant: () => number;
    readonly quantum_wave_normalize: (a: number, b: number) => number;
    readonly quantum_wave_probability_density: (a: number) => number;
    readonly quantum_wkb_transmission: (a: number, b: number) => number;
    readonly quantum_zero_point_energy: (a: number, b: number) => number;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_exn_store: (a: number) => void;
    readonly __externref_table_alloc: () => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
