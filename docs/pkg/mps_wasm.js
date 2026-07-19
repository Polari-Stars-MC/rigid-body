/* @ts-self-types="./mps_wasm.d.ts" */

/**
 * Snapshot entry for one body — used for batch sync with Three.js.
 * Layout per body: 13 f64 values.
 */
export class BodySnapshotEntry {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        BodySnapshotEntryFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_bodysnapshotentry_free(ptr, 0);
    }
    /**
     * Angular velocity (x, y, z) at offset 10
     * @returns {number}
     */
    get angvel_x() {
        const ret = wasm.__wbg_get_bodysnapshotentry_angvel_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get angvel_y() {
        const ret = wasm.__wbg_get_bodysnapshotentry_angvel_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get angvel_z() {
        const ret = wasm.__wbg_get_bodysnapshotentry_angvel_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * Position (x, y, z) at offset 0
     * @returns {number}
     */
    get pos_x() {
        const ret = wasm.__wbg_get_bodysnapshotentry_pos_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get pos_y() {
        const ret = wasm.__wbg_get_bodysnapshotentry_pos_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get pos_z() {
        const ret = wasm.__wbg_get_bodysnapshotentry_pos_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * Rotation quaternion (i, j, k, w) at offset 3
     * @returns {number}
     */
    get rot_i() {
        const ret = wasm.__wbg_get_bodysnapshotentry_rot_i(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get rot_j() {
        const ret = wasm.__wbg_get_bodysnapshotentry_rot_j(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get rot_k() {
        const ret = wasm.__wbg_get_bodysnapshotentry_rot_k(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get rot_w() {
        const ret = wasm.__wbg_get_bodysnapshotentry_rot_w(this.__wbg_ptr);
        return ret;
    }
    /**
     * Linear velocity (x, y, z) at offset 7
     * @returns {number}
     */
    get vel_x() {
        const ret = wasm.__wbg_get_bodysnapshotentry_vel_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get vel_y() {
        const ret = wasm.__wbg_get_bodysnapshotentry_vel_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get vel_z() {
        const ret = wasm.__wbg_get_bodysnapshotentry_vel_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * Angular velocity (x, y, z) at offset 10
     * @param {number} arg0
     */
    set angvel_x(arg0) {
        wasm.__wbg_set_bodysnapshotentry_angvel_x(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set angvel_y(arg0) {
        wasm.__wbg_set_bodysnapshotentry_angvel_y(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set angvel_z(arg0) {
        wasm.__wbg_set_bodysnapshotentry_angvel_z(this.__wbg_ptr, arg0);
    }
    /**
     * Position (x, y, z) at offset 0
     * @param {number} arg0
     */
    set pos_x(arg0) {
        wasm.__wbg_set_bodysnapshotentry_pos_x(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set pos_y(arg0) {
        wasm.__wbg_set_bodysnapshotentry_pos_y(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set pos_z(arg0) {
        wasm.__wbg_set_bodysnapshotentry_pos_z(this.__wbg_ptr, arg0);
    }
    /**
     * Rotation quaternion (i, j, k, w) at offset 3
     * @param {number} arg0
     */
    set rot_i(arg0) {
        wasm.__wbg_set_bodysnapshotentry_rot_i(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set rot_j(arg0) {
        wasm.__wbg_set_bodysnapshotentry_rot_j(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set rot_k(arg0) {
        wasm.__wbg_set_bodysnapshotentry_rot_k(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set rot_w(arg0) {
        wasm.__wbg_set_bodysnapshotentry_rot_w(this.__wbg_ptr, arg0);
    }
    /**
     * Linear velocity (x, y, z) at offset 7
     * @param {number} arg0
     */
    set vel_x(arg0) {
        wasm.__wbg_set_bodysnapshotentry_vel_x(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set vel_y(arg0) {
        wasm.__wbg_set_bodysnapshotentry_vel_y(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set vel_z(arg0) {
        wasm.__wbg_set_bodysnapshotentry_vel_z(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) BodySnapshotEntry.prototype[Symbol.dispose] = BodySnapshotEntry.prototype.free;

/**
 * Body status (matches rapier3d BodyStatus).
 * @enum {0 | 1 | 2 | 3}
 */
export const BodyStatus = Object.freeze({
    Dynamic: 0, "0": "Dynamic",
    Fixed: 1, "1": "Fixed",
    KinematicPositionBased: 2, "2": "KinematicPositionBased",
    KinematicVelocityBased: 3, "3": "KinematicVelocityBased",
});

/**
 * @enum {0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9}
 */
export const CelestialBody = Object.freeze({
    Sun: 0, "0": "Sun",
    Mercury: 1, "1": "Mercury",
    Venus: 2, "2": "Venus",
    Earth: 3, "3": "Earth",
    Moon: 4, "4": "Moon",
    Mars: 5, "5": "Mars",
    Jupiter: 6, "6": "Jupiter",
    Saturn: 7, "7": "Saturn",
    Uranus: 8, "8": "Uranus",
    Neptune: 9, "9": "Neptune",
});

export class CelestialParams {
    static __wrap(ptr) {
        const obj = Object.create(CelestialParams.prototype);
        obj.__wbg_ptr = ptr;
        CelestialParamsFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        CelestialParamsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_celestialparams_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    get equatorial_radius() {
        const ret = wasm.__wbg_get_celestialparams_equatorial_radius(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get flattening() {
        const ret = wasm.__wbg_get_celestialparams_flattening(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get gm() {
        const ret = wasm.__wbg_get_celestialparams_gm(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get j2() {
        const ret = wasm.__wbg_get_celestialparams_j2(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get j3() {
        const ret = wasm.__wbg_get_celestialparams_j3(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get j4() {
        const ret = wasm.__wbg_get_celestialparams_j4(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get j5() {
        const ret = wasm.__wbg_get_celestialparams_j5(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get j6() {
        const ret = wasm.__wbg_get_celestialparams_j6(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get max_degree() {
        const ret = wasm.__wbg_get_celestialparams_max_degree(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {number}
     */
    get ref_radius() {
        const ret = wasm.__wbg_get_celestialparams_ref_radius(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get rotation_rate() {
        const ret = wasm.__wbg_get_celestialparams_rotation_rate(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} arg0
     */
    set equatorial_radius(arg0) {
        wasm.__wbg_set_celestialparams_equatorial_radius(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set flattening(arg0) {
        wasm.__wbg_set_celestialparams_flattening(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set gm(arg0) {
        wasm.__wbg_set_celestialparams_gm(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set j2(arg0) {
        wasm.__wbg_set_celestialparams_j2(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set j3(arg0) {
        wasm.__wbg_set_celestialparams_j3(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set j4(arg0) {
        wasm.__wbg_set_celestialparams_j4(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set j5(arg0) {
        wasm.__wbg_set_celestialparams_j5(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set j6(arg0) {
        wasm.__wbg_set_celestialparams_j6(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set max_degree(arg0) {
        wasm.__wbg_set_celestialparams_max_degree(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set ref_radius(arg0) {
        wasm.__wbg_set_celestialparams_ref_radius(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set rotation_rate(arg0) {
        wasm.__wbg_set_celestialparams_rotation_rate(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) CelestialParams.prototype[Symbol.dispose] = CelestialParams.prototype.free;

/**
 * Descriptor for creating a collider.
 */
export class ColliderDescriptor {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        ColliderDescriptorFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_colliderdescriptor_free(ptr, 0);
    }
    constructor() {
        const ret = wasm.colliderdescriptor_new();
        this.__wbg_ptr = ret;
        ColliderDescriptorFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Shape parameters: (radius, half_x, half_y, half_z) depending on type.
     * @returns {number}
     */
    get a() {
        const ret = wasm.__wbg_get_colliderdescriptor_a(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get b() {
        const ret = wasm.__wbg_get_colliderdescriptor_b(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get c() {
        const ret = wasm.__wbg_get_colliderdescriptor_c(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get collision_group() {
        const ret = wasm.__wbg_get_colliderdescriptor_collision_group(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {number}
     */
    get collision_mask() {
        const ret = wasm.__wbg_get_colliderdescriptor_collision_mask(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {number}
     */
    get d() {
        const ret = wasm.__wbg_get_colliderdescriptor_d(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get density() {
        const ret = wasm.__wbg_get_colliderdescriptor_density(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get friction() {
        const ret = wasm.__wbg_get_colliderdescriptor_friction(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {boolean}
     */
    get is_sensor() {
        const ret = wasm.__wbg_get_colliderdescriptor_is_sensor(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {number}
     */
    get restitution() {
        const ret = wasm.__wbg_get_colliderdescriptor_restitution(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Quat}
     */
    get rotation() {
        const ret = wasm.__wbg_get_colliderdescriptor_rotation(this.__wbg_ptr);
        return Quat.__wrap(ret);
    }
    /**
     * @returns {ShapeType}
     */
    get shape_type() {
        const ret = wasm.__wbg_get_colliderdescriptor_shape_type(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Vec3}
     */
    get translation() {
        const ret = wasm.__wbg_get_colliderdescriptor_translation(this.__wbg_ptr);
        return Vec3.__wrap(ret);
    }
    /**
     * Shape parameters: (radius, half_x, half_y, half_z) depending on type.
     * @param {number} arg0
     */
    set a(arg0) {
        wasm.__wbg_set_colliderdescriptor_a(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set b(arg0) {
        wasm.__wbg_set_colliderdescriptor_b(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set c(arg0) {
        wasm.__wbg_set_colliderdescriptor_c(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set collision_group(arg0) {
        wasm.__wbg_set_colliderdescriptor_collision_group(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set collision_mask(arg0) {
        wasm.__wbg_set_colliderdescriptor_collision_mask(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set d(arg0) {
        wasm.__wbg_set_colliderdescriptor_d(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set density(arg0) {
        wasm.__wbg_set_colliderdescriptor_density(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set friction(arg0) {
        wasm.__wbg_set_colliderdescriptor_friction(this.__wbg_ptr, arg0);
    }
    /**
     * @param {boolean} arg0
     */
    set is_sensor(arg0) {
        wasm.__wbg_set_colliderdescriptor_is_sensor(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set restitution(arg0) {
        wasm.__wbg_set_colliderdescriptor_restitution(this.__wbg_ptr, arg0);
    }
    /**
     * @param {Quat} arg0
     */
    set rotation(arg0) {
        _assertClass(arg0, Quat);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_colliderdescriptor_rotation(this.__wbg_ptr, ptr0);
    }
    /**
     * @param {ShapeType} arg0
     */
    set shape_type(arg0) {
        wasm.__wbg_set_colliderdescriptor_shape_type(this.__wbg_ptr, arg0);
    }
    /**
     * @param {Vec3} arg0
     */
    set translation(arg0) {
        _assertClass(arg0, Vec3);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_colliderdescriptor_translation(this.__wbg_ptr, ptr0);
    }
}
if (Symbol.dispose) ColliderDescriptor.prototype[Symbol.dispose] = ColliderDescriptor.prototype.free;

/**
 * Collision event record.
 */
export class CollisionEvent {
    static __wrap(ptr) {
        const obj = Object.create(CollisionEvent.prototype);
        obj.__wbg_ptr = ptr;
        CollisionEventFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        CollisionEventFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_collisionevent_free(ptr, 0);
    }
    /**
     * @returns {bigint}
     */
    get collider1() {
        const ret = wasm.__wbg_get_collisionevent_collider1(this.__wbg_ptr);
        return BigInt.asUintN(64, ret);
    }
    /**
     * @returns {bigint}
     */
    get collider2() {
        const ret = wasm.__wbg_get_collisionevent_collider2(this.__wbg_ptr);
        return BigInt.asUintN(64, ret);
    }
    /**
     * @returns {boolean}
     */
    get sensor() {
        const ret = wasm.__wbg_get_collisionevent_sensor(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {boolean}
     */
    get started() {
        const ret = wasm.__wbg_get_collisionevent_started(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @param {bigint} arg0
     */
    set collider1(arg0) {
        wasm.__wbg_set_collisionevent_collider1(this.__wbg_ptr, arg0);
    }
    /**
     * @param {bigint} arg0
     */
    set collider2(arg0) {
        wasm.__wbg_set_collisionevent_collider2(this.__wbg_ptr, arg0);
    }
    /**
     * @param {boolean} arg0
     */
    set sensor(arg0) {
        wasm.__wbg_set_collisionevent_sensor(this.__wbg_ptr, arg0);
    }
    /**
     * @param {boolean} arg0
     */
    set started(arg0) {
        wasm.__wbg_set_collisionevent_started(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) CollisionEvent.prototype[Symbol.dispose] = CollisionEvent.prototype.free;

/**
 * Contact force event record.
 */
export class ContactForceEvent {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        ContactForceEventFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_contactforceevent_free(ptr, 0);
    }
    /**
     * @returns {bigint}
     */
    get collider1() {
        const ret = wasm.__wbg_get_contactforceevent_collider1(this.__wbg_ptr);
        return BigInt.asUintN(64, ret);
    }
    /**
     * @returns {bigint}
     */
    get collider2() {
        const ret = wasm.__wbg_get_contactforceevent_collider2(this.__wbg_ptr);
        return BigInt.asUintN(64, ret);
    }
    /**
     * @returns {number}
     */
    get force_x() {
        const ret = wasm.__wbg_get_contactforceevent_force_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get force_y() {
        const ret = wasm.__wbg_get_contactforceevent_force_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get force_z() {
        const ret = wasm.__wbg_get_contactforceevent_force_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get max_force_dir_x() {
        const ret = wasm.__wbg_get_contactforceevent_max_force_dir_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get max_force_dir_y() {
        const ret = wasm.__wbg_get_contactforceevent_max_force_dir_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get max_force_dir_z() {
        const ret = wasm.__wbg_get_contactforceevent_max_force_dir_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get max_force_magnitude() {
        const ret = wasm.__wbg_get_contactforceevent_max_force_magnitude(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get total_force_magnitude() {
        const ret = wasm.__wbg_get_contactforceevent_total_force_magnitude(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {bigint} arg0
     */
    set collider1(arg0) {
        wasm.__wbg_set_contactforceevent_collider1(this.__wbg_ptr, arg0);
    }
    /**
     * @param {bigint} arg0
     */
    set collider2(arg0) {
        wasm.__wbg_set_contactforceevent_collider2(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set force_x(arg0) {
        wasm.__wbg_set_contactforceevent_force_x(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set force_y(arg0) {
        wasm.__wbg_set_contactforceevent_force_y(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set force_z(arg0) {
        wasm.__wbg_set_contactforceevent_force_z(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set max_force_dir_x(arg0) {
        wasm.__wbg_set_contactforceevent_max_force_dir_x(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set max_force_dir_y(arg0) {
        wasm.__wbg_set_contactforceevent_max_force_dir_y(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set max_force_dir_z(arg0) {
        wasm.__wbg_set_contactforceevent_max_force_dir_z(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set max_force_magnitude(arg0) {
        wasm.__wbg_set_contactforceevent_max_force_magnitude(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set total_force_magnitude(arg0) {
        wasm.__wbg_set_contactforceevent_total_force_magnitude(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) ContactForceEvent.prototype[Symbol.dispose] = ContactForceEvent.prototype.free;

/**
 * A physics world with configurable gravity and body management.
 */
export class PhysicsWorld {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        PhysicsWorldFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_physicsworld_free(ptr, 0);
    }
    /**
     * @param {bigint} handle
     * @param {number} fx
     * @param {number} fy
     * @param {number} fz
     * @param {boolean} wake_up
     */
    add_force(handle, fx, fy, fz, wake_up) {
        wasm.physicsworld_add_force(this.__wbg_ptr, handle, fx, fy, fz, wake_up);
    }
    /**
     * @param {bigint} handle
     * @param {number} tx
     * @param {number} ty
     * @param {number} tz
     * @param {boolean} wake_up
     */
    add_torque(handle, tx, ty, tz, wake_up) {
        wasm.physicsworld_add_torque(this.__wbg_ptr, handle, tx, ty, tz, wake_up);
    }
    /**
     * @param {bigint} handle
     * @param {number} ix
     * @param {number} iy
     * @param {number} iz
     * @param {boolean} wake_up
     */
    apply_impulse(handle, ix, iy, iz, wake_up) {
        wasm.physicsworld_apply_impulse(this.__wbg_ptr, handle, ix, iy, iz, wake_up);
    }
    /**
     * Cast a ray into the world. Returns collider handle or 0.
     * @param {number} ox
     * @param {number} oy
     * @param {number} oz
     * @param {number} dx
     * @param {number} dy
     * @param {number} dz
     * @param {number} max_toi
     * @returns {bigint}
     */
    cast_ray(ox, oy, oz, dx, dy, dz, max_toi) {
        const ret = wasm.physicsworld_cast_ray(this.__wbg_ptr, ox, oy, oz, dx, dy, dz, max_toi);
        return BigInt.asUintN(64, ret);
    }
    clear_events() {
        wasm.physicsworld_clear_events(this.__wbg_ptr);
    }
    /**
     * Create a dynamic box at position with given half-extents and mass.
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} hx
     * @param {number} hy
     * @param {number} hz
     * @param {number} mass
     * @returns {bigint}
     */
    create_dynamic_box(px, py, pz, hx, hy, hz, mass) {
        const ret = wasm.physicsworld_create_dynamic_box(this.__wbg_ptr, px, py, pz, hx, hy, hz, mass);
        return BigInt.asUintN(64, ret);
    }
    /**
     * Create a dynamic sphere at position with given radius and mass.
     * @param {number} px
     * @param {number} py
     * @param {number} pz
     * @param {number} radius
     * @param {number} mass
     * @returns {bigint}
     */
    create_dynamic_sphere(px, py, pz, radius, mass) {
        const ret = wasm.physicsworld_create_dynamic_sphere(this.__wbg_ptr, px, py, pz, radius, mass);
        return BigInt.asUintN(64, ret);
    }
    /**
     * Create a ground plane (halfspace) collider attached to a fixed body.
     * @param {number} nx
     * @param {number} ny
     * @param {number} nz
     * @param {number} dist
     * @returns {bigint}
     */
    create_ground_plane(nx, ny, nz, dist) {
        const ret = wasm.physicsworld_create_ground_plane(this.__wbg_ptr, nx, ny, nz, dist);
        return BigInt.asUintN(64, ret);
    }
    /**
     * Destroy the world.
     */
    destroy() {
        wasm.physicsworld_destroy(this.__wbg_ptr);
    }
    /**
     * Enable/disable CCD. FFI: rigid_body_enable_ccd(world, handle, Bool)
     * @param {bigint} handle
     * @param {boolean} enabled
     */
    enable_ccd(handle, enabled) {
        wasm.physicsworld_enable_ccd(this.__wbg_ptr, handle, enabled);
    }
    /**
     * @param {bigint} handle
     * @returns {Vec3}
     */
    get_body_angular_velocity(handle) {
        const ret = wasm.physicsworld_get_body_angular_velocity(this.__wbg_ptr, handle);
        return Vec3.__wrap(ret);
    }
    /**
     * Returns body count as i32 (matches FFI).
     * @returns {number}
     */
    get_body_count() {
        const ret = wasm.physicsworld_get_body_count(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {bigint} handle
     * @returns {Vec3}
     */
    get_body_linear_velocity(handle) {
        const ret = wasm.physicsworld_get_body_linear_velocity(this.__wbg_ptr, handle);
        return Vec3.__wrap(ret);
    }
    /**
     * @param {bigint} handle
     * @returns {Quat}
     */
    get_body_rotation(handle) {
        const ret = wasm.physicsworld_get_body_rotation(this.__wbg_ptr, handle);
        return Quat.__wrap(ret);
    }
    /**
     * Batch snapshot for Three.js sync.
     * Returns Float64Array: [pos(3), quat(4), vel(3), angvel(3)] per body = 13 f64 each.
     * @returns {Float64Array}
     */
    get_body_snapshot() {
        const ret = wasm.physicsworld_get_body_snapshot(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {bigint} handle
     * @returns {Vec3}
     */
    get_body_translation(handle) {
        const ret = wasm.physicsworld_get_body_translation(this.__wbg_ptr, handle);
        return Vec3.__wrap(ret);
    }
    /**
     * Get parameters of a built-in celestial body.
     * @param {number} body_id
     * @returns {CelestialParams | undefined}
     */
    get_celestial_params(body_id) {
        const ret = wasm.physicsworld_get_celestial_params(this.__wbg_ptr, body_id);
        return ret === 0 ? undefined : CelestialParams.__wrap(ret);
    }
    /**
     * Returns collider count as i32 (matches FFI).
     * @returns {number}
     */
    get_collider_count() {
        const ret = wasm.physicsworld_get_collider_count(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} index
     * @returns {CollisionEvent}
     */
    get_collision_event(index) {
        const ret = wasm.physicsworld_get_collision_event(this.__wbg_ptr, index);
        return CollisionEvent.__wrap(ret);
    }
    /**
     * @returns {number}
     */
    get_collision_event_count() {
        const ret = wasm.physicsworld_get_collision_event_count(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {Array<any>}
     */
    get_collision_events() {
        const ret = wasm.physicsworld_get_collision_events(this.__wbg_ptr);
        return ret;
    }
    /**
     * Insert a collider from a descriptor. Optionally attach to a parent body.
     * @param {ColliderDescriptor} desc
     * @param {bigint} parent_body
     * @returns {bigint}
     */
    insert_collider(desc, parent_body) {
        _assertClass(desc, ColliderDescriptor);
        const ret = wasm.physicsworld_insert_collider(this.__wbg_ptr, desc.__wbg_ptr, parent_body);
        return BigInt.asUintN(64, ret);
    }
    /**
     * @param {RigidBodyDescriptor} desc
     * @returns {bigint}
     */
    insert_rigid_body(desc) {
        _assertClass(desc, RigidBodyDescriptor);
        const ret = wasm.physicsworld_insert_rigid_body(this.__wbg_ptr, desc.__wbg_ptr);
        return BigInt.asUintN(64, ret);
    }
    /**
     * Count bodies intersecting an AABB.
     * @param {number} min_x
     * @param {number} min_y
     * @param {number} min_z
     * @param {number} max_x
     * @param {number} max_y
     * @param {number} max_z
     * @returns {number}
     */
    intersect_aabb_count(min_x, min_y, min_z, max_x, max_y, max_z) {
        const ret = wasm.physicsworld_intersect_aabb_count(this.__wbg_ptr, min_x, min_y, min_z, max_x, max_y, max_z);
        return ret >>> 0;
    }
    /**
     * @param {number} gx
     * @param {number} gy
     * @param {number} gz
     */
    constructor(gx, gy, gz) {
        const ret = wasm.physicsworld_new(gx, gy, gz);
        this.__wbg_ptr = ret;
        PhysicsWorldFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Register a celestial body's gravity. Returns the force law handle (>0) or 0 on error.
     * @param {number} body_id
     * @param {number} degree
     * @returns {bigint}
     */
    register_celestial_gravity(body_id, degree) {
        const ret = wasm.physicsworld_register_celestial_gravity(this.__wbg_ptr, body_id, degree);
        return BigInt.asUintN(64, ret);
    }
    /**
     * Remove a collider by handle.
     * @param {bigint} handle
     */
    remove_collider(handle) {
        wasm.physicsworld_remove_collider(this.__wbg_ptr, handle);
    }
    /**
     * @param {bigint} handle
     * @param {boolean} remove_colliders
     */
    remove_rigid_body(handle, remove_colliders) {
        wasm.physicsworld_remove_rigid_body(this.__wbg_ptr, handle, remove_colliders);
    }
    /**
     * @param {bigint} handle
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    set_body_translation(handle, x, y, z) {
        wasm.physicsworld_set_body_translation(this.__wbg_ptr, handle, x, y, z);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    set_gravity(x, y, z) {
        wasm.physicsworld_set_gravity(this.__wbg_ptr, x, y, z);
    }
    /**
     * Put a body to sleep. FFI: rigid_body_sleep(world, handle) -> Bool
     * @param {bigint} handle
     */
    sleep(handle) {
        wasm.physicsworld_sleep(this.__wbg_ptr, handle);
    }
    /**
     * @param {number} dt
     */
    step(dt) {
        wasm.physicsworld_step(this.__wbg_ptr, dt);
    }
    /**
     * Wake up a body. FFI: rigid_body_wake_up(world, handle, strong: Bool)
     * @param {bigint} handle
     */
    wake_up(handle) {
        wasm.physicsworld_wake_up(this.__wbg_ptr, handle);
    }
}
if (Symbol.dispose) PhysicsWorld.prototype[Symbol.dispose] = PhysicsWorld.prototype.free;

/**
 * Quaternion (i, j, k, w) — compatible with Three.js ordering.
 */
export class Quat {
    static __wrap(ptr) {
        const obj = Object.create(Quat.prototype);
        obj.__wbg_ptr = ptr;
        QuatFinalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        QuatFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_quat_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    get i() {
        const ret = wasm.__wbg_get_quat_i(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get j() {
        const ret = wasm.__wbg_get_quat_j(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get k() {
        const ret = wasm.__wbg_get_quat_k(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get w() {
        const ret = wasm.__wbg_get_quat_w(this.__wbg_ptr);
        return ret;
    }
    /**
     * Identity quaternion.
     * @returns {Quat}
     */
    static identity() {
        const ret = wasm.quat_identity();
        return Quat.__wrap(ret);
    }
    /**
     * @param {number} i
     * @param {number} j
     * @param {number} k
     * @param {number} w
     */
    constructor(i, j, k, w) {
        const ret = wasm.quat_new(i, j, k, w);
        this.__wbg_ptr = ret;
        QuatFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} arg0
     */
    set i(arg0) {
        wasm.__wbg_set_quat_i(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set j(arg0) {
        wasm.__wbg_set_quat_j(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set k(arg0) {
        wasm.__wbg_set_quat_k(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set w(arg0) {
        wasm.__wbg_set_quat_w(this.__wbg_ptr, arg0);
    }
}
if (Symbol.dispose) Quat.prototype[Symbol.dispose] = Quat.prototype.free;

/**
 * Descriptor for creating a rigid body.
 */
export class RigidBodyDescriptor {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        RigidBodyDescriptorFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_rigidbodydescriptor_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    get additional_mass() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_additional_mass(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get angular_damping() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_angular_damping(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Vec3}
     */
    get angular_velocity() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_angular_velocity(this.__wbg_ptr);
        return Vec3.__wrap(ret);
    }
    /**
     * @returns {boolean}
     */
    get can_sleep() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_can_sleep(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {boolean}
     */
    get ccd_enabled() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_ccd_enabled(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {number}
     */
    get gravity_scale() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_gravity_scale(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get linear_damping() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_linear_damping(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Vec3}
     */
    get linear_velocity() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_linear_velocity(this.__wbg_ptr);
        return Vec3.__wrap(ret);
    }
    /**
     * @returns {Quat}
     */
    get rotation() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_rotation(this.__wbg_ptr);
        return Quat.__wrap(ret);
    }
    /**
     * @returns {BodyStatus}
     */
    get status() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_status(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Vec3}
     */
    get translation() {
        const ret = wasm.__wbg_get_rigidbodydescriptor_translation(this.__wbg_ptr);
        return Vec3.__wrap(ret);
    }
    constructor() {
        const ret = wasm.rigidbodydescriptor_new();
        this.__wbg_ptr = ret;
        RigidBodyDescriptorFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} arg0
     */
    set additional_mass(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_additional_mass(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set angular_damping(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_angular_damping(this.__wbg_ptr, arg0);
    }
    /**
     * @param {Vec3} arg0
     */
    set angular_velocity(arg0) {
        _assertClass(arg0, Vec3);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_rigidbodydescriptor_angular_velocity(this.__wbg_ptr, ptr0);
    }
    /**
     * @param {boolean} arg0
     */
    set can_sleep(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_can_sleep(this.__wbg_ptr, arg0);
    }
    /**
     * @param {boolean} arg0
     */
    set ccd_enabled(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_ccd_enabled(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set gravity_scale(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_gravity_scale(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set linear_damping(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_linear_damping(this.__wbg_ptr, arg0);
    }
    /**
     * @param {Vec3} arg0
     */
    set linear_velocity(arg0) {
        _assertClass(arg0, Vec3);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_rigidbodydescriptor_linear_velocity(this.__wbg_ptr, ptr0);
    }
    /**
     * @param {Quat} arg0
     */
    set rotation(arg0) {
        _assertClass(arg0, Quat);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_rigidbodydescriptor_rotation(this.__wbg_ptr, ptr0);
    }
    /**
     * @param {BodyStatus} arg0
     */
    set status(arg0) {
        wasm.__wbg_set_rigidbodydescriptor_status(this.__wbg_ptr, arg0);
    }
    /**
     * @param {Vec3} arg0
     */
    set translation(arg0) {
        _assertClass(arg0, Vec3);
        var ptr0 = arg0.__destroy_into_raw();
        wasm.__wbg_set_rigidbodydescriptor_translation(this.__wbg_ptr, ptr0);
    }
}
if (Symbol.dispose) RigidBodyDescriptor.prototype[Symbol.dispose] = RigidBodyDescriptor.prototype.free;

/**
 * Shape types for colliders.
 * @enum {0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8}
 */
export const ShapeType = Object.freeze({
    Ball: 0, "0": "Ball",
    Cuboid: 1, "1": "Cuboid",
    Capsule: 2, "2": "Capsule",
    Cylinder: 3, "3": "Cylinder",
    Cone: 4, "4": "Cone",
    Halfspace: 5, "5": "Halfspace",
    Heightfield: 6, "6": "Heightfield",
    ConvexHull: 7, "7": "ConvexHull",
    TriangleMesh: 8, "8": "TriangleMesh",
});

/**
 * 3D vector (matches rapier3d/Three.js convention: right-handed Y-up).
 */
export class Vec3 {
    static __wrap(ptr) {
        const obj = Object.create(Vec3.prototype);
        obj.__wbg_ptr = ptr;
        Vec3Finalization.register(obj, obj.__wbg_ptr, obj);
        return obj;
    }
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        Vec3Finalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_vec3_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    get x() {
        const ret = wasm.__wbg_get_vec3_x(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get y() {
        const ret = wasm.__wbg_get_vec3_y(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    get z() {
        const ret = wasm.__wbg_get_vec3_z(this.__wbg_ptr);
        return ret;
    }
    /**
     * @param {number} arg0
     */
    set x(arg0) {
        wasm.__wbg_set_vec3_x(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set y(arg0) {
        wasm.__wbg_set_vec3_y(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} arg0
     */
    set z(arg0) {
        wasm.__wbg_set_vec3_z(this.__wbg_ptr, arg0);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    constructor(x, y, z) {
        const ret = wasm.vec3_new(x, y, z);
        this.__wbg_ptr = ret;
        Vec3Finalization.register(this, this.__wbg_ptr, this);
        return this;
    }
}
if (Symbol.dispose) Vec3.prototype[Symbol.dispose] = Vec3.prototype.free;

/**
 * Initialize the WASM module (sets panic hook).
 */
export function init() {
    wasm.init();
}

/**
 * Returns the version string.
 * @returns {string}
 */
export function version() {
    let deferred1_0;
    let deferred1_1;
    try {
        const ret = wasm.version();
        deferred1_0 = ret[0];
        deferred1_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
    }
}
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg___wbindgen_throw_ea4887a5f8f9a9db: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbg_error_a6fa202b58aa1cd3: function(arg0, arg1) {
            let deferred0_0;
            let deferred0_1;
            try {
                deferred0_0 = arg0;
                deferred0_1 = arg1;
                console.error(getStringFromWasm0(arg0, arg1));
            } finally {
                wasm.__wbindgen_free(deferred0_0, deferred0_1, 1);
            }
        },
        __wbg_new_227d7c05414eb861: function() {
            const ret = new Error();
            return ret;
        },
        __wbg_new_2e117a478906f062: function() {
            const ret = new Object();
            return ret;
        },
        __wbg_new_36e147a8ced3c6e0: function() {
            const ret = new Array();
            return ret;
        },
        __wbg_push_f724b5db8acf89d2: function(arg0, arg1) {
            const ret = arg0.push(arg1);
            return ret;
        },
        __wbg_set_4564f7dc44fcb0c9: function() { return handleError(function (arg0, arg1, arg2) {
            const ret = Reflect.set(arg0, arg1, arg2);
            return ret;
        }, arguments); },
        __wbg_stack_3b0d974bbf31e44f: function(arg0, arg1) {
            const ret = arg1.stack;
            const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len1 = WASM_VECTOR_LEN;
            getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
        },
        __wbindgen_cast_0000000000000001: function(arg0, arg1) {
            // Cast intrinsic for `Ref(Slice(F64)) -> NamedExternref("Float64Array")`.
            const ret = getArrayF64FromWasm0(arg0, arg1);
            return ret;
        },
        __wbindgen_cast_0000000000000002: function(arg0, arg1) {
            // Cast intrinsic for `Ref(String) -> Externref`.
            const ret = getStringFromWasm0(arg0, arg1);
            return ret;
        },
        __wbindgen_cast_0000000000000003: function(arg0) {
            // Cast intrinsic for `U64 -> Externref`.
            const ret = BigInt.asUintN(64, arg0);
            return ret;
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./mps_wasm_bg.js": import0,
    };
}

const BodySnapshotEntryFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_bodysnapshotentry_free(ptr, 1));
const CelestialParamsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_celestialparams_free(ptr, 1));
const ColliderDescriptorFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_colliderdescriptor_free(ptr, 1));
const CollisionEventFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_collisionevent_free(ptr, 1));
const ContactForceEventFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_contactforceevent_free(ptr, 1));
const PhysicsWorldFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_physicsworld_free(ptr, 1));
const QuatFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_quat_free(ptr, 1));
const RigidBodyDescriptorFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_rigidbodydescriptor_free(ptr, 1));
const Vec3Finalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_vec3_free(ptr, 1));

function addToExternrefTable0(obj) {
    const idx = wasm.__externref_table_alloc();
    wasm.__wbindgen_externrefs.set(idx, obj);
    return idx;
}

function _assertClass(instance, klass) {
    if (!(instance instanceof klass)) {
        throw new Error(`expected instance of ${klass.name}`);
    }
}

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

let cachedDataViewMemory0 = null;
function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function handleError(f, args) {
    try {
        return f.apply(this, args);
    } catch (e) {
        const idx = addToExternrefTable0(e);
        wasm.__wbindgen_exn_store(idx);
    }
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedDataViewMemory0 = null;
    cachedFloat64ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('mps_wasm_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
