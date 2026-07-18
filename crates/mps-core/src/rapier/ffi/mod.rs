pub mod convert;
pub mod types;

pub(crate) use convert::*;
pub use types::*;

#[cfg(test)]
mod tests {
    use rapier3d::prelude::{
        ColliderHandle, ImpulseJointHandle as RapierImpulseJointHandle, RigidBodyHandle,
    };

    use super::convert::{
        pack_collider_handle, pack_impulse_joint_handle, pack_rigid_body_handle,
        unpack_collider_handle, unpack_impulse_joint_handle, unpack_rigid_body_handle,
    };

    #[test]
    fn packed_arena_handles_reserve_zero_for_null() {
        let body = RigidBodyHandle::from_raw_parts(0, 0);
        let collider = ColliderHandle::from_raw_parts(0, 0);
        let joint = RapierImpulseJointHandle::from_raw_parts(0, 0);

        assert_ne!(pack_rigid_body_handle(body), 0);
        assert_ne!(pack_collider_handle(collider), 0);
        assert_ne!(pack_impulse_joint_handle(joint), 0);

        assert_eq!(
            unpack_rigid_body_handle(pack_rigid_body_handle(body)).into_raw_parts(),
            (0, 0)
        );
        assert_eq!(
            unpack_collider_handle(pack_collider_handle(collider)).into_raw_parts(),
            (0, 0)
        );
        assert_eq!(
            unpack_impulse_joint_handle(pack_impulse_joint_handle(joint)).into_raw_parts(),
            (0, 0)
        );
    }
}
