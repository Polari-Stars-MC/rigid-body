package org.polaris2023.msp_rigid_body.util;

import org.polaris2023.msp_rigid_body.RigidBody;

public final class PhysicsWorld implements AutoCloseable {
    Long handle;
    double deltaSeconds = 1.0 / 60.0;
    Long build, rigidBody;
    public PhysicsWorld(double gravityX, double gravityY, double gravityZ) {
        handle = RigidBody.worldCreate(gravityX, gravityY, gravityZ);
    }

    public boolean isEmpty() {
        return handle == 0L;
    }

    public boolean bodyEmpty() {
        return build == 0L;
    }
    public boolean physicsEmpty() {
        return rigidBody == 0L;
    }


    public PhysicsWorld translation(double x, double y, double z) {
        RigidBody.rigidBodyBuilderSetTranslation(build, x, y, z);
        return this;
    }

    public double[] translation() {
        return RigidBody.rigidBodyGetTranslation(handle, rigidBody);
    }
    public double translationX() {
        return translation()[0];
    }
    public double translationY() {
        return translation()[1];
    }
    public double translationZ() {
        return translation()[2];
    }



    public long build() {
        return build;
    }

    public PhysicsWorld body(int status) {
        build = RigidBody.rigidBodyBuilderCreate(status);
        return this;
    }

    public PhysicsWorld insert() {
        rigidBody = RigidBody.worldInsertRigidBody(handle, build);
        return this;
    }

    public long handle() {
        return handle;
    }

    public double[] gravity() {
        return RigidBody.worldGetGravity(handle);
    }

    public double gravityX() {
        return gravity()[0];
    }

    public double gravityY() {
        return gravity()[1];
    }

    public double gravityZ() {
        return gravity()[2];
    }



    public PhysicsWorld set(double gravityX, double gravityY, double gravityZ) {
        RigidBody.worldSetGravity(handle, gravityX, gravityY, gravityZ);
        return this;
    }

    public PhysicsWorld step() {
        RigidBody.worldStep(handle, deltaSeconds);
        return this;
    }

    public PhysicsWorld deltaSeconds(double deltaSeconds) {
        this.deltaSeconds = deltaSeconds;
        return this;
    }

    @Override
    public void close() throws Exception {
        RigidBody.worldDestroy(handle);
        RigidBody.rigidBodyBuilderDestroy(build);
        build = null;
        handle = null;
    }
}
