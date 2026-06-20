package org.polaris2023.msp_rigid_body.util;

import org.polaris2023.msp_rigid_body.RigidBodyNative;

public final class SpaceFormulas {
    private SpaceFormulas() {
    }

    public static double keplerPeriod(double mu, double semiMajorAxis) {
        return RigidBodyNative.spaceKeplerPeriod(mu, semiMajorAxis);
    }

    public static double keplerSemiMajorAxis(double mu, double period) {
        return RigidBodyNative.spaceKeplerSemiMajorAxis(mu, period);
    }

    public static HohmannTransfer hohmannTransfer(double mu, double radius1, double radius2) {
        try (NativeMemory out = new NativeMemory(32)) {
            if (!RigidBodyNative.spaceHohmannTransfer(mu, radius1, radius2, out.address())) {
                throw new IllegalArgumentException("invalid Hohmann transfer parameters");
            }
            return new HohmannTransfer(out.getDouble(0), out.getDouble(8), out.getDouble(16), out.getDouble(24));
        }
    }

    public static double[] atmosphericDragAcceleration(
            double vx, double vy, double vz,
            double atmosphereVx, double atmosphereVy, double atmosphereVz,
            double density, double dragCoefficient, double area, double mass) {
        try (NativeMemory out = new NativeMemory(24)) {
            if (!RigidBodyNative.spaceAtmosphericDragAcceleration(
                    vx, vy, vz,
                    atmosphereVx, atmosphereVy, atmosphereVz,
                    density, dragCoefficient, area, mass,
                    out.address())) {
                throw new IllegalArgumentException("invalid atmospheric drag parameters");
            }
            return out.getVec3(0);
        }
    }

    public static double[] applyAtmosphericDragToBody(
            PhysicsWorld world,
            RigidBody body,
            double atmosphereVx, double atmosphereVy, double atmosphereVz,
            double density, double dragCoefficient, double area, double mass,
            boolean wakeUp) {
        try (NativeMemory out = new NativeMemory(24)) {
            if (!RigidBodyNative.spaceApplyAtmosphericDragToBody(
                    world.handle(),
                    body.handle(),
                    atmosphereVx, atmosphereVy, atmosphereVz,
                    density, dragCoefficient, area, mass,
                    wakeUp ? 1 : 0,
                    out.address())) {
                throw new IllegalArgumentException("invalid atmospheric drag body parameters");
            }
            return out.getVec3(0);
        }
    }

    public static QuaternionDerivative quaternionDerivative(
            double qi, double qj, double qk, double qw,
            double wx, double wy, double wz) {
        try (NativeMemory out = new NativeMemory(32)) {
            if (!RigidBodyNative.spaceQuaternionDerivative(qi, qj, qk, qw, wx, wy, wz, out.address())) {
                throw new IllegalArgumentException("invalid quaternion derivative parameters");
            }
            return new QuaternionDerivative(out.getDouble(0), out.getDouble(8), out.getDouble(16), out.getDouble(24));
        }
    }

    public static ScalarKalman ekfPredictScalar(
            double state, double covariance, double nonlinearDelta, double jacobian, double processNoise) {
        try (NativeMemory out = new NativeMemory(16)) {
            if (!RigidBodyNative.spaceEkfPredictScalar(state, covariance, nonlinearDelta, jacobian, processNoise, out.address())) {
                throw new IllegalArgumentException("invalid EKF prediction parameters");
            }
            return new ScalarKalman(out.getDouble(0), out.getDouble(8));
        }
    }

    public static double ekfGainScalar(double covariance, double measurementJacobian, double measurementNoise) {
        return RigidBodyNative.spaceEkfGainScalar(covariance, measurementJacobian, measurementNoise);
    }

    public static ScalarKalman ekfUpdateScalar(
            double predictedState, double predictedCovariance,
            double measurement, double predictedMeasurement,
            double kalmanGain, double measurementJacobian) {
        try (NativeMemory out = new NativeMemory(16)) {
            if (!RigidBodyNative.spaceEkfUpdateScalar(
                    predictedState, predictedCovariance,
                    measurement, predictedMeasurement,
                    kalmanGain, measurementJacobian,
                    out.address())) {
                throw new IllegalArgumentException("invalid EKF update parameters");
            }
            return new ScalarKalman(out.getDouble(0), out.getDouble(8));
        }
    }

    public record HohmannTransfer(double deltaV1, double deltaV2, double totalDeltaV, double transferTime) {
    }

    public record QuaternionDerivative(double iDot, double jDot, double kDot, double wDot) {
    }

    public record ScalarKalman(double value, double covariance) {
    }
}
