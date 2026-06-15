package org.polaris2023.msp_rigid_body;

import net.neoforged.neoforge.common.ModConfigSpec;

final class MpsRigidBodyConfig {
    static final ModConfigSpec SPEC;

    static final ModConfigSpec.BooleanValue ENABLED;
    static final ModConfigSpec.DoubleValue GRAVITY_X;
    static final ModConfigSpec.DoubleValue GRAVITY_Y;
    static final ModConfigSpec.DoubleValue GRAVITY_Z;
    static final ModConfigSpec.DoubleValue STEP_SECONDS;
    static final ModConfigSpec.IntValue SOLVER_ITERATIONS;
    static final ModConfigSpec.IntValue CCD_SUBSTEPS;
    static final ModConfigSpec.IntValue MAX_VOXEL_AREA_BLOCKS;
    static final ModConfigSpec.IntValue MAX_COLLIDERS;
    static final ModConfigSpec.DoubleValue DEFAULT_VOXEL_SIZE;
    static final ModConfigSpec.BooleanValue DEBUG_PARTICLES;
    static final ModConfigSpec.BooleanValue ASYNC_VOXEL_BUILD;
    static final ModConfigSpec.BooleanValue AUTO_CHUNK_COLLIDERS;
    static final ModConfigSpec.BooleanValue AUTO_BLOCK_UPDATES;
    static final ModConfigSpec.IntValue MAX_CONTACT_EVENTS_PER_TICK;

    static {
        ModConfigSpec.Builder builder = new ModConfigSpec.Builder();
        builder.push("physics");
        ENABLED = builder.define("enabled", true);
        GRAVITY_X = builder.defineInRange("gravityX", 0.0, -256.0, 256.0);
        GRAVITY_Y = builder.defineInRange("gravityY", -9.81, -256.0, 256.0);
        GRAVITY_Z = builder.defineInRange("gravityZ", 0.0, -256.0, 256.0);
        STEP_SECONDS = builder.defineInRange("stepSeconds", 1.0 / 20.0, 1.0e-5, 1.0);
        SOLVER_ITERATIONS = builder.defineInRange("solverIterations", 8, 1, 128);
        CCD_SUBSTEPS = builder.defineInRange("ccdSubsteps", 4, 0, 64);
        MAX_COLLIDERS = builder.defineInRange("maxColliders", 4096, 1, 1_000_000);
        MAX_CONTACT_EVENTS_PER_TICK = builder.defineInRange("maxContactEventsPerTick", 64, 0, 16_384);
        builder.pop();

        builder.push("voxels");
        MAX_VOXEL_AREA_BLOCKS = builder.defineInRange("maxVoxelAreaBlocks", 65_536, 1, 16_777_216);
        DEFAULT_VOXEL_SIZE = builder.defineInRange("defaultVoxelSize", 1.0, 0.01, 16.0);
        ASYNC_VOXEL_BUILD = builder.define("asyncVoxelBuild", true);
        AUTO_CHUNK_COLLIDERS = builder.define("autoChunkColliders", false);
        AUTO_BLOCK_UPDATES = builder.define("autoBlockUpdates", false);
        builder.pop();

        builder.push("debug");
        DEBUG_PARTICLES = builder.define("debugParticles", false);
        builder.pop();

        SPEC = builder.build();
    }

    private MpsRigidBodyConfig() {
    }
}
