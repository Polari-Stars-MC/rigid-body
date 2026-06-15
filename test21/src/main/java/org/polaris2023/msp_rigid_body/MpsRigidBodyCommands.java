package org.polaris2023.msp_rigid_body;

import com.mojang.brigadier.CommandDispatcher;
import com.mojang.brigadier.arguments.DoubleArgumentType;
import com.mojang.brigadier.arguments.IntegerArgumentType;
import com.mojang.brigadier.arguments.StringArgumentType;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.commands.arguments.EntityArgument;
import net.minecraft.commands.arguments.coordinates.BlockPosArgument;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.entity.Entity;

final class MpsRigidBodyCommands {
    private MpsRigidBodyCommands() {
    }

    static void register(CommandDispatcher<CommandSourceStack> dispatcher, PhysicsRuntime runtime) {
        dispatcher.register(Commands.literal("mpsrb")
                .requires(source -> source.hasPermission(2))
                .then(Commands.literal("status").executes(context -> status(context.getSource(), runtime)))
                .then(Commands.literal("client_status").executes(context -> clientStatus(context.getSource())))
                .then(Commands.literal("profile").executes(context -> profile(context.getSource(), runtime)))
                .then(Commands.literal("reset").executes(context -> reset(context.getSource(), runtime)))
                .then(Commands.literal("clear").executes(context -> reset(context.getSource(), runtime)))
                .then(Commands.literal("enable").executes(context -> setEnabled(context.getSource(), runtime, true)))
                .then(Commands.literal("disable").executes(context -> setEnabled(context.getSource(), runtime, false)))
                .then(Commands.literal("gravity")
                        .then(Commands.argument("x", DoubleArgumentType.doubleArg(-256.0, 256.0))
                                .then(Commands.argument("y", DoubleArgumentType.doubleArg(-256.0, 256.0))
                                        .then(Commands.argument("z", DoubleArgumentType.doubleArg(-256.0, 256.0))
                                                .executes(context -> gravity(
                                                        context.getSource(),
                                                        runtime,
                                                        DoubleArgumentType.getDouble(context, "x"),
                                                        DoubleArgumentType.getDouble(context, "y"),
                                                        DoubleArgumentType.getDouble(context, "z")))))))
                .then(Commands.literal("step")
                        .then(Commands.argument("ticks", IntegerArgumentType.integer(1, 200))
                                .executes(context -> step(
                                        context.getSource(),
                                        runtime,
                                        IntegerArgumentType.getInteger(context, "ticks")))))
                .then(Commands.literal("voxelize_area")
                        .then(Commands.argument("from", BlockPosArgument.blockPos())
                                .then(Commands.argument("to", BlockPosArgument.blockPos())
                                        .then(Commands.argument("voxelSize", DoubleArgumentType.doubleArg(0.01, 16.0))
                                                .executes(context -> voxelizeArea(
                                                        context.getSource(),
                                                        runtime,
                                                        BlockPosArgument.getLoadedBlockPos(context, "from"),
                                                        BlockPosArgument.getLoadedBlockPos(context, "to"),
                                                        DoubleArgumentType.getDouble(context, "voxelSize")))))))
                .then(Commands.literal("voxelize_area_async")
                        .then(Commands.argument("from", BlockPosArgument.blockPos())
                                .then(Commands.argument("to", BlockPosArgument.blockPos())
                                        .then(Commands.argument("voxelSize", DoubleArgumentType.doubleArg(0.01, 16.0))
                                                .executes(context -> voxelizeAreaAsync(
                                                        context.getSource(),
                                                        runtime,
                                                        BlockPosArgument.getLoadedBlockPos(context, "from"),
                                                        BlockPosArgument.getLoadedBlockPos(context, "to"),
                                                        DoubleArgumentType.getDouble(context, "voxelSize")))))))
                .then(Commands.literal("save_area")
                        .then(Commands.argument("id", StringArgumentType.word())
                                .then(Commands.argument("from", BlockPosArgument.blockPos())
                                        .then(Commands.argument("to", BlockPosArgument.blockPos())
                                                .then(Commands.argument("voxelSize", DoubleArgumentType.doubleArg(0.01, 16.0))
                                                        .executes(context -> saveArea(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                BlockPosArgument.getLoadedBlockPos(context, "from"),
                                                                BlockPosArgument.getLoadedBlockPos(context, "to"),
                                                                DoubleArgumentType.getDouble(context, "voxelSize"))))))))
                .then(Commands.literal("areas")
                        .then(Commands.literal("list").executes(context -> listAreas(context.getSource(), runtime)))
                        .then(Commands.literal("remove")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .executes(context -> removeArea(context.getSource(), runtime, StringArgumentType.getString(context, "id")))))
                        .then(Commands.literal("clear").executes(context -> clearAreas(context.getSource(), runtime)))
                        .then(Commands.literal("rebuild").executes(context -> rebuildAreas(context.getSource(), runtime))))
                .then(Commands.literal("entity")
                        .then(Commands.literal("bind")
                                .then(Commands.argument("target", EntityArgument.entity())
                                        .then(Commands.argument("halfExtent", DoubleArgumentType.doubleArg(0.05, 8.0))
                                                .executes(context -> bindEntity(
                                                        context.getSource(),
                                                        runtime,
                                                        EntityArgument.getEntity(context, "target"),
                                                        DoubleArgumentType.getDouble(context, "halfExtent"))))))
                        .then(Commands.literal("save_bind")
                                .then(Commands.argument("target", EntityArgument.entity())
                                        .then(Commands.argument("halfExtent", DoubleArgumentType.doubleArg(0.05, 8.0))
                                                .executes(context -> bindEntityPersistent(
                                                        context.getSource(),
                                                        runtime,
                                                        EntityArgument.getEntity(context, "target"),
                                                        DoubleArgumentType.getDouble(context, "halfExtent"))))))
                        .then(Commands.literal("list").executes(context -> listBindings(context.getSource(), runtime)))
                        .then(Commands.literal("rebuild").executes(context -> rebuildBindings(context.getSource(), runtime)))
                        .then(Commands.literal("clear").executes(context -> clearBindings(context.getSource(), runtime))))
                .then(Commands.literal("joint")
                        .then(Commands.literal("fixed")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .executes(context -> createJoint(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                "fixed",
                                                                EntityArgument.getEntity(context, "first"),
                                                                EntityArgument.getEntity(context, "second")))))))
                        .then(Commands.literal("save_fixed")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .executes(context -> saveJoint(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                "fixed",
                                                                EntityArgument.getEntity(context, "first"),
                                                                EntityArgument.getEntity(context, "second")))))))
                        .then(Commands.literal("revolute")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .executes(context -> createJoint(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                "revolute",
                                                                EntityArgument.getEntity(context, "first"),
                                                                EntityArgument.getEntity(context, "second")))))))
                        .then(Commands.literal("prismatic")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .then(Commands.argument("axisX", DoubleArgumentType.doubleArg(-1.0, 1.0))
                                                                .then(Commands.argument("axisY", DoubleArgumentType.doubleArg(-1.0, 1.0))
                                                                        .then(Commands.argument("axisZ", DoubleArgumentType.doubleArg(-1.0, 1.0))
                                                                                .executes(context -> createJointAdvanced(
                                                                                        context.getSource(),
                                                                                        runtime,
                                                                                        StringArgumentType.getString(context, "id"),
                                                                                        "prismatic",
                                                                                        EntityArgument.getEntity(context, "first"),
                                                                                        EntityArgument.getEntity(context, "second"),
                                                                                        DoubleArgumentType.getDouble(context, "axisX"),
                                                                                        DoubleArgumentType.getDouble(context, "axisY"),
                                                                                        DoubleArgumentType.getDouble(context, "axisZ"),
                                                                                        0.0,
                                                                                        0.0)))))))))
                        .then(Commands.literal("rope")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .then(Commands.argument("maxDistance", DoubleArgumentType.doubleArg(0.0, 128.0))
                                                                .executes(context -> createJointAdvanced(
                                                                        context.getSource(),
                                                                        runtime,
                                                                        StringArgumentType.getString(context, "id"),
                                                                        "rope",
                                                                        EntityArgument.getEntity(context, "first"),
                                                                        EntityArgument.getEntity(context, "second"),
                                                                        0.0,
                                                                        1.0,
                                                                        0.0,
                                                                        DoubleArgumentType.getDouble(context, "maxDistance"),
                                                                        0.0)))))))
                        .then(Commands.literal("spring")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .then(Commands.argument("restLength", DoubleArgumentType.doubleArg(0.0, 128.0))
                                                                .then(Commands.argument("stiffness", DoubleArgumentType.doubleArg(0.0, 1_000_000.0))
                                                                        .then(Commands.argument("damping", DoubleArgumentType.doubleArg(0.0, 1_000_000.0))
                                                                                .executes(context -> createJointAdvanced(
                                                                                        context.getSource(),
                                                                                        runtime,
                                                                                        StringArgumentType.getString(context, "id"),
                                                                                        "spring",
                                                                                        EntityArgument.getEntity(context, "first"),
                                                                                        EntityArgument.getEntity(context, "second"),
                                                                                        DoubleArgumentType.getDouble(context, "restLength"),
                                                                                        0.0,
                                                                                        0.0,
                                                                                        DoubleArgumentType.getDouble(context, "stiffness"),
                                                                                        DoubleArgumentType.getDouble(context, "damping"))))))))))
                        .then(Commands.literal("spherical")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("first", EntityArgument.entity())
                                                .then(Commands.argument("second", EntityArgument.entity())
                                                        .executes(context -> createJoint(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                "spherical",
                                                                EntityArgument.getEntity(context, "first"),
                                                                EntityArgument.getEntity(context, "second")))))))
                        .then(Commands.literal("motor_velocity")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .then(Commands.argument("type", StringArgumentType.word())
                                                .then(Commands.argument("first", EntityArgument.entity())
                                                        .then(Commands.argument("second", EntityArgument.entity())
                                                                .then(Commands.argument("targetVelocity", DoubleArgumentType.doubleArg(-1_000_000.0, 1_000_000.0))
                                                                        .then(Commands.argument("factor", DoubleArgumentType.doubleArg(0.0, 1_000_000.0))
                                                                                .executes(context -> motorJoint(
                                                                                        context.getSource(),
                                                                                        runtime,
                                                                                        StringArgumentType.getString(context, "id"),
                                                                                        StringArgumentType.getString(context, "type"),
                                                                                        EntityArgument.getEntity(context, "first"),
                                                                                        EntityArgument.getEntity(context, "second"),
                                                                                        DoubleArgumentType.getDouble(context, "targetVelocity"),
                                                                                        DoubleArgumentType.getDouble(context, "factor"))))))))))
                        .then(Commands.literal("remove")
                                .then(Commands.argument("id", StringArgumentType.word())
                                        .executes(context -> removeJoint(
                                                context.getSource(),
                                                runtime,
                                                StringArgumentType.getString(context, "id")))))
                        .then(Commands.literal("list").executes(context -> listJoints(context.getSource(), runtime)))
                        .then(Commands.literal("rebuild").executes(context -> rebuildJoints(context.getSource(), runtime))))
                .then(Commands.literal("shapes")
                        .executes(context -> shapes(context.getSource()))
                        .then(Commands.literal("insert")
                                .then(Commands.argument("id", StringArgumentType.string())
                                        .then(Commands.argument("from", BlockPosArgument.blockPos())
                                                .then(Commands.argument("to", BlockPosArgument.blockPos())
                                                        .executes(context -> insertShape(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                BlockPosArgument.getLoadedBlockPos(context, "from"),
                                                                BlockPosArgument.getLoadedBlockPos(context, "to")))))))
                        .then(Commands.literal("queue")
                                .then(Commands.argument("id", StringArgumentType.string())
                                        .then(Commands.argument("from", BlockPosArgument.blockPos())
                                                .then(Commands.argument("to", BlockPosArgument.blockPos())
                                                        .executes(context -> queueShape(
                                                                context.getSource(),
                                                                runtime,
                                                                StringArgumentType.getString(context, "id"),
                                                                BlockPosArgument.getLoadedBlockPos(context, "from"),
                                                                BlockPosArgument.getLoadedBlockPos(context, "to"))))))))
                .then(Commands.literal("materials").executes(context -> materials(context.getSource())))
                .then(Commands.literal("query")
                        .then(Commands.literal("ray")
                                .then(Commands.argument("dx", DoubleArgumentType.doubleArg(-1.0, 1.0))
                                        .then(Commands.argument("dy", DoubleArgumentType.doubleArg(-1.0, 1.0))
                                                .then(Commands.argument("dz", DoubleArgumentType.doubleArg(-1.0, 1.0))
                                                        .then(Commands.argument("maxDistance", DoubleArgumentType.doubleArg(0.01, 1024.0))
                                                                .executes(context -> queryRay(
                                                                        context.getSource(),
                                                                        runtime,
                                                                        DoubleArgumentType.getDouble(context, "dx"),
                                                                        DoubleArgumentType.getDouble(context, "dy"),
                                                                        DoubleArgumentType.getDouble(context, "dz"),
                                                                        DoubleArgumentType.getDouble(context, "maxDistance"))))))))
                        .then(Commands.literal("aabb")
                                .then(Commands.argument("from", BlockPosArgument.blockPos())
                                        .then(Commands.argument("to", BlockPosArgument.blockPos())
                                                .executes(context -> queryAabb(
                                                        context.getSource(),
                                                        runtime,
                                                        BlockPosArgument.getLoadedBlockPos(context, "from"),
                                                        BlockPosArgument.getLoadedBlockPos(context, "to"))))))
                        .then(Commands.literal("sphere")
                                .then(Commands.argument("radius", DoubleArgumentType.doubleArg(0.01, 256.0))
                                        .executes(context -> querySphere(
                                                context.getSource(),
                                                runtime,
                                                DoubleArgumentType.getDouble(context, "radius"))))))
                .then(Commands.literal("inspect")
                        .then(Commands.argument("pos", BlockPosArgument.blockPos())
                                .executes(context -> inspect(context.getSource(), runtime, BlockPosArgument.getLoadedBlockPos(context, "pos")))))
                .then(Commands.literal("remove_near")
                        .then(Commands.argument("pos", BlockPosArgument.blockPos())
                                .executes(context -> removeNear(context.getSource(), runtime, BlockPosArgument.getLoadedBlockPos(context, "pos")))))
                .then(Commands.literal("rebuild_chunk")
                        .then(Commands.argument("x", IntegerArgumentType.integer())
                                .then(Commands.argument("z", IntegerArgumentType.integer())
                                        .executes(context -> rebuildChunk(
                                                context.getSource(),
                                                runtime,
                                                IntegerArgumentType.getInteger(context, "x"),
                                                IntegerArgumentType.getInteger(context, "z"))))))
                .then(Commands.literal("export").executes(context -> export(context.getSource(), runtime))));
    }

    private static int status(CommandSourceStack source, PhysicsRuntime runtime) {
        PhysicsRuntime.Status status = runtime.status();
        source.sendSuccess(() -> Component.literal(
                "mps_rigid_body loaded=" + status.loaded()
                        + " enabled=" + status.enabled()
                        + " ticks=" + status.ticks()
                        + " bodies=" + status.rigidBodies()
                        + " colliders=" + status.colliders()
                        + " joints=" + runtime.jointCount()
                        + " boundEntities=" + status.boundEntities()
                        + " queuedVoxelBuilds=" + status.queuedVoxelBuilds()), false);
        return 1;
    }

    private static int clientStatus(CommandSourceStack source) {
        PhysicsRuntime.Status status = MpsRigidBodyNetwork.lastClientStatus();
        source.sendSuccess(() -> Component.literal(
                "client status loaded=" + status.loaded()
                        + " enabled=" + status.enabled()
                        + " ticks=" + status.ticks()
                        + " bodies=" + status.rigidBodies()
                        + " colliders=" + status.colliders()), false);
        return 1;
    }

    private static int profile(CommandSourceStack source, PhysicsRuntime runtime) {
        PhysicsRuntime.Profile profile = runtime.profile();
        source.sendSuccess(() -> Component.literal(
                "profile lastStepMs=" + profile.lastStepMillis()
                        + " lastVoxelMs=" + profile.lastVoxelBuildMillis()
                        + " avgVoxelMs=" + profile.averageVoxelBuildMillis()
                        + " voxelBuilds=" + profile.totalVoxelBuilds()
                        + " chunkColliders=" + profile.chunkColliders()), false);
        return 1;
    }

    private static int reset(CommandSourceStack source, PhysicsRuntime runtime) {
        runtime.start(source.getServer());
        source.sendSuccess(() -> Component.literal("mps_rigid_body physics world reset"), true);
        return 1;
    }

    private static int setEnabled(CommandSourceStack source, PhysicsRuntime runtime, boolean enabled) {
        runtime.setEnabled(enabled);
        source.sendSuccess(() -> Component.literal("mps_rigid_body enabled=" + enabled), true);
        return 1;
    }

    private static int gravity(CommandSourceStack source, PhysicsRuntime runtime, double x, double y, double z) {
        runtime.setGravity(x, y, z);
        source.sendSuccess(() -> Component.literal("gravity set to " + x + ", " + y + ", " + z), true);
        return 1;
    }

    private static int step(CommandSourceStack source, PhysicsRuntime runtime, int ticks) {
        for (int i = 0; i < ticks; i++) {
            runtime.tick();
        }
        source.sendSuccess(() -> Component.literal("stepped physics " + ticks + " ticks"), true);
        return ticks;
    }

    private static int voxelizeArea(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            net.minecraft.core.BlockPos from,
            net.minecraft.core.BlockPos to,
            double voxelSize) {
        ServerLevel level = source.getLevel();
        PhysicsRuntime.VoxelInsertResult result = runtime.voxelizeArea(level, from, to, voxelSize);
        source.sendSuccess(() -> Component.literal(
                "voxel collider inserted handle=" + result.colliderHandle()
                        + " solids=" + result.solidBlocks()
                        + " scanned=" + result.scannedBlocks()
                        + " colliders=" + runtime.status().colliders()), true);
        return 1;
    }

    private static int voxelizeAreaAsync(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            net.minecraft.core.BlockPos from,
            net.minecraft.core.BlockPos to,
            double voxelSize) {
        String id = runtime.queueVoxelizeArea(source.getLevel(), from, to, voxelSize);
        source.sendSuccess(() -> Component.literal("queued async voxel build " + id), true);
        return 1;
    }

    private static int saveArea(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            String id,
            net.minecraft.core.BlockPos from,
            net.minecraft.core.BlockPos to,
            double voxelSize) {
        var area = runtime.saveArea(source.getLevel(), id, from, to, voxelSize);
        source.sendSuccess(() -> Component.literal("saved voxel area " + area.id()), true);
        return 1;
    }

    private static int listAreas(CommandSourceStack source, PhysicsRuntime runtime) {
        var areas = runtime.savedAreas();
        source.sendSuccess(() -> Component.literal("saved voxel areas: " + areas.size()), false);
        for (var area : areas) {
            source.sendSuccess(() -> Component.literal(
                    area.id() + " " + area.dimension().location()
                            + " from=" + area.from().toShortString()
                            + " to=" + area.to().toShortString()
                            + " voxelSize=" + area.voxelSize()), false);
        }
        return areas.size();
    }

    private static int removeArea(CommandSourceStack source, PhysicsRuntime runtime, String id) {
        boolean removed = runtime.removeSavedArea(id);
        source.sendSuccess(() -> Component.literal("removed=" + removed + " id=" + id), true);
        return removed ? 1 : 0;
    }

    private static int clearAreas(CommandSourceStack source, PhysicsRuntime runtime) {
        int count = runtime.clearSavedAreas();
        source.sendSuccess(() -> Component.literal("cleared saved voxel areas: " + count), true);
        return count;
    }

    private static int rebuildAreas(CommandSourceStack source, PhysicsRuntime runtime) {
        int count = runtime.rebuildPersistentAreas();
        source.sendSuccess(() -> Component.literal("queued persistent voxel rebuilds: " + count), true);
        return count;
    }

    private static int bindEntity(CommandSourceStack source, PhysicsRuntime runtime, Entity entity, double halfExtent) {
        long handle = runtime.bindEntity(entity, halfExtent);
        source.sendSuccess(() -> Component.literal("bound entity " + entity.getStringUUID() + " body=" + handle), true);
        return 1;
    }

    private static int bindEntityPersistent(CommandSourceStack source, PhysicsRuntime runtime, Entity entity, double halfExtent) {
        long handle = runtime.bindEntity(entity, halfExtent, 1.0, 0.6, 0.0, true);
        source.sendSuccess(() -> Component.literal("saved bound entity " + entity.getStringUUID() + " body=" + handle), true);
        return 1;
    }

    private static int clearBindings(CommandSourceStack source, PhysicsRuntime runtime) {
        runtime.clearBindings();
        source.sendSuccess(() -> Component.literal("cleared entity bindings"), true);
        return 1;
    }

    private static int listBindings(CommandSourceStack source, PhysicsRuntime runtime) {
        var bindings = runtime.savedBindings();
        source.sendSuccess(() -> Component.literal("saved entity bindings: " + bindings.size()), false);
        for (var binding : bindings) {
            source.sendSuccess(() -> Component.literal(
                    binding.entity() + " " + binding.dimension().location()
                            + " halfExtent=" + binding.halfExtent()
                            + " density=" + binding.density()
                            + " friction=" + binding.friction()
                            + " restitution=" + binding.restitution()), false);
        }
        return bindings.size();
    }

    private static int rebuildBindings(CommandSourceStack source, PhysicsRuntime runtime) {
        int count = runtime.rebuildPersistentBindings();
        source.sendSuccess(() -> Component.literal("rebuilt persistent entity bindings: " + count), true);
        return count;
    }

    private static int shapes(CommandSourceStack source) {
        var shapes = MpsRigidBodyShapeLoader.shapes();
        source.sendSuccess(() -> Component.literal("loaded shape configs: " + shapes.size()), false);
        shapes.forEach((id, shape) -> source.sendSuccess(() -> Component.literal(
                id + " type=" + shape.type()
                        + " voxelSize=" + shape.voxelSize()
                        + " friction=" + shape.friction()
                        + " restitution=" + shape.restitution()), false));
        return shapes.size();
    }

    private static int materials(CommandSourceStack source) {
        var materials = MpsRigidBodyShapeLoader.materials();
        source.sendSuccess(() -> Component.literal("loaded materials: " + materials.size()), false);
        materials.forEach((id, material) -> source.sendSuccess(() -> Component.literal(
                id + " friction=" + material.friction()
                        + " restitution=" + material.restitution()
                        + " density=" + material.density()
                        + " damageScale=" + material.damageScale()
                        + " breakThreshold=" + material.breakThreshold()
                        + " blocks=" + material.blocks().size()
                        + " tags=" + material.tags().size()), false));
        return materials.size();
    }

    private static int createJoint(CommandSourceStack source, PhysicsRuntime runtime, String id, String type, Entity first, Entity second) {
        long handle = runtime.createJoint(id, type, first, second);
        source.sendSuccess(() -> Component.literal("created " + type + " joint " + id + " handle=" + handle), true);
        return 1;
    }

    private static int saveJoint(CommandSourceStack source, PhysicsRuntime runtime, String id, String type, Entity first, Entity second) {
        long handle = runtime.saveJoint(id, type, first, second, 0.0, 1.0, 0.0, 0.0, 0.0);
        source.sendSuccess(() -> Component.literal("saved " + type + " joint " + id + " handle=" + handle), true);
        return 1;
    }

    private static int createJointAdvanced(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            String id,
            String type,
            Entity first,
            Entity second,
            double axisX,
            double axisY,
            double axisZ,
            double valueB,
            double valueC) {
        long handle = runtime.createJoint(id, type, first, second, axisX, axisY, axisZ, valueB, valueC, Double.NaN, Double.NaN, false);
        source.sendSuccess(() -> Component.literal("created " + type + " joint " + id + " handle=" + handle), true);
        return 1;
    }

    private static int motorJoint(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            String id,
            String type,
            Entity first,
            Entity second,
            double targetVelocity,
            double factor) {
        long handle = runtime.createMotorJoint(id, type, first, second, targetVelocity, factor);
        source.sendSuccess(() -> Component.literal("created motor " + type + " joint " + id + " handle=" + handle), true);
        return 1;
    }

    private static int removeJoint(CommandSourceStack source, PhysicsRuntime runtime, String id) {
        boolean removed = runtime.removeJoint(id);
        source.sendSuccess(() -> Component.literal("removed joint " + id + "=" + removed), true);
        return removed ? 1 : 0;
    }

    private static int listJoints(CommandSourceStack source, PhysicsRuntime runtime) {
        var joints = runtime.savedJoints();
        source.sendSuccess(() -> Component.literal("saved joints: " + joints.size() + " active=" + runtime.jointCount()), false);
        for (var joint : joints) {
            source.sendSuccess(() -> Component.literal(
                    joint.id() + " type=" + joint.type()
                            + " first=" + joint.first()
                            + " second=" + joint.second()), false);
        }
        return joints.size();
    }

    private static int rebuildJoints(CommandSourceStack source, PhysicsRuntime runtime) {
        int count = runtime.rebuildPersistentJoints();
        source.sendSuccess(() -> Component.literal("rebuilt persistent joints: " + count), true);
        return count;
    }

    private static int insertShape(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            String shapeId,
            net.minecraft.core.BlockPos from,
            net.minecraft.core.BlockPos to) {
        int solids = runtime.insertShape(source.getLevel(), shapeId, from, to);
        source.sendSuccess(() -> Component.literal("inserted shape " + shapeId + " solids=" + solids), true);
        return 1;
    }

    private static int queueShape(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            String shapeId,
            net.minecraft.core.BlockPos from,
            net.minecraft.core.BlockPos to) {
        runtime.queueShape(source.getLevel(), shapeId, from, to);
        source.sendSuccess(() -> Component.literal("queued shape " + shapeId), true);
        return 1;
    }

    private static int queryRay(CommandSourceStack source, PhysicsRuntime runtime, double dx, double dy, double dz, double maxDistance) {
        var pos = source.getPosition();
        PhysicsRuntime.QueryResult result = runtime.raycast(pos.x, pos.y, pos.z, dx, dy, dz, maxDistance);
        if (result.isEmpty()) {
            source.sendSuccess(() -> Component.literal("ray hit none"), false);
            return 0;
        }
        double[] normal = result.normal();
        source.sendSuccess(() -> Component.literal(
                "ray hit collider=" + result.collider()
                        + " toi=" + result.timeOfImpact()
                        + " normal=" + normal[0] + "," + normal[1] + "," + normal[2]), false);
        return 1;
    }

    private static int queryAabb(
            CommandSourceStack source,
            PhysicsRuntime runtime,
            net.minecraft.core.BlockPos from,
            net.minecraft.core.BlockPos to) {
        int count = runtime.countAabb(from, to);
        source.sendSuccess(() -> Component.literal("aabb hits=" + count), false);
        return count;
    }

    private static int querySphere(CommandSourceStack source, PhysicsRuntime runtime, double radius) {
        var pos = source.getPosition();
        int count = runtime.countSphere(pos.x, pos.y, pos.z, radius);
        source.sendSuccess(() -> Component.literal("sphere hits=" + count), false);
        return count;
    }

    private static int inspect(CommandSourceStack source, PhysicsRuntime runtime, net.minecraft.core.BlockPos pos) {
        source.sendSuccess(() -> Component.literal("profile " + runtime.profile()), false);
        source.sendSuccess(() -> Component.literal("client status " + MpsRigidBodyNetwork.lastClientStatus()), false);
        source.sendSuccess(() -> Component.literal("block " + pos.toShortString()), false);
        return 1;
    }

    private static int rebuildChunk(CommandSourceStack source, PhysicsRuntime runtime, int x, int z) {
        runtime.refreshChunkCollider(source.getLevel(), new net.minecraft.world.level.ChunkPos(x, z));
        source.sendSuccess(() -> Component.literal("queued chunk rebuild " + x + "," + z), true);
        return 1;
    }

    private static int removeNear(CommandSourceStack source, PhysicsRuntime runtime, net.minecraft.core.BlockPos pos) {
        String removed = runtime.removeNearestSavedArea(source.getLevel(), pos);
        source.sendSuccess(() -> Component.literal(removed.isEmpty() ? "no saved area found" : "removed saved area " + removed), true);
        return removed.isEmpty() ? 0 : 1;
    }

    private static int export(CommandSourceStack source, PhysicsRuntime runtime) {
        var areas = runtime.savedAreas();
        source.sendSuccess(() -> Component.literal("exported areas=" + areas.size()), false);
        for (var area : areas) {
            source.sendSuccess(() -> Component.literal(area.id() + "@" + area.dimension().location()), false);
        }
        return areas.size();
    }
}
