package org.polaris2023.msp_rigid_body;

import net.minecraft.core.BlockPos;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.sounds.SoundEvents;
import net.minecraft.tags.TagKey;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.ChunkPos;
import net.minecraft.world.level.block.state.BlockState;
import org.polaris2023.msp_rigid_body.util.Collider;
import org.polaris2023.msp_rigid_body.util.Joint;
import org.polaris2023.msp_rigid_body.util.PhysicsWorld;
import org.polaris2023.msp_rigid_body.util.Query;
import org.polaris2023.msp_rigid_body.util.RigidBody;
import org.polaris2023.msp_rigid_body.util.VoxelGrid;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;

public final class PhysicsRuntime implements AutoCloseable {
    private static final Logger LOGGER = LoggerFactory.getLogger(PhysicsRuntime.class);
    private static final int VOXEL_MODE_AUTO = 0;
    private static final ThreadFactory VOXEL_THREAD_FACTORY = runnable -> {
        Thread thread = new Thread(runnable, "mps-rigid-body-voxel-builder");
        thread.setDaemon(true);
        return thread;
    };

    private final ExecutorService voxelExecutor = Executors.newSingleThreadExecutor(VOXEL_THREAD_FACTORY);
    private final Queue<PendingVoxelBuild> pendingVoxelBuilds = new ArrayDeque<>();
    private final Map<UUID, BoundEntity> boundEntities = new HashMap<>();
    private final Map<ChunkKey, Collider> chunkColliders = new HashMap<>();
    private final Map<ChunkKey, Area> chunkAreas = new HashMap<>();
    private final Map<String, Joint> joints = new HashMap<>();
    private final Map<Long, MpsRigidBodyShapeLoader.MaterialConfig> colliderMaterials = new HashMap<>();
    private final AtomicInteger generatedAreaIds = new AtomicInteger();

    private PhysicsWorld world;
    private MinecraftServer server;
    private long ticks;
    private boolean enabled = true;
    private long lastStepNanos;
    private long lastVoxelBuildNanos;
    private long totalVoxelBuilds;
    private long totalVoxelBuildNanos;

    public void start(MinecraftServer server) {
        this.server = server;
        start();
        rebuildPersistentAreas();
        rebuildPersistentBindings();
        rebuildPersistentJoints();
    }

    public void start() {
        closeWorld();
        enabled = MpsRigidBodyConfig.ENABLED.get();
        world = new PhysicsWorld(
                MpsRigidBodyConfig.GRAVITY_X.get(),
                MpsRigidBodyConfig.GRAVITY_Y.get(),
                MpsRigidBodyConfig.GRAVITY_Z.get())
                .integrationParameters(
                        MpsRigidBodyConfig.STEP_SECONDS.get(),
                        MpsRigidBodyConfig.SOLVER_ITERATIONS.get(),
                        MpsRigidBodyConfig.CCD_SUBSTEPS.get());
        ticks = 0L;
        pendingVoxelBuilds.clear();
        boundEntities.clear();
        chunkColliders.clear();
        chunkAreas.clear();
        joints.clear();
        colliderMaterials.clear();
        lastStepNanos = 0L;
        lastVoxelBuildNanos = 0L;
        totalVoxelBuilds = 0L;
        totalVoxelBuildNanos = 0L;
        LOGGER.info("Created mps_rigid_body physics world");
    }

    public void tick() {
        drainVoxelBuilds();
        if (!enabled || world == null || world.isEmpty()) {
            return;
        }
        syncEntitiesToPhysics();
        if (!world.isEmpty()) {
            long started = System.nanoTime();
            world.step();
            lastStepNanos = System.nanoTime() - started;
            ticks++;
            syncPhysicsToEntities();
            emitContactEvents();
            if (ticks % 20L == 0L && server != null) {
                MpsRigidBodyNetwork.syncStatusToAll(status());
            }
        }
    }

    public Status status() {
        if (world == null || world.isEmpty()) {
            return new Status(false, enabled, ticks, 0, 0, boundEntities.size(), pendingVoxelBuilds.size());
        }
        return new Status(
                true,
                enabled,
                ticks,
                world.rigidBodyCount(),
                world.colliderCount(),
                boundEntities.size(),
                pendingVoxelBuilds.size());
    }

    public int jointCount() {
        return joints.size();
    }

    public Profile profile() {
        double averageVoxelMillis = totalVoxelBuilds == 0L ? 0.0 : nanosToMillis(totalVoxelBuildNanos / totalVoxelBuilds);
        return new Profile(
                nanosToMillis(lastStepNanos),
                nanosToMillis(lastVoxelBuildNanos),
                averageVoxelMillis,
                totalVoxelBuilds,
                chunkColliders.size());
    }

    public void syncDebugStateToClient() {
        if (server == null) {
            return;
        }
        java.util.List<MpsRigidBodyNetwork.DebugAabb> boxes = new java.util.ArrayList<>();
        for (MpsRigidBodySavedData.VoxelArea area : savedAreas()) {
            boxes.add(new MpsRigidBodyNetwork.DebugAabb(
                    area.from().getX(), area.from().getY(), area.from().getZ(),
                    area.to().getX() + 1.0, area.to().getY() + 1.0, area.to().getZ() + 1.0,
                    0x55AAFF));
        }
        for (Area area : chunkAreas.values()) {
            boxes.add(new MpsRigidBodyNetwork.DebugAabb(
                    area.from().getX(), area.from().getY(), area.from().getZ(),
                    area.to().getX() + 1.0, area.to().getY() + 1.0, area.to().getZ() + 1.0,
                    0x55FF55));
        }
        MpsRigidBodyNetwork.syncDebugAabbsToAll(boxes);
    }

    public void syncContactToClient(long collider1, long collider2, double force) {
        MpsRigidBodyNetwork.syncContactToAll(new MpsRigidBodyNetwork.ContactInfo(collider1, collider2, force));
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setGravity(double x, double y, double z) {
        requireWorld();
        world.set(x, y, z);
    }

    public VoxelInsertResult voxelizeArea(ServerLevel level, BlockPos first, BlockPos second, double voxelSize) {
        requireWorld();
        Area area = checkedArea(first, second, voxelSize);
        enforceColliderLimit();

        try (VoxelGrid grid = new VoxelGrid(area.sizeX(), area.sizeY(), area.sizeZ())) {
            long started = System.nanoTime();
            MaterialAccumulator material = new MaterialAccumulator();
            int solid = fillGrid(level, area, grid, material);
            recordVoxelBuild(System.nanoTime() - started);
            Collider collider = insertVoxelCollider(grid.toByteArray(), area, solid, material.material());
            debugArea(level, area);
            return new VoxelInsertResult(collider.handle(), solid, area.volume());
        }
    }

    public String queueVoxelizeArea(ServerLevel level, BlockPos first, BlockPos second, double voxelSize) {
        Area area = checkedArea(first, second, voxelSize);
        String id = nextAreaId();
        CompletableFuture<VoxelBuildResult> future = CompletableFuture.supplyAsync(() -> buildVoxelBytes(level, area), voxelExecutor);
        pendingVoxelBuilds.add(new PendingVoxelBuild(id, level.dimension(), area, future, null));
        return id;
    }

    public MpsRigidBodySavedData.VoxelArea saveArea(ServerLevel level, String id, BlockPos first, BlockPos second, double voxelSize) {
        Area area = checkedArea(first, second, voxelSize);
        MpsRigidBodySavedData.VoxelArea saved = new MpsRigidBodySavedData.VoxelArea(
                id == null || id.isBlank() ? nextAreaId() : id,
                level.dimension(),
                area.from(),
                area.to(),
                area.voxelSize(),
                false);
        MpsRigidBodySavedData.get(level.getServer()).add(saved);
        return saved;
    }

    public int rebuildPersistentAreas() {
        if (server == null) {
            return 0;
        }
        int queued = 0;
        for (MpsRigidBodySavedData.VoxelArea area : MpsRigidBodySavedData.get(server).areas()) {
            ServerLevel level = server.getLevel(area.dimension());
            if (level != null) {
                queueVoxelizeArea(level, area.from(), area.to(), area.voxelSize());
                queued++;
            }
        }
        return queued;
    }

    public boolean removeSavedArea(String id) {
        if (server == null) {
            return false;
        }
        return MpsRigidBodySavedData.get(server).remove(id);
    }

    public String removeNearestSavedArea(ServerLevel level, BlockPos pos) {
        if (server == null) {
            return "";
        }
        String nearest = "";
        double best = Double.MAX_VALUE;
        for (MpsRigidBodySavedData.VoxelArea area : MpsRigidBodySavedData.get(server).areas()) {
            if (!area.dimension().equals(level.dimension())) {
                continue;
            }
            double centerX = (area.from().getX() + area.to().getX()) * 0.5;
            double centerY = (area.from().getY() + area.to().getY()) * 0.5;
            double centerZ = (area.from().getZ() + area.to().getZ()) * 0.5;
            double dx = centerX - pos.getX();
            double dy = centerY - pos.getY();
            double dz = centerZ - pos.getZ();
            double distance = dx * dx + dy * dy + dz * dz;
            if (distance < best) {
                best = distance;
                nearest = area.id();
            }
        }
        if (!nearest.isEmpty() && removeSavedArea(nearest)) {
            return nearest;
        }
        return "";
    }

    public int clearSavedAreas() {
        if (server == null) {
            return 0;
        }
        MpsRigidBodySavedData data = MpsRigidBodySavedData.get(server);
        int count = data.areas().size() + data.bindings().size() + data.joints().size();
        data.clear();
        return count;
    }

    public java.util.List<MpsRigidBodySavedData.VoxelArea> savedAreas() {
        if (server == null) {
            return java.util.List.of();
        }
        return MpsRigidBodySavedData.get(server).areas();
    }

    public java.util.List<MpsRigidBodySavedData.EntityBinding> savedBindings() {
        if (server == null) {
            return java.util.List.of();
        }
        return MpsRigidBodySavedData.get(server).bindings();
    }

    public java.util.List<MpsRigidBodySavedData.JointBinding> savedJoints() {
        if (server == null) {
            return java.util.List.of();
        }
        return MpsRigidBodySavedData.get(server).joints();
    }

    public void queueChunkCollider(ServerLevel level, ChunkPos chunkPos) {
        if (!MpsRigidBodyConfig.AUTO_CHUNK_COLLIDERS.get()) {
            return;
        }
        ChunkKey key = new ChunkKey(level.dimension(), chunkPos.x, chunkPos.z);
        if (chunkColliders.containsKey(key)) {
            return;
        }
        int minY = level.getMinBuildHeight();
        int maxY = Math.min(level.getMaxBuildHeight() - 1, minY + 31);
        Area area = checkedArea(
                new BlockPos(chunkPos.getMinBlockX(), minY, chunkPos.getMinBlockZ()),
                new BlockPos(chunkPos.getMaxBlockX(), maxY, chunkPos.getMaxBlockZ()),
                MpsRigidBodyConfig.DEFAULT_VOXEL_SIZE.get());
        String id = "chunk_" + chunkPos.x + "_" + chunkPos.z;
        CompletableFuture<VoxelBuildResult> future = CompletableFuture.supplyAsync(() -> buildVoxelBytes(level, area), voxelExecutor);
        pendingVoxelBuilds.add(new PendingVoxelBuild(id, level.dimension(), area, future, key));
        chunkAreas.put(key, area);
    }

    public boolean unloadChunkCollider(ServerLevel level, ChunkPos chunkPos) {
        ChunkKey key = new ChunkKey(level.dimension(), chunkPos.x, chunkPos.z);
        chunkAreas.remove(key);
        Collider collider = chunkColliders.remove(key);
        if (collider != null) {
            colliderMaterials.remove(collider.handle());
        }
        return collider != null && !collider.isEmpty() && collider.remove(false);
    }

    public void refreshChunkCollider(ServerLevel level, ChunkPos chunkPos) {
        if (!MpsRigidBodyConfig.AUTO_CHUNK_COLLIDERS.get()) {
            return;
        }
        unloadChunkCollider(level, chunkPos);
        queueChunkCollider(level, chunkPos);
    }

    public int insertShape(ServerLevel level, String shapeId, BlockPos from, BlockPos to) {
        ResourceLocation id = ResourceLocation.tryParse(shapeId);
        if (id == null) {
            throw new IllegalArgumentException("invalid shape id: " + shapeId);
        }
        MpsRigidBodyShapeLoader.ShapeConfig shape = MpsRigidBodyShapeLoader.shapes().get(id);
        if (shape == null) {
            throw new IllegalArgumentException("unknown shape id: " + shapeId);
        }
        return insertShapeConfig(level, shape, from, to);
    }

    public void queueShape(ServerLevel level, String shapeId, BlockPos from, BlockPos to) {
        ResourceLocation id = ResourceLocation.tryParse(shapeId);
        if (id == null) {
            throw new IllegalArgumentException("invalid shape id: " + shapeId);
        }
        MpsRigidBodyShapeLoader.ShapeConfig shape = MpsRigidBodyShapeLoader.shapes().get(id);
        if (shape == null) {
            throw new IllegalArgumentException("unknown shape id: " + shapeId);
        }
        if ("voxel_aabb".equals(shape.type())) {
            queueVoxelizeArea(level, from, to, shape.voxelSize());
            return;
        }
        insertShapeConfig(level, shape, from, to);
    }

    public VoxelInsertResult insertVoxelAabb(ServerLevel level, BlockPos first, BlockPos second, double voxelSize) {
        requireWorld();
        Area area = checkedArea(first, second, voxelSize);
        enforceColliderLimit();
        Collider collider = world.voxelAabbCollider(
                        area.from().getX(), area.from().getY(), area.from().getZ(),
                        area.to().getX() + 1.0, area.to().getY() + 1.0, area.to().getZ() + 1.0,
                        voxelSize,
                        VOXEL_MODE_AUTO,
                        false,
                        128,
                        20_000)
                .friction(0.8)
                .restitution(0.0)
                .insert();
        colliderMaterials.put(collider.handle(), MaterialAccumulator.defaultMaterial());
        debugArea(level, area);
        return new VoxelInsertResult(collider.handle(), Math.toIntExact(area.volume()), area.volume());
    }

    public long bindEntity(Entity entity, double halfExtent) {
        return bindEntity(entity, halfExtent, 1.0, 0.6, 0.0, false);
    }

    public long bindEntity(Entity entity, double halfExtent, double density, double friction, double restitution, boolean persistent) {
        requireWorld();
        if (entity == null) {
            throw new IllegalArgumentException("entity is required");
        }
        if (!Double.isFinite(halfExtent) || halfExtent <= 0.0) {
            throw new IllegalArgumentException("halfExtent must be positive and finite");
        }
        RigidBody body = world.body(0)
                .translation(entity.getX(), entity.getY(), entity.getZ())
                .damping(0.2, 0.2)
                .body(world);
        Collider collider = world.cuboidCollider(halfExtent, halfExtent, halfExtent)
                .density(density)
                .friction(friction)
                .restitution(restitution)
                .insert(body);
        colliderMaterials.put(collider.handle(), MaterialAccumulator.defaultMaterial());
        boundEntities.put(entity.getUUID(), new BoundEntity(body));
        if (persistent && server != null && entity.level() instanceof ServerLevel level) {
            MpsRigidBodySavedData.get(server).addBinding(new MpsRigidBodySavedData.EntityBinding(
                    entity.getUUID(), level.dimension(), halfExtent, density, friction, restitution));
        }
        return body.handle();
    }

    public void clearBindings() {
        boundEntities.clear();
    }

    public boolean removeSavedBinding(String uuid) {
        if (server == null) {
            return false;
        }
        return MpsRigidBodySavedData.get(server).removeBinding(uuid);
    }

    public int clearSavedBindings() {
        if (server == null) {
            return 0;
        }
        MpsRigidBodySavedData data = MpsRigidBodySavedData.get(server);
        int count = data.bindings().size();
        data.clearBindings();
        return count;
    }

    public int clearSavedJoints() {
        if (server == null) {
            return 0;
        }
        MpsRigidBodySavedData data = MpsRigidBodySavedData.get(server);
        int count = data.joints().size();
        data.clearJoints();
        return count;
    }

    public int rebuildPersistentBindings() {
        if (server == null) {
            return 0;
        }
        int count = 0;
        for (MpsRigidBodySavedData.EntityBinding binding : MpsRigidBodySavedData.get(server).bindings()) {
            ServerLevel level = server.getLevel(binding.dimension());
            if (level == null) {
                continue;
            }
            Entity entity = level.getEntity(binding.entity());
            if (entity == null) {
                continue;
            }
            bindEntity(entity, binding.halfExtent(), binding.density(), binding.friction(), binding.restitution(), false);
            count++;
        }
        return count;
    }

    public int rebuildPersistentJoints() {
        if (server == null) {
            return 0;
        }
        int count = 0;
        for (MpsRigidBodySavedData.JointBinding joint : MpsRigidBodySavedData.get(server).joints()) {
            Entity first = findEntity(joint.first());
            Entity second = findEntity(joint.second());
            if (first == null || second == null) {
                continue;
            }
            createJoint(joint.id(), joint.type(), first, second, joint.axisX(), joint.axisY(), joint.axisZ(), joint.valueB(), joint.valueC(), Double.NaN, Double.NaN, false);
            count++;
        }
        return count;
    }

    @Override
    public void close() {
        closeWorld();
        server = null;
    }

    private void closeWorld() {
        if (world != null) {
            world.close();
            world = null;
        }
        pendingVoxelBuilds.clear();
        boundEntities.clear();
        chunkColliders.clear();
        chunkAreas.clear();
        joints.clear();
        colliderMaterials.clear();
    }

    private void requireWorld() {
        if (world == null || world.isEmpty()) {
            start();
        }
    }

    private void enforceColliderLimit() {
        if (world != null && !world.isEmpty() && world.colliderCount() >= MpsRigidBodyConfig.MAX_COLLIDERS.get()) {
            throw new IllegalStateException("collider limit reached: " + MpsRigidBodyConfig.MAX_COLLIDERS.get());
        }
    }

    private Area checkedArea(BlockPos first, BlockPos second, double voxelSize) {
        if (!Double.isFinite(voxelSize) || voxelSize <= 0.0) {
            throw new IllegalArgumentException("voxelSize must be positive and finite");
        }
        int minX = Math.min(first.getX(), second.getX());
        int minY = Math.min(first.getY(), second.getY());
        int minZ = Math.min(first.getZ(), second.getZ());
        int maxX = Math.max(first.getX(), second.getX());
        int maxY = Math.max(first.getY(), second.getY());
        int maxZ = Math.max(first.getZ(), second.getZ());
        int sizeX = maxX - minX + 1;
        int sizeY = maxY - minY + 1;
        int sizeZ = maxZ - minZ + 1;
        long volume = (long) sizeX * sizeY * sizeZ;
        if (volume > MpsRigidBodyConfig.MAX_VOXEL_AREA_BLOCKS.get()) {
            throw new IllegalArgumentException("voxel area is too large: " + volume);
        }
        return new Area(new BlockPos(minX, minY, minZ), new BlockPos(maxX, maxY, maxZ), sizeX, sizeY, sizeZ, voxelSize, volume);
    }

    private VoxelBuildResult buildVoxelBytes(ServerLevel level, Area area) {
        byte[] voxels = new byte[Math.toIntExact(area.volume())];
        int solid = 0;
        MaterialAccumulator material = new MaterialAccumulator();
        for (int z = 0; z < area.sizeZ(); z++) {
            for (int y = 0; y < area.sizeY(); y++) {
                for (int x = 0; x < area.sizeX(); x++) {
                    BlockPos pos = area.from().offset(x, y, z);
                    BlockState state = level.getBlockState(pos);
                    if (isPhysicsSolid(level, pos, state)) {
                        voxels[(z * area.sizeY() + y) * area.sizeX() + x] = 1;
                        material.accept(state);
                        solid++;
                    }
                }
            }
        }
        return new VoxelBuildResult(voxels, solid, material.material());
    }

    private int fillGrid(ServerLevel level, Area area, VoxelGrid grid, MaterialAccumulator material) {
        int solid = 0;
        for (int z = 0; z < area.sizeZ(); z++) {
            for (int y = 0; y < area.sizeY(); y++) {
                for (int x = 0; x < area.sizeX(); x++) {
                    BlockPos pos = area.from().offset(x, y, z);
                    BlockState state = level.getBlockState(pos);
                    if (isPhysicsSolid(level, pos, state)) {
                        grid.set(x, y, z, true);
                        material.accept(state);
                        solid++;
                    }
                }
            }
        }
        return solid;
    }

    private static boolean isPhysicsSolid(ServerLevel level, BlockPos pos, BlockState state) {
        return !state.isAir() && !state.getCollisionShape(level, pos).isEmpty();
    }

    private int insertShapeConfig(ServerLevel level, MpsRigidBodyShapeLoader.ShapeConfig shape, BlockPos from, BlockPos to) {
        requireWorld();
        Area area = checkedArea(from, to, Math.max(0.01, shape.voxelSize()));
        enforceColliderLimit();
        double cx = (area.from().getX() + area.to().getX() + 1.0) * 0.5;
        double cy = (area.from().getY() + area.to().getY() + 1.0) * 0.5;
        double cz = (area.from().getZ() + area.to().getZ() + 1.0) * 0.5;
        double hx = Math.max(0.01, area.sizeX() * 0.5);
        double hy = Math.max(0.01, area.sizeY() * 0.5);
        double hz = Math.max(0.01, area.sizeZ() * 0.5);
        if ("voxel_aabb".equals(shape.type())) {
            VoxelInsertResult result = voxelizeArea(level, from, to, shape.voxelSize());
            return result.solidBlocks();
        }
        Collider collider = switch (shape.type()) {
            case "box", "cuboid" -> world.cuboidCollider(hx, hy, hz)
                    .friction(shape.friction())
                    .restitution(shape.restitution())
                    .density(shape.density())
                    .translation(cx, cy, cz)
                    .insert();
            case "sphere" -> world.sphereCollider(cx, cy, cz, Math.max(0.01, shape.radius()))
                    .friction(shape.friction())
                    .restitution(shape.restitution())
                    .density(shape.density())
                    .insert();
            case "capsule" -> world.capsuleCollider(cx, cy - Math.max(0.01, shape.halfHeight()), cz, cx, cy + Math.max(0.01, shape.halfHeight()), cz, Math.max(0.01, shape.radius()))
                    .friction(shape.friction())
                    .restitution(shape.restitution())
                    .density(shape.density())
                    .insert();
            case "cylinder" -> world.cylinderCollider(cx, cy, cz, Math.max(0.01, shape.radius()), Math.max(0.01, shape.halfHeight()))
                    .friction(shape.friction())
                    .restitution(shape.restitution())
                    .density(shape.density())
                    .insert();
            default -> throw new IllegalArgumentException("unsupported shape type: " + shape.type());
        };
        colliderMaterials.put(collider.handle(), new MpsRigidBodyShapeLoader.MaterialConfig(
                shape.friction(), shape.restitution(), shape.density(), 0.0,
                Double.POSITIVE_INFINITY, 0.2, 0.2, 1.0,
                "minecraft:block.anvil.land", "minecraft:crit",
                java.util.Set.of(), java.util.Set.of()));
        debugArea(level, area);
        return 1;
    }

    private Collider insertVoxelCollider(byte[] voxels, Area area, int solid, MpsRigidBodyShapeLoader.MaterialConfig material) {
        requireWorld();
        enforceColliderLimit();
        if (solid == 0) {
            throw new IllegalArgumentException("voxel area contains no solid blocks");
        }
        Collider collider = world.voxelCollider(
                        voxels,
                        area.sizeX(), area.sizeY(), area.sizeZ(),
                        area.voxelSize(),
                        area.from().getX(), area.from().getY(), area.from().getZ(),
                        VOXEL_MODE_AUTO,
                false,
                128,
                20_000)
                .friction(material.friction())
                .restitution(material.restitution())
                .insert();
        colliderMaterials.put(collider.handle(), material);
        return collider;
    }

    private void drainVoxelBuilds() {
        if (pendingVoxelBuilds.isEmpty()) {
            return;
        }
        int count = pendingVoxelBuilds.size();
        for (int i = 0; i < count; i++) {
            PendingVoxelBuild build = pendingVoxelBuilds.peek();
            if (build == null || !build.future().isDone()) {
                break;
            }
            pendingVoxelBuilds.remove();
            try {
                ServerLevel level = server == null ? null : server.getLevel(build.dimension());
                if (level == null) {
                    continue;
                }
                VoxelBuildResult result = build.future().join();
                Collider collider = insertVoxelCollider(result.voxels(), build.area(), result.solid(), result.material());
                if (build.chunkKey() != null) {
                    chunkColliders.put(build.chunkKey(), collider);
                }
                debugArea(level, build.area());
                LOGGER.info("Inserted async voxel area {} as collider {} solids={}", build.id(), collider.handle(), result.solid());
            } catch (RuntimeException exception) {
                LOGGER.warn("Failed to insert async voxel area {}", build.id(), exception);
            }
        }
    }

    private void syncEntitiesToPhysics() {
        if (server == null || boundEntities.isEmpty()) {
            return;
        }
        java.util.List<PhysicsWorld.BodyPoseUpdate> updates = new java.util.ArrayList<>();
        boundEntities.forEach((uuid, binding) -> {
            Entity entity = findEntity(uuid);
            if (entity != null) {
                updates.add(new PhysicsWorld.BodyPoseUpdate(
                        binding.bodyHandle(),
                        new double[] {entity.getX(), entity.getY(), entity.getZ()},
                        new double[] {0.0, 0.0, 0.0, 1.0}));
            }
        });
        if (!updates.isEmpty()) {
            world.updateBodyPoses(updates.toArray(PhysicsWorld.BodyPoseUpdate[]::new), true);
        }
    }

    private void syncPhysicsToEntities() {
        if (server == null || boundEntities.isEmpty()) {
            return;
        }
        Map<Long, PhysicsWorld.BodySnapshot> snapshots = new HashMap<>();
        for (PhysicsWorld.BodySnapshot snapshot : world.bodySnapshot()) {
            snapshots.put(snapshot.handle(), snapshot);
        }
        boundEntities.forEach((uuid, binding) -> {
            Entity entity = findEntity(uuid);
            PhysicsWorld.BodySnapshot snapshot = snapshots.get(binding.bodyHandle());
            if (entity != null && snapshot != null) {
                double[] position = snapshot.translation();
                entity.teleportTo(position[0], position[1], position[2]);
            }
        });
    }

    private Entity findEntity(UUID uuid) {
        if (server == null) {
            return null;
        }
        for (ServerLevel level : server.getAllLevels()) {
            Entity entity = level.getEntity(uuid);
            if (entity != null) {
                return entity;
            }
        }
        return null;
    }

    private void emitContactEvents() {
        int max = MpsRigidBodyConfig.MAX_CONTACT_EVENTS_PER_TICK.get();
        if (max <= 0) {
            world.clearEvents();
            return;
        }
        PhysicsWorld.ContactForceEvent[] forces = world.contactForceEvents();
        for (int i = 0; i < Math.min(max, forces.length); i++) {
            PhysicsWorld.ContactForceEvent event = forces[i];
            if (event.totalForceMagnitude() > 1.0) {
                LOGGER.debug("mps_rigid_body contact {} <-> {} force={}", event.collider1(), event.collider2(), event.totalForceMagnitude());
                syncContactToClient(event.collider1(), event.collider2(), event.totalForceMagnitude());
                emitMinecraftContactEvent(event.collider1(), event.collider2(), event.totalForceMagnitude());
            }
        }
        world.clearEvents();
    }

    private void emitMinecraftContactEvent(long collider1, long collider2, double force) {
        if (server == null || force < 10.0) {
            return;
        }
        Entity entity = firstBoundEntity();
        if (entity == null || !(entity.level() instanceof ServerLevel level)) {
            return;
        }
        MpsRigidBodyShapeLoader.MaterialConfig material = strongerMaterial(collider1, collider2);
        entity.playSound(SoundEvents.ANVIL_LAND, Math.min(2.0F, (float) force / 50.0F), 1.0F);
        level.sendParticles(ParticleTypes.CRIT, entity.getX(), entity.getY() + 0.5, entity.getZ(), 8, 0.2, 0.2, 0.2, 0.05);
        double damageScale = Math.max(materialDamageScale(collider1), materialDamageScale(collider2));
        if (force >= 40.0 && damageScale > 0.0) {
            entity.hurt(entity.damageSources().generic(), Math.min(10.0F, (float) (force * damageScale / 20.0)));
        }
        if (material != null && force >= material.breakThreshold()) {
            level.destroyBlock(entity.blockPosition(), true);
        }
    }

    private double materialDamageScale(long collider) {
        MpsRigidBodyShapeLoader.MaterialConfig material = colliderMaterials.get(collider);
        return material == null ? 0.0 : material.damageScale();
    }

    private MpsRigidBodyShapeLoader.MaterialConfig strongerMaterial(long collider1, long collider2) {
        MpsRigidBodyShapeLoader.MaterialConfig first = colliderMaterials.get(collider1);
        MpsRigidBodyShapeLoader.MaterialConfig second = colliderMaterials.get(collider2);
        if (first == null) {
            return second;
        }
        if (second == null) {
            return first;
        }
        return first.damageScale() >= second.damageScale() ? first : second;
    }

    private Entity firstBoundEntity() {
        if (boundEntities.isEmpty()) {
            return null;
        }
        return findEntity(boundEntities.keySet().iterator().next());
    }

    public long createJoint(String id, String type, Entity first, Entity second) {
        return createJoint(id, type, first, second, 0.0, 1.0, 0.0, 0.0, 0.0, Double.NaN, Double.NaN, false);
    }

    public long saveJoint(String id, String type, Entity first, Entity second, double axisX, double axisY, double axisZ, double valueB, double valueC) {
        long handle = createJoint(id, type, first, second, axisX, axisY, axisZ, valueB, valueC, Double.NaN, Double.NaN, false);
        if (server != null) {
            MpsRigidBodySavedData.get(server).addJoint(new MpsRigidBodySavedData.JointBinding(
                    id, first.getUUID(), second.getUUID(), type, axisX, axisY, axisZ, valueB, valueC));
        }
        return handle;
    }

    public long createJoint(
            String id,
            String type,
            Entity first,
            Entity second,
            double axisX,
            double axisY,
            double axisZ,
            double valueB,
            double valueC,
            double limitMin,
            double limitMax,
            boolean contactsEnabled) {
        requireWorld();
        BoundEntity body1 = boundEntities.get(first.getUUID());
        BoundEntity body2 = boundEntities.get(second.getUUID());
        if (body1 == null || body2 == null) {
            throw new IllegalArgumentException("both entities must be bound first");
        }
        Joint.Builder builder = switch (type) {
            case "revolute" -> Joint.Builder.revolute(world, axisX, axisY, axisZ);
            case "prismatic" -> Joint.Builder.prismatic(world, axisX, axisY, axisZ);
            case "rope" -> Joint.Builder.rope(world, valueB);
            case "spring" -> Joint.Builder.spring(world, axisX, valueB, valueC);
            case "spherical" -> Joint.Builder.spherical(world);
            default -> world.fixedJoint();
        };
        builder.contactsEnabled(contactsEnabled);
        if (Double.isFinite(limitMin) && Double.isFinite(limitMax)) {
            builder.limits(Joint.AXIS_Y, limitMin, limitMax);
        }
        Joint joint = builder.insert(body1.body(), body2.body(), true);
        Joint previous = joints.put(id, joint);
        if (previous != null) {
            previous.remove(true);
        }
        return joint.handle();
    }

    public QueryResult raycast(double ox, double oy, double oz, double dx, double dy, double dz, double maxToi) {
        requireWorld();
        Query.RayHit hit = world.query().castRay(ox, oy, oz, dx, dy, dz, maxToi);
        if (hit.isEmpty()) {
            return QueryResult.empty();
        }
        return new QueryResult(hit.collider(), hit.timeOfImpact(), hit.normal());
    }

    public int countAabb(BlockPos first, BlockPos second) {
        requireWorld();
        Area area = checkedArea(first, second, MpsRigidBodyConfig.DEFAULT_VOXEL_SIZE.get());
        return world.query().countAabb(
                area.from().getX(), area.from().getY(), area.from().getZ(),
                area.to().getX() + 1.0, area.to().getY() + 1.0, area.to().getZ() + 1.0);
    }

    public int countSphere(double x, double y, double z, double radius) {
        requireWorld();
        return world.query().intersectSphere(x, y, z, radius, MpsRigidBodyConfig.MAX_CONTACT_EVENTS_PER_TICK.get()).length;
    }

    public long createMotorJoint(String id, String type, Entity first, Entity second, double targetVelocity, double factor) {
        requireWorld();
        BoundEntity body1 = boundEntities.get(first.getUUID());
        BoundEntity body2 = boundEntities.get(second.getUUID());
        if (body1 == null || body2 == null) {
            throw new IllegalArgumentException("both entities must be bound first");
        }
        Joint.Builder builder = switch (type) {
            case "prismatic" -> Joint.Builder.prismatic(world, 0.0, 1.0, 0.0);
            default -> Joint.Builder.revolute(world, 0.0, 1.0, 0.0);
        };
        Joint joint = builder.contactsEnabled(false)
                .motorVelocity(Joint.AXIS_Y, targetVelocity, factor)
                .insert(body1.body(), body2.body(), true);
        joints.put(id, joint);
        return joint.handle();
    }

    public boolean removeJoint(String id) {
        Joint joint = joints.remove(id);
        boolean removed = joint != null && joint.remove(true);
        if (server != null) {
            removed |= MpsRigidBodySavedData.get(server).removeJoint(id);
        }
        return removed;
    }

    private void debugArea(ServerLevel level, Area area) {
        if (!MpsRigidBodyConfig.DEBUG_PARTICLES.get()) {
            return;
        }
        BlockPos min = area.from();
        BlockPos max = area.to();
        for (int x = min.getX(); x <= max.getX(); x += Math.max(1, area.sizeX() / 8)) {
            level.sendParticles(ParticleTypes.END_ROD, x + 0.5, min.getY() + 0.5, min.getZ() + 0.5, 1, 0.0, 0.0, 0.0, 0.0);
            level.sendParticles(ParticleTypes.END_ROD, x + 0.5, max.getY() + 0.5, max.getZ() + 0.5, 1, 0.0, 0.0, 0.0, 0.0);
        }
    }

    private String nextAreaId() {
        return "area_" + generatedAreaIds.incrementAndGet();
    }

    private void recordVoxelBuild(long nanos) {
        lastVoxelBuildNanos = nanos;
        totalVoxelBuilds++;
        totalVoxelBuildNanos += nanos;
    }

    private static double nanosToMillis(long nanos) {
        return nanos / 1_000_000.0;
    }

    public record Status(boolean loaded, boolean enabled, long ticks, int rigidBodies, int colliders, int boundEntities, int queuedVoxelBuilds) {
    }

    public record VoxelInsertResult(long colliderHandle, int solidBlocks, long scannedBlocks) {
    }

    public record Profile(double lastStepMillis, double lastVoxelBuildMillis, double averageVoxelBuildMillis, long totalVoxelBuilds, int chunkColliders) {
    }

    public record QueryResult(long collider, double timeOfImpact, double[] normal) {
        static QueryResult empty() {
            return new QueryResult(0L, 0.0, new double[] {0.0, 0.0, 0.0});
        }

        public boolean isEmpty() {
            return collider == 0L;
        }
    }

    private record Area(BlockPos from, BlockPos to, int sizeX, int sizeY, int sizeZ, double voxelSize, long volume) {
    }

    private record VoxelBuildResult(byte[] voxels, int solid, MpsRigidBodyShapeLoader.MaterialConfig material) {
    }

    private record PendingVoxelBuild(String id, net.minecraft.resources.ResourceKey<net.minecraft.world.level.Level> dimension, Area area, CompletableFuture<VoxelBuildResult> future, ChunkKey chunkKey) {
    }

    private record BoundEntity(RigidBody body) {
        long bodyHandle() {
            return body.handle();
        }
    }

    private record ChunkKey(net.minecraft.resources.ResourceKey<net.minecraft.world.level.Level> dimension, int x, int z) {
    }

    private static final class MaterialAccumulator {
        private MpsRigidBodyShapeLoader.MaterialConfig material = defaultMaterial();

        void accept(BlockState state) {
            ResourceLocation blockId = BuiltInRegistries.BLOCK.getKey(state.getBlock());
            for (MpsRigidBodyShapeLoader.MaterialConfig candidate : MpsRigidBodyShapeLoader.materials().values()) {
                if (candidate.blocks().contains(blockId.toString()) || matchesTag(state, candidate)) {
                    material = candidate;
                    return;
                }
            }
        }

        MpsRigidBodyShapeLoader.MaterialConfig material() {
            return material;
        }

        private static boolean matchesTag(BlockState state, MpsRigidBodyShapeLoader.MaterialConfig candidate) {
            for (String tagId : candidate.tags()) {
                ResourceLocation location = ResourceLocation.tryParse(tagId.startsWith("#") ? tagId.substring(1) : tagId);
                if (location != null && state.is(TagKey.create(net.minecraft.core.registries.Registries.BLOCK, location))) {
                    return true;
                }
            }
            return false;
        }

        private static MpsRigidBodyShapeLoader.MaterialConfig defaultMaterial() {
            return new MpsRigidBodyShapeLoader.MaterialConfig(
                    0.8, 0.0, 1.0, 0.0,
                    Double.POSITIVE_INFINITY, 0.2, 0.2, 1.0,
                    "minecraft:block.anvil.land", "minecraft:crit",
                    java.util.Set.of(), java.util.Set.of());
        }
    }
}
