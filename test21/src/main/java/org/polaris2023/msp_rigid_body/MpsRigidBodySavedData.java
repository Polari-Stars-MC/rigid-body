package org.polaris2023.msp_rigid_body;

import net.minecraft.core.BlockPos;
import net.minecraft.core.HolderLookup;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.nbt.ListTag;
import net.minecraft.nbt.Tag;
import net.minecraft.resources.ResourceKey;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.saveddata.SavedData;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public final class MpsRigidBodySavedData extends SavedData {
    private static final String NAME = MpsRigidBodyMod.MOD_ID + "_areas";
    private final List<VoxelArea> areas = new ArrayList<>();
    private final List<EntityBinding> bindings = new ArrayList<>();
    private final List<JointBinding> joints = new ArrayList<>();

    public static MpsRigidBodySavedData get(net.minecraft.server.MinecraftServer server) {
        return server.overworld().getDataStorage().computeIfAbsent(new Factory<>(
                MpsRigidBodySavedData::new,
                MpsRigidBodySavedData::load,
                null), NAME);
    }

    private static MpsRigidBodySavedData load(CompoundTag tag, HolderLookup.Provider provider) {
        MpsRigidBodySavedData data = new MpsRigidBodySavedData();
        ListTag list = tag.getList("areas", Tag.TAG_COMPOUND);
        for (int i = 0; i < list.size(); i++) {
            VoxelArea.fromTag(list.getCompound(i)).ifPresent(data.areas::add);
        }
        ListTag bindings = tag.getList("bindings", Tag.TAG_COMPOUND);
        for (int i = 0; i < bindings.size(); i++) {
            EntityBinding.fromTag(bindings.getCompound(i)).ifPresent(data.bindings::add);
        }
        ListTag joints = tag.getList("joints", Tag.TAG_COMPOUND);
        for (int i = 0; i < joints.size(); i++) {
            JointBinding.fromTag(joints.getCompound(i)).ifPresent(data.joints::add);
        }
        return data;
    }

    public List<VoxelArea> areas() {
        return List.copyOf(areas);
    }

    public List<EntityBinding> bindings() {
        return List.copyOf(bindings);
    }

    public List<JointBinding> joints() {
        return List.copyOf(joints);
    }

    public void add(VoxelArea area) {
        areas.removeIf(existing -> existing.id().equals(area.id()));
        areas.add(area);
        setDirty();
    }

    public void addBinding(EntityBinding binding) {
        bindings.removeIf(existing -> existing.entity().equals(binding.entity()));
        bindings.add(binding);
        setDirty();
    }

    public void addJoint(JointBinding joint) {
        joints.removeIf(existing -> existing.id().equals(joint.id()));
        joints.add(joint);
        setDirty();
    }

    public boolean remove(String id) {
        boolean removed = areas.removeIf(area -> area.id().equals(id));
        if (removed) {
            setDirty();
        }
        return removed;
    }

    public void clear() {
        if (!areas.isEmpty() || !bindings.isEmpty() || !joints.isEmpty()) {
            areas.clear();
            bindings.clear();
            joints.clear();
            setDirty();
        }
    }

    public void clearBindings() {
        if (!bindings.isEmpty()) {
            bindings.clear();
            setDirty();
        }
    }

    public void clearJoints() {
        if (!joints.isEmpty()) {
            joints.clear();
            setDirty();
        }
    }

    public boolean removeBinding(String uuid) {
        boolean removed = bindings.removeIf(binding -> binding.entity().toString().equals(uuid));
        if (removed) {
            setDirty();
        }
        return removed;
    }

    public boolean removeJoint(String id) {
        boolean removed = joints.removeIf(joint -> joint.id().equals(id));
        if (removed) {
            setDirty();
        }
        return removed;
    }

    @Override
    public CompoundTag save(CompoundTag tag, HolderLookup.Provider provider) {
        ListTag list = new ListTag();
        for (VoxelArea area : areas) {
            list.add(area.toTag());
        }
        tag.put("areas", list);
        ListTag bindingList = new ListTag();
        for (EntityBinding binding : bindings) {
            bindingList.add(binding.toTag());
        }
        tag.put("bindings", bindingList);
        ListTag jointList = new ListTag();
        for (JointBinding joint : joints) {
            jointList.add(joint.toTag());
        }
        tag.put("joints", jointList);
        return tag;
    }

    public record VoxelArea(
            String id,
            ResourceKey<Level> dimension,
            BlockPos from,
            BlockPos to,
            double voxelSize,
            boolean autoChunk) {
        CompoundTag toTag() {
            CompoundTag tag = new CompoundTag();
            tag.putString("id", id);
            tag.putString("dimension", dimension.location().toString());
            tag.putLong("from", from.asLong());
            tag.putLong("to", to.asLong());
            tag.putDouble("voxelSize", voxelSize);
            tag.putBoolean("autoChunk", autoChunk);
            return tag;
        }

        static Optional<VoxelArea> fromTag(CompoundTag tag) {
            try {
                ResourceLocation dimensionId = ResourceLocation.parse(tag.getString("dimension"));
                ResourceKey<Level> dimension = ResourceKey.create(net.minecraft.core.registries.Registries.DIMENSION, dimensionId);
                return Optional.of(new VoxelArea(
                        tag.getString("id"),
                        dimension,
                        BlockPos.of(tag.getLong("from")),
                        BlockPos.of(tag.getLong("to")),
                        tag.getDouble("voxelSize"),
                        tag.getBoolean("autoChunk")));
            } catch (RuntimeException exception) {
                return Optional.empty();
            }
        }
    }

    public record EntityBinding(
            java.util.UUID entity,
            ResourceKey<Level> dimension,
            double halfExtent,
            double density,
            double friction,
            double restitution) {
        CompoundTag toTag() {
            CompoundTag tag = new CompoundTag();
            tag.putUUID("entity", entity);
            tag.putString("dimension", dimension.location().toString());
            tag.putDouble("halfExtent", halfExtent);
            tag.putDouble("density", density);
            tag.putDouble("friction", friction);
            tag.putDouble("restitution", restitution);
            return tag;
        }

        static Optional<EntityBinding> fromTag(CompoundTag tag) {
            try {
                ResourceLocation dimensionId = ResourceLocation.parse(tag.getString("dimension"));
                ResourceKey<Level> dimension = ResourceKey.create(net.minecraft.core.registries.Registries.DIMENSION, dimensionId);
                return Optional.of(new EntityBinding(
                        tag.getUUID("entity"),
                        dimension,
                        tag.getDouble("halfExtent"),
                        tag.contains("density") ? tag.getDouble("density") : 1.0,
                        tag.contains("friction") ? tag.getDouble("friction") : 0.6,
                        tag.contains("restitution") ? tag.getDouble("restitution") : 0.0));
            } catch (RuntimeException exception) {
                return Optional.empty();
            }
        }
    }

    public record JointBinding(
            String id,
            java.util.UUID first,
            java.util.UUID second,
            String type,
            double axisX,
            double axisY,
            double axisZ,
            double valueB,
            double valueC) {
        CompoundTag toTag() {
            CompoundTag tag = new CompoundTag();
            tag.putString("id", id);
            tag.putUUID("first", first);
            tag.putUUID("second", second);
            tag.putString("type", type);
            tag.putDouble("axisX", axisX);
            tag.putDouble("axisY", axisY);
            tag.putDouble("axisZ", axisZ);
            tag.putDouble("valueB", valueB);
            tag.putDouble("valueC", valueC);
            return tag;
        }

        static Optional<JointBinding> fromTag(CompoundTag tag) {
            try {
                return Optional.of(new JointBinding(
                        tag.getString("id"),
                        tag.getUUID("first"),
                        tag.getUUID("second"),
                        tag.getString("type"),
                        tag.contains("axisX") ? tag.getDouble("axisX") : 0.0,
                        tag.contains("axisY") ? tag.getDouble("axisY") : 1.0,
                        tag.contains("axisZ") ? tag.getDouble("axisZ") : 0.0,
                        tag.contains("valueB") ? tag.getDouble("valueB") : 0.0,
                        tag.contains("valueC") ? tag.getDouble("valueC") : 0.0));
            } catch (RuntimeException exception) {
                return Optional.empty();
            }
        }
    }
}
