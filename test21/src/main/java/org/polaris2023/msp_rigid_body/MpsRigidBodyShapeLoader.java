package org.polaris2023.msp_rigid_body;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.packs.resources.ResourceManager;
import net.minecraft.server.packs.resources.SimpleJsonResourceReloadListener;
import net.minecraft.util.profiling.ProfilerFiller;

import java.util.HashMap;
import java.util.Map;

public final class MpsRigidBodyShapeLoader extends SimpleJsonResourceReloadListener {
    private static final Gson GSON = new Gson();
    private static final Map<ResourceLocation, ShapeConfig> SHAPES = new HashMap<>();
    private static final Map<ResourceLocation, MaterialConfig> MATERIALS = new HashMap<>();

    public MpsRigidBodyShapeLoader() {
        super(GSON, "mps_rigid_body/shapes");
    }

    public static Map<ResourceLocation, ShapeConfig> shapes() {
        return Map.copyOf(SHAPES);
    }

    public static Map<ResourceLocation, MaterialConfig> materials() {
        return Map.copyOf(MATERIALS);
    }

    @Override
    protected void apply(Map<ResourceLocation, JsonElement> objects, ResourceManager manager, ProfilerFiller profiler) {
        SHAPES.clear();
        MATERIALS.clear();
        objects.forEach((id, element) -> {
            if (!element.isJsonObject()) {
                return;
            }
            JsonObject object = element.getAsJsonObject();
            if ("material".equals(string(object, "kind", ""))) {
                MATERIALS.put(id, new MaterialConfig(
                        number(object, "friction", 0.8),
                        number(object, "restitution", 0.0),
                        number(object, "density", 1.0),
                        number(object, "damage_scale", 0.0),
                        number(object, "break_threshold", Double.POSITIVE_INFINITY),
                        number(object, "linear_damping", 0.2),
                        number(object, "angular_damping", 0.2),
                        number(object, "gravity_scale", 1.0),
                        string(object, "sound", "minecraft:block.anvil.land"),
                        string(object, "particle", "minecraft:crit"),
                        strings(object, "blocks"),
                        strings(object, "tags")));
                return;
            }
            String type = string(object, "type", "voxel_aabb");
            double voxelSize = number(object, "voxel_size", MpsRigidBodyConfig.DEFAULT_VOXEL_SIZE.get());
            double friction = number(object, "friction", 0.8);
            double restitution = number(object, "restitution", 0.0);
            double density = number(object, "density", 1.0);
            double radius = number(object, "radius", 0.5);
            double halfExtent = number(object, "half_extent", 0.5);
            double halfHeight = number(object, "half_height", 0.5);
            SHAPES.put(id, new ShapeConfig(type, voxelSize, friction, restitution, density, radius, halfExtent, halfHeight));
        });
    }

    public static MaterialConfig materialForBlock(String blockId) {
        for (MaterialConfig material : MATERIALS.values()) {
            if (material.blocks().contains(blockId)) {
                return material;
            }
        }
        return null;
    }

    public static MaterialConfig materialForTag(String tagId) {
        String normalized = tagId.startsWith("#") ? tagId.substring(1) : tagId;
        for (MaterialConfig material : MATERIALS.values()) {
            if (material.tags().contains(normalized) || material.tags().contains("#" + normalized)) {
                return material;
            }
        }
        return null;
    }

    private static String string(JsonObject object, String key, String fallback) {
        return object.has(key) ? object.get(key).getAsString() : fallback;
    }

    private static double number(JsonObject object, String key, double fallback) {
        return object.has(key) ? object.get(key).getAsDouble() : fallback;
    }

    private static java.util.Set<String> strings(JsonObject object, String key) {
        if (!object.has(key) || !object.get(key).isJsonArray()) {
            return java.util.Set.of();
        }
        java.util.Set<String> values = new java.util.HashSet<>();
        JsonArray array = object.getAsJsonArray(key);
        for (JsonElement element : array) {
            values.add(element.getAsString());
        }
        return java.util.Set.copyOf(values);
    }

    public record ShapeConfig(
            String type,
            double voxelSize,
            double friction,
            double restitution,
            double density,
            double radius,
            double halfExtent,
            double halfHeight) {
    }

    public record MaterialConfig(
            double friction,
            double restitution,
            double density,
            double damageScale,
            double breakThreshold,
            double linearDamping,
            double angularDamping,
            double gravityScale,
            String sound,
            String particle,
            java.util.Set<String> blocks,
            java.util.Set<String> tags) {
    }
}
