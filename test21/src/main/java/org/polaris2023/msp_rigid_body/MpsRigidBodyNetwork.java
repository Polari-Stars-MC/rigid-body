package org.polaris2023.msp_rigid_body;

import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.phys.AABB;
import net.neoforged.neoforge.network.PacketDistributor;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;

import java.util.ArrayList;
import java.util.List;

public final class MpsRigidBodyNetwork {
    private static volatile PhysicsRuntime.Status lastClientStatus = new PhysicsRuntime.Status(false, false, 0L, 0, 0, 0, 0);
    private static volatile List<DebugAabb> lastDebugAabbs = List.of();
    private static volatile ContactInfo lastContact = ContactInfo.empty();

    private MpsRigidBodyNetwork() {
    }

    static void register(RegisterPayloadHandlersEvent event) {
        event.registrar("1")
                .playToClient(StatusPayload.TYPE, StatusPayload.STREAM_CODEC, (payload, context) -> lastClientStatus = payload.status())
                .playToClient(DebugAabbsPayload.TYPE, DebugAabbsPayload.STREAM_CODEC, (payload, context) -> lastDebugAabbs = payload.aabbs())
                .playToClient(ContactPayload.TYPE, ContactPayload.STREAM_CODEC, (payload, context) -> lastContact = payload.contact());
    }

    static void syncStatusToAll(PhysicsRuntime.Status status) {
        PacketDistributor.sendToAllPlayers(new StatusPayload(status));
    }

    static void syncDebugAabbsToAll(List<DebugAabb> aabbs) {
        PacketDistributor.sendToAllPlayers(new DebugAabbsPayload(aabbs));
    }

    static void syncContactToAll(ContactInfo contact) {
        PacketDistributor.sendToAllPlayers(new ContactPayload(contact));
    }

    public static PhysicsRuntime.Status lastClientStatus() {
        return lastClientStatus;
    }

    public static List<DebugAabb> lastDebugAabbs() {
        return lastDebugAabbs;
    }

    public static ContactInfo lastContact() {
        return lastContact;
    }

    public record StatusPayload(PhysicsRuntime.Status status) implements CustomPacketPayload {
        static final Type<StatusPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(MpsRigidBodyMod.MOD_ID, "status"));
        static final StreamCodec<RegistryFriendlyByteBuf, StatusPayload> STREAM_CODEC = CustomPacketPayload.codec(
                StatusPayload::write,
                StatusPayload::read);

        private static StatusPayload read(RegistryFriendlyByteBuf buffer) {
            return new StatusPayload(new PhysicsRuntime.Status(
                    buffer.readBoolean(),
                    buffer.readBoolean(),
                    buffer.readLong(),
                    buffer.readVarInt(),
                    buffer.readVarInt(),
                    buffer.readVarInt(),
                    buffer.readVarInt()));
        }

        private void write(RegistryFriendlyByteBuf buffer) {
            buffer.writeBoolean(status.loaded());
            buffer.writeBoolean(status.enabled());
            buffer.writeLong(status.ticks());
            buffer.writeVarInt(status.rigidBodies());
            buffer.writeVarInt(status.colliders());
            buffer.writeVarInt(status.boundEntities());
            buffer.writeVarInt(status.queuedVoxelBuilds());
        }

        @Override
        public Type<? extends CustomPacketPayload> type() {
            return TYPE;
        }
    }

    public record DebugAabbsPayload(List<DebugAabb> aabbs) implements CustomPacketPayload {
        static final Type<DebugAabbsPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(MpsRigidBodyMod.MOD_ID, "debug_aabbs"));
        static final StreamCodec<RegistryFriendlyByteBuf, DebugAabbsPayload> STREAM_CODEC = CustomPacketPayload.codec(
                DebugAabbsPayload::write,
                DebugAabbsPayload::read);

        private static DebugAabbsPayload read(RegistryFriendlyByteBuf buffer) {
            int count = Math.min(buffer.readVarInt(), 256);
            List<DebugAabb> boxes = new ArrayList<>(count);
            for (int i = 0; i < count; i++) {
                boxes.add(new DebugAabb(
                        buffer.readDouble(), buffer.readDouble(), buffer.readDouble(),
                        buffer.readDouble(), buffer.readDouble(), buffer.readDouble(),
                        buffer.readInt()));
            }
            return new DebugAabbsPayload(List.copyOf(boxes));
        }

        private void write(RegistryFriendlyByteBuf buffer) {
            int count = Math.min(aabbs.size(), 256);
            buffer.writeVarInt(count);
            for (int i = 0; i < count; i++) {
                DebugAabb box = aabbs.get(i);
                buffer.writeDouble(box.minX());
                buffer.writeDouble(box.minY());
                buffer.writeDouble(box.minZ());
                buffer.writeDouble(box.maxX());
                buffer.writeDouble(box.maxY());
                buffer.writeDouble(box.maxZ());
                buffer.writeInt(box.color());
            }
        }

        @Override
        public Type<? extends CustomPacketPayload> type() {
            return TYPE;
        }
    }

    public record ContactPayload(ContactInfo contact) implements CustomPacketPayload {
        static final Type<ContactPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(MpsRigidBodyMod.MOD_ID, "contact"));
        static final StreamCodec<RegistryFriendlyByteBuf, ContactPayload> STREAM_CODEC = CustomPacketPayload.codec(
                ContactPayload::write,
                ContactPayload::read);

        private static ContactPayload read(RegistryFriendlyByteBuf buffer) {
            return new ContactPayload(new ContactInfo(buffer.readLong(), buffer.readLong(), buffer.readDouble()));
        }

        private void write(RegistryFriendlyByteBuf buffer) {
            buffer.writeLong(contact.collider1());
            buffer.writeLong(contact.collider2());
            buffer.writeDouble(contact.force());
        }

        @Override
        public Type<? extends CustomPacketPayload> type() {
            return TYPE;
        }
    }

    public record DebugAabb(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, int color) {
        public AABB toAabb() {
            return new AABB(minX, minY, minZ, maxX, maxY, maxZ);
        }
    }

    public record ContactInfo(long collider1, long collider2, double force) {
        static ContactInfo empty() {
            return new ContactInfo(0L, 0L, 0.0);
        }
    }
}
