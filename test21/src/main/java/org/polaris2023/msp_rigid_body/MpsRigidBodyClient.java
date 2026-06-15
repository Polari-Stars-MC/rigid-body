package org.polaris2023.msp_rigid_body;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.network.chat.Component;
import net.minecraft.world.phys.Vec3;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientPlayerNetworkEvent;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import net.neoforged.neoforge.client.event.RenderGuiEvent;
import net.neoforged.neoforge.client.event.RenderLevelStageEvent;

@EventBusSubscriber(modid = MpsRigidBodyMod.MOD_ID, value = Dist.CLIENT)
public final class MpsRigidBodyClient {
    private static boolean showHud = true;

    private MpsRigidBodyClient() {
    }

    @SubscribeEvent
    public static void onClientLogin(ClientPlayerNetworkEvent.LoggingIn event) {
        showHud = true;
    }

    @SubscribeEvent
    public static void onClientLogout(ClientPlayerNetworkEvent.LoggingOut event) {
        showHud = false;
    }

    @SubscribeEvent
    public static void onClientTick(ClientTickEvent.Post event) {
        if (Minecraft.getInstance().player == null) {
            showHud = false;
        }
    }

    @SubscribeEvent
    public static void onRenderGui(RenderGuiEvent.Post event) {
        if (!showHud) {
            return;
        }
        PhysicsRuntime.Status status = MpsRigidBodyNetwork.lastClientStatus();
        GuiGraphics graphics = event.getGuiGraphics();
        graphics.drawString(Minecraft.getInstance().font,
                Component.literal("mpsrb " + status.rigidBodies() + "/" + status.colliders() + " ticks=" + status.ticks()),
                8, 8, 0xFFFFFF, true);
        graphics.drawString(Minecraft.getInstance().font,
                Component.literal("bound=" + status.boundEntities() + " queued=" + status.queuedVoxelBuilds() + " enabled=" + status.enabled()),
                8, 18, 0xA0A0A0, true);
        MpsRigidBodyNetwork.ContactInfo contact = MpsRigidBodyNetwork.lastContact();
        if (contact.force() > 0.0) {
            graphics.drawString(Minecraft.getInstance().font,
                    Component.literal("contact force=" + String.format(java.util.Locale.ROOT, "%.2f", contact.force())),
                    8, 28, 0xFFCC66, true);
        }
    }

    @SubscribeEvent
    public static void onRenderLevelStage(RenderLevelStageEvent event) {
        if (!showHud || event.getStage() != RenderLevelStageEvent.Stage.AFTER_BLOCK_ENTITIES) {
            return;
        }
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.level == null) {
            return;
        }
        PoseStack poseStack = event.getPoseStack();
        Vec3 camera = event.getCamera().getPosition();
        VertexConsumer consumer = minecraft.renderBuffers().bufferSource().getBuffer(RenderType.lines());
        for (MpsRigidBodyNetwork.DebugAabb box : MpsRigidBodyNetwork.lastDebugAabbs()) {
            float red = ((box.color() >> 16) & 0xff) / 255.0F;
            float green = ((box.color() >> 8) & 0xff) / 255.0F;
            float blue = (box.color() & 0xff) / 255.0F;
            LevelRenderer.renderLineBox(
                    poseStack,
                    consumer,
                    box.toAabb().move(-camera.x, -camera.y, -camera.z),
                    red,
                    green,
                    blue,
                    1.0F);
        }
    }
}
