package org.polaris2023.msp_rigid_body;

import net.neoforged.bus.api.IEventBus;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.ModContainer;
import net.neoforged.fml.common.Mod;
import net.neoforged.fml.config.ModConfig;
import net.neoforged.neoforge.common.NeoForge;
import net.neoforged.neoforge.event.AddReloadListenerEvent;
import net.neoforged.neoforge.event.RegisterCommandsEvent;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;
import net.neoforged.neoforge.event.level.BlockEvent;
import net.neoforged.neoforge.event.level.ChunkEvent;
import net.neoforged.neoforge.event.server.ServerStartedEvent;
import net.neoforged.neoforge.event.server.ServerStoppingEvent;
import net.neoforged.neoforge.event.tick.ServerTickEvent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Mod(MpsRigidBodyMod.MOD_ID)
public final class MpsRigidBodyMod {
    public static final String MOD_ID = "mps_rigid_body";
    private static final Logger LOGGER = LoggerFactory.getLogger(MpsRigidBodyMod.class);
    private final PhysicsRuntime runtime = new PhysicsRuntime();

    public MpsRigidBodyMod(IEventBus modBus, ModContainer container) {
        container.registerConfig(ModConfig.Type.SERVER, MpsRigidBodyConfig.SPEC);
        modBus.addListener(MpsRigidBodyNetwork::register);
        NeoForge.EVENT_BUS.register(this);
        LOGGER.info("mps_rigid_body mod loaded");
    }

    @SubscribeEvent
    public void onServerStarted(ServerStartedEvent event) {
        runtime.start(event.getServer());
    }

    @SubscribeEvent
    public void onServerStopping(ServerStoppingEvent event) {
        runtime.close();
    }

    @SubscribeEvent
    public void onServerPostTick(ServerTickEvent.Post event) {
        runtime.tick();
        if (runtime.status().ticks() % 20L == 0L) {
            runtime.syncDebugStateToClient();
        }
    }

    @SubscribeEvent
    public void onRegisterCommands(RegisterCommandsEvent event) {
        MpsRigidBodyCommands.register(event.getDispatcher(), runtime);
    }

    @SubscribeEvent
    public void onAddReloadListeners(AddReloadListenerEvent event) {
        event.addListener(new MpsRigidBodyShapeLoader());
    }

    @SubscribeEvent
    public void onChunkLoad(ChunkEvent.Load event) {
        if (event.getLevel() instanceof net.minecraft.server.level.ServerLevel level) {
            runtime.queueChunkCollider(level, event.getChunk().getPos());
        }
    }

    @SubscribeEvent
    public void onChunkUnload(ChunkEvent.Unload event) {
        if (event.getLevel() instanceof net.minecraft.server.level.ServerLevel level) {
            runtime.unloadChunkCollider(level, event.getChunk().getPos());
        }
    }

    @SubscribeEvent
    public void onBlockBreak(BlockEvent.BreakEvent event) {
        if (MpsRigidBodyConfig.AUTO_BLOCK_UPDATES.get() && event.getLevel() instanceof net.minecraft.server.level.ServerLevel level) {
            runtime.refreshChunkCollider(level, new net.minecraft.world.level.ChunkPos(event.getPos()));
        }
    }

    @SubscribeEvent
    public void onBlockPlace(BlockEvent.EntityPlaceEvent event) {
        if (MpsRigidBodyConfig.AUTO_BLOCK_UPDATES.get() && event.getLevel() instanceof net.minecraft.server.level.ServerLevel level) {
            runtime.refreshChunkCollider(level, new net.minecraft.world.level.ChunkPos(event.getPos()));
        }
    }
}
