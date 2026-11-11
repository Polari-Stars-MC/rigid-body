use dioxus::prelude::*;
use dioxus_i18n::t;

use crate::metrics::{CORE_FFI_COUNT, JNI_METHOD_COUNT, VERSION};

/// Java JNI Bindings — the 312-method `org.polaris2023.mps.rapier.RapierNative`
/// surface exported by `mps-jni` via `jni!` / `jni_e_c!` macros.
pub fn Jni() -> Element {
    rsx! {
        section { id: "sec-jni", class: "doc-section",

        div { class: "page-head",
            div {
                div { class: "page-tag", { t!("jni-tag") } }
                h1 { class: "page-title", { t!("jni-title") } }
                p { class: "page-desc", { t!("jni-desc", methods: JNI_METHOD_COUNT) } }
            }
            div { class: "page-index", "11" }
        }

        // ── Macro codegen ────────────────────────────────────────────────────
        div { class: "section-card",
            h2 { { t!("jni-codegen-title") } }
            p { class: "p-lead", { t!("jni-codegen-lead") } }
            p { class: "p-muted", { t!("jni-codegen-body") } }
            div { class: "code-block",
                pre { code {
                    r#"// mps-jni/src/lib.rs
jni!(long worldCreate(double dt, int iters, int ccd) {{
    to_jlong(wo::world_create(dt, iters as usize, ccd as usize))
}});

// jni_e_c! adds JNIEnv / jclass for callbacks needing VM access
jni_e_c!(void collisionEventInstall(long world, long callbackPtr) {{
    // ...
}});"#
                } }
            }
            p { class: "p-note", { t!("jni-codegen-note") } }
        }

        // ── Panic isolation ─────────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("jni-panic-title") } }
            p { class: "p-lead", { t!("jni-panic-lead") } }
            div { class: "callout-note",
                p { { t!("jni-panic-body") } }
            }
        }

        // ── Symbol mangling ─────────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("jni-mangle-title") } }
            p { class: "p-muted", { t!("jni-mangle-desc") } }
            div { class: "table-wrap",
                table {
                    thead { tr {
                        th { { t!("jni-col-class") } }
                        th { { t!("jni-col-symbol") } }
                    } }
                    tbody {
                        tr {
                            td { "org.polaris2023.mps.rapier.RapierNative" }
                            td { "Java_org_polaris2023_mps_rapier_RapierNative_<m>" }
                        }
                        tr {
                            td { "org.cn_grass_block.kelvin.physical.SpaceNative" }
                            td { "Java_org_cn_1grass_1block_kelvin_physical_SpaceNative_<m>" }
                        }
                    }
                }
            }
            p { class: "p-note", { t!("jni-mangle-note") } }
        }

        // ── API surface groups ─────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("jni-groups-title", ffi: CORE_FFI_COUNT) } }
            div { class: "feature-grid",
                div { class: "feature-card",
                    h3 { { t!("jni-group-abi-title") } }
                    p { { t!("jni-group-abi-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-world-title") } }
                    p { { t!("jni-group-world-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-rb-title") } }
                    p { { t!("jni-group-rb-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-collider-title") } }
                    p { { t!("jni-group-collider-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-query-title") } }
                    p { { t!("jni-group-query-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-events-title") } }
                    p { { t!("jni-group-events-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-forces-title") } }
                    p { { t!("jni-group-forces-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-aero-title") } }
                    p { { t!("jni-group-aero-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-arena-title") } }
                    p { { t!("jni-group-arena-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-cosmos-title") } }
                    p { { t!("jni-group-cosmos-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-spaceflight-title") } }
                    p { { t!("jni-group-spaceflight-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-disaster-title") } }
                    p { { t!("jni-group-disaster-desc") } }
                }
                div { class: "feature-card",
                    h3 { { t!("jni-group-volcano-title") } }
                    p { { t!("jni-group-volcano-desc") } }
                }
            }
        }

        // ── Handle packing ──────────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("jni-handle-title") } }
            p { class: "p-lead", { t!("jni-handle-lead") } }
            div { class: "code-block",
                pre { code {
                    "// RigidBodyHandle → jlong (high 32 = index, low 32 = generation)\n// 与 Rapier RigidBodyHandle::into_raw_parts() 顺序一致\nlong handle = worldCreate(...);\nlong ptr      = cosmosInsertBody(world, builder);"
                } }
            }
            p { class: "p-note", { t!("jni-handle-note") } }
        }

        // ── Zero-copy Arena bridge ──────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("jni-arena-title") } }
            p { class: "p-lead", { t!("jni-arena-lead") } }
            div { class: "code-block",
                pre { code {
                    r#"// arenaAsDirectByteBuffer 通过 NewDirectByteBuffer 把原生 Arena
// 包装成 java.nio.ByteBuffer，Java 端读写零 JNI 往返
ByteBuffer buf = RapierNative.arenaAsDirectByteBuffer(world);
DoubleBuffer db = buf.asDoubleBuffer();
db.position(OFFSET_BODY_0_LINVEL);
db.put(linvelX); db.put(linvelY); db.put(linvelZ);"#
                } }
            }
            p { class: "p-muted", { t!("jni-arena-body") } }
        }

        // ── Deployment ──────────────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("jni-deploy-title") } }
            ul { class: "ul-plain",
                li { { t!("jni-deploy-lib") } }
                li { { t!("jni-deploy-load") } }
                li { { t!("jni-deploy-version", version: VERSION) } }
            }
        }

        }
    }
}
