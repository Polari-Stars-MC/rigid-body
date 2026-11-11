use dioxus::prelude::*;
use dioxus_i18n::t;

/// Debug Replay — Jolt-Viewer 风格录制回放：`debugRecording*` JNI 面的
/// 导出 blob 布局文档 + Java 端最小 Canvas 回放示例（SSR 文档页，
/// 无 JS，代码示例供宿主复制）。
pub fn DebugReplay() -> Element {
    rsx! {
        section { id: "sec-debug-replay", class: "doc-section",

        div { class: "page-head",
            div {
                div { class: "page-tag", { t!("replay-tag") } }
                h1 { class: "page-title", { t!("replay-title") } }
                p { class: "page-desc", { t!("replay-desc") } }
            }
            div { class: "page-index", "13" }
        }

        // ── Recording lifecycle ─────────────────────────────────────────────
        div { class: "section-card",
            h2 { { t!("replay-lifecycle-title") } }
            p { class: "p-lead", { t!("replay-lifecycle-lead") } }
            div { class: "code-block",
                pre { code {
                    r#"// 录制环形缓冲：溢出覆盖最旧帧（Jolt Viewer 语义）
debugRecordingStart(world, 600);          // 环容量 = 600 帧（10s @60Hz）
// 每帧推进后捕获（未录制时是静默 no-op，热路径可无条件调用）：
worldStep(world, dt);
debugRecordingCapture(world);
// 停止后读回：
int frames = debugRecordingStop(world);   // 返回保留帧数
debugRecordingDrop(world);                // 释放内存（可选）"#
                } }
            }
            p { class: "p-note", { t!("replay-lifecycle-note") } }
        }

        // ── Export blob layout ──────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("replay-blob-title") } }
            p { class: "p-muted", { t!("replay-blob-desc") } }
            div { class: "table-wrap",
                table {
                    thead { tr {
                        th { { t!("replay-col-slot") } }
                        th { { t!("replay-col-value") } }
                        th { { t!("replay-col-meaning") } }
                    } }
                    tbody {
                        tr { td { "[0]" } td { "frame_count" } td { { t!("replay-row-frame-count") } } }
                        tr { td { "[1]" } td { "dropped" } td { { t!("replay-row-dropped") } } }
                        tr { td { "逐帧" } td { "magic" } td { { t!("replay-row-magic") } } }
                        tr { td { "" } td { "body_count" } td { { t!("replay-row-body-count") } } }
                        tr { td { "" } td { "dt" } td { { t!("replay-row-dt") } } }
                        tr { td { "" } td { "handle × N" } td { { t!("replay-row-handle") } } }
                        tr { td { "" } td { "13 lanes × N" } td { { t!("replay-row-lanes") } } }
                    }
                }
            }
            p { class: "p-note", { t!("replay-blob-note") } }
        }

        // ── Minimal Java replay loop ────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("replay-java-title") } }
            p { class: "p-lead", { t!("replay-java-lead") } }
            div { class: "code-block",
                pre { code {
                    r#"// 两段式导出：先查大小，再分配 native 缓冲
int slots = debugRecordingExport(world, 0, 0);
long blob = Unsafe.allocateMemory((long) slots * 8);
debugRecordingExport(world, blob, slots);
DoubleBuffer db = new DirectByteBuffer(blob, slots * 8L)
        .order(ByteOrder.nativeOrder()).asDoubleBuffer();

// 逐帧解码（13 lanes: pos3 + quat4 + linvel3 + angvel3）
int frameCount = (int) db.get(0);
int cursor = 2;                          // 跳过 [frame_count, dropped]
for (int f = 0; f < frameCount; f++) {
    double magic = db.get(cursor++);     // MPSDR01 位型，可校验
    int bodies    = (int) db.get(cursor++);
    double dt     = db.get(cursor++);
    for (int b = 0; b < bodies; b++) {
        long handle = Double.doubleToRawLongBits(db.get(cursor++));
        double x = db.get(cursor++), y = db.get(cursor++), z = db.get(cursor++);
        // quat(i,j,k,w) 在 lanes 3..7 —— 渲染时只需 pose 的话可跳过
        cursor += 10;                    // quat4 + linvel3 + angvel3
        render(handle, x, y, z);         // 正交投影到 Canvas，逐帧绘制
    }
}"#
                } }
            }
            p { class: "p-muted", { t!("replay-java-note") } }
        }

        // ── When to recenter instead ────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("replay-recenter-title") } }
            p { class: "p-lead", { t!("replay-recenter-lead") } }
            div { class: "code-block",
                pre { code {
                    r#"// 浮动原点：长距航线坐标超过 1e7 m 后以主飞船为 pivot 重定位
int ok = cosmosWorldRecenter(world, shipX, shipY, shipZ, offsetPtr);
// offsetPtr 收到实际平移量（3×f64）；所有位置 += offset，物理不变。
// 必须在两次 cosmosWorldStep 之间调用，录制帧坐标会随之跳变，
// 回放器按帧内 handle 对齐、跨帧位置差渲染即可自然衔接。"#
                } }
            }
            p { class: "p-note", { t!("replay-recenter-note") } }
        }

        }
    }
}
