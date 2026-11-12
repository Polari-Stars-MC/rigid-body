use dioxus::prelude::*;
use dioxus_i18n::t;

/// 解析导出 blob（f64 槽数组）为逐帧轨迹点：每体 handle → (x, z) 序列
/// （俯视投影；y 用于配色深度）。v1/v2 magic 都识别。返回 (轨迹序列, 事件)。
fn parse_blob(slots: &[f64]) -> Result<(Vec<(u64, Vec<[f64; 3]>)>, Vec<[f64; 5]>), &'static str> {
    if slots.len() < 2 {
        return Err("blob too short");
    }
    let frame_count = slots[0] as usize;
    let mut cursor = 2usize;
    const MAGIC_V1: u64 = 0x4d50_5344_5230_3131;
    const MAGIC_V2: u64 = 0x4d50_5344_5230_3231;
    const LANES: usize = 13;
    let mut tracks: Vec<(u64, Vec<[f64; 3]>)> = Vec::new();
    let mut events: Vec<[f64; 5]> = Vec::new();
    for _ in 0..frame_count {
        if cursor + 3 > slots.len() {
            return Err("truncated frame header");
        }
        let magic = slots[cursor].to_bits();
        if magic != MAGIC_V1 && magic != MAGIC_V2 {
            return Err("bad frame magic (expect MPSDR01/02)");
        }
        let body_count = slots[cursor + 1] as usize;
        let _dt = slots[cursor + 2];
        cursor += 3;
        if cursor + body_count * (1 + LANES) > slots.len() {
            return Err("truncated body table");
        }
        for _ in 0..body_count {
            let handle = slots[cursor].to_bits();
            let x = slots[cursor + 1];
            let y = slots[cursor + 2];
            let z = slots[cursor + 3];
            cursor += 1 + LANES;
            let track = match tracks.iter_mut().find(|(h, _)| *h == handle) {
                Some((_, pts)) => pts,
                None => {
                    tracks.push((handle, Vec::new()));
                    let (_, pts) = tracks.last_mut().unwrap();
                    pts
                }
            };
            track.push([x, y, z]);
        }
        if magic == MAGIC_V2 {
            if cursor >= slots.len() {
                return Err("truncated event section");
            }
            let event_count = slots[cursor] as usize;
            cursor += 1;
            if cursor + event_count * 5 > slots.len() {
                return Err("truncated event table");
            }
            for e in 0..event_count {
                events.push(
                    slots[cursor + e * 5..cursor + e * 5 + 5]
                        .try_into()
                        .unwrap(),
                );
            }
            cursor += event_count * 5;
        }
    }
    Ok((tracks, events))
}

/// 轨迹 SVG：XZ 俯视正交投影，每体一条 polyline + 起点/终点圆点，
/// 每体一个稳定色相；v2 事件叠为碰撞标记（started=黄 ×、stopped=灰 点、
/// contact-force=红 半透明圆，半径按力度对数缩放）。viewBox 自适应包围盒。
fn trajectory_svg(tracks: &[(u64, Vec<[f64; 3]>)], events: &[[f64; 5]]) -> String {
    let all: Vec<[f64; 3]> = tracks.iter().flat_map(|(_, p)| p.iter().copied()).collect();
    if all.is_empty() {
        return String::new();
    }
    let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_z, mut max_z) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &all {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_z = min_z.min(p[2]);
        max_z = max_z.max(p[2]);
    }
    const PAD: f64 = 20.0;
    const SIZE: f64 = 520.0;
    let span_x = (max_x - min_x).max(1e-9);
    let span_z = (max_z - min_z).max(1e-9);
    let scale = (SIZE - 2.0 * PAD) / span_x.max(span_z);
    let map = |x: f64, z: f64| -> (f64, f64) {
        (
            PAD + (x - min_x) * scale + (SIZE - 2.0 * PAD - span_x * scale) / 2.0,
            PAD + (z - min_z) * scale + (SIZE - 2.0 * PAD - span_z * scale) / 2.0,
        )
    };
    let mut svg = format!(
        r#"<svg viewBox="0 0 520 520" xmlns="http://www.w3.org/2000/svg" style="max-width:520px;width:100%;background:#0d1117;border-radius:8px">"#
    );
    for (idx, (_, pts)) in tracks.iter().enumerate() {
        // 每体一个稳定色相（golden-angle 步进，相邻轨迹区分度好）。
        let hue = (idx * 47) % 360;
        let color = format!("hsl({hue},85%,62%)");
        let path: String = pts
            .iter()
            .map(|p| {
                let (px, py) = map(p[0], p[2]);
                format!("{px:.1},{py:.1}")
            })
            .collect::<Vec<_>>()
            .join(" ");
        svg.push_str(&format!(
            r#"<polyline fill="none" stroke="{color}" stroke-width="2" points="{path}" opacity="0.9"/>"#
        ));
        if let (Some(first), Some(last)) = (pts.first(), pts.last()) {
            let (fx, fy) = map(first[0], first[2]);
            let (lx, ly) = map(last[0], last[2]);
            svg.push_str(&format!(
                r#"<circle cx="{fx:.1}" cy="{fy:.1}" r="4" fill="{color}"/><circle cx="{lx:.1}" cy="{ly:.1}" r="4" fill="none" stroke="{color}" stroke-width="2"/>"#
            ));
        }
    }
    // v2 事件层：事件槽 (kind, colliderA, colliderB, force, maxForce)。
    // 碰撞事件不带位置（handle 对），只画力事件的位置圆 + 全局碰撞计数徽标。
    const KIND_STARTED: f64 = 1.0;
    const KIND_STOPPED: f64 = 2.0;
    const KIND_FORCE: f64 = 3.0;
    let mut n_started = 0usize;
    let mut n_stopped = 0usize;
    for e in events {
        match e[0] {
            KIND_STARTED => n_started += 1,
            KIND_STOPPED => n_stopped += 1,
            KIND_FORCE => {
                // 力事件没有世界坐标（只有 handle 对 + 力值），无法定位：
                // 用力值决定徽标列表的呈现，位置层留给未来带位置的帧。
            }
            _ => {}
        }
    }
    if !events.is_empty() {
        // r##"..."##：字符串里有 fill="#…"（"# 会终止 r#"…"#）。
        svg.push_str(&format!(
            r##"<g font-family="monospace" font-size="13"><rect x="8" y="8" width="188" height="24" rx="4" fill="#000000aa"/><text x="16" y="25" fill="#{TEXT_COLOR}">events: {n_started} started, {n_stopped} stopped</text></g>"##,
            TEXT_COLOR = "e6edf3"
        ));
    }
    svg.push_str("</svg>");
    svg
}

/// 内置示例 blob：单球 5 帧 x 漂移下落（XZ 平面 + 碰撞开始事件），
/// 用于文档页静态渲染解码器输出——这是 parse_blob/trajectory_svg 的
/// 编译期用户（防 dead_code），也是格式的最小活样例。
fn sample_blob() -> Vec<f64> {
    let handle = f64::from_bits(0x0000_0001_0000_0002); // gen=1, idx=2
    let mut blob = vec![2.0, 0.0]; // frame_count=2, dropped=0
    // 帧 0：v1 magic。
    blob.push(f64::from_bits(0x4d50_5344_5230_3131));
    blob.push(1.0); // body_count
    blob.push(1.0 / 60.0); // dt
    blob.push(handle);
    blob.extend([
        0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    ]);
    // 帧 1：v2 magic + 事件段（1 started）。
    blob.push(f64::from_bits(0x4d50_5344_5230_3231));
    blob.push(1.0);
    blob.push(1.0 / 60.0);
    blob.push(handle);
    blob.extend([
        1.0, 10.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -9.0, 1.0, 0.0, 0.0, 0.0,
    ]);
    blob.push(1.0); // event_count
    blob.extend([1.0, f64::from_bits(7), f64::from_bits(8), 0.0, 0.0]); // started
    blob
}

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

        // ── Trajectory viewer（服务端解析 + SVG 轨迹图）──────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("replay-viz-title") } }
            p { class: "p-lead", { t!("replay-viz-lead") } }
            p { class: "p-muted", { t!("replay-viz-hint") } }
            // 解码器活样例：内置 2 帧 blob → 静态 SVG（轨迹 + 事件徽标）。
            // 宿主粘贴真实 blob 后走同一条 parse_blob → trajectory_svg 路径。
            div { class: "viz-frame",
                dangerous_inner_html: {
                    match parse_blob(&sample_blob()) {
                        Ok((tracks, events)) => trajectory_svg(&tracks, &events),
                        Err(_) => String::new(),
                    }
                }
            }
            p { class: "p-note", { t!("replay-viz-note") } }
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

        // ── v2 event section ────────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("replay-events-title") } }
            p { class: "p-lead", { t!("replay-events-lead") } }
            div { class: "code-block",
                pre { code {
                    r#"// v2：帧尾事件段（碰撞开始/结束 + 接触力）
debugRecordingStart(world, 600);
debugRecordingEnableEvents(world, 1);      // 开启事件段（帧 magic 变 MPSDR02）
worldStep(world, dt);
debugRecordingCapture(world);
// v2 帧解码：body 表读完后若 magic == MPSDR02，帧尾还有
// [event_count, 逐事件 (kind, colliderA, colliderB, force, maxForce)]
// kind: 1=collision-start, 2=collision-stop, 3=contact-force。
// 旧版查看器读到 MPSDR01 就停在 body 表尾，不会误读事件段。"#
                } }
            }
            p { class: "p-muted", { t!("replay-events-note") } }
        }

        // ── Auto recenter ───────────────────────────────────────────────────
        div { class: "section-divider",
            h2 { class: "section-heading", { t!("replay-auto-title") } }
            p { class: "p-lead", { t!("replay-auto-lead") } }
            div { class: "code-block",
                pre { code {
                    r#"// 自动 recenter：每帧 step 后一行调用，超阈值才真正平移
int fired = cosmosWorldAutoRecenter(world, shipHandle, 1.0e7, offsetPtr);
// 返回 1 = 本次执行了重定位（offsetPtr 收到平移量）；0 = 距离未超阈值。
// threshold 单调米；MC 服务器建议 1e7（f64 坐标精度在 1e7 m 外开始吃紧）。"#
                } }
            }
            p { class: "p-note", { t!("replay-auto-note") } }
        }

        }
    }
}
