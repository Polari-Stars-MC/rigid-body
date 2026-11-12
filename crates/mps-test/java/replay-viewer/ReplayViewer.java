import javax.swing.*;
import java.awt.*;
import java.awt.geom.Path2D;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * MPS Debug Recording Viewer — 独立离线回放工具（Java Swing，无仓库内依赖）。
 *
 * 用法：{@code java ReplayViewer /path/to/blob.mpsdr}
 * 拖拽/文件选择加载 `debugRecordingExport` 导出的 blob，渲染 XZ 俯视轨迹
 * （每体一条折线，起点实心圆 / 终点空心圆）+ v2 事件统计徽标。
 *
 * 这是 web 文档页轨迹查看器（Rust 参考实现）的 Swing 版本——同一份
 * blob 布局契约：头部 [frame_count, dropped]，逐帧 magic|count|dt|
 * (handle + 13 lanes)，v2 帧尾 [event_count, 5 槽/事件]。
 */
public final class ReplayViewer extends JPanel {
    // ── 解码结果 ────────────────────────────────────────────────────────
    record Track(long handle, List<double[]> points) {}
    record Event(int kind, long a, long b, double force) {}

    static final long MAGIC_V1 = 0x4D50534452303131L;
    static final long MAGIC_V2 = 0x4D50534452303231L;
    static final int KIND_STARTED = 1, KIND_STOPPED = 2, KIND_FORCE = 3;

    final List<Track> tracks = new ArrayList<>();
    final List<Event> events = new ArrayList<>();
    int frameCount, dropped;
    double minX, maxX, minZ, maxZ;

    ReplayViewer() { setPreferredSize(new Dimension(760, 700)); setBackground(new Color(0x0D1117)); }

    void load(Path file) throws IOException {
        byte[] raw = Files.readAllBytes(file);
        ByteBuffer db = ByteBuffer.wrap(raw).order(ByteOrder.nativeOrder());
        tracks.clear(); events.clear();
        frameCount = (int) db.getDouble(0);
        dropped = (int) db.getDouble(1);
        Map<Long, Track> byHandle = new LinkedHashMap<>();
        int cursor = 2;
        for (int f = 0; f < frameCount; f++) {
            long magic = Double.doubleToRawLongBits(db.getDouble(cursor));
            int bodies = (int) db.getDouble(cursor + 1);
            cursor += 3;
            for (int b = 0; b < bodies; b++) {
                long handle = Double.doubleToRawLongBits(db.getDouble(cursor));
                double x = db.getDouble(cursor + 1);
                double y = db.getDouble(cursor + 2);
                double z = db.getDouble(cursor + 3);
                cursor += 1 + 13;
                Track t = byHandle.computeIfAbsent(handle, h -> new Track(h, new ArrayList<>()));
                t.points().add(new double[]{x, y, z});
            }
            if (magic == MAGIC_V2) {
                int n = (int) db.getDouble(cursor++);
                for (int e = 0; e < n; e++) {
                    events.add(new Event((int) db.getDouble(cursor),
                            Double.doubleToRawLongBits(db.getDouble(cursor + 1)),
                            Double.doubleToRawLongBits(db.getDouble(cursor + 2)),
                            db.getDouble(cursor + 3)));
                    cursor += 5;
                }
            }
        }
        tracks.addAll(byHandle.values());
        computeBounds();
        repaint();
    }

    void computeBounds() {
        minX = minZ = Double.POSITIVE_INFINITY;
        maxX = maxZ = Double.NEGATIVE_INFINITY;
        for (Track t : tracks) for (double[] p : t.points()) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minZ = Math.min(minZ, p[2]); maxZ = Math.max(maxZ, p[2]);
        }
        if (tracks.isEmpty()) { minX = maxX = minZ = maxZ = 0; }
    }

    @Override protected void paintComponent(Graphics g0) {
        super.paintComponent(g0);
        Graphics2D g = (Graphics2D) g0;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        int w = getWidth(), h = getHeight();
        int pad = 40;
        double spanX = Math.max(maxX - minX, 1e-9), spanZ = Math.max(maxZ - minZ, 1e-9);
        double scale = Math.min(w - 2.0 * pad, h - 2.0 * pad) / Math.max(spanX, spanZ);
        double ox = (w - spanX * scale) / 2.0, oz = (h - spanZ * scale) / 2.0;
        java.util.function.DoubleUnaryOperator mapX = x -> ox + (x - minX) * scale;
        java.util.function.DoubleUnaryOperator mapZ = z -> oz + (z - minZ) * scale;

        // 轨迹层：每体一个稳定色相（golden-angle 步进，与 web 版一致）。
        for (int i = 0; i < tracks.size(); i++) {
            Track t = tracks.get(i);
            Color c = Color.getHSBColor((i * 47 % 360) / 360.0f, 0.85f, 0.62f);
            g.setColor(c);
            g.setStroke(new BasicStroke(1.8f));
            if (t.points().size() >= 2) {
                Path2D path = new Path2D.Double();
                double[] p0 = t.points().get(0);
                path.moveTo(mapX.applyAsDouble(p0[0]), mapZ.applyAsDouble(p0[2]));
                for (double[] p : t.points().subList(1, t.points().size()))
                    path.lineTo(mapX.applyAsDouble(p[0]), mapZ.applyAsDouble(p[2]));
                g.draw(path);
            }
            double[] first = t.points().get(0), last = t.points().get(t.points().size() - 1);
            g.fillOval((int) mapX.applyAsDouble(first[0]) - 4, (int) mapZ.applyAsDouble(first[2]) - 4, 8, 8);
            g.drawOval((int) mapX.applyAsDouble(last[0]) - 5, (int) mapZ.applyAsDouble(last[2]) - 5, 10, 10);
        }

        // 状态徽标。
        g.setColor(new Color(0xE6EDF3));
        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));
        String line1 = "frames " + frameCount + " (dropped " + dropped + ")  bodies " + tracks.size();
        String line2 = "events: " + events.stream().filter(e -> e.kind() == KIND_STARTED).count() + " started, "
                + events.stream().filter(e -> e.kind() == KIND_STOPPED).count() + " stopped, "
                + events.stream().filter(e -> e.kind() == KIND_FORCE).count() + " forces";
        g.drawString(line1, 16, 24);
        g.drawString(line2, 16, 42);
    }

    public static void main(String[] args) throws Exception {
        SwingUtilities.invokeLater(() -> {
            JFrame f = new JFrame("MPS Replay Viewer");
            ReplayViewer view = new ReplayViewer();
            f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            f.add(view, BorderLayout.CENTER);
            JToolBar bar = new JToolBar();
            JButton open = new JButton("Open blob…");
            open.addActionListener(ev -> {
                JFileChooser fc = new JFileChooser();
                if (fc.showOpenDialog(f) == JFileChooser.APPROVE_OPTION) {
                    try { view.load(fc.getSelectedFile().toPath()); }
                    catch (Exception e) {
                        JOptionPane.showMessageDialog(f, "decode failed: " + e.getMessage(),
                                "Error", JOptionPane.ERROR_MESSAGE);
                    }
                }
            });
            bar.add(open);
            f.add(bar, BorderLayout.NORTH);
            f.pack();
            f.setVisible(true);
            // 命令行直开。
            if (args.length > 0) {
                try { view.load(Path.of(args[0])); }
                catch (Exception e) { JOptionPane.showMessageDialog(f, "decode failed: " + e.getMessage()); }
            }
        });
    }
}
