package org.firstinspires.ftc.pedroext.logging;

import com.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.pedroext.sim.SimulatedRobot;
import org.firstinspires.ftc.pedroext.sim.SimulationLoop;
import org.firstinspires.ftc.pedroext.sim.SimulationResult;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class LoggingTest {

    private static final double EPS = 1e-9;

    private LogFrame frame(double t, double poseX, double voltage) {
        return LogFrame.builder().set("t", t).set("poseX", poseX).set("batteryVoltage", voltage).build();
    }

    // --- LogFrame ------------------------------------------------------------

    @Test
    void builderSetsNamedChannels() {
        LogFrame f = frame(0.5, 12.0, 11.8);
        assertEquals(0.5, f.time(), EPS);
        assertEquals(12.0, f.get("poseX"), EPS);
        assertEquals(11.8, f.get("batteryVoltage"), EPS);
        assertEquals(LogFrame.CHANNELS.length, f.values().length);
    }

    @Test
    void unknownChannelThrows() {
        assertThrows(IllegalArgumentException.class, () -> LogFrame.builder().set("nope", 1.0));
    }

    // --- CSV -----------------------------------------------------------------

    @Test
    void csvWriterEmitsHeaderAndRows() throws IOException {
        StringWriter sw = new StringWriter();
        try (CsvDataLogWriter w = new CsvDataLogWriter(sw)) {
            w.writeFrame(frame(0.0, 1.0, 12.0));
            w.writeFrame(frame(0.02, 2.0, 11.9));
        }
        String[] lines = sw.toString().trim().split("\\R");
        assertEquals(String.join(",", LogFrame.CHANNELS), lines[0]);
        assertEquals(3, lines.length);  // header + 2 rows
        int poseX = LogFrame.channelIndex("poseX");
        assertEquals(2.0, Double.parseDouble(lines[2].split(",")[poseX]), EPS);
    }

    // --- WPILOG round-trip ---------------------------------------------------

    @Test
    void wpilogRoundTripsChannelsAndValues() throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        try (WpilogWriter w = new WpilogWriter(out)) {
            w.writeFrame(frame(0.0, 1.0, 12.0));
            w.writeFrame(frame(0.02, 2.0, 11.9));
        }
        DecodedLog log = decode(out.toByteArray());

        assertArrayEquals(LogFrame.CHANNELS, log.channelNames());
        // ts in microseconds
        assertEquals(1.0, log.value(0L, "poseX"), EPS);
        assertEquals(12.0, log.value(0L, "batteryVoltage"), EPS);
        assertEquals(2.0, log.value(20_000L, "poseX"), EPS);
        assertEquals(11.9, log.value(20_000L, "batteryVoltage"), EPS);
    }

    // --- B -> D pairing: a short sim run logs through the shared schema -------

    @Test
    void simulatedRunLogsToWpilogAndCsv() throws IOException {
        SimulatedRobot r = SimulatedRobot.create(new FollowerConstants());
        Path path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(20, 0, 0)));
        path.setConstantHeadingInterpolation(0);
        SimulationResult result = SimulationLoop.fixedWindow(0.4, 0.005, 0.05)
                .followPath(r.getFollower(), r.getPlant(), r.getLocalizer(), r.getDrivetrain(), path, new Pose(0, 0, 0));

        ByteArrayOutputStream wpilog = new ByteArrayOutputStream();
        try (WpilogWriter w = new WpilogWriter(wpilog)) {
            LogFrames.write(result, w);
        }
        StringWriter csv = new StringWriter();
        try (CsvDataLogWriter w = new CsvDataLogWriter(csv)) {
            LogFrames.write(result, w);
        }

        int frames = result.getSamples().size();
        assertTrue(frames > 5, "sim should produce several frames");
        assertEquals(frames + 1, csv.toString().trim().split("\\R").length); // + header
        DecodedLog log = decode(wpilog.toByteArray());
        assertArrayEquals(LogFrame.CHANNELS, log.channelNames());
        assertEquals(frames, log.timestamps().size());
    }

    // ---------------------------------------------------------------- decoder

    /** Minimal WPILOG reader used only to verify the writer. */
    private static DecodedLog decode(byte[] data) throws IOException {
        ByteArrayInputStream in = new ByteArrayInputStream(data);
        byte[] magic = new byte[6];
        assertEquals(6, in.read(magic));
        assertEquals("WPILOG", new String(magic, StandardCharsets.UTF_8));
        assertEquals(0x00, in.read());
        assertEquals(0x01, in.read());
        long extraLen = readU32(in);
        in.skip(extraLen);

        Map<Integer, String> channelById = new HashMap<>();
        TreeMap<Long, Map<String, Double>> byTimestamp = new TreeMap<>();

        int bitfield;
        while ((bitfield = in.read()) != -1) {
            int idLen = (bitfield & 0x3) + 1;
            int sizeLen = ((bitfield >> 2) & 0x3) + 1;
            int tsLen = ((bitfield >> 4) & 0x7) + 1;
            int entryId = (int) readLE(in, idLen);
            int size = (int) readLE(in, sizeLen);
            long ts = readLE(in, tsLen);
            byte[] payload = new byte[size];
            assertEquals(size, in.read(payload));

            if (entryId == 0) {                       // control record
                assertEquals(0, payload[0]);          // Start
                int pos = 1;
                int definedId = (int) leFromBytes(payload, pos, 4); pos += 4;
                int nameLen = (int) leFromBytes(payload, pos, 4); pos += 4;
                String name = new String(payload, pos, nameLen, StandardCharsets.UTF_8); pos += nameLen;
                channelById.put(definedId, name);
            } else {
                double value = Double.longBitsToDouble(leFromBytes(payload, 0, 8));
                byTimestamp.computeIfAbsent(ts, k -> new HashMap<>())
                        .put(channelById.get(entryId), value);
            }
        }
        return new DecodedLog(channelById, byTimestamp);
    }

    private static long readU32(ByteArrayInputStream in) {
        return readLE(in, 4);
    }

    private static long readLE(ByteArrayInputStream in, int n) {
        long value = 0;
        for (int i = 0; i < n; i++) {
            value |= ((long) (in.read() & 0xFF)) << (8 * i);
        }
        return value;
    }

    private static long leFromBytes(byte[] b, int offset, int n) {
        long value = 0;
        for (int i = 0; i < n; i++) {
            value |= ((long) (b[offset + i] & 0xFF)) << (8 * i);
        }
        return value;
    }

    private static final class DecodedLog {
        private final Map<Integer, String> channelById;
        private final TreeMap<Long, Map<String, Double>> byTimestamp;

        DecodedLog(Map<Integer, String> channelById, TreeMap<Long, Map<String, Double>> byTimestamp) {
            this.channelById = channelById;
            this.byTimestamp = byTimestamp;
        }

        String[] channelNames() {
            String[] names = new String[channelById.size()];
            for (Map.Entry<Integer, String> e : channelById.entrySet()) {
                names[e.getKey() - 1] = e.getValue();   // entry ids are 1-based, in order
            }
            return names;
        }

        double value(long ts, String channel) {
            return byTimestamp.get(ts).get(channel);
        }

        TreeMap<Long, Map<String, Double>> timestamps() {
            return byTimestamp;
        }
    }
}
