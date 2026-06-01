package org.firstinspires.ftc.pedroext.logging;

import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

/**
 * Writes {@link LogFrame}s as a WPILOG (WPILib data log) binary stream, which
 * AdvantageScope loads directly. Each {@link LogFrame#CHANNELS} entry becomes a
 * {@code double} channel; one data record per channel per frame.
 *
 * <p>Encoding (WPILOG 1.0): a {@code "WPILOG"} magic + version + extra-header,
 * then records. For simplicity every record uses the maximum field widths
 * (4-byte entry id, 4-byte payload size, 8-byte timestamp), so the record header
 * bitfield byte is always {@code 0x7F}. Channel definitions are emitted as
 * control "Start" records (entry id 0); samples as data records. All integers are
 * little-endian; timestamps are microseconds.
 */
public final class WpilogWriter implements DataLogWriter {

    private static final byte[] MAGIC = {'W', 'P', 'I', 'L', 'O', 'G'};
    private static final int RECORD_HEADER_BITFIELD = 0x7F;
    private static final int CONTROL_ENTRY_ID = 0;
    private static final int CONTROL_START = 0;

    private final OutputStream out;
    private boolean started;

    public WpilogWriter(OutputStream out) {
        this.out = new BufferedOutputStream(out);
    }

    private void writeFileHeaderAndChannels() throws IOException {
        out.write(MAGIC);
        out.write(new byte[]{0x00, 0x01});   // version 1.0 (LE: minor=0, major=1)
        writeU32(out, 0);                     // extra header length

        for (int i = 0; i < LogFrame.CHANNELS.length; i++) {
            int entryId = i + 1;              // 0 is reserved for control records
            ByteArrayOutputStream payload = new ByteArrayOutputStream();
            payload.write(CONTROL_START);
            writeU32(payload, entryId);
            writeString(payload, LogFrame.CHANNELS[i]);
            writeString(payload, "double");
            writeString(payload, "");         // metadata
            writeRecord(CONTROL_ENTRY_ID, payload.toByteArray(), 0L);
        }
        started = true;
    }

    @Override
    public void writeFrame(LogFrame frame) throws IOException {
        if (!started) {
            writeFileHeaderAndChannels();
        }
        long timestampMicros = Math.round(frame.time() * 1_000_000.0);
        double[] values = frame.values();
        for (int i = 0; i < values.length; i++) {
            writeRecord(i + 1, doubleToLeBytes(values[i]), timestampMicros);
        }
    }

    private void writeRecord(int entryId, byte[] payload, long timestampMicros) throws IOException {
        out.write(RECORD_HEADER_BITFIELD);
        writeU32(out, entryId);
        writeU32(out, payload.length);
        writeU64(out, timestampMicros);
        out.write(payload);
    }

    @Override
    public void close() throws IOException {
        if (!started) {
            writeFileHeaderAndChannels();
        }
        out.flush();
        out.close();
    }

    private static void writeString(OutputStream os, String s) throws IOException {
        byte[] bytes = s.getBytes(StandardCharsets.UTF_8);
        writeU32(os, bytes.length);
        os.write(bytes);
    }

    private static void writeU32(OutputStream os, int value) throws IOException {
        os.write(value & 0xFF);
        os.write((value >>> 8) & 0xFF);
        os.write((value >>> 16) & 0xFF);
        os.write((value >>> 24) & 0xFF);
    }

    private static void writeU64(OutputStream os, long value) throws IOException {
        for (int i = 0; i < 8; i++) {
            os.write((int) ((value >>> (8 * i)) & 0xFF));
        }
    }

    private static byte[] doubleToLeBytes(double value) {
        long bits = Double.doubleToLongBits(value);
        byte[] bytes = new byte[8];
        for (int i = 0; i < 8; i++) {
            bytes[i] = (byte) ((bits >>> (8 * i)) & 0xFF);
        }
        return bytes;
    }
}
