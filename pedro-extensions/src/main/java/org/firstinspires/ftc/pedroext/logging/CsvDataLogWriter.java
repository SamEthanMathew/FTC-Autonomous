package org.firstinspires.ftc.pedroext.logging;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * Writes {@link LogFrame}s as CSV with a header row of {@link LogFrame#CHANNELS}.
 * This is the documented, human-readable schema; because sim and robot emit the
 * same columns, two runs diff directly in a spreadsheet or AdvantageScope's CSV
 * import.
 */
public final class CsvDataLogWriter implements DataLogWriter {

    private final BufferedWriter writer;
    private boolean headerWritten;

    public CsvDataLogWriter(Writer writer) {
        this.writer = new BufferedWriter(writer);
    }

    @Override
    public void writeFrame(LogFrame frame) throws IOException {
        if (!headerWritten) {
            writer.write(String.join(",", LogFrame.CHANNELS));
            writer.newLine();
            headerWritten = true;
        }
        double[] values = frame.values();
        StringBuilder row = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                row.append(',');
            }
            row.append(values[i]);
        }
        writer.write(row.toString());
        writer.newLine();
    }

    @Override
    public void close() throws IOException {
        writer.flush();
        writer.close();
    }
}
