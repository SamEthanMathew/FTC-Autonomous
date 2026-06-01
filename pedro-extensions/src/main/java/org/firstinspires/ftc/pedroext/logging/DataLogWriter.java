package org.firstinspires.ftc.pedroext.logging;

import java.io.Closeable;
import java.io.IOException;

/**
 * Sink for {@link LogFrame}s. Implemented as CSV (human/spreadsheet friendly) and
 * WPILOG (binary, directly loadable in AdvantageScope). The header is written
 * lazily on the first frame.
 */
public interface DataLogWriter extends Closeable {

    void writeFrame(LogFrame frame) throws IOException;
}
