package frc.lib2960.telemetry;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Class to make a measure sendable
 */
public class SendableMeasure<U extends Unit> implements Sendable {

    private final Measure<U> measure;

    /**
     * Constructor
     * 
     * @param measure The measure to make sendable
     */
    public SendableMeasure(Measure<U> measure) {
        this.measure = measure;
    }

    /**
     * Implements sendable initialization
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(String.format("Measure: %s", measure.unit().name()));
        builder.addStringProperty("Value", () -> measure.toShortString(), null);
    }
}
