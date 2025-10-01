package frc.lib2960.telemetry;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableElevatorFeedForward implements Sendable {
    private final ElevatorFeedforward ff;

    public SendableElevatorFeedForward(ElevatorFeedforward ff) {
        this.ff = ff;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Feed Forward");
        builder.addDoubleProperty("kS", ff::getKs, ff::setKs);
        builder.addDoubleProperty("kV", ff::getKv, ff::setKv);
        builder.addDoubleProperty("kG", ff::getKg, ff::setKg);
        builder.addDoubleProperty("kA", ff::getKa, ff::setKa);
    }
    
}
