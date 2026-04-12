package frc.robot.Subsystems.Hopper.Utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Constants.States.HopperStates.HopperRollerState;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants.TelemetryConstants;

public class HopperControlData implements Sendable {

    public HopperRollerState hopperRollerState = HopperRollerState.ZERO;
    public HopperControlState hopperControlState = HopperControlState.ZERO;
    public HopperControlState prevHopperControlState = HopperControlState.ZERO;

    public double hopperGoalVelocity = 0;
    public double hopperVelocity = 0;
    public double hopperError = 0;

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HopperControlData");
        builder.addStringProperty("HopperControlState", () -> hopperControlState.toString(), null);
        builder.addStringProperty("HopperPrevControlState", () -> prevHopperControlState.toString(), null);
        builder.addStringProperty("HopperRollerState", () -> hopperRollerState.toString(), null);
        builder.addDoubleProperty("HopperGoalRPM", () -> TelemetryConstants.roundTelemetry(hopperGoalVelocity * 60), null);
        builder.addDoubleProperty("HopperRPM", () -> TelemetryConstants.roundTelemetry(hopperVelocity * 60), null);
        builder.addDoubleProperty("HopperErrorRPM", () -> TelemetryConstants.roundTelemetry(hopperError * 60), null);
    }
}
