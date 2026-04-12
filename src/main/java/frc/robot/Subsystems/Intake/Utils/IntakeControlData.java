package frc.robot.Subsystems.Intake.Utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Constants.States.IntakeStates.IntakePositionState;
import frc.robot.Constants.States.IntakeStates.IntakeRollerState;
import frc.robot.Constants.TelemetryConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

public class IntakeControlData implements Sendable {

    public IntakeRollerState intakeRollerState = IntakeRollerState.ZERO;
    public IntakeControlState intakeControlState = IntakeControlState.CLOSE;
    public IntakeControlState prevIntakeControlState = IntakeControlState.CLOSE;
    public IntakePositionState intakePositionState = IntakePositionState.RETRACTED;

    public double intakeGoalVelocity = 0;
    public double intakeVelocity = 0;
    public double intakeError = 0;

    public double intakeArmAngle = 0;
    public double intakeGoalArmAngle = 0;
    public double intakeArmError = 0;

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("IntakeControlData");
        builder.addStringProperty("IntakeControlState", () -> intakeControlState.toString(), null);
        builder.addStringProperty("IntakePrevControlState", () -> prevIntakeControlState.toString(), null);
        builder.addStringProperty("IntakeRollerState", () -> intakeRollerState.toString(), null);
        builder.addStringProperty("IntakePositionState", () -> intakePositionState.toString(), null);
        builder.addDoubleProperty("IntakeGoalRPM", () -> TelemetryConstants.roundTelemetry(intakeGoalVelocity * 60), null);
        builder.addDoubleProperty("IntakeRPM", () -> TelemetryConstants.roundTelemetry(intakeVelocity * 60), null);
        builder.addDoubleProperty("IntakeErrorRPM", () -> TelemetryConstants.roundTelemetry(intakeError * 60), null);
        builder.addDoubleProperty("IntakeArmAngleDegrees", () -> TelemetryConstants.roundTelemetry(intakeArmAngle * 360), null);
        builder.addDoubleProperty("IntakeGoalArmAngleDegrees", () -> TelemetryConstants.roundTelemetry(intakeGoalArmAngle * 360), null);
        builder.addDoubleProperty("IntakeArmErrorDegrees", () -> TelemetryConstants.roundTelemetry(intakeArmError * 360), null);
    }
}
