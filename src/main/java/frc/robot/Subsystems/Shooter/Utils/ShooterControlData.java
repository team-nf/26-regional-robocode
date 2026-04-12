package frc.robot.Subsystems.Shooter.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.States.ShooterStates.*;
import frc.robot.Constants.TelemetryConstants;

public class ShooterControlData implements Sendable{

    public FlywheelState flywheelStateL = FlywheelState.ZERO;
    public FlywheelState flywheelStateR = FlywheelState.ZERO;

    public HoodState hoodState = HoodState.ZERO;
    public ShooterControlState shooterControlState = ShooterControlState.ZERO;
    public ShooterControlState prevShooterControlState = ShooterControlState.ZERO;

    public double flywheelGoalVelocity = 0.0;
    public double hoodGoalAngle = 0.0;

    public double flywheelVelocityL = 0.0;
    public double flywheelVelocityR = 0.0;
    public double hoodAngle = 0.0;

    public double flywheelErrorL = 0.0;
    public double flywheelErrorR = 0.0;
    public double hoodError = 0.0;
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterControlData");
        builder.addStringProperty("ShooterControlState", () -> shooterControlState.toString(), null);
        builder.addStringProperty("PrevShooterControlState", () -> prevShooterControlState.toString(), null);
        builder.addStringProperty("FlywheelState-L", () -> flywheelStateL.toString(), null);
        builder.addStringProperty("FlywheelState-R", () -> flywheelStateR.toString(), null);
        builder.addStringProperty("HoodState", () -> hoodState.toString(), null);
        builder.addDoubleProperty("FlywheelGoalRPM", () -> TelemetryConstants.roundTelemetry(flywheelGoalVelocity * 60), null);
        builder.addDoubleProperty("HoodGoalAngleDegrees", () -> TelemetryConstants.roundTelemetry(hoodGoalAngle * 360), null);
        builder.addDoubleProperty("FlywheelRPM-L", () -> TelemetryConstants.roundTelemetry(flywheelVelocityL * 60), null);
        builder.addDoubleProperty("FlywheelRPM-R", () -> TelemetryConstants.roundTelemetry(flywheelVelocityR * 60), null);
        builder.addDoubleProperty("HoodAngleDegrees", () -> TelemetryConstants.roundTelemetry(hoodAngle * 360), null);
        builder.addDoubleProperty("FlywheelErrorRPM-L", () -> TelemetryConstants.roundTelemetry(flywheelErrorL * 60), null);
        builder.addDoubleProperty("FlywheelErrorRPM-R", () -> TelemetryConstants.roundTelemetry(flywheelErrorR * 60), null);
        builder.addDoubleProperty("HoodErrorDegrees", () -> TelemetryConstants.roundTelemetry(hoodError * 360), null);
    }

}
