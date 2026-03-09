package frc.robot.Subsystems.Shooter.Hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class ShooterRealHardware implements ShooterHardware {

    protected final TalonFX firstShootMotor;
    protected final TalonFX secondShootMotor;
    protected final TalonFX thirdShootMotor;
    protected final TalonFX fourthShootMotor;
    protected final TalonFX hoodMotor;

    private TalonFXConfiguration shootMotorConfigL;
    private TalonFXConfiguration shootMotorConfigR;

    private TalonFXConfiguration hoodMotorConfig;

    private final VelocityVoltage shootVelocityControlL;
    private final VelocityVoltage shootVelocityControlR;
    private final PositionVoltage hoodPositionControl;

    private double flywheelVelocityLeft = 0;
    private double flywheelVelocityRight = 0;

    private double shootErrorLeft = 0.0;
    private double shootErrorRight = 0.0;

    private double hoodPosition = 0;
    private double hoodError = 0.0;

    private double testFlywheelGoal= 35;
    private double testHoodGoal = 20.0/360.0;

    public ShooterRealHardware() {
        firstShootMotor = new TalonFX(ShooterConstants.FIRST_SHOOTER_MOTOR_ID);
        secondShootMotor = new TalonFX(ShooterConstants.SECOND_SHOOTER_MOTOR_ID);
        thirdShootMotor = new TalonFX(ShooterConstants.THIRD_SHOOTER_MOTOR_ID);
        fourthShootMotor = new TalonFX(ShooterConstants.FOURTH_SHOOTER_MOTOR_ID);

        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

        setShootMotorConfig(ShooterConstants.SHOOTER_CONFIG);
        setHoodMotorConfig(ShooterConstants.HOOD_CONFIG);
        updateMotorConfigs();

        secondShootMotor.setControl(new Follower(firstShootMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        fourthShootMotor.setControl(new Follower(thirdShootMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        shootVelocityControlL = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();
        shootVelocityControlR = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();

        hoodPositionControl = ShooterConstants.HOOD_POSITION_CONTROL.clone();
    }

    @Override
    public void updateMotorConfigs() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = firstShootMotor.getConfigurator().apply(shootMotorConfigL);
            if (status.isOK()) break;
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = thirdShootMotor.getConfigurator().apply(shootMotorConfigR);
            if (status.isOK()) break;
        }
        
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hoodMotor.getConfigurator().apply(hoodMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void setShootMotorConfig(TalonFXConfiguration config) {
        shootMotorConfigR = config.clone();
        shootMotorConfigL = config.clone().withMotorOutput(config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    }

    public void setHoodMotorConfig(TalonFXConfiguration config) {
        hoodMotorConfig = config.clone();
    }

    @Override
    public TalonFX getFirstShootMotor() {
        return firstShootMotor;
    }

    @Override
    public TalonFX getSecondShootMotor() {
        return secondShootMotor;
    }

    @Override
    public TalonFX getThirdShootMotor() {
        return thirdShootMotor;
    }

    @Override
    public TalonFX getFourthShootMotor() {
        return fourthShootMotor;
    }

    @Override
    public double getFlywheelVelocityL() {
        // Returns the velocity of the first shooter motor in rotations per second
        return flywheelVelocityLeft;
    }

    @Override
    public double getFlywheelVelocityR() {
        return flywheelVelocityRight;
    }

    @Override
    public void flywheelSetPower(double power) {
        // Sets the power for the first shooter motor (others are followers)
        firstShootMotor.set(power);
        thirdShootMotor.set(power);
    }

    @Override
    public void setFlywheelSpeed(double velocity) {
        // Set velocity in rotations per second
        firstShootMotor.setControl(
            shootVelocityControlL.withVelocity(velocity*(ShooterConstants.FLYWHEEL_GEAR_REDUCTION))
        );
        thirdShootMotor.setControl(
            shootVelocityControlR.withVelocity(velocity*(ShooterConstants.FLYWHEEL_GEAR_REDUCTION))
        );
    }

    @Override
    public void flywheelStop() {
        firstShootMotor.set(0.0);
        thirdShootMotor.set(0.0);
    }

    @Override
    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    @Override
    public double getHoodPosition() {
        // Example: Returns the hood position in rotations
        return hoodPosition;
    }

    @Override
    public void setHoodAngle(double hoodAngle) {
        // Set hood position in rotations

        hoodAngle = Math.max(ShooterConstants.MIN_HOOD_ANGLE.in(Rotations),
            Math.min(hoodAngle, ShooterConstants.MAX_HOOD_ANGLE.in(Rotations)));

        hoodMotor.setControl(
            hoodPositionControl.withPosition(hoodAngle*(ShooterConstants.HOOD_GEAR_REDUCTION))
        );
    }

    @Override
    public void hoodZero() {
        setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE.in(Rotations));
    }

    @Override
    public void setShooter(double velocity, double hoodAngle) {
        setFlywheelSpeed(velocity);
        setHoodAngle(hoodAngle);
    }

    @Override
    public void testShooter() {
        setShooter(testFlywheelGoal, testHoodGoal);
    }

    
    @Override
    public void updateVariables() {
        flywheelVelocityLeft = firstShootMotor.getVelocity().getValue().div(ShooterConstants.FLYWHEEL_GEAR_REDUCTION).in(RotationsPerSecond);
        flywheelVelocityRight = thirdShootMotor.getVelocity().getValue().div(ShooterConstants.FLYWHEEL_GEAR_REDUCTION).in(RotationsPerSecond);

        shootErrorLeft = firstShootMotor.getClosedLoopError().getValue();
        shootErrorRight = thirdShootMotor.getClosedLoopError().getValue();

        hoodPosition = hoodMotor.getPosition().getValue().div(ShooterConstants.HOOD_GEAR_REDUCTION).in(Rotations);
        hoodError = hoodMotor.getClosedLoopError().getValue();
    }

    @Override
    public void update() {
        updateVariables();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    builder.addDoubleProperty("Flywheel Velocity Left", () -> TelemetryConstants.roundTelemetry(flywheelVelocityLeft*60), null);
    builder.addDoubleProperty("Flywheel Velocity Right", () -> TelemetryConstants.roundTelemetry(flywheelVelocityRight*60), null);
    builder.addDoubleProperty("Shoot Error Left", () -> TelemetryConstants.roundTelemetry(shootErrorLeft), null);
    builder.addDoubleProperty("Shoot Error Right", () -> TelemetryConstants.roundTelemetry(shootErrorRight), null);
    builder.addDoubleProperty("Hood Position", () -> TelemetryConstants.roundTelemetry(hoodPosition*360), null);
    builder.addDoubleProperty("Hood Error", () -> TelemetryConstants.roundTelemetry(hoodError*360), null);
    }

    @Override
    public double getTestFlywheelGoal()
    {
        return testFlywheelGoal;
    }

    @Override
    public double getTestHoodGoal()
    {
        return testHoodGoal;
    }
}