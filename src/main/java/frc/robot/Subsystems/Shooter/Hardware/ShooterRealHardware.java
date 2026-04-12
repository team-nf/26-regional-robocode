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

    private double shootVelocityLeft = 0;
    private double shootVelocityRight = 0;

    private double shootVoltageLeft = 0;
    private double shootVoltageRight = 0;

    private double shootCurrentLeft = 0;
    private double shootCurrentRight = 0;

    private double shootErrorLeft = 0.0;
    private double shootErrorRight = 0.0;

    private double shootReferenceLeft = 0.0;
    private double shootReferenceRight = 0.0;


    private double hoodPosition = 0;
    private double hoodMotorPosition = 0;
    private double hoodVoltage = 0;
    private double hoodCurrent = 0;
    private double hoodReference = 0.0;
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

        //shootVelocityLeft = firstShootMotor.getVelocity().getValue();
        //shootVelocityRight = thirdShootMotor.getVelocity().getValue();

        //shootVoltageLeft = firstShootMotor.getMotorVoltage().getValue();
        //shootVoltageRight = thirdShootMotor.getMotorVoltage().getValue();

        //shootCurrentLeft = firstShootMotor.getStatorCurrent().getValue();
        //shootCurrentRight = thirdShootMotor.getStatorCurrent().getValue();

        shootErrorLeft = firstShootMotor.getClosedLoopError().getValue();
        shootErrorRight = thirdShootMotor.getClosedLoopError().getValue();

        //shootReferenceLeft = firstShootMotor.getClosedLoopReference().getValue();
        //shootReferenceRight = thirdShootMotor.getClosedLoopReference().getValue();

        hoodPosition = hoodMotor.getPosition().getValue().div(ShooterConstants.HOOD_GEAR_REDUCTION).in(Rotations);
        //hoodMotorPosition = hoodMotor.getPosition().getValue();
        //hoodVoltage = hoodMotor.getMotorVoltage().getValue();
        //hoodCurrent = hoodMotor.getStatorCurrent().getValue();
        //hoodReference = hoodMotor.getClosedLoopReference().getValue();
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
    builder.addDoubleProperty("Shoot Motor Velocity Left", () -> TelemetryConstants.roundTelemetry(shootVelocityLeft*(60)), null);
    builder.addDoubleProperty("Shoot Motor Velocity Right", () -> TelemetryConstants.roundTelemetry(shootVelocityRight*(60)), null);
    builder.addDoubleProperty("Shoot Voltage Left", () -> TelemetryConstants.roundTelemetry(shootVoltageLeft), null);
    builder.addDoubleProperty("Shoot Voltage Right", () -> TelemetryConstants.roundTelemetry(shootVoltageRight), null);
    builder.addDoubleProperty("Shoot Current Left", () -> TelemetryConstants.roundTelemetry(shootCurrentLeft), null);
    builder.addDoubleProperty("Shoot Current Right", () -> TelemetryConstants.roundTelemetry(shootCurrentRight), null);
    builder.addDoubleProperty("Shoot Reference Left", () -> TelemetryConstants.roundTelemetry(shootReferenceLeft), null);
    builder.addDoubleProperty("Shoot Reference Right", () -> TelemetryConstants.roundTelemetry(shootReferenceRight), null);
    builder.addDoubleProperty("Shoot Error Left", () -> TelemetryConstants.roundTelemetry(shootErrorLeft), null);
    builder.addDoubleProperty("Shoot Error Right", () -> TelemetryConstants.roundTelemetry(shootErrorRight), null);
    builder.addDoubleProperty("Hood Position", () -> TelemetryConstants.roundTelemetry(hoodPosition*360), null);
    builder.addDoubleProperty("Hood Motor Position", () -> TelemetryConstants.roundTelemetry(hoodMotorPosition*360), null);
    builder.addDoubleProperty("Hood Voltage", () -> TelemetryConstants.roundTelemetry(hoodVoltage), null);
    builder.addDoubleProperty("Hood Current", () -> TelemetryConstants.roundTelemetry(hoodCurrent), null);
    builder.addDoubleProperty("Hood Reference", () -> TelemetryConstants.roundTelemetry(hoodReference), null);
    builder.addDoubleProperty("Hood Error", () -> TelemetryConstants.roundTelemetry(hoodError*360), null);
/*
        builder.addDoubleProperty("Test Flywheel Goal", () -> testFlywheelGoal*(60), (val) -> {
            testFlywheelGoal = (val/60);
        });

        builder.addDoubleProperty("Test Hood Goal", () -> testHoodGoal*360, (val) -> {
            testHoodGoal = val/360;
        });

        builder.addDoubleProperty("Shoot KS", () -> shootMotorConfigR.Slot0.kS, (val) -> {
            shootMotorConfigR.Slot0.kS = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KV", () -> shootMotorConfigR.Slot0.kV, (val) -> {
            shootMotorConfigR.Slot0.kV = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KP", () -> shootMotorConfigR.Slot0.kP, (val) -> {
            shootMotorConfigR.Slot0.kP = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KI", () -> shootMotorConfigR.Slot0.kI, (val) -> {
            shootMotorConfigR.Slot0.kI = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Shoot KD", () -> shootMotorConfigR.Slot0.kD, (val) -> {
            shootMotorConfigR.Slot0.kD = val;
            updateMotorConfigs();
        });

        builder.addDoubleProperty("Hood KS", () -> hoodMotorConfig.Slot0.kS, (val) -> {
            hoodMotorConfig.Slot0.kS = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KV", () -> hoodMotorConfig.Slot0.kV, (val) -> {
            hoodMotorConfig.Slot0.kV = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KP", () -> hoodMotorConfig.Slot0.kP, (val) -> {
            hoodMotorConfig.Slot0.kP = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KI", () -> hoodMotorConfig.Slot0.kI, (val) -> {
            hoodMotorConfig.Slot0.kI = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KD", () -> hoodMotorConfig.Slot0.kD, (val) -> {
            hoodMotorConfig.Slot0.kD = val;
            updateMotorConfigs();
        });
        builder.addDoubleProperty("Hood KG", () -> hoodMotorConfig.Slot0.kG, (val) -> {
            hoodMotorConfig.Slot0.kG = val;
            updateMotorConfigs();
        });*/
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