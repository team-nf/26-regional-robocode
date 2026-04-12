package frc.robot.Subsystems.Hopper.Hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.TelemetryConstants;

public class HopperRealHardware implements HopperHardware {

    protected TalonFX hopperMotor;
    
    private TalonFX hopperMotor2;


    private TalonFXConfiguration hopperMotorConfig;

    private final VelocityVoltage hopperVelocityControl;

    private double hopperVelocity = 0;
    private double hopperMotorVelocity = 0;
    private double hopperVoltage = 0;
    private double hopperCurrent = 0;
    private double hopperReference = 0.0;
    private double hopperError = 0.0;
    private double testHopperGoal = 0;

    public HopperRealHardware() {
        hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID);

        hopperMotor2 = new TalonFX(HopperConstants.HOPPER_MOTOR_2_ID);

        setHopperMotorConfig(HopperConstants.HOPPER_MOTOR_CONFIG);
        updateMotorConfig();

        hopperVelocityControl = HopperConstants.HOPPER_VELOCITY_CONTROL.clone();

        hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateMotorConfig() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hopperMotor.getConfigurator().apply(hopperMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hopperMotor2.getConfigurator().apply(hopperMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void setHopperMotorConfig(TalonFXConfiguration config) {
        hopperMotorConfig = config.clone();
    }

    @Override
    public TalonFX getHopperMotor() {
        return hopperMotor;
    }

    @Override
    public double getHopperVelocity() {
        return hopperMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void hopperSetPower(double power) {
        hopperMotor.set(power);
    }

    @Override
    public void setHopperSpeed(double velocity) {
        hopperMotor.setControl(
            hopperVelocityControl.withVelocity(velocity * HopperConstants.HOPPER_GEAR_REDUCTION)
        );
    }

    @Override
    public void hopperStop() {
        hopperMotor.set(0.0);
    }

    @Override
    public void testHopper() {
        setHopperSpeed(testHopperGoal);
    }
    
    @Override
    public void updateVariables() {
        hopperVelocity = hopperMotor.getVelocity().getValue().div(HopperConstants.HOPPER_GEAR_REDUCTION).in(RotationsPerSecond);
        //hopperMotorVelocity = hopperMotor.getVelocity().getValue();
        //hopperVoltage = hopperMotor.getMotorVoltage().getValue();
        //hopperCurrent = hopperMotor.getStatorCurrent().getValue();
        //hopperReference = hopperMotor.getClosedLoopReference().getValue();
        hopperError = hopperMotor.getClosedLoopError().getValue();
    }

    @Override
    public void update() {
        updateVariables();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HopperHardware");

        builder.addDoubleProperty("Hopper Velocity (RPM)", () -> TelemetryConstants.roundTelemetry(hopperVelocity*(60)), null);
        builder.addDoubleProperty("Hopper Motor Velocity (RPM)", () -> TelemetryConstants.roundTelemetry(hopperMotorVelocity*(60)), null);
        builder.addDoubleProperty("Hopper Voltage (V)", () -> TelemetryConstants.roundTelemetry(hopperVoltage), null);
        builder.addDoubleProperty("Hopper Current (A)", () -> TelemetryConstants.roundTelemetry(hopperCurrent), null);
        builder.addDoubleProperty("Hopper Reference", () -> TelemetryConstants.roundTelemetry(hopperReference), null);
        builder.addDoubleProperty("Hopper Error", () -> TelemetryConstants.roundTelemetry(hopperError), null);
            
            builder.addDoubleProperty("Test Hopper Goal (RPM)", () -> testHopperGoal*(60),
                (val) -> testHopperGoal = (val / 60));

        builder.addDoubleProperty("Hopper KS", () -> hopperMotorConfig.Slot0.kS, 
            (val) -> {
                hopperMotorConfig.Slot0.kS = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Hopper KV", () -> hopperMotorConfig.Slot0.kV, 
            (val) -> {
                hopperMotorConfig.Slot0.kV = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Hopper KP", () -> hopperMotorConfig.Slot0.kP, 
            (val) -> {
                hopperMotorConfig.Slot0.kP = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Hopper KI", () -> hopperMotorConfig.Slot0.kI, 
            (val) -> {
                hopperMotorConfig.Slot0.kI = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Hopper KD", () -> hopperMotorConfig.Slot0.kD,
            (val) -> {
                hopperMotorConfig.Slot0.kD = val;
                updateMotorConfig();
            });
    }
}
