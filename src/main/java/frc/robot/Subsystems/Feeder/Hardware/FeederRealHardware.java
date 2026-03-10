package frc.robot.Subsystems.Feeder.Hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.TelemetryConstants;

public class FeederRealHardware implements FeederHardware {
    
    protected TalonFX feederMotor;

    private TalonFXConfiguration feederMotorConfig;

    private final VelocityVoltage feederVelocityControl;

    private double feederVelocity = 0;
    private double feederError = 0.0;
    private double testFeederGoal = 0;

    public FeederRealHardware() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID);

        setFeederMotorConfig(FeederConstants.FEEDER_MOTOR_CONFIG);
        updateMotorConfig();

        feederVelocityControl = FeederConstants.FEEDER_VELOCITY_CONTROL.clone();
    }

    @Override
    public void updateMotorConfig() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = feederMotor.getConfigurator().apply(feederMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void setFeederMotorConfig(TalonFXConfiguration config) {
        feederMotorConfig = config.clone();
    }

    @Override
    public TalonFX getFeederMotor() {
        return feederMotor;
    }

    @Override
    public double getFeederVelocity() {
        return feederVelocity;
    }

    @Override
    public void feederSetPower(double power) {
        feederMotor.set(power);
    }

    @Override
    public void setFeederSpeed(double velocity) {
        feederMotor.setControl(
            feederVelocityControl.withVelocity(velocity * FeederConstants.FEEDER_GEAR_REDUCTION)
        );
    }

    @Override
    public void feederStop() {
        feederMotor.set(0.0);
    }

    @Override
    public void testFeeder() {
        setFeederSpeed(testFeederGoal);
    }
    
    @Override
    public void updateVariables() {
        feederVelocity = feederMotor.getVelocity().getValue().div(FeederConstants.FEEDER_GEAR_REDUCTION).in(RotationsPerSecond);
        feederError = feederMotor.getClosedLoopError().getValue();
    }

    @Override
    public void update() {
        updateVariables();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FeederHardware");

    builder.addDoubleProperty("Feeder Velocity (RPM)", () -> TelemetryConstants.roundTelemetry(feederVelocity * 60), null);
    builder.addDoubleProperty("Feeder Error", () -> TelemetryConstants.roundTelemetry(feederError), null);
    
        builder.addDoubleProperty("Test Feeder Goal (RPM)", () -> testFeederGoal * 60,
            (val) -> testFeederGoal = val / 60);
        
        builder.addDoubleProperty("Feeder KS", () -> feederMotorConfig.Slot0.kS, 
            (val) -> {
                feederMotorConfig.Slot0.kS = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Feeder KV", () -> feederMotorConfig.Slot0.kV, 
            (val) -> {
                feederMotorConfig.Slot0.kV = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Feeder KP", () -> feederMotorConfig.Slot0.kP, 
            (val) -> {
                feederMotorConfig.Slot0.kP = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Feeder KI", () -> feederMotorConfig.Slot0.kI, 
            (val) -> {
                feederMotorConfig.Slot0.kI = val;
                updateMotorConfig();
            });

        builder.addDoubleProperty("Feeder KD", () -> feederMotorConfig.Slot0.kD,
            (val) -> {
                feederMotorConfig.Slot0.kD = val;
                updateMotorConfig();
            });
    }

}