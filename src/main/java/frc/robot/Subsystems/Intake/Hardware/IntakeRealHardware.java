package frc.robot.Subsystems.Intake.Hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TelemetryConstants;

public class IntakeRealHardware implements IntakeHardware {

    protected TalonFX intakeMotor;
    protected TalonFX intakeArmMotor;

    private TalonFXConfiguration intakeMotorConfig;
    private TalonFXConfiguration intakeArmMotorConfig;

    private final VelocityVoltage intakeVelocityControl;
    private final PositionVoltage intakeArmPositionControl;

    private double intakeVelocity = 0;
    private double testIntakeGoal = 0;

    private double intakeArmPosition = IntakeConstants.INTAKE_ARM_START_ANGLE.in(Rotation);
    private double testIntakeArmGoal = 0;

    public IntakeRealHardware() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

        // No separate arm motor configured in constants by default. Use the intake motor as a
        // fallback for arm telemetry/control when a dedicated motor isn't supplied.
        intakeArmMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_ID);

        if(!Utils.isSimulation())
        {
        intakeArmMotor.setPosition(IntakeConstants.INTAKE_ARM_START_ANGLE.times(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION));
        }
        setIntakeMotorConfig(IntakeConstants.INTAKE_MOTOR_CONFIG);
        setIntakeArmMotorConfig(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);

        updateMotorConfig();

        intakeVelocityControl = IntakeConstants.INTAKE_VELOCITY_CONTROL.clone();
        // Use a sensible default PositionVoltage for arm control if none provided
        intakeArmPositionControl = IntakeConstants.INTAKE_ARM_POSITION_CONTROL.clone();
    }
    @Override
    public void updateMotorConfig() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(intakeMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeArmMotor.getConfigurator().apply(intakeArmMotorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    @Override
    public void setIntakeMotorConfig(TalonFXConfiguration config) {
        intakeMotorConfig = config.clone();
    }

    @Override
    public void setIntakeArmMotorConfig(TalonFXConfiguration config) {
        intakeArmMotorConfig = config.clone();
    }

    @Override
    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }

    @Override
    public TalonFX getIntakeArmMotor() {
        return intakeArmMotor;
    }

    @Override
    public double getIntakeVelocity() {
        return intakeVelocity;
    }

    @Override
    public double getIntakeArmPosition() {
        return intakeArmPosition;
    }

    @Override
    public void intakeSetPower(double power) {
        intakeMotor.set(power);
    }

    @Override
    public void setIntakeSpeed(double velocity) {
        intakeMotor.setControl(
            intakeVelocityControl.withVelocity(velocity*(IntakeConstants.INTAKE_GEAR_REDUCTION))
        );
    }

    @Override
    public void intakeStop() {
        intakeMotor.set(0.0);
    }

    @Override
    public void testIntake() {
        setIntakeSpeed(testIntakeGoal);
    }

    @Override
    public void intakeArmSetPower(double power) {
        intakeArmMotor.set(power);
    }

    @Override
    public void setIntakeArmPosition(double position) {
        intakeArmMotor.setControl(
            intakeArmPositionControl.withPosition(position*(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION))
        );
    }

    @Override
    public void intakeArmZero() {
        setIntakeArmPosition(IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE.in(Rotations));
    }

    @Override
    public void testArmIntake() {
        setIntakeArmPosition(testIntakeArmGoal);
    }

    @Override
    public void updateVariables() {
        intakeVelocity = intakeMotor.getVelocity().getValue().div(IntakeConstants.INTAKE_GEAR_REDUCTION).in(RotationsPerSecond);
        intakeArmPosition = intakeArmMotor.getPosition().getValue().div(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION).in(Rotations);
    }

    @Override
    public void update() {
        updateVariables();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("IntakeHardware");

    builder.addDoubleProperty("Intake Velocity (RPM)", () -> TelemetryConstants.roundTelemetry(intakeVelocity*60), null);
    builder.addDoubleProperty("Intake Arm Position (deg)", () -> TelemetryConstants.roundTelemetry(intakeArmPosition), null);

        builder.addDoubleProperty("Test Intake Goal RPM", () -> testIntakeGoal*60,
            (value) -> testIntakeGoal = (value / 60.0));
        
        builder.addDoubleProperty("Test Intake Arm Goal Degrees", () -> testIntakeArmGoal*360,
            (value) -> testIntakeArmGoal = value/360.0);

        // Allow tuning of the arm PID/FF slot0 values from the dashboard
        builder.addDoubleProperty("Intake Arm KS", () -> intakeArmMotorConfig.Slot0.kS, (value) -> {
            intakeArmMotorConfig.Slot0.kS = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KV", () -> intakeArmMotorConfig.Slot0.kV, (value) -> {
            intakeArmMotorConfig.Slot0.kV = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KP", () -> intakeArmMotorConfig.Slot0.kP, (value) -> {
            intakeArmMotorConfig.Slot0.kP = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KI", () -> intakeArmMotorConfig.Slot0.kI, (value) -> {
            intakeArmMotorConfig.Slot0.kI = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KD", () -> intakeArmMotorConfig.Slot0.kD, (value) -> {
            intakeArmMotorConfig.Slot0.kD = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake Arm KG", () -> intakeArmMotorConfig.Slot0.kG, (value) -> {
            intakeArmMotorConfig.Slot0.kG = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KS", () -> intakeMotorConfig.Slot0.kS, (value) -> {
            intakeMotorConfig.Slot0.kS = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KV", () -> intakeMotorConfig.Slot0.kV, (value) -> {
            intakeMotorConfig.Slot0.kV = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KP", () -> intakeMotorConfig.Slot0.kP, (value) -> {
            intakeMotorConfig.Slot0.kP = value;
            updateMotorConfig();
        });
        
        builder.addDoubleProperty("Intake KI", () -> intakeMotorConfig.Slot0.kI, (value) -> {
            intakeMotorConfig.Slot0.kI = value;
            updateMotorConfig();
        });

        builder.addDoubleProperty("Intake KD", () -> intakeMotorConfig.Slot0.kD, (value) -> {
            intakeMotorConfig.Slot0.kD = value;
            updateMotorConfig();
        });

        
    }
}
