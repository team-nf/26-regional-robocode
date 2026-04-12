package frc.robot.Subsystems.Intake.Hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeHardware extends Sendable {

    public void updateMotorConfig();
    public void setIntakeMotorConfig(TalonFXConfiguration config);
    public void setIntakeArmMotorConfig(TalonFXConfiguration config);

    public TalonFX getIntakeMotor();
    public TalonFX getIntakeArmMotor();

    public double getIntakeVelocity();
    public double getIntakeArmPosition();

    public void intakeSetPower(double power);
    public void setIntakeSpeed(double velocity);
    public void intakeStop();
    public void testIntake();

    // Arm / position control (abstracted)
    public void intakeArmSetPower(double power);
    public void setIntakeArmPosition(double position);
    public void intakeArmZero();
    public void testArmIntake();

    public void updateVariables();
    public void update();
}
