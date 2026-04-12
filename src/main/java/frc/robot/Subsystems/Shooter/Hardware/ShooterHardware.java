package frc.robot.Subsystems.Shooter.Hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;

public interface ShooterHardware extends Sendable{

    public void updateMotorConfigs();
    public void setShootMotorConfig(TalonFXConfiguration config);
    public void setHoodMotorConfig(TalonFXConfiguration config);

    public double getTestFlywheelGoal();
    public double getTestHoodGoal();

    public TalonFX getFirstShootMotor();
    public TalonFX getSecondShootMotor();
    public TalonFX getThirdShootMotor();
    public TalonFX getFourthShootMotor();

    public double getFlywheelVelocityL();
    public double getFlywheelVelocityR();

    public void flywheelSetPower(double power);
    public void setFlywheelSpeed(double velocity);
    public void flywheelStop();

    public TalonFX getHoodMotor();

    public double getHoodPosition();

    public void setHoodAngle(double hoodAngle);
    public void hoodZero();

    public void setShooter(double velocity, double hoodAngle);
    public void testShooter();

    public void updateVariables();
    public void update();
}

