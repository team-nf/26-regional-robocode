package frc.robot.Subsystems.Feeder.Hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface FeederHardware extends Sendable{

    public void updateMotorConfig();
    public void setFeederMotorConfig(TalonFXConfiguration config);

    public TalonFX getFeederMotor();

    public double getFeederVelocity();

    public void feederSetPower(double power);
    public void setFeederSpeed(double velocity);
    public void feederStop();
    
    public void testFeeder();

    public void updateVariables();
    public void update();
}
