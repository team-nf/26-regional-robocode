package frc.robot.Subsystems.Hopper.Hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.units.measure.AngularVelocity;

public interface HopperHardware extends Sendable {

    public void updateMotorConfig();
    public void setHopperMotorConfig(TalonFXConfiguration config);

    public TalonFX getHopperMotor();

    public double getHopperVelocity();

    public void hopperSetPower(double power);
    public void setHopperSpeed(double velocity);
    public void hopperStop();
    public void testHopper();

    public void updateVariables();
    public void update();
}
