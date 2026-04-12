package frc.robot.Subsystems.Shooter.Hardware;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class ShooterSimHardware extends ShooterRealHardware {

    // Simulations
    private final DCMotor shDcMotor;
    private final LinearSystem<N2, N1, N2> shFlywheelSystem;
    private final DCMotorSim shFlywheelSim;

    private final DCMotor hoodDcMotor;
    private final SingleJointedArmSim hoodArmSim;

    private boolean isSimulationInitialized = false;

    public ShooterSimHardware() {
        super();

        // Flywheel systems and sims
        shDcMotor = DCMotor.getKrakenX60(ShooterConstants.NUMBER_OF_FLYWHEEL_MOTORS);
        shFlywheelSystem = LinearSystemId.createDCMotorSystem(
            shDcMotor,
            ShooterConstants.FLYWHEEL_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            ShooterConstants.FLYWHEEL_GEAR_REDUCTION);

        shFlywheelSim = new DCMotorSim(shFlywheelSystem, shDcMotor);

        // Hood systems and sims
        hoodDcMotor = DCMotor.getKrakenX60(1);
        hoodArmSim = new SingleJointedArmSim(
            hoodDcMotor,
            ShooterConstants.HOOD_GEAR_REDUCTION,
            ShooterConstants.HOOD_INERTIA.in(KilogramSquareMeters),
            ShooterConstants.HOOD_LENGTH.in(Meters),
            ShooterConstants.MIN_HOOD_ANGLE.in(Radians),
            ShooterConstants.MAX_HOOD_ANGLE.in(Radians),
            true,
            ShooterConstants.MIN_HOOD_ANGLE.in(Radians),
            0.0,
            0.0
        );
                
    }

    @Override
    public void update() {
        super.update();
        if (!isSimulationInitialized) {
            isSimulationInitialized = true;
            firstShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            firstShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            secondShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            secondShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            thirdShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            thirdShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            fourthShootMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            fourthShootMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            hoodMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            hoodMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
            final var firstShootMotorSimState = firstShootMotor.getSimState();
            final var secondShootMotorSimState = secondShootMotor.getSimState();
            final var thirdShootMotorSimState = thirdShootMotor.getSimState();
            final var fourthShootMotorSimState = fourthShootMotor.getSimState();

            /* First set the supply voltage of all the devices */
            firstShootMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            secondShootMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            thirdShootMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            fourthShootMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            /* Then calculate the new velocity of the simulated flywheel */
            shFlywheelSim.setInputVoltage(firstShootMotorSimState.getMotorVoltage());
            shFlywheelSim.update(0.002);

            /* Apply the new rotor velocity to the motors (before gear ratio) */
            firstShootMotorSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            secondShootMotorSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            thirdShootMotorSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fourthShootMotorSimState.setRawRotorPosition(
                shFlywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            

            firstShootMotorSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            secondShootMotorSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            thirdShootMotorSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fourthShootMotorSimState.setRotorVelocity(
                shFlywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            /* Now do the same for the hood */
            final var hoodSimState = hoodMotor.getSimState();
            hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            hoodArmSim.setInput(hoodSimState.getMotorVoltage());
            hoodArmSim.update(0.002);

            hoodSimState.setRawRotorPosition(
                Radians.of(hoodArmSim.getAngleRads()).times(ShooterConstants.HOOD_GEAR_REDUCTION));

            hoodSimState.setRotorVelocity(
                RadiansPerSecond.of(hoodArmSim.getVelocityRadPerSec()).times(ShooterConstants.HOOD_GEAR_REDUCTION));

        }

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("HoodSim", () -> Radians.of(hoodArmSim.getAngleRads()).in(Degrees) + " deg, MOI: " + ShooterConstants.HOOD_INERTIA.in(KilogramSquareMeters) + " kg*m^2", null);
    }

}
