package frc.robot.Subsystems.Hopper.Hardware;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.HopperConstants;

import static edu.wpi.first.units.Units.*;

public class HopperSimHardware extends HopperRealHardware {

    private final DCMotor hopperDcMotor;
    private final LinearSystem<N2, N1, N2> hopperSystem;
    private final DCMotorSim hopperSim;

    private boolean isSimulationInitialized = false;

    public HopperSimHardware() {
        super();

        hopperDcMotor = DCMotor.getKrakenX60(1);
        hopperSystem = LinearSystemId.createDCMotorSystem(
            hopperDcMotor,
            HopperConstants.HOPPER_MOMENT_OF_INERTIA.in(KilogramSquareMeters), // Moment of Inertia
            HopperConstants.HOPPER_GEAR_REDUCTION);

        hopperSim = new DCMotorSim(hopperSystem, hopperDcMotor);
    }

    @Override
    public void update() {
        super.update();

        if (!isSimulationInitialized) {
            isSimulationInitialized = true;
            hopperMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            hopperMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
            final var hopperMotorSimState = hopperMotor.getSimState();

            hopperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            
            hopperSim.setInputVoltage(hopperMotorSimState.getMotorVoltage());
            hopperSim.update(0.002);

            hopperMotorSimState.setRawRotorPosition(
                hopperSim.getAngularPosition().times(HopperConstants.HOPPER_GEAR_REDUCTION));

            hopperMotorSimState.setRotorVelocity(
                hopperSim.getAngularVelocity().times(HopperConstants.HOPPER_GEAR_REDUCTION));
        }   
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("HopperSim", () -> hopperSim.getAngularPosition().in(Degrees) + " deg, MOI: " + HopperConstants.HOPPER_MOMENT_OF_INERTIA.in(KilogramSquareMeters) + " kg*m^2", null);
    }

}
