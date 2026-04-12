// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Feeder.Hardware;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.FeederConstants;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class FeederSimHardware extends FeederRealHardware {

    private final DCMotor feederDcMotor;
    private final LinearSystem<N2, N1, N2> feederSystem;
    private final DCMotorSim feederSim;

    private boolean isSimulationInitialized = false;

    public FeederSimHardware() {
        super();

        feederDcMotor = DCMotor.getKrakenX60(1);
        feederSystem = LinearSystemId.createDCMotorSystem(
            feederDcMotor,
            FeederConstants.FEEDER_MOMENT_OF_INERTIA.in(KilogramSquareMeters), // Moment of Inertia
            FeederConstants.FEEDER_GEAR_REDUCTION);

        feederSim = new DCMotorSim(feederSystem, feederDcMotor);
    }

    @Override
    public void update() {
        super.update();

        if (!isSimulationInitialized) {
            isSimulationInitialized = true;
            feederMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            feederMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
            final var feederMotorSimState = feederMotor.getSimState();

            feederMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            
            feederSim.setInputVoltage(feederMotorSimState.getMotorVoltage());
            feederSim.update(0.002);

            feederMotorSimState.setRawRotorPosition(
                feederSim.getAngularPosition().times(FeederConstants.FEEDER_GEAR_REDUCTION));

            feederMotorSimState.setRotorVelocity(
                feederSim.getAngularVelocity().times(FeederConstants.FEEDER_GEAR_REDUCTION));
        }   
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("FeederSim", () -> feederSim.getAngularPosition().per(Degrees) + " " + FeederConstants.FEEDER_MOMENT_OF_INERTIA.per(KilogramMetersPerSecond), null);
    }

}
