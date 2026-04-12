package frc.robot.Subsystems.Intake.Hardware;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeSimHardware extends IntakeRealHardware {

    private final DCMotor intakeDcMotor;
    private final LinearSystem<N2, N1, N2> intakeSystem;
    private final DCMotorSim intakeSim;
    
    private final DCMotor intakeArmDcMotor;
    private final SingleJointedArmSim intakeArmSim;

    private boolean isSimulationInitialized = false;

    public IntakeSimHardware() {
        super();

        intakeDcMotor = DCMotor.getKrakenX60(1);
        intakeSystem = LinearSystemId.createDCMotorSystem(
            intakeDcMotor,
            IntakeConstants.INTAKE_MOMENT_OF_INERTIA.in(KilogramSquareMeters), // Moment of Inertia
            IntakeConstants.INTAKE_GEAR_REDUCTION);

        intakeSim = new DCMotorSim(intakeSystem, intakeDcMotor);

        // Arm sim: model a single-jointed arm using default arm constants
        intakeArmDcMotor = DCMotor.getKrakenX60(1);
        intakeArmSim = new SingleJointedArmSim(
            intakeArmDcMotor,
            IntakeConstants.INTAKE_ARM_GEAR_REDUCTION,
            IntakeConstants.INTAKE_ARM_INERTIA,
            IntakeConstants.INTAKE_ARM_LENGTH.in(Meters),
            IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Radians), // min angle (deployed)
            IntakeConstants.INTAKE_ARM_START_ANGLE.in(Radians), // max angle (retracted)
            true,
            IntakeConstants.INTAKE_ARM_START_ANGLE.in(Radians),
            0.0,
            0.0
        );
    }

    @Override
    public void update() {
        super.update();
        if (!isSimulationInitialized) {
            isSimulationInitialized = true;
            getIntakeMotor().getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            getIntakeMotor().getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            getIntakeArmMotor().getSimState().Orientation = ChassisReference.Clockwise_Positive;
            getIntakeArmMotor().getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
            final var intakeMotorSimState = getIntakeMotor().getSimState();
            final var intakeArmMotorSimState = getIntakeArmMotor().getSimState();

            /* First set the supply voltage of the devices */
            intakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            intakeArmMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            /* Roller sim update */
            intakeSim.setInputVoltage(intakeMotorSimState.getMotorVoltage());
            intakeSim.update(0.002);

            intakeMotorSimState.setRawRotorPosition(
                intakeSim.getAngularPosition().times(IntakeConstants.INTAKE_GEAR_REDUCTION));
            intakeMotorSimState.setRotorVelocity(
                intakeSim.getAngularVelocity().times(IntakeConstants.INTAKE_GEAR_REDUCTION));

            /* Arm sim update */
            intakeArmSim.setInput(intakeArmMotorSimState.getMotorVoltage());
            intakeArmSim.update(0.002);

            intakeArmMotorSimState.setRawRotorPosition(
                Radians.of(intakeArmSim.getAngleRads()).times(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION));

            intakeArmMotorSimState.setRotorVelocity(
                RadiansPerSecond.of(intakeArmSim.getVelocityRadPerSec()).times(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION));
        }   
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("IntakeSim", () -> intakeSim.getAngularPosition().in(Degrees) + " deg, MOI: " + IntakeConstants.INTAKE_MOMENT_OF_INERTIA.in(KilogramSquareMeters) + " kg*m^2", null);
        builder.addStringProperty("IntakeArmSim", () -> Radians.of(intakeArmSim.getAngleRads()).per(Degrees) + " " + IntakeConstants.INTAKE_ARM_INERTIA, null);
    }

}
