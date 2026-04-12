package frc.robot.Subsystems.Intake.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Constants.States.IntakeStates;

public class IntakeWithOffsetRequest extends InstantCommand {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeWithOffsetRequest(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeState(IntakeStates.IntakeControlState.INTAKE_WITH_OFFSET);
    }
}
