package frc.robot.Subsystems.Intake.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Constants.States.IntakeStates;

public class IntakeWithOffsetAction extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeWithOffsetAction(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Use the currently stored offset in intakeData
        intakeSubsystem.intakeWithOffset();
    }

    @Override
    public boolean isFinished() {
        // Stay active while in INTAKE_WITH_OFFSET state; state machine will change it
        return intakeSubsystem.intakeData.intakeControlState != IntakeStates.IntakeControlState.INTAKE_WITH_OFFSET;
    }
}
