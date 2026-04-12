package frc.robot.Subsystems.Intake.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.States.IntakeStates.IntakeControlState;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IntakeIntakeRequest extends InstantCommand {

  private final IntakeSubsystem intakeSubsystem;

  public IntakeIntakeRequest(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeState(IntakeControlState.INTAKE);
  }
}
