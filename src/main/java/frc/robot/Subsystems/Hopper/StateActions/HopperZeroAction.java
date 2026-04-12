package frc.robot.Subsystems.Hopper.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Subsystems.Hopper.HopperSubsystem;

public class HopperZeroAction extends Command {

  private final HopperSubsystem hopperSubsystem;

  public HopperZeroAction(HopperSubsystem hopperSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void initialize() {
    hopperSubsystem.zero();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !hopperSubsystem.isHopperState(HopperControlState.ZERO);
  }
}
