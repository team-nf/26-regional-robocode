package frc.robot.Subsystems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineReverseAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterZeroRequest(),
        theMachine.feederReverseRequest(),
        theMachine.hopperReverseRequest(),
        theMachine.intakeReverseRequest()
    ).until(() -> (theMachine.getState() != TheMachineControlState.REVERSE));
  }
}