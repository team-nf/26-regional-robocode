package frc.robot.Subsystems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineGetReadyActionPas {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterTestRequest(),
        theMachine.feederReverseRequest(),
        theMachine.hopperZeroRequest(),
        theMachine.intakeDeployRequest()
    ).until(() -> (theMachine.getState() != TheMachineControlState.GET_READY_PAS));
  }
}
