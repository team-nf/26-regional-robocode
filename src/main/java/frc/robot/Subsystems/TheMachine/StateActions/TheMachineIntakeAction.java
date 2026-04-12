package frc.robot.Subsystems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineIntakeAction {

  public static Command get(TheMachine theMachine) {
    return new ParallelCommandGroup(
        theMachine.shooterRestRequest(),
        theMachine.feederReverseRequest(),
        theMachine.hopperZeroRequest(),
        new ConditionalCommand(theMachine.intakeIntakeRequest(), theMachine.intakeWithOffsetRequest(), theMachine::isIntakeWithoutOffset)
    ).until(() -> (theMachine.getState() != TheMachineControlState.INTAKE));
  }
}