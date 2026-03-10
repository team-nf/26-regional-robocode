package frc.robot.Subsystems.TheMachine.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineShootAction {

  public static Command get(TheMachine theMachine) {

    Command commandWithShake = new ParallelCommandGroup(
        theMachine.shooterShootRequest(),
        theMachine.waitForShooter()
        .andThen(
            theMachine.feederFeedRequest(),
                    (theMachine.hopperReverseRequest()
                      .andThen(new WaitCommand(0.5))),

                      (theMachine.hopperFeedRequest()))
        .andThen(theMachine.intakeIdleBetweenRequest())
        .andThen(new WaitCommand(1))
        .andThen(theMachine.intakeFeedRequest())
        .andThen(new WaitCommand(2.5)));
        

    return commandWithShake.until(() -> (theMachine.getState() != TheMachineControlState.SHOOT));
  }
}