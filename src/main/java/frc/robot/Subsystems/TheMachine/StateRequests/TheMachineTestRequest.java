package frc.robot.Subsystems.TheMachine.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineTestRequest extends InstantCommand {

    private final TheMachine theMachine;

    public TheMachineTestRequest(TheMachine theMachine) {
        this.theMachine = theMachine;
    }

    @Override
    public void initialize() {
        theMachine.setState(TheMachineControlState.TEST);
    }
}