package frc.robot.Subsystems.TheMachine.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;   
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;
import frc.robot.Subsystems.TheMachine.TheMachine;

public class TheMachineIntakeToggledRequest extends InstantCommand {

    private final TheMachine theMachine;

    public TheMachineIntakeToggledRequest(TheMachine theMachine) {
        this.theMachine = theMachine;
    }

    @Override
    public void initialize() {
        if (theMachine.isState(TheMachineControlState.INTAKE)) {
            theMachine.setState(TheMachineControlState.IDLE_DEPLOYED);
        } else {    
        theMachine.setState(TheMachineControlState.INTAKE);
       }
    }
}       