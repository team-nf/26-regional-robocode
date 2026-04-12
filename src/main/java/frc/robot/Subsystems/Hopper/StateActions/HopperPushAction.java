package frc.robot.Subsystems.Hopper.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Subsystems.Hopper.HopperSubsystem;

public class HopperPushAction extends Command {

    private final HopperSubsystem hopperSubsystem;

    public HopperPushAction(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization logic for pushing
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !hopperSubsystem.isHopperState(HopperControlState.PUSH); // Modify this condition as needed
    }
}