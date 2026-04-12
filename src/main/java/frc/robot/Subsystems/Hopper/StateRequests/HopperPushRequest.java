package frc.robot.Subsystems.Hopper.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.States.HopperStates.HopperControlState;
import frc.robot.Subsystems.Hopper.HopperSubsystem;

public class HopperPushRequest extends InstantCommand {

    private final HopperSubsystem hopperSubsystem;

    public HopperPushRequest(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize() {
        hopperSubsystem.setHopperState(HopperControlState.PUSH);
    }
}