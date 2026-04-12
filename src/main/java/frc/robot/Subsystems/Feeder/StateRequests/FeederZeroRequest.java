// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Feeder.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.States.FeederStates.FeederControlState;
import frc.robot.Subsystems.Feeder.FeederSubsystem;

import static edu.wpi.first.units.Units.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederZeroRequest extends InstantCommand {

  private final FeederSubsystem feederSubsystem;

  public FeederZeroRequest(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setFeederState(FeederControlState.ZERO);
  }
}
