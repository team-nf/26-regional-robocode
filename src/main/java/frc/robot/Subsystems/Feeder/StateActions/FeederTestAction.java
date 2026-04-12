// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Feeder.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.FeederStates.FeederControlState;
import frc.robot.Subsystems.Feeder.FeederSubsystem;

public class FeederTestAction extends Command {

  private final FeederSubsystem feederSubsystem;

  public FeederTestAction(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    addRequirements(feederSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.test();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !feederSubsystem.isFeederState(FeederControlState.TEST);
  }
}
