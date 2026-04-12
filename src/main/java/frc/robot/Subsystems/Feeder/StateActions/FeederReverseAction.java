// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Feeder.StateActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.FeederStates.FeederControlState;
import frc.robot.Constants.States.ShooterStates.ShooterControlState;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederReverseAction extends Command {
  /** Creates a new ShooterIdleAction. */
  
  private final FeederSubsystem feederSubsystem;

  public FeederReverseAction(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    addRequirements(feederSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.reverse();
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
    return !feederSubsystem.isFeederState(FeederControlState.REVERSE);
  }
}
