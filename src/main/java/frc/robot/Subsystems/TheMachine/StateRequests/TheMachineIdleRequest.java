// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.TheMachine.StateRequests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.TheMachine.TheMachine;
import frc.robot.Constants.States.TheMachineStates.TheMachineControlState;

public class TheMachineIdleRequest extends InstantCommand {

  private final TheMachine theMachine;

  public TheMachineIdleRequest(TheMachine theMachine) {
    this.theMachine = theMachine;
  }

  @Override
  public void initialize() {
    theMachine.setState(TheMachineControlState.IDLE);
  }
}